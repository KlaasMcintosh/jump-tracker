// Jump tracker: ADXL345 @ 200 Hz -> flight-time jump detection -> Serial + BLE.
//
// Samples are drained from the ADXL345's 32-entry FIFO, so a stalled loop()
// (later: BLE) delays processing by up to 160 ms without losing samples or
// skewing their timing. Sample timestamps come from the sensor's own clock,
// with its true rate calibrated continuously against the ESP32's crystal.
//
// Serial commands (115200 baud):  r = toggle raw magnitude stream (for Serial
// Plotter / threshold tuning),  s = status,  x = reset detector.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADXL345_U.h>
#include <cmath>

#include "ble_link.h"
#include "jump_detector.h"

namespace {
constexpr uint8_t kSdaPin = 21;
constexpr uint8_t kSclPin = 22;
constexpr uint32_t kI2cClockHz = 400000;
constexpr uint8_t kAdxlAddress = 0x53;

constexpr uint8_t kRegDataX0 = 0x32;
constexpr uint8_t kRegFifoCtl = 0x38;
constexpr uint8_t kRegFifoStatus = 0x39;
constexpr uint8_t kFifoModeBypass = 0x00;
constexpr uint8_t kFifoModeStream = 0x80;
constexpr uint8_t kFifoDepth = 32;
constexpr float kGPerLsb = 0.0039f;  // full-resolution mode, any range

constexpr double kNominalSamplePeriodUs = 5000.0;  // 200 Hz
constexpr uint32_t kRateCalibrationMinSamples = 400;

Adafruit_ADXL345_Unified accelerometer(12345);
JumpDetector detector;

bool rawStreamEnabled = false;
uint64_t sampleTimeUs = 0;           // timestamp of the latest sample (sensor clock)
double samplePeriodUs = kNominalSamplePeriodUs;
uint32_t calibrationStartUs = 0;
uint32_t calibrationSamples = 0;
uint32_t fifoOverflows = 0;
uint32_t i2cErrors = 0;

const char* stateName(JumpDetector::State state) {
  switch (state) {
    case JumpDetector::State::Settling: return "settling";
    case JumpDetector::State::Armed: return "armed";
    case JumpDetector::State::Flight: return "flight";
    case JumpDetector::State::Landing: return "landing";
  }
  return "?";
}

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kAdxlAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length) {
  Wire.beginTransmission(kAdxlAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(kAdxlAddress, length) != length) return false;
  for (uint8_t i = 0; i < length; ++i) buffer[i] = Wire.read();
  return true;
}

// One 6-byte burst pops exactly one FIFO entry and keeps X/Y/Z from the same
// sample (Adafruit's getEvent() reads each axis separately, which does not).
bool readSampleG(float& magnitudeG) {
  uint8_t raw[6];
  if (!readRegisters(kRegDataX0, raw, sizeof(raw))) return false;
  const float x = static_cast<int16_t>(raw[1] << 8 | raw[0]) * kGPerLsb;
  const float y = static_cast<int16_t>(raw[3] << 8 | raw[2]) * kGPerLsb;
  const float z = static_cast<int16_t>(raw[5] << 8 | raw[4]) * kGPerLsb;
  magnitudeG = sqrtf(x * x + y * y + z * z);
  return true;
}

void restartRateCalibration() {
  calibrationStartUs = micros();
  calibrationSamples = 0;
}

// Continuously estimates the ADXL345's real output data rate (its internal
// oscillator is not crystal accurate) so flight time is measured in true time.
void updateRateCalibration(uint8_t samplesRead) {
  calibrationSamples += samplesRead;
  if (calibrationSamples >= kRateCalibrationMinSamples) {
    samplePeriodUs = static_cast<double>(micros() - calibrationStartUs) / calibrationSamples;
  }
}

void printJump(const JumpResult& jump) {
  Serial.printf("JUMP #%lu | flight %6.1f ms | height %5.1f cm (%4.1f in) | peak impact %5.2f g\n",
                static_cast<unsigned long>(jump.number), jump.flightTimeMs, jump.heightCm,
                jump.heightIn, jump.peakImpactG);
}

void printStatus() {
  Serial.printf("status | state %s | ble %s | ODR %.2f Hz | fifo overflows %lu | i2c errors %lu\n",
                stateName(detector.state()), ble_link::isConnected() ? "connected" : "advertising",
                1e6 / samplePeriodUs,
                static_cast<unsigned long>(fifoOverflows), static_cast<unsigned long>(i2cErrors));
}

void handleEvent(JumpDetector::Event event) {
  if (event == JumpDetector::Event::None) return;
  if (event == JumpDetector::Event::Jump) ble_link::publishJump(detector.lastJump());
  ble_link::publishStatus(detector, event);

  switch (event) {
    case JumpDetector::Event::Armed:
      if (!rawStreamEnabled) Serial.println("ready  | standing still, jump when ready");
      break;
    case JumpDetector::Event::Jump:
      printJump(detector.lastJump());
      break;
    case JumpDetector::Event::Rejected:
      if (!rawStreamEnabled) {
        Serial.printf("ignored| %s (%.0f ms)\n", JumpDetector::describe(detector.lastRejectReason()),
                      detector.lastRejectedFlightMs());
      }
      break;
    case JumpDetector::Event::Takeoff:
    case JumpDetector::Event::None:
      break;
  }
}

void drainFifo() {
  uint8_t status = 0;
  if (!readRegisters(kRegFifoStatus, &status, 1)) {
    ++i2cErrors;
    return;
  }
  const uint8_t entries = status & 0x3F;
  if (entries == 0) return;
  // A full FIFO means stream mode may have overwritten unread samples.
  const bool overflowed = entries >= kFifoDepth;

  for (uint8_t i = 0; i < entries; ++i) {
    float magnitudeG;
    if (!readSampleG(magnitudeG)) {
      ++i2cErrors;
      return;
    }
    sampleTimeUs += static_cast<uint64_t>(samplePeriodUs + 0.5);
    const JumpDetector::Event event = detector.update(sampleTimeUs, magnitudeG);
    if (rawStreamEnabled) {
      Serial.printf("g:%.3f,state:%d\n", magnitudeG, static_cast<int>(detector.state()));
    }
    handleEvent(event);
  }

  if (overflowed) {
    // Lost samples would bias the rate estimate; start it over after this backlog.
    ++fifoOverflows;
    restartRateCalibration();
  } else {
    updateRateCalibration(entries);
  }
}

void handleSerialCommands() {
  while (Serial.available()) {
    switch (Serial.read()) {
      case 'r':
        rawStreamEnabled = !rawStreamEnabled;
        if (!rawStreamEnabled) Serial.println("raw stream off");
        break;
      case 's':
        printStatus();
        break;
      case 'x':
        detector.reset();
        Serial.println("detector reset");
        break;
      default:
        break;
    }
  }
}

void scanI2c(uint8_t sdaPin, uint8_t sclPin) {
  Wire.end();
  Wire.begin(sdaPin, sclPin);
  Serial.printf("I2C scan (SDA=GPIO%u, SCL=GPIO%u):", sdaPin, sclPin);
  uint8_t found = 0;
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" 0x%02X", address);
      ++found;
    }
  }
  Serial.println(found ? "" : " no devices found");
}

// Wiring diagnostics for when the ADXL345 does not answer. With the ESP32's
// weak internal pull-downs enabled, a line only reads HIGH if the powered
// Grove module's pull-up resistor is on it, i.e. the wire is really connected.
void printI2cDiagnostics() {
  Wire.end();
  pinMode(kSdaPin, INPUT_PULLDOWN);
  pinMode(kSclPin, INPUT_PULLDOWN);
  delay(5);
  Serial.printf("Sensor pull-ups seen: GPIO%u=%s  GPIO%u=%s  (YES = wire reaches a powered sensor)\n",
                kSdaPin, digitalRead(kSdaPin) ? "YES" : "NO", kSclPin,
                digitalRead(kSclPin) ? "YES" : "NO");
  scanI2c(kSdaPin, kSclPin);
  scanI2c(kSclPin, kSdaPin);  // finds the sensor if white/yellow are swapped
  Wire.end();
  Wire.begin(kSdaPin, kSclPin);
}

[[noreturn]] void haltWithError(const char* message) {
  while (true) {
    Serial.println(message);
    delay(2000);
  }
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nJump tracker");
  ble_link::begin();  // slow; do it before the sensor FIFO starts filling

  Wire.begin(kSdaPin, kSclPin);
  if (!accelerometer.begin(kAdxlAddress)) {
    printI2cDiagnostics();
    haltWithError("ERROR: ADXL345 not found at 0x53. Check SDA=21, SCL=22, 3V3, GND.");
  }
  Wire.setClock(kI2cClockHz);

  accelerometer.setRange(ADXL345_RANGE_16_G);  // also enables full-resolution mode
  accelerometer.setDataRate(ADXL345_DATARATE_200_HZ);
  // The sensor keeps its FIFO running across ESP32 resets; bypass mode clears
  // the stale backlog before streaming restarts.
  if (!writeRegister(kRegFifoCtl, kFifoModeBypass) || !writeRegister(kRegFifoCtl, kFifoModeStream)) {
    haltWithError("ERROR: failed to configure ADXL345 FIFO.");
  }

  restartRateCalibration();
  Serial.println("Commands: r = raw stream, s = status, x = reset. Stand still to arm.");
}

void loop() {
  handleSerialCommands();
  drainFifo();
  delay(2);  // FIFO holds 160 ms, so polling every few ms is plenty
}
