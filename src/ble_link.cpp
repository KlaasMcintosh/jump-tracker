#include "ble_link.h"

#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <cmath>

namespace ble_link {
namespace {
constexpr char kDeviceName[] = "JumpTracker-ESP32";
constexpr char kServiceUuid[] = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
constexpr char kJumpUuid[] = "8f3d0a51-2c1e-4a8b-9f0d-6b1c2e7a9d01";
constexpr char kStatusUuid[] = "8f3d0a51-2c1e-4a8b-9f0d-6b1c2e7a9d02";
constexpr uint8_t kJumpPacketVersion = 1;

BLECharacteristic* jumpCharacteristic = nullptr;
BLECharacteristic* statusCharacteristic = nullptr;
volatile bool connected = false;

class ServerCallbacks final : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    connected = true;
    Serial.println("ble    | dashboard connected");
  }

  void onDisconnect(BLEServer*) override {
    connected = false;
    Serial.println("ble    | dashboard disconnected, advertising again");
    BLEDevice::startAdvertising();
  }
};

uint16_t toFixed(float value, float scale) {
  const float scaled = roundf(value * scale);
  if (scaled <= 0) return 0;
  if (scaled >= 65535) return 65535;
  return static_cast<uint16_t>(scaled);
}

void putU16(uint8_t* out, uint16_t value) {
  out[0] = value & 0xFF;
  out[1] = value >> 8;
}

BLECharacteristic* addReadNotifyCharacteristic(BLEService* service, const char* uuid) {
  BLECharacteristic* characteristic = service->createCharacteristic(
      uuid, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  characteristic->addDescriptor(new BLE2902());
  return characteristic;
}
}  // namespace

void begin() {
  BLEDevice::init(kDeviceName);
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService* service = server->createService(kServiceUuid);
  jumpCharacteristic = addReadNotifyCharacteristic(service, kJumpUuid);
  statusCharacteristic = addReadNotifyCharacteristic(service, kStatusUuid);
  uint8_t empty[5] = {};
  statusCharacteristic->setValue(empty, sizeof(empty));
  service->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(kServiceUuid);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.printf("ble    | advertising as %s\n", kDeviceName);
}

bool isConnected() { return connected; }

void publishJump(const JumpResult& jump) {
  uint8_t packet[9];
  packet[0] = kJumpPacketVersion;
  putU16(&packet[1], static_cast<uint16_t>(jump.number));
  putU16(&packet[3], toFixed(jump.flightTimeMs, 10));
  putU16(&packet[5], toFixed(jump.heightCm, 10));
  putU16(&packet[7], toFixed(jump.peakImpactG, 100));
  jumpCharacteristic->setValue(packet, sizeof(packet));
  if (connected) jumpCharacteristic->notify();
}

void publishStatus(const JumpDetector& detector, JumpDetector::Event event) {
  const bool rejected = event == JumpDetector::Event::Rejected;
  uint8_t packet[5];
  packet[0] = static_cast<uint8_t>(detector.state());
  packet[1] = static_cast<uint8_t>(event);
  packet[2] = rejected ? static_cast<uint8_t>(detector.lastRejectReason()) : 0;
  putU16(&packet[3], rejected ? toFixed(detector.lastRejectedFlightMs(), 1) : 0);
  statusCharacteristic->setValue(packet, sizeof(packet));
  if (connected) statusCharacteristic->notify();
}

}  // namespace ble_link
