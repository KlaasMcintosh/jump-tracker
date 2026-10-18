# Jump Tracker

Chest-worn vertical jump tester. An ESP32 reads an ADXL345 accelerometer at 200 Hz,
detects takeoff and landing, and computes jump height from flight time
(h = g·t² / 8). Results stream over Serial and BLE to a Web Bluetooth dashboard.

**Dashboard:** https://klaasmcintosh.github.io/jump-tracker/ (Chrome on Android or desktop)

## Hardware

- ESP32 NodeMCU (WROOM-32, 38-pin)
- Seeed Grove ADXL345 (I2C 0x53): GND → GND, VCC → 3V3, SDA → GPIO 21, SCL → GPIO 22
- Power: USB power bank

## Layout

| Path | Contents |
|---|---|
| `lib/jump_detector/` | Hardware-independent takeoff/landing state machine |
| `src/main.cpp` | ADXL345 FIFO sampling, serial output and commands |
| `src/ble_link.*` | BLE GATT service (packet format documented in `ble_link.h`) |
| `test/` | Host unit tests for the detector |
| `web/index.html` | Web Bluetooth dashboard, deployed to GitHub Pages |

## Build

```sh
pio run -t upload          # flash the ESP32
pio device monitor         # 115200 baud; r = raw stream, s = status, x = reset
pio test -e native         # detector unit tests on the host
```
