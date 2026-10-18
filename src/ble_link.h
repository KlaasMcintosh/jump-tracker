#pragma once

// BLE GATT link to the Web Bluetooth dashboard (web/index.html).
//
// Service 4fafc201-1fb5-459e-8fcc-c5c9c331914b, device name "JumpTracker-ESP32".
// All values are little-endian and fit the default 20-byte notification
// payload, so no MTU negotiation is needed.
//
// Jump characteristic   (read, notify) 9 bytes, sent once per accepted jump:
//   u8  version (= 1)
//   u16 jump number
//   u16 flight time      [0.1 ms]
//   u16 jump height      [0.1 cm]
//   u16 peak impact      [0.01 g]
//
// Status characteristic (read, notify) 5 bytes, sent on every detector event:
//   u8  detector state   0 settling, 1 armed, 2 flight, 3 landing
//   u8  last event       0 none, 1 armed, 2 takeoff, 3 jump, 4 rejected
//   u8  reject reason    0 none, 1 flight too short, 2 free fall too long, 3 no landing impact
//   u16 rejected flight  [ms]

#include "jump_detector.h"

namespace ble_link {

void begin();
bool isConnected();
void publishJump(const JumpResult& jump);
void publishStatus(const JumpDetector& detector, JumpDetector::Event event);

}  // namespace ble_link
