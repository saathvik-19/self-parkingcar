# Bill of materials

## Car
| Qty | Part | Notes |
|---|---|---|
| 1 | ESP32 DevKit (30-pin) | Controller, Wi-Fi client |
| 1 | L298N motor driver | One channel per side |
| 4 | TT gear motors + wheels | Two in parallel per side |
| 1 | SG90 micro servo | Rotates the sensor |
| 1 | HC-SR04 ultrasonic sensor | Mounted on the servo |
| 1 | 1 kΩ + 2 kΩ resistors | Echo voltage divider (5 V to 3.3 V) |
| 2 | 18650 Li-ion cells + holder + switch | Motor supply (about 7.4 V) |
| 1 | Power bank + USB cable | Powers the ESP32 separately from the motors |
| 1 | Cardboard box chassis, jumper wires, mini breadboard | |

## Parking lot controller (ESP32)
| Qty | Part |
|---|---|
| 1 | ESP32 DevKit |
| 4 | HC-SR04 sensors (entry + 3 slots), each Echo line through a 1 kΩ + 2 kΩ divider |
| 3 | LEDs with resistors (slot status) |
| 1 | Model: corridor, 3 slots, ceiling frame |
