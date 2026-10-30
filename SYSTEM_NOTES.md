# Smart Parking: ESP32 car + ESP32 lot over Wi-Fi

Two boards, no router, no internet.

- **Lot controller** (`parking_controller/`): ESP32 that senses the 3 slots and the gate, hosts the Wi-Fi network `VIT-SmartParking`, and serves a dashboard.
- **Car** (`parking_car_esp32/`): ESP32 that joins that network, drives to the gate, takes a slot number, drives there, and parks.

Checked with a mock compile only (syntax and types, both ESP32 core 2.x and 3.x paths). Not run on hardware.

## How a run goes

1. The car powers on, joins Wi-Fi, waits 3 s, drives to the gate line (front sensor reads `GATE_FRONT_CM`).
2. It stops and sends `arrived`. The lot's entry sensor sees the car too.
3. After 1.5 s the lot picks the nearest free slot (slot 1 is nearest the gate) and queues `GO:n`.
4. The car polls `/car/cmd` every 0.4 s, gets `GO:n`, replies `ack`.
5. It drives straight until the front sensor reads `SLOT_FRONT_CM[n-1]` (the pre-mapped route).
6. The servo swings to the side; the car checks that the slot opening is there (side reading above `OPENING_CM`).
7. It turns right, creeps in until the back wall is `PARK_CM` away, sends `parked`.
8. The ceiling sensor over the slot confirms the car is there and the dashboard shows it.

## Wi-Fi protocol (lot = 192.168.4.1)

| Request | Meaning |
|---|---|
| `GET /car/cmd` | reply `NONE`, `GO:n` or `STOP` |
| `GET /car/event?e=arrived` | car is at the gate |
| `GET /car/event?e=ack&slot=n` | car heard `GO:n` |
| `GET /car/event?e=parked&slot=n` | car stopped inside slot n |
| `GET /car/event?e=failed&slot=n` | car gave up (`n=0`: never reached the gate) |

`GO:n` stays queued until the car acks, so a dropped packet just means it asks again. You can test the lot from a phone browser: join `VIT-SmartParking` (password `park1234`) and open `http://192.168.4.1/car/cmd`.

## Car wiring (ESP32 DevKit, 30-pin)

| ESP32 pin | Goes to |
|---|---|
| GPIO 25 | L298N ENA |
| GPIO 26 | L298N IN1 |
| GPIO 27 | L298N IN2 |
| GPIO 32 | L298N IN3 |
| GPIO 33 | L298N IN4 |
| GPIO 13 | L298N ENB |
| GPIO 18 | Servo signal |
| GPIO 19 | HC-SR04 TRIG |
| GPIO 34 | HC-SR04 ECHO **through a divider** (ECHO, 1 kΩ, junction to GPIO 34, 2 kΩ from junction to GND) |
| VIN | L298N 5V terminal (also feeds sensor VCC and servo red) |
| GND | L298N GND, battery −, sensor GND, servo GND |

Battery + goes to the L298N 12V terminal. Left motor pair on OUT1/OUT2, right pair on OUT3/OUT4. Remove the ENA/ENB jumper caps. Put a 470-1000 µF capacitor across 5V and GND near the servo. Full picture: `car_wiring_diagram.png`.

## Lot wiring (unchanged, minus the HC-05)

| ESP32 pin | Goes to |
|---|---|
| GPIO 13 / 36 | Entry sensor TRIG / ECHO |
| GPIO 14 / 39 | Slot 1 sensor TRIG / ECHO |
| GPIO 27 / 34 | Slot 2 sensor TRIG / ECHO |
| GPIO 26 / 35 | Slot 3 sensor TRIG / ECHO |
| GPIO 25 / 33 / 32 | Slot 1 / 2 / 3 LEDs (with resistors) |

**Every ECHO line needs a 1 kΩ + 2 kΩ divider.** GPIO 16/17 are free now; remove the HC-05 and anything on them.

## Geometry this assumes

- The corridor runs straight to a flat **end wall**. The car's **front sensor** measures its distance to that wall.
- Slots open on the **right** side of the corridor, slot 1 closest to the gate.
- The gate is a line across the corridor. The **entry sensor** is mounted on the **left** wall at that line, looking across at the car's side (the car passes within 15 cm of it). If you mount it differently, adjust `ENTRY_TRIGGER_CM` in the controller.
- The car starts behind the gate line, facing down the corridor.

## Calibration (do this in order)

1. Set `CALIBRATE_ONLY = true` in the car sketch and upload. The car does not drive; it prints `front` and `side` distances.
2. Servo: confirm which angle looks straight ahead and which looks at the slot side. Fix `SERVO_FORWARD` / `SERVO_SIDE` (use 5 instead of 175 if it looks left).
3. Carry the car to the **gate line**, centred in the corridor, and read `front`. Put it in `GATE_FRONT_CM`.
4. Place the car **centred on slot 1**, then slot 2, then slot 3, and read `front` each time. Put them in `SLOT_FRONT_CM`.
5. With the car in front of a slot opening, check `side` is above `OPENING_CM` (30) there, and below it (about 10-20) beside a divider wall.
6. Set `CALIBRATE_ONLY = false`. Test driving alone (lot off is fine: the car just waits at the gate).
7. If the car veers left, raise `LEFT_TRIM` (the left side is too slow). If it veers right, raise `RIGHT_TRIM`. A few units at a time.
8. Tune `TURN_90_MS` until the car turns a clean 90° on your floor.
9. Do a full run for each slot. Log how close to centre the car stops, over about 10 runs per slot. That's your results table.

## Known limits (say these in the review)

- The car only listens for commands while standing still (gate and idle), because a Wi-Fi request can block for most of a second. A `STOP` sent mid-route is not heard.
- If the car fails mid-route it stops where it is; it does not drive back to the gate.
- There is no exit flow yet.
- The 90° turn is timed, so it drifts as the battery drains.
- The stored distances only hold for this layout. Move a wall and you recalibrate.

## Libraries

Install **ESP32Servo** (Library Manager). Wi-Fi, HTTPClient and WebServer come with the ESP32 core. Board: **ESP32 Dev Module**.
