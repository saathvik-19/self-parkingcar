/*
  servo_sweep_test.ino  -  ESP32, servo only (no sensor, no motors)

  Sweeps the servo slowly: 90 -> 150 -> 30 -> 90, forever.
  Servo wires:  signal (orange/yellow) -> GPIO 18
                red                    -> VIN (5V)
                brown/black            -> GND
  Library: ESP32Servo. Open the Serial Monitor at 115200.

  If "=== Servo sweep test started ===" keeps appearing, the ESP32 is
  resetting (power problem), not the servo failing.
*/
#include <ESP32Servo.h>

const int SERVO_PIN = 18;
Servo scanner;

void goTo(int from, int to) {              // slow move, gentle on the supply
  int step = (to > from) ? 2 : -2;
  for (int a = from; a != to; a += step) {
    scanner.write(a);
    delay(20);
  }
  scanner.write(to);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== Servo sweep test started ===");
  scanner.setPeriodHertz(50);
  scanner.attach(SERVO_PIN, 500, 2400);
  scanner.write(90);
  delay(1000);
}

void loop() {
  Serial.println("to 150");  goTo(90, 150);   delay(800);
  Serial.println("to 30");   goTo(150, 30);   delay(800);
  Serial.println("to 90");   goTo(30, 90);    delay(1500);
}
