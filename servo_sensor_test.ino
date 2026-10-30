/*
  servo_sensor_test.ino  -  ESP32 obstacle-avoiding car, for testing the
  servo and sensor (no Wi-Fi, no parking logic).

  Drives forward. When something is closer than SAFE_CM it stops, backs up a
  little, swings the servo left and right, measures both, and turns toward
  the side with more room. If both sides are blocked it does a U-turn.

  TEST IN THIS ORDER
    1. MOTORS_ENABLED = false, car on the bench, USB plugged in. Open the
       Serial Monitor at 115200. Move your hand in front of the sensor and
       watch the numbers. Put a book on the car's LEFT side and wait for a
       scan: it should print "Left: small  Right: big" and "Turning RIGHT".
       If it picks the wrong side, set SWAP_SERVO_SIDES = true.
    2. MOTORS_ENABLED = true, wheels in the air, battery on.
    3. Wheels on the floor, with 2 m of clear space.

  Pins:  motors ENA 25 IN1 26 IN2 27 | ENB 13 IN3 32 IN4 33
         servo 18 | sensor TRIG 19, ECHO 34 (sensor on 3V3, or 5V + divider)
  Library: ESP32Servo
*/
#include <ESP32Servo.h>

const int ENA = 25, IN1 = 26, IN2 = 27;     // left side
const int ENB = 13, IN3 = 32, IN4 = 33;     // right side
const int SERVO_PIN = 18;
const int TRIG = 19, ECHO = 34;

// ---- change these ----
const bool MOTORS_ENABLED   = true;         // false = bench test, wheels never move
const bool SWAP_SERVO_SIDES = false;        // true if the car turns toward the obstacle
const int  SERVO_CENTER = 90;
const int  SERVO_LEFT   = 150;
const int  SERVO_RIGHT  = 30;
const float SAFE_CM     = 25.0;             // avoid anything closer than this
const float TIE_CM      = 5.0;              // left/right within this = a tie
const int  DRIVE_SPEED  = 150;              // 0-255
const int  CREEP_SPEED  = 120;
const int  TURN_SPEED   = 170;
const int  TURN_90_MS   = 1400;             // from your ramp test
const int  BACKUP_MS    = 400;
const int  LEFT_TRIM    = 0;                // trim adds speed to that side: veers left = raise LEFT_TRIM
const int  RIGHT_TRIM   = 0;
const bool SOFT_START   = true;
const int  RAMP_STEP    = 40;
const unsigned long RAMP_MS = 30;

const int PWM_FREQ = 1000, PWM_BITS = 8;
const int CH_ENA = 4, CH_ENB = 5;           // only used on ESP32 core 2.x

Servo scanner;
int  curL = 0, curR = 0;
int  noEcho = 0;
bool lastTurnWasLeft = false;

// ------------------------------------------------------------- motors
void pwmWrite(int pin, int ch, int duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(ch, duty);
#endif
}

int rampSide(int cur, int target, bool stepNow) {
  if (!SOFT_START || target == 0) return target;
  int sign = target > 0 ? 1 : -1, mag = abs(target);
  int curMag = (cur * sign > 0) ? abs(cur) : 0;
  if (curMag >= mag) return target;
  if (!stepNow) return sign * curMag;
  return sign * min(mag, curMag + RAMP_STEP);
}

int trimmed(int s, int trim) {
  if (s == 0) return 0;
  int mag = constrain(abs(s) + trim, 0, 255);
  return s > 0 ? mag : -mag;
}

void drive(int left, int right) {           // -255..255, negative = reverse
  if (!MOTORS_ENABLED) { left = 0; right = 0; }
  static unsigned long lastStep = 0;
  bool stepNow = millis() - lastStep >= RAMP_MS;
  if (stepNow) lastStep = millis();
  left  = rampSide(curL, trimmed(left,  LEFT_TRIM),  stepNow);
  right = rampSide(curR, trimmed(right, RIGHT_TRIM), stepNow);
  curL = left; curR = right;
  digitalWrite(IN1, left  > 0); digitalWrite(IN2, left  < 0);
  digitalWrite(IN3, right > 0); digitalWrite(IN4, right < 0);
  pwmWrite(ENA, CH_ENA, abs(left));
  pwmWrite(ENB, CH_ENB, abs(right));
}

void halt() { drive(0, 0); }

void driveFor(int left, int right, unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) { drive(left, right); delay(10); }
}

// ------------------------------------------------------------- sensor
float pingOnce() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(3);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long us = pulseIn(ECHO, HIGH, 25000UL);
  return us == 0 ? -1.0 : us / 58.0;
}

float readCm() {                            // median of 3, -1 if no echo
  float v[3]; int n = 0;
  for (int i = 0; i < 3; i++) { float d = pingOnce(); if (d > 0) v[n++] = d; delay(8); }
  if (n == 0) return -1.0;
  if (n == 1) return v[0];
  if (n == 2) return min(v[0], v[1]);
  float hi = max(v[0], max(v[1], v[2])), lo = min(v[0], min(v[1], v[2]));
  return v[0] + v[1] + v[2] - hi - lo;
}

// Swing the servo, wait for it to arrive, then measure. No echo counts as
// "far" here (400 cm), but it is printed so you can see it happen.
float scanAt(int angle) {
  scanner.write(angle);
  delay(450);
  float d = readCm();
  if (d < 0) d = readCm();
  if (d < 0) { Serial.println("  (no echo at " + String(angle) + " deg, counting as far)"); d = 400; }
  return d;
}

// --------------------------------------------------------------- moves
void spin(bool left, int ms) {
  Serial.println(left ? "Turning LEFT" : "Turning RIGHT");
  if (left) driveFor(-TURN_SPEED,  TURN_SPEED, ms);
  else      driveFor( TURN_SPEED, -TURN_SPEED, ms);
  halt();
  delay(200);
}

void avoid() {
  Serial.println("--- obstacle: stop, back up, scan ---");
  halt(); delay(200);
  driveFor(-CREEP_SPEED, -CREEP_SPEED, BACKUP_MS);
  halt(); delay(150);

  int leftAngle  = SWAP_SERVO_SIDES ? SERVO_RIGHT : SERVO_LEFT;
  int rightAngle = SWAP_SERVO_SIDES ? SERVO_LEFT  : SERVO_RIGHT;
  float l = scanAt(leftAngle);
  float r = scanAt(rightAngle);
  scanner.write(SERVO_CENTER);
  Serial.println("Left: " + String(l, 0) + " cm   Right: " + String(r, 0) + " cm");

  bool deadEnd = (l <= SAFE_CM && r <= SAFE_CM);
  bool goLeft;
  if (fabs(l - r) <= TIE_CM) goLeft = !lastTurnWasLeft;      // tie: alternate
  else                       goLeft = l > r;
  lastTurnWasLeft = goLeft;

  if (deadEnd) Serial.println("Dead end: U-turn");
  spin(goLeft, deadEnd ? TURN_90_MS * 2 : TURN_90_MS);
  delay(300);                               // let the servo reach centre before sensing
}

// --------------------------------------------------------------- setup
void setup() {
  Serial.begin(115200);
  int outs[] = {IN1, IN2, IN3, IN4, TRIG};
  for (int i = 0; i < 5; i++) pinMode(outs[i], OUTPUT);
  pinMode(ECHO, INPUT);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(ENA, PWM_FREQ, PWM_BITS);
  ledcAttach(ENB, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(CH_ENA, PWM_FREQ, PWM_BITS); ledcAttachPin(ENA, CH_ENA);
  ledcSetup(CH_ENB, PWM_FREQ, PWM_BITS); ledcAttachPin(ENB, CH_ENB);
#endif
  halt();
  scanner.setPeriodHertz(50);
  scanner.attach(SERVO_PIN, 500, 2400);
  scanner.write(SERVO_CENTER);

  Serial.println(MOTORS_ENABLED ? "Motors ON. Starting in 5 s, put the car down."
                                : "BENCH MODE: motors disabled, readings and decisions only.");
  delay(MOTORS_ENABLED ? 5000 : 1000);
}

// ---------------------------------------------------------------- loop
void loop() {
  float front = readCm();

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 300) {
    lastPrint = millis();
    Serial.println(front < 0 ? "Front: no echo" : "Front: " + String(front, 1) + " cm");
  }

  if (front < 0) {                          // nothing in range, or a wiring problem
    if (++noEcho > 8) {
      halt();
      Serial.println("No echo for a while. Check TRIG/ECHO wiring and the sensor power.");
      delay(500);
      return;
    }
    front = 400;                            // brief dropout: treat as clear
  } else {
    noEcho = 0;
  }

  if (front > SAFE_CM) {
    drive(front < SAFE_CM + 15 ? CREEP_SPEED : DRIVE_SPEED,
          front < SAFE_CM + 15 ? CREEP_SPEED : DRIVE_SPEED);
  } else {
    avoid();
    if (!MOTORS_ENABLED) delay(1500);       // bench mode: pause so the output is readable
  }
}
