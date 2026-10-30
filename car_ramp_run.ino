/*
  car_ramp_run.ino  -  ESP32 + L298N, motors only (no sensors, no Wi-Fi)

  Power on -> wait 5 s -> forward 5 s -> turn left -> turn right ->
  turn 180 -> forward 5 s -> stop and stay stopped.
  (Left then right cancels out, so after the 180 the second leg heads back
  the way it came.) Press the EN (reset) button on the ESP32 to run again.

  Wiring: ENA 25, IN1 26, IN2 27, ENB 13, IN3 32, IN4 33
  Remove the ENA/ENB jumper caps on the L298N.
  Leave about 2 m of clear floor in front of the car.
*/

const int ENA = 25, IN1 = 26, IN2 = 27;   // left side
const int ENB = 13, IN3 = 32, IN4 = 33;   // right side

// ---- change these ----
const int  START_DELAY_MS = 5000;   // wait before moving
const int  DRIVE_SPEED    = 150;    // 0-255 for the straight legs
const int  TURN_SPEED     = 170;    // 0-255 for spinning in place
const int  FORWARD_MS     = 5000;   // length of each straight leg (includes the ramp-up)
const int  RAMP_MS        = 800;    // time to speed up at the start of a move
const int  TURN_90_MS     = 700;    // time for a 90 degree spin. CALIBRATE THIS.
const int  PAUSE_MS       = 600;    // rest between moves so the car settles
const bool TURN_180_LEFT  = true;   // which way the 180 spins
const bool GO_FORWARD     = true;   // set false if the whole car drives backward
const int  LEFT_TRIM      = 0;      // trim ADDS speed to that side. Car veers LEFT = left side
const int  RIGHT_TRIM     = 0;      //   is too slow: raise LEFT_TRIM. Veers RIGHT: raise RIGHT_TRIM.

const int PWM_FREQ = 1000, PWM_BITS = 8;
const int CH_ENA = 4, CH_ENB = 5;   // only used on ESP32 core 2.x

int curL = 0, curR = 0;             // signed speed each side is getting now

void pwmWrite(int pin, int ch, int duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(ch, duty);
#endif
}

// Signed speeds, -255..255. Positive = forward. GO_FORWARD flips everything
// together, so forward AND the turn directions stay consistent.
void drive(int left, int right) {
  if (!GO_FORWARD) { left = -left; right = -right; }
  curL = left; curR = right;
  int l = left  != 0 ? constrain(abs(left)  + LEFT_TRIM,  0, 255) : 0;
  int r = right != 0 ? constrain(abs(right) + RIGHT_TRIM, 0, 255) : 0;
  digitalWrite(IN1, left  > 0); digitalWrite(IN2, left  < 0);
  digitalWrite(IN3, right > 0); digitalWrite(IN4, right < 0);
  pwmWrite(ENA, CH_ENA, l);
  pwmWrite(ENB, CH_ENB, r);
}

void stopNow() { drive(0, 0); }

// Smoothly change from the current speeds to new ones over ms.
void rampTo(int left, int right, int ms) {
  int fromL = GO_FORWARD ? curL : -curL;     // undo the flip so we ramp in "logical" units
  int fromR = GO_FORWARD ? curR : -curR;
  unsigned long t0 = millis();
  while (millis() - t0 < (unsigned long)ms) {
    float f = (float)(millis() - t0) / ms;
    drive(fromL + (int)((left - fromL) * f), fromR + (int)((right - fromR) * f));
    delay(15);
  }
  drive(left, right);
}

// Speed up, hold, then stop. totalMs counts from the start of the ramp.
void move(const char* name, int left, int right, int totalMs, int rampMs) {
  Serial.println(name);
  rampTo(left, right, rampMs);
  delay(totalMs - rampMs);
  stopNow();
  delay(PAUSE_MS);
}

void setup() {
  Serial.begin(115200);
  int pins[] = {IN1, IN2, IN3, IN4};
  for (int i = 0; i < 4; i++) pinMode(pins[i], OUTPUT);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(ENA, PWM_FREQ, PWM_BITS);
  ledcAttach(ENB, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(CH_ENA, PWM_FREQ, PWM_BITS); ledcAttachPin(ENA, CH_ENA);
  ledcSetup(CH_ENB, PWM_FREQ, PWM_BITS); ledcAttachPin(ENB, CH_ENB);
#endif
  stopNow();

  Serial.println("Starting in 5 s... put the car down.");
  delay(START_DELAY_MS);

  move("Forward 5 s",  DRIVE_SPEED,  DRIVE_SPEED, FORWARD_MS, RAMP_MS);
  move("Turn left",   -TURN_SPEED,   TURN_SPEED,  TURN_90_MS, 150);
  move("Turn right",   TURN_SPEED,  -TURN_SPEED,  TURN_90_MS, 150);

  if (TURN_180_LEFT) move("Turn 180 (left)",  -TURN_SPEED,  TURN_SPEED, TURN_90_MS * 2, 150);
  else               move("Turn 180 (right)",  TURN_SPEED, -TURN_SPEED, TURN_90_MS * 2, 150);

  move("Forward 5 s",  DRIVE_SPEED,  DRIVE_SPEED, FORWARD_MS, RAMP_MS);

  stopNow();
  Serial.println("Done. Press EN to run again.");
}

void loop() {
  // nothing: the car stays stopped
}
