/*
  Step 1: motor test (ESP32 + L298N)
  Runs each side forward and back, then spins. Full speed, no PWM yet.
  Take the car OFF the floor (wheels in the air) for the first run.
*/
const int ENA = 25, IN1 = 26, IN2 = 27;   // left side
const int ENB = 13, IN3 = 32, IN4 = 33;   // right side

void side(int en, int a, int b, int dir) {  // dir: 1 forward, -1 back, 0 stop
  digitalWrite(a, dir > 0);
  digitalWrite(b, dir < 0);
  digitalWrite(en, dir != 0);
}

void setup() {
  int pins[] = {ENA, IN1, IN2, ENB, IN3, IN4};
  for (int p : pins) pinMode(p, OUTPUT);
  Serial.begin(115200);
}

void step(const char* name, int left, int right) {
  Serial.println(name);
  side(ENA, IN1, IN2, left);
  side(ENB, IN3, IN4, right);
  delay(1500);
  side(ENA, IN1, IN2, 0);
  side(ENB, IN3, IN4, 0);
  delay(800);
}

void loop() {
  step("LEFT forward",   1,  0);
  step("RIGHT forward",  0,  1);
  step("BOTH forward",   1,  1);
  step("BOTH backward", -1, -1);
  step("SPIN right",     1, -1);
  step("SPIN left",     -1,  1);
  Serial.println("--- done, repeating in 3 s ---");
  delay(3000);
}
