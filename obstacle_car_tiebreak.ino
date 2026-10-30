/*
  Obstacle-avoiding 4WD car - v2 with tie-breaking
  Same wiring as before (L298N / L293D, HC-SR04 on servo).

  New in this version:
  1. If left and right are nearly equal (within TIE_MARGIN cm),
     the car alternates its turn direction instead of always turning right.
  2. If BOTH sides are blocked (dead end), it does a U-turn.
*/
#include <Servo.h>

// L298N pins
const int ENA = 5;
const int IN1 = 2;
const int IN2 = 3;
const int ENB = 11;  // PWM pin. NOT 10: Servo library disables PWM on pins 9 and 10
const int IN3 = 4;
const int IN4 = 8;

// Ultrasonic sensor + servo
const int TRIG = 7;
const int ECHO = 6;
const int SERVO_PIN = 9;

// Tuning
const int SAFE_DISTANCE = 25;  // cm
const int TIE_MARGIN = 5;      // cm: sides closer than this count as a tie
const int SPEED = 200;
const int TURN_TIME = 500;     // ms for a ~90 degree pivot (tune on real car)
const int BACK_TIME = 400;     // ms

const int LOOK_LEFT = 150;
const int LOOK_CENTER = 90;
const int LOOK_RIGHT = 30;

Servo head;
bool lastTurnWasLeft = false;  // remembers the previous turn for tie-breaking

long readDistanceCm() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duration = pulseIn(ECHO, HIGH, 30000);
  if (duration == 0) return 400;
  return duration * 0.034 / 2;
}

void drive(int left, int right) {
  digitalWrite(IN1, left > 0);   digitalWrite(IN2, left < 0);
  digitalWrite(IN3, right > 0);  digitalWrite(IN4, right < 0);
  analogWrite(ENA, abs(left));
  analogWrite(ENB, abs(right));
}

long lookAt(int angle) {
  head.write(angle);
  delay(500);
  return readDistanceCm();
}

// Decide which way to turn. Returns true for LEFT, false for RIGHT.
bool chooseTurnLeft(long left, long right) {
  if (abs(left - right) <= TIE_MARGIN) {
    return !lastTurnWasLeft;     // tie: alternate so it doesn't loop one way
  }
  return left > right;           // otherwise pick the bigger gap
}

void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  head.attach(SERVO_PIN);
  head.write(LOOK_CENTER);
  Serial.begin(9600);
}

void loop() {
  long front = lookAt(LOOK_CENTER);
  Serial.print("Front: "); Serial.print(front); Serial.println(" cm");

  if (front > SAFE_DISTANCE) {
    drive(SPEED, SPEED);
  } else {
    drive(0, 0);
    delay(200);
    drive(-SPEED, -SPEED);
    delay(BACK_TIME);
    drive(0, 0);

    long left  = lookAt(LOOK_LEFT);
    long right = lookAt(LOOK_RIGHT);
    head.write(LOOK_CENTER);
    Serial.print("Left: ");  Serial.print(left);
    Serial.print("  Right: "); Serial.println(right);

    bool turnLeft = chooseTurnLeft(left, right);
    bool deadEnd = (left <= SAFE_DISTANCE && right <= SAFE_DISTANCE);

    int turnTime = deadEnd ? TURN_TIME * 2 : TURN_TIME;  // U-turn if boxed in
    Serial.println(deadEnd ? "Dead end: U-turn" :
                   (turnLeft ? "Turning LEFT" : "Turning RIGHT"));

    if (turnLeft) drive(-SPEED, SPEED);
    else          drive(SPEED, -SPEED);
    delay(turnTime);
    drive(0, 0);

    lastTurnWasLeft = turnLeft;
  }
  delay(50);
}
