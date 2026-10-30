/* =====================================================================
   parking_car.ino   —  ESP32 on the car.  ONE sketch, two modes.
   ---------------------------------------------------------------------
   MODE 1  ROAM      drives around on the floor avoiding obstacles
   MODE 2  PARK      drives a fixed route into a chosen bay

   How a run goes:
     1. Power on. The car joins the lot's Wi-Fi and starts ROAMING on
        the floor. Put a hand in front of it and it turns away.
     2. Someone taps a bay on the website. The car HALTS wherever it is.
     3. You pick it up and set it on the start line.
     4. The car notices it is on the start line — the wall in front is
        back at the distance it should be, and holding steady — and
        drives the route by itself.
     5. It parks. The page shows a Release button.
     6. Release: it reverses out, turns, and goes back to roaming.

   WHY YOU HAVE TO CARRY IT TO THE START LINE
     The route works by measuring distance to a known wall from a known
     start point and heading. After roaming, the car has no idea where
     it is or which way it is facing, and with one ultrasonic sensor and
     no wheel encoders it cannot work that out. Rather than pretend, the
     car waits to be placed and then CHECKS that it has been placed
     correctly before it moves.

   THE LOT IS L-SHAPED:

        +-------------------------------------------+
        | STRAIGHT   | CORRIDOR    | CORRIDOR       |
        | _AHEAD     | _BAY[0]     | _BAY[1]        |
        +----   -----+----   ------+----   ---------+
        |                                           |
        |            <=== leg 2, facing EAST ===>   |
        |      ^                          EAST WALL |
        |      |  leg 1, facing NORTH               |
        |    [CAR]  start line                      |
        +-------------------------------------------+

   The three bays are still called Slot 1, 2 and 3 on the website. Which
   NUMBER sits in which POSITION is set by STRAIGHT_AHEAD_BAY and
   CORRIDOR_BAY below -- the route is described by position, so renumber
   there and nothing else has to change.

   Libraries: ESP32Servo. Board: "ESP32 Dev Module".
   ===================================================================== */

// ===================================================== SERVO SWITCH ==
//   0 = no servo at all. The sensor is bolted facing forward. The
//       ESP32Servo library is not even compiled in, so it cannot touch
//       the LEDC hardware the motors use. THIS IS THE DEFAULT.
//   1 = sweeping head. Read the note on LEDC channels further down
//       before you turn this on.
#define CAR_HAS_SERVO 1

#include <WiFi.h>
#include <HTTPClient.h>
#if CAR_HAS_SERVO
  #include <ESP32Servo.h>
#endif

// =============================================== the model, in numbers
const float CAR_WIDTH_CM  = 20.0;
const float CAR_LEN_CM    = 20.0;
const float BAY_DEPTH_CM  = 16.0;
const float CORRIDOR_CM   = 30.0;   // depth of the leg-2 corridor ("A")
const float CORRIDOR_MIN_CM = 24.1; // 10 + half the car's diagonal

// ---------------------------------------------------------------- pins
const int ENA = 25, IN1 = 26, IN2 = 27;     // left motors
const int ENB = 13, IN3 = 32, IN4 = 33;     // right motors
const int SERVO_PIN = 18;
const int TRIG = 19;
const int ECHO = 34;                        // sensor on 3V3, no divider

const int PWM_FREQ = 1000, PWM_BITS = 8;

// WHY THE MOTORS SIT ON LEDC CHANNELS 6 AND 7
//   The ESP32 has 16 LEDC channels sharing 4 timers: channel/2 % 4 picks
//   the timer. ESP32Servo's allocator tracks only the channels IT handed
//   out -- a channel set up with a raw ledcSetup() call, as the motors
//   are, is invisible to it. Searching upward from timer 0 it can hand
//   the servo a channel the motors are already driving, and then every
//   motor speed write lands on the servo's channel as a garbage pulse
//   width. The servo slams to one end and buzzes there.
//   So: the servo is pinned to timer 0 (channels 0/1) with
//   allocateTimer(0), and the motors are moved to 6/7, on timer 3.
const int CH_ENA = 6, CH_ENB = 7;           // core 2.x only

// ------------------------------------------------------------- network
const char* WIFI_SSID = "VIT-SmartParking";
const char* WIFI_PASS = "park1234";
const char* LOT_URL   = "http://192.168.4.1";

// ================================================== BENCH SWITCHES ==
const bool CALIBRATE_ONLY = false;  // true = print distances, never drive
                                    // set true again to re-measure the route

// Set by CAR_HAS_SERVO at the top of the file; do not edit this line.
// With no servo, roaming nudges and re-checks instead of looking left
// and right, and parks without the side check. It loses nothing it
// cannot do another way, and in simulation it actually drives MORE of
// the time, because it never stops to sweep.
constexpr bool HAS_SERVO = (CAR_HAS_SERVO == 1);
// --------------------------------------------- servo mounting fixes
// Use these instead of re-seating the horn, which you cannot do.
//
// SERVO_TRIM: degrees added to every angle so that "forward" really is
//   forward. Only for SMALL errors, roughly -25..+25. A 90-degree error
//   cannot be trimmed away -- the three look angles would collapse on
//   top of each other -- so for that, edit SERVO_FORWARD / LEFT / RIGHT
//   near the bottom of the constants instead.
//
// SERVO_MIRROR: set 1 if the servo is mounted upside down, so that
//   turning the angle up swings the head the wrong way. Left and right
//   then swap, and the car would turn towards obstacles instead of away.
//
// FIRST FLASH IT WITH BOTH AT THE DEFAULT. The slam-to-the-right you saw
// was the PWM clash, not the mounting; it very likely centres correctly
// now. Watch the boot sweep before changing either of these.
const int  SERVO_TRIM   = 0;
#define    SERVO_MIRROR 0

// Nudges the wheels at boot, before anything else, so you can see at
// once whether the motors, the driver board and the battery are alive.
// Put the car on a book so the wheels spin free.
const bool BOOT_MOTOR_TEST = true;

// ------------------------------------------- what to do when idle
// true  = drive around avoiding obstacles (the roaming demo)
// false = stand still and wait for someone to tap a slot
// Flip this depending on which half you are demonstrating. The car
// still obeys taps either way.
//
// SHIPPED false ON PURPOSE. A roaming car drives into the parking model
// because the model is the nearest thing in front of it, halts 40 cm
// short, and sweeps its head looking for a way round -- which looks
// exactly like a parked car misbehaving. Standing still by default
// means anything that moves, moved because you told it to.
const bool ROAM_WHEN_IDLE = false;

// ------------------------------------- obstacle checks ON THE ROUTE
// While driving to a slot the car cannot steer around anything: the
// route is a fixed path measured off the walls. So "avoiding" can only
// mean ABORTING. With this true it gives up and reports blocked when
// something appears ahead; with it false it drives to the slot and
// stops, whatever walks past.
//   Set false if the route keeps aborting on its own. The thing it
//   mistakes for an obstacle is usually the bay's own back wall coming
//   into view as the car turns in -- that is a real 10 cm jump.
const bool ROUTE_WATCHES_FOR_OBSTACLES = false;

// Prints what the car is seeing and deciding while it roams, one line
// at a time, at 115200 baud. Set false for a quiet monitor.
const bool TRACE_ROAM = true;
const unsigned long TRACE_MS = 250;        // at most one line this often

// WHICH SLOT NUMBER SITS WHERE ON THE ROUTE
//   STRAIGHT_AHEAD_BAY is the slot at the end of leg 1 -- the one the
//   car drives straight into without turning.
//   CORRIDOR_BAY lists the other two IN THE ORDER THE CAR MEETS THEM
//   after it turns east, nearest first. They appear on its left.
//   Every number must be 1, 2 or 3, and all three must appear once.
const int STRAIGHT_AHEAD_BAY = 3;   // renamed: was slot 1
const int CORRIDOR_BAY[2]    = {1, 2};   // renamed: was {2, 3}

// --- leg 1: facing NORTH, measuring to STRAIGHT_AHEAD_BAY's back wall -
const float START_FRONT_CM  = 10.0;
const float CORNER_FRONT_CM = 31.0;
// --- leg 2: facing EAST, measuring to the EAST WALL ---
//   Same order as CORRIDOR_BAY: first the nearer bay, then the farther.
const float BAY_FRONT_CM[2] = {55.0, 28.0};

const float PARK_CM = 6.0;          // stop this far off a bay's back wall

// How close the front reading must be to START_FRONT_CM, and how steady,
// before the car accepts that it is on the start line and starts driving.
const float PLACE_TOL_CM  = 6.0;
const int   PLACE_STABLE  = 6;      // agreeing reads in a row
const float PLACE_JITTER  = 2.5;    // reads must agree within this

// ---------------------------------------------------------- roaming
// A 20 cm wide car needs to stop at 40 cm: the beam is a cone, so at
// distance d it only covers 2*d*tan(15deg). Below about 37 cm that is
// narrower than the car, and it clips things its beam passed either
// side of. 35 cm was right when the car was 15 cm wide; it is not now.
const float ROAM_STOP_CM  = 10.0;
const float ROAM_SLOW_CM  = 12.0;
const float ROAM_PANIC_CM = 6.0;   // too close to sweep the head safely
const float ROAM_CLEAR_CM = 30.0;   // a side counts as open above this
const int   ROAM_CORNER_SCANS = 3;
const unsigned long ROAM_GOOD_RUN_MS = 2500;
const unsigned long ROAM_POLL_MS     = 3000;  // halt this often to check orders

// ---------------------------------------------------------- the rest
const int SERVO_FORWARD   = 90;
const int SERVO_LEFT_LOOK = 5;
const int SERVO_RIGHT_LOOK = 175;
const int SERVO_SETTLE_MS = 300;

const int DRIVE_SPEED = 255, CREEP_SPEED = 185, TURN_SPEED = 255, BACK_SPEED = 230;
const bool SOFT_START = true;
const int  RAMP_STEP  = 40;
const unsigned long RAMP_MS = 30;
const int LEFT_TRIM = 0, RIGHT_TRIM = 0;

const unsigned long TURN_RIGHT_MS = 1500;   // corner: north -> east
const unsigned long TURN_LEFT_MS  = 1500;   // into a bay: east -> north
const unsigned long TURN_45_MS    = 700;
const unsigned long TURN_180_MS   = 2800;
const unsigned long BACKUP_MS     = 500;

const float SLOWDOWN_CM       = 14.0;
const float PAST_TOLERANCE_CM = 7.0;
const float OPENING_CM        = 26.0;

// An obstacle in the beam is indistinguishable from the wall being
// nearer — same reading. What gives it away is the JUMP: the car moves
// about a centimetre per reading, so a drop of 10 cm between two reads
// is something arriving, not the car travelling.
const float BLOCK_JUMP_CM = 10.0;
const int   BLOCK_CONFIRM = 2;

const int  DROPOUT_CREEP = 3, DROPOUT_STOP = 8;
const bool VERIFY_BAY = true;
const int  VERIFY_STEPS = 5;
const unsigned long VERIFY_STEP_MS = 110;

const unsigned long BOOT_DELAY_MS   = 2000;
const unsigned long WIFI_RETRY_MS   = 10000;
const unsigned long NAV_TIMEOUT_MS  = 60000;

// --------------------------------------------------------------- state
enum CarState {
  CONNECTING,
  ROAM_DRIVE, ROAM_BACK, ROAM_SCAN, ROAM_TURN,       // mode 1
  ARMED,                                             // told a bay, waiting to be placed
  IDLE_WAIT,                                         // standing still, waiting for a tap
  LEG1, TURN_CORNER, LEG2, VERIFY, TURN_INTO_BAY, ENTERING,   // mode 2
  PARKED, LEAVING, FAILED
};
CarState state = ROAM_DRIVE;   // roaming starts at once, Wi-Fi or no Wi-Fi

#if CAR_HAS_SERVO
  Servo scanner;
#endif
int  targetBay = 0, verifyTries = 0;
int  hits = 0, dropouts = 0, blockHits = 0, scanCount = 0;
float prevFront = -1.0;
bool  turnLeftNext = true;
unsigned long turnFor = 0;
unsigned long stateStarted = 0, navStarted = 0, lastPoll = 0, lastArrived = 0;
unsigned long clearSince = 0, lastRoamPoll = 0, wifiTry = 0;
bool wifiWasUp = false;

// Prints distance in a fixed width so the columns line up, and "--" for
// a dropout rather than a number, because a missing echo is not a
// measurement and should never look like one.
void traceCm(float d) {
  if (d < 0)       Serial.print("  --  ");
  else if (d < 10) Serial.print("  ");
  else if (d < 100) Serial.print(" ");
  if (d >= 0) { Serial.print(d, 1); Serial.print(" "); }
}
unsigned long lastTrace = 0;
bool traceDue() {
  if (!TRACE_ROAM) return false;
  if (millis() - lastTrace < TRACE_MS) return false;
  lastTrace = millis();
  return true;
}

bool headParked = false;
unsigned long lastBeat = 0;

void enter(CarState s) {
  state = s; stateStarted = millis();
  headParked = false;
  hits = 0; dropouts = 0; blockHits = 0; prevFront = -1.0;
}

// ======================================================= motor control
void pwmWrite(int pin, int ch, int duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(ch, duty);
#endif
}
void pwmBegin() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(ENA, PWM_FREQ, PWM_BITS); ledcAttach(ENB, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(CH_ENA, PWM_FREQ, PWM_BITS); ledcAttachPin(ENA, CH_ENA);
  ledcSetup(CH_ENB, PWM_FREQ, PWM_BITS); ledcAttachPin(ENB, CH_ENB);
#endif
}
int trimmed(int s, int t) {
  if (s == 0) return 0;
  int mag = constrain(abs(s) + t, 0, 255);
  return s > 0 ? mag : -mag;
}
int curL = 0, curR = 0;
int rampSide(int cur, int target, bool stepNow) {
  if (!SOFT_START || target == 0) return target;
  int sign = target > 0 ? 1 : -1, mag = abs(target);
  int curMag = (cur * sign > 0) ? abs(cur) : 0;
  if (curMag >= mag) return target;
  if (!stepNow) return sign * curMag;
  return sign * min(mag, curMag + RAMP_STEP);
}
void drive(int left, int right) {
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
void driveFor(int l, int r, unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) { drive(l, r); delay(10); }
}
void halt()          { drive(0, 0); }
void forward(int s)  { drive(s, s); }
void backward(int s) { drive(-s, -s); }
void spinRight(int s){ drive(s, -s); }
void spinLeft(int s) { drive(-s, s); }

// ====================================================== sensor reading
float pingOnce() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(3);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long us = pulseIn(ECHO, HIGH, 25000UL);
  if (us == 0) return -1.0;
  return us / 58.0;
}
// Median of the valid pings. A dropout is discarded, never replaced with
// a number: reading "no echo" as "far" is how a car drives into a wall
// it could not see.
float readCm() {
  float v[3]; int n = 0;
  for (int i = 0; i < 3; i++) { float d = pingOnce(); if (d > 0) v[n++] = d; delay(8); }
  if (n == 0) return -1.0;
  if (n == 1) return v[0];
  if (n == 2) return min(v[0], v[1]);
  float hi = max(v[0], max(v[1], v[2])), lo = min(v[0], min(v[1], v[2]));
  return v[0] + v[1] + v[2] - hi - lo;
}
void look(int angle, int settle = SERVO_SETTLE_MS) {
#if CAR_HAS_SERVO
  int a = angle;
#if SERVO_MIRROR
  a = 180 - a;
#endif
  a += SERVO_TRIM;
  if (a < 0) a = 0;
  if (a > 180) a = 180;
  scanner.write(a);
  delay(settle);
#else
  (void) angle; (void) settle;
  delay(20);                                 // head is bolted; nothing to turn
#endif
}
float readAt(int angle) { look(angle); return readCm(); }

// ===================================================== link to the lot
bool httpGet(const String &path, String *body = nullptr) {
  if (WiFi.status() != WL_CONNECTED) return false;
  HTTPClient http;
  http.setConnectTimeout(900);
  http.setTimeout(1200);
  http.begin(String(LOT_URL) + path);
  int code = http.GET();
  if (code == 200 && body) { *body = http.getString(); body->trim(); }
  http.end();
  return code == 200;
}
bool sendEvent(const char* e, int bay) {
  String path = String("/car/event?e=") + e + "&slot=" + String(bay);
  for (int i = 0; i < 3; i++) {
    if (httpGet(path)) { Serial.println(String("-> ") + e + ":" + String(bay)); return true; }
    delay(150);
  }
  Serial.println(String("!! could not send ") + e);
  return false;
}
void fail(const char* why) {
  halt();
  Serial.println(String("FAILED: ") + why);
  look(SERVO_FORWARD, 200);
  sendEvent("failed", targetBay);
  enter(FAILED);
}
void blocked() {
  halt();
  Serial.println("BLOCKED: something came into the beam");
  sendEvent("blocked", targetBay);
  enter(FAILED);
}

// Called every pass. The car does NOT wait for the lot's Wi-Fi before it
// starts roaming: obstacle avoidance is self-contained, and a car that
// refuses to move because a second board is switched off is useless on a
// bench. If the lot appears later, orders start arriving; if it vanishes,
// the car keeps roaming.
void maintainWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiWasUp) {
      wifiWasUp = true;
      Serial.println("lot found, IP " + WiFi.localIP().toString());
      Serial.println("tap a bay on the website when you want me to park");
    }
    return;
  }
  if (wifiWasUp) { wifiWasUp = false; Serial.println("lost the lot - still roaming"); }
  if (millis() - wifiTry > WIFI_RETRY_MS) {
    wifiTry = millis();
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.println("(no lot Wi-Fi yet - roaming anyway)");
  }
}

// Only ever called while the car is standing still: an HTTP request can
// block for most of a second, and a moving car must not go blind.
void pollCommand() {
  String cmd;
  if (!httpGet("/car/cmd", &cmd)) return;

  if (cmd.startsWith("GO:")) {
    int bay = cmd.substring(3).toInt();
    if (bay < 1 || bay > 3) return;
    targetBay = bay;
    sendEvent("ack", bay);
    halt();
    Serial.println("GO bay " + String(bay) + " - halting. Put me on the start line.");
    enter(ARMED);
  } else if (cmd.startsWith("RELEASE")) {
    if (state == PARKED) { Serial.println("released"); enter(LEAVING); }
  } else if (cmd.startsWith("STOP")) {
    halt(); enter(ROAM_WHEN_IDLE ? ROAM_DRIVE : IDLE_WAIT);
  }
}

// Position of a slot number along leg 2, or -1 if it is not there.
int corridorIndex(int bay) {
  for (int i = 0; i < 2; i++) if (CORRIDOR_BAY[i] == bay) return i;
  return -1;
}

bool reached(float front, float target) {
  if (front > 0 && front <= target) hits++; else hits = 0;
  return hits >= 2;
}
// True when the reading has jumped nearer by more than travel explains.
bool somethingArrived(float front) {
  if (!ROUTE_WATCHES_FOR_OBSTACLES) { if (front > 0) prevFront = front; return false; }
  bool jump = (front > 0 && prevFront > 0 && (prevFront - front) > BLOCK_JUMP_CM);
  if (jump) blockHits++; else blockHits = 0;
  if (front > 0) prevFront = front;
  return blockHits >= BLOCK_CONFIRM;
}

// ================================================================ setup
void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG, OUTPUT); digitalWrite(TRIG, LOW);
  pinMode(ECHO, INPUT);

  // The servo claims its LEDC timer FIRST, before any motor channel
  // exists, so that whichever core version is in use the two cannot be
  // handed the same hardware.
#if CAR_HAS_SERVO
  ESP32PWM::allocateTimer(0);                // servo owns timer 0 only
  scanner.setPeriodHertz(50);
  scanner.attach(SERVO_PIN, 500, 2400);
#endif

  pwmBegin(); halt();

  Serial.println();
  Serial.println("=== car: roam + auto-park ===");

  // Say the layout out loud, and refuse to pretend a bad one is fine.
  // A duplicated or missing number here sends the car confidently to
  // the wrong bay, which looks like a sensor fault and is not one.
  Serial.println(ROAM_WHEN_IDLE ? "idle mode: ROAM (obstacle avoiding)"
                                : "idle mode: STAND STILL (wait for a tap)");
  Serial.println(ROUTE_WATCHES_FOR_OBSTACLES
                 ? "on the route: aborts if something appears ahead"
                 : "on the route: ignores obstacles, drives to the slot and stops");
  Serial.print("route: straight ahead = slot "); Serial.println(STRAIGHT_AHEAD_BAY);
  Serial.print("       corridor, near to far = slot ");
  Serial.print(CORRIDOR_BAY[0]); Serial.print(", slot ");
  Serial.println(CORRIDOR_BAY[1]);
  {
    int seen[4] = {0, 0, 0, 0};
    bool bad = false;
    int all[3] = { STRAIGHT_AHEAD_BAY, CORRIDOR_BAY[0], CORRIDOR_BAY[1] };
    for (int i = 0; i < 3; i++) {
      if (all[i] < 1 || all[i] > 3) bad = true; else seen[all[i]]++;
    }
    for (int b = 1; b <= 3; b++) if (seen[b] != 1) bad = true;
    if (bad) {
      Serial.println("*** BAD LAYOUT: STRAIGHT_AHEAD_BAY and CORRIDOR_BAY must");
      Serial.println("    together use slots 1, 2 and 3 exactly once each. ***");
    }
  }

  if (BOOT_MOTOR_TEST) {
    Serial.println("motor test: forward 0.4s, back 0.4s, stop");
    driveFor(DRIVE_SPEED, DRIVE_SPEED, 400);
    halt(); delay(300);
    driveFor(-DRIVE_SPEED, -DRIVE_SPEED, 400);
    halt(); delay(300);
    Serial.println("  nothing moved? -> battery, ENA/ENB jumpers, or L298N 5V");
  }

#if CAR_HAS_SERVO
  Serial.println("servo test: forward, left, right, forward");
  look(SERVO_FORWARD, 700);
  look(SERVO_LEFT_LOOK, 700);
  look(SERVO_RIGHT_LOOK, 700);
  look(SERVO_FORWARD, 700);
  Serial.println("  buzzing or slammed to one end?");
  Serial.println("     -> power the servo from the POWER BANK 5V, not the");
  Serial.println("        L298N's 5V pin, grounds commoned. If it still");
  Serial.println("        buzzes, set CAR_HAS_SERVO 0 and bolt it forward.");
  Serial.println("  head centred but a few degrees off? -> SERVO_TRIM");
  Serial.println("  left and right swapped?             -> SERVO_MIRROR 1");
  Serial.println("  a full 90 deg out? -> edit SERVO_FORWARD/LEFT/RIGHT,");
  Serial.println("     e.g. forward 0, left 90, right 180. Trim cannot fix");
  Serial.println("     that: the three angles would collapse together.");
#else
  Serial.println("CAR_HAS_SERVO 0: head bolted forward, D18 unused,");
  Serial.println("  ESP32Servo not compiled in, no LEDC conflict possible");
#endif
  if (CORRIDOR_CM < CORRIDOR_MIN_CM) {
    Serial.print("WARNING: leg-2 corridor is "); Serial.print(CORRIDOR_CM, 1);
    Serial.print(" cm but a 20x20 car needs "); Serial.print(CORRIDOR_MIN_CM, 1);
    Serial.println(" cm to turn in.");
    Serial.println("  BAYS 2 AND 3 ARE NOT REACHABLE until it is widened.");
  }
  if (CAR_LEN_CM + 4.0 > BAY_DEPTH_CM) {
    Serial.print("NOTE: about ");
    Serial.print(CAR_LEN_CM - (BAY_DEPTH_CM - PARK_CM), 0);
    Serial.println(" cm of car will stick out of the bay. Cosmetic.");
  }

  // The four route distances below are the ones I shipped as examples.
  // Roaming needs none of them and works as it is; the parking route is
  // meaningless until they are measured on your own model.
  if (START_FRONT_CM == 75.0 && CORNER_FRONT_CM == 31.0 &&
      BAY_FRONT_CM[0] == 55.0 && BAY_FRONT_CM[1] == 28.0) {
    Serial.println();
    Serial.println("*** ROUTE NOT CALIBRATED - still the shipped example numbers. ***");
    Serial.println("    Roaming works. Parking will drive to the WRONG PLACE.");
    Serial.println("    Set CALIBRATE_ONLY = true, measure the four distances,");
    Serial.println("    paste them in, then set it false again.");
  }

  if (CALIBRATE_ONLY) {
    Serial.println();
    Serial.println("CALIBRATE_ONLY - motors off. Push the car by hand and note");
    Serial.println("'front' at each of these, then paste them in:");
    Serial.println("  1. on the start line, facing bay 1   -> START_FRONT_CM");
    Serial.println("  2. at the corner, facing bay 1       -> CORNER_FRONT_CM");
    Serial.println("  3. facing EAST, centred on bay 2     -> BAY_FRONT_CM[0]");
    Serial.println("  4. facing EAST, centred on bay 3     -> BAY_FRONT_CM[1]");
    Serial.println();
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  wifiTry = millis();
  Serial.println(String("looking for ") + WIFI_SSID + " in the background");

  delay(BOOT_DELAY_MS);
  lastRoamPoll = millis();
  if (ROAM_WHEN_IDLE) {
    Serial.println("roaming now - put a hand in front of me");
    enter(ROAM_DRIVE);
  } else {
    Serial.println("standing by - tap a slot on the website");
    enter(IDLE_WAIT);
  }
}

// ================================================================= loop
void loop() {

  if (CALIBRATE_ONLY) {
    float f = readAt(SERVO_FORWARD);
    Serial.print("front ");
    if (f < 0) Serial.print("--"); else Serial.print(f, 1);
    Serial.println(" cm");
    return;
  }

  maintainWifi();

  // Safety net over the driving half of the route.
  if (state >= LEG1 && state <= ENTERING && millis() - navStarted > NAV_TIMEOUT_MS) {
    fail("route timed out");
    return;
  }

  switch (state) {

    // Kept only so the enum stays in order; the car never waits here now.
    case CONNECTING:
      enter(ROAM_WHEN_IDLE ? ROAM_DRIVE : IDLE_WAIT);
      break;

    // Motors off, head still, just listening for a tap.
    case IDLE_WAIT: {
      halt();
      if (millis() - lastPoll > 700) { lastPoll = millis(); pollCommand(); }
      static unsigned long lastSay = 0;
      if (TRACE_ROAM && millis() - lastSay > 5000) {
        lastSay = millis();
        float d = readCm();
        Serial.print("IDLE  standing by, front ");
        traceCm(d);
        Serial.println("cm");
      }
      break;
    }

    // ============================================= MODE 1: roaming
    case ROAM_DRIVE: {
      float d = readCm();
      dropouts = (d < 0) ? dropouts + 1 : 0;

      // Stop every few seconds to ask whether anyone wants the car. It
      // cannot poll while moving, so this is how a tap reaches it.
      if (millis() - lastRoamPoll > ROAM_POLL_MS) {
        halt(); delay(60);
        lastRoamPoll = millis();
        pollCommand();
        if (state != ROAM_DRIVE) break;
      }

      if (d > 0 && d < ROAM_PANIC_CM) {
        halt();
        Serial.print("PANIC front "); traceCm(d);
        Serial.print("cm < "); Serial.print(ROAM_PANIC_CM, 0);
        Serial.println(" - backing up");
        enter(ROAM_BACK); break;
      }
      if ((d > 0 && d < ROAM_STOP_CM) || dropouts >= DROPOUT_STOP) {
        halt();
        Serial.print("STOP  front "); traceCm(d);
        if (d > 0) { Serial.print("cm < "); Serial.print(ROAM_STOP_CM, 0); Serial.println(" - looking for a way round"); }
        else       { Serial.println("cm - blind, looking for a way round"); }
        enter(ROAM_SCAN); break;
      }
      bool wary = (d > 0 && d < ROAM_SLOW_CM) || dropouts >= DROPOUT_CREEP;
      forward(wary ? CREEP_SPEED : DRIVE_SPEED);

      if (traceDue()) {
        Serial.print("ROAM  front ");
        traceCm(d);
        Serial.print("cm  ");
        if (d > 0 && d < ROAM_SLOW_CM) Serial.print("| closing ");
        else if (dropouts >= DROPOUT_CREEP) Serial.print("| no echo ");
        else                                Serial.print("| clear   ");
        Serial.print("| speed ");
        Serial.print(wary ? CREEP_SPEED : DRIVE_SPEED);
        if (dropouts > 0) { Serial.print("  (dropouts "); Serial.print(dropouts); Serial.print(")"); }
        Serial.println();
      }
      if (wary) clearSince = 0;
      else if (clearSince == 0) clearSince = millis();
      else if (millis() - clearSince > ROAM_GOOD_RUN_MS) scanCount = 0;
      break;
    }

    case ROAM_BACK:
      driveFor(-BACK_SPEED, -BACK_SPEED, BACKUP_MS);
      halt(); delay(150);
      enter(ROAM_SCAN);
      break;

    case ROAM_SCAN: {
      // With the head bolted forward the car cannot compare left against
      // right, so it turns a fixed step away, re-checks, and alternates
      // direction so it cannot lock itself into a corner.
      if (!HAS_SERVO) {
        scanCount++;
        if (scanCount >= ROAM_CORNER_SCANS) {
          turnFor = TURN_180_MS; scanCount = 0;
          Serial.println("boxed in - turning around");
        } else {
          turnLeftNext = !turnLeftNext;
          turnFor = TURN_45_MS;
        }
        clearSince = 0; dropouts = 0;
        if (TRACE_ROAM) {
          Serial.print("SCAN  no servo, nudging ");
          Serial.print(turnLeftNext ? "LEFT " : "RIGHT ");
          Serial.print(turnFor); Serial.println(" ms");
        }
        enter(ROAM_TURN);
        break;
      }

      float l = readAt(SERVO_LEFT_LOOK);
      float r = readAt(SERVO_RIGHT_LOOK);
      look(SERVO_FORWARD);
      // A dropout here means "cannot see", not "open". Scoring it zero
      // stops the car turning confidently into the one direction it has
      // no information about.
      float ls = (l < 0) ? 0.0 : l, rs = (r < 0) ? 0.0 : r;
      scanCount++; dropouts = 0;

      Serial.print("SCAN  left "); traceCm(l);
      Serial.print("cm   right "); traceCm(r);
      Serial.print("cm   (scan "); Serial.print(scanCount);
      Serial.print(" of "); Serial.print(ROAM_CORNER_SCANS); Serial.print(")  -> ");

      if ((ls < ROAM_CLEAR_CM && rs < ROAM_CLEAR_CM) || scanCount >= ROAM_CORNER_SCANS) {
        turnLeftNext = true; turnFor = TURN_180_MS;
        scanCount = 0; clearSince = 0;
        Serial.println("boxed in, turning around");
      } else {
        turnLeftNext = (ls > rs);
        turnFor = (fabs(ls - rs) > 20.0) ? TURN_RIGHT_MS : TURN_45_MS;
        Serial.print(turnLeftNext ? "turn LEFT " : "turn RIGHT ");
        Serial.print(turnFor); Serial.println(" ms");
      }
      enter(ROAM_TURN);
      break;
    }

    case ROAM_TURN:
      if (turnLeftNext) spinLeft(TURN_SPEED); else spinRight(TURN_SPEED);
      if (millis() - stateStarted > turnFor) {
        halt(); delay(150);
        float after = readCm();
        if (TRACE_ROAM) {
          Serial.print("TURN  done, front now "); traceCm(after);
          Serial.println("cm - driving on");
        }
        clearSince = 0;
        lastTrace = 0;
        enter(ROAM_DRIVE);
      }
      break;

    // ======================== told a bay; waiting to be set down ======
    // The car checks it really is on the start line before it drives:
    // the wall in front must be at START_FRONT_CM and must hold steady,
    // which it will not while the car is being carried.
    case ARMED: {
      halt();
      static int    agree = 0;
      static float  lastRead = -1.0;
      float f = readCm();

      bool nearEnough = (f > 0) && (fabs(f - START_FRONT_CM) <= PLACE_TOL_CM);
      bool steady     = (lastRead > 0) && (fabs(f - lastRead) <= PLACE_JITTER);
      if (f > 0) lastRead = f;

      agree = (nearEnough && steady) ? agree + 1 : 0;

      static unsigned long lastSay = 0;
      if (millis() - lastSay > 1500) {
        lastSay = millis();
        Serial.print("waiting on the start line... front ");
        if (f < 0) Serial.print("--"); else Serial.print(f, 1);
        Serial.print(" cm, want "); Serial.print(START_FRONT_CM, 0);
        Serial.print(" +/- "); Serial.print(PLACE_TOL_CM, 0);
        Serial.print("   ["); Serial.print(agree); Serial.println("/6]");
      }

      if (agree >= PLACE_STABLE) {
        agree = 0; lastRead = -1.0;
        navStarted = millis();
        sendEvent("driving", targetBay);
        Serial.println("on the start line - going to bay " + String(targetBay));
        enter(LEG1);
      }
      // Still reachable by a STOP from the page while standing here.
      if (millis() - lastPoll > 700) { lastPoll = millis(); pollCommand(); }
      break;
    }

    // ============================================= MODE 2: the route
    case LEG1: {
      float target = (targetBay == STRAIGHT_AHEAD_BAY) ? PARK_CM : CORNER_FRONT_CM;
      float front  = readCm();
      dropouts = (front < 0) ? dropouts + 1 : 0;
      if (dropouts > 12) { fail("front sensor lost its echo"); break; }
      if (somethingArrived(front)) { blocked(); break; }

      if (front > 0 && front < target - PAST_TOLERANCE_CM) { fail("overshot on leg 1"); break; }
      if (reached(front, target)) {
        halt();
        if (targetBay == STRAIGHT_AHEAD_BAY) {
          Serial.println("parked in slot " + String(targetBay) +
                         " (straight ahead), front " + String(front, 1) + " cm");
          sendEvent("parked", targetBay);
          enter(PARKED);
        } else {
          Serial.println("at the corner, front " + String(front, 1) + " cm");
          enter(TURN_CORNER);
        }
      } else {
        forward((front > 0 && front < target + SLOWDOWN_CM) ? CREEP_SPEED : DRIVE_SPEED);
      }
      break;
    }

    case TURN_CORNER:
      spinRight(TURN_SPEED);
      if (millis() - stateStarted > TURN_RIGHT_MS) {
        halt(); delay(250);
        Serial.println("turned east");
        enter(LEG2);
      }
      break;

    case LEG2: {
      int ci = corridorIndex(targetBay);
      if (ci < 0) { fail("slot is not on the route - check CORRIDOR_BAY"); break; }
      float target = BAY_FRONT_CM[ci];
      float front  = readCm();
      dropouts = (front < 0) ? dropouts + 1 : 0;
      if (dropouts > 12) { fail("front sensor lost its echo"); break; }
      if (somethingArrived(front)) { blocked(); break; }

      if (front > 0 && front < target - PAST_TOLERANCE_CM) { fail("overshot on leg 2"); break; }
      if (reached(front, target)) {
        halt();
        Serial.println("at slot " + String(targetBay) + " (corridor " +
                       String(ci + 1) + "), front " + String(front, 1));
        look(SERVO_LEFT_LOOK);
        verifyTries = 0;
        enter(VERIFY);
      } else {
        forward((front > 0 && front < target + SLOWDOWN_CM) ? CREEP_SPEED : DRIVE_SPEED);
      }
      break;
    }

    // The stored distance says we are here; the side sensor confirms the
    // bay is open. This is what stops the car turning into a bay that
    // filled up while it was driving.
    case VERIFY: {
      if (!VERIFY_BAY || !HAS_SERVO || !ROUTE_WATCHES_FOR_OBSTACLES) {
        look(SERVO_FORWARD); enter(TURN_INTO_BAY); break;
      }
      float side = readCm();
      Serial.println("left side " + String(side, 1) + " cm");
      if (side > OPENING_CM) {
        look(SERVO_FORWARD);
        enter(TURN_INTO_BAY);
      } else if (verifyTries++ < VERIFY_STEPS) {
        driveFor(CREEP_SPEED, CREEP_SPEED, VERIFY_STEP_MS); halt(); delay(150);
      } else {
        fail("no opening on the left - bay occupied?");
      }
      break;
    }

    case TURN_INTO_BAY:
      spinLeft(TURN_SPEED);
      if (millis() - stateStarted > TURN_LEFT_MS) {
        halt(); delay(250);
        enter(ENTERING);
      }
      break;

    case ENTERING: {
      float front = readCm();
      dropouts = (front < 0) ? dropouts + 1 : 0;
      if (dropouts > 12) { fail("front sensor lost its echo"); break; }
      if (reached(front, PARK_CM)) {
        halt();
        Serial.println("parked in bay " + String(targetBay) +
                       ", front " + String(front, 1) + " cm");
        sendEvent("parked", targetBay);
        enter(PARKED);
      } else {
        forward(CREEP_SPEED);
      }
      break;
    }

    // Sits in the bay until Release is tapped on the page. Motors off,
    // head centred and still: nothing here reads the sensor to steer by,
    // so if the head is sweeping, the car is NOT in this state.
    case PARKED:
      halt();
      if (!headParked) { headParked = true; look(SERVO_FORWARD, 200); }
      if (millis() - lastPoll > 700) { lastPoll = millis(); pollCommand(); }
      if (TRACE_ROAM && millis() - lastBeat > 5000) {
        lastBeat = millis();
        Serial.println("PARKED in slot " + String(targetBay) +
                       " - motors off, waiting for Release");
      }
      break;

    // Reverse out of the bay, turn away, and go back to roaming.
    case LEAVING:
      driveFor(-BACK_SPEED, -BACK_SPEED, 1600);
      halt(); delay(200);
      spinRight(TURN_SPEED);
      delay(TURN_RIGHT_MS);
      halt(); delay(200);
      sendEvent("left", targetBay);
      targetBay = 0;
      scanCount = 0; clearSince = 0; lastRoamPoll = millis();
      Serial.println(ROAM_WHEN_IDLE ? "back to roaming" : "out of the bay, standing by");
      enter(ROAM_WHEN_IDLE ? ROAM_DRIVE : IDLE_WAIT);
      break;

    case FAILED:
      halt();
      if (!headParked) { headParked = true; look(SERVO_FORWARD, 200); }
      if (millis() - lastPoll > 700) { lastPoll = millis(); pollCommand(); }
      if (TRACE_ROAM && millis() - lastBeat > 5000) {
        lastBeat = millis();
        Serial.println("FAILED - motors off. Power-cycle me to roam again.");
      }
      break;
  }
}
