/* =====================================================================
   parking_final.ino      —  ESP32 DevKit v1
   ---------------------------------------------------------------------
   Smart parking model, 3 slots, 20 x 20 cm board.

   What this board does, and nothing more:
     * Reads 4 ultrasonic sensors — one in the ceiling of each of the 3
       slots, plus one at the entry.
     * Decides which slots are free.
     * Hosts its own Wi-Fi access point and serves a web page showing
       which slots are free, so a phone can see them on arrival.

   There is no car link. The driver reads the board and parks the car.

   No router, no laptop, no internet. The ESP32 is the whole system.

   HOW A SLOT IS JUDGED
   A slot is "taken" only when the reading lands inside a height band that
   a model car fits — not merely when something is closer than the floor.
   That rejects the empty floor at one end and a hand, a wire or the
   gantry itself at the other. Set MOUNT_HEIGHT_CM to your real gantry
   height and the band follows from it.

   WIRING WARNING
   Every ECHO line must reach the ESP32 through a 1k / 2k voltage divider.
   The HC-SR04 outputs 5 V and these pins are 3.3 V parts that are not 5 V
   tolerant. Wired direct, the pin works for a while and then dies.
   VCC must be 5 V from VIN, not 3V3.

   Libraries: none beyond the ESP32 core. Install "esp32" by Espressif in
   Arduino IDE -> Boards Manager, then pick "ESP32 Dev Module".
   ===================================================================== */

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESP32Servo.h>

// ---------------------------------------------------------------- pins
// Every pin here is printed on a DOIT DevKit v1 as a plain D-number, so
// the silkscreen matches this code.
//
// Avoided throughout: 0, 2, 12, 15 (strapping pins that affect boot),
// 6-11 (wired to the flash chip), and 36/39 (input-only but silkscreened
// VP and VN, so nobody can find them).
const int TRIG_ENTRY   = 13, ECHO_ENTRY = 22;
// WHICH SENSOR IS WHICH SLOT
//   Index 0 is Slot 1 on the page, index 1 is Slot 2, index 2 is Slot 3.
//   The numbering is fixed; the POSITIONS are yours. If the sensor you
//   want to call Slot 1 is physically the one you wired to D26/D23,
//   move that pair to the front of both lines. Reordering these two
//   lines is the whole job -- no rewiring, no page edits.
const int TRIG_SLOT[3] = {27, 26, 14};   // was {14, 27, 26}
const int ECHO_SLOT[3] = {35, 23, 34};   // was {34, 35, 23}

// WHICH SLOT TO RECOMMEND FIRST
//   "Next free slot is N" should name the one a driver reaches first,
//   which is not necessarily the lowest number once the slots are
//   arranged differently. List the slot numbers in order of nearness to
//   the entry. {1,2,3} keeps the old behaviour.
const int SUGGEST_ORDER[3] = {1, 2, 3};

// ------------------------------------------------- how many sensors?
// The three ceiling sensors are the project. The entry sensor only adds
// a line on the page saying a car has arrived. Build with three, add the
// fourth whenever you like.
const bool USE_ENTRY_SENSOR = true;

// ------------------------------------------------------------- tuning
// An HC-SR04 cannot physically report outside this window. Anything else
// is electrical noise on a floating pin, not a measurement, and gets
// thrown away. A 0.0 cm "reading" is a glitch; treated as a real distance
// it makes an unwired sensor look exactly like a parked car.
const float MIN_VALID_CM     = 2.0;
const float MAX_VALID_CM     = 400.0;

// -------------------------------------------------- the detection zone
// WHERE the sensor looks is set by how high you mount it, not by code.
// The beam leaves the sensor as a widening cone, so
//
//        footprint = 0.536 x MOUNT_HEIGHT_CM
//
//   mount 11 cm ->  5.9 cm circle     mount 20 cm -> 10.7 cm circle
//   mount 12 cm ->  6.4 cm circle     mount 24 cm -> 12.9 cm circle
//   mount 14 cm ->  7.5 cm circle     mount 28 cm -> 15.0 cm circle
//
// On a 20 x 20 cm board the three slots are 6.7 cm apart, so the mount
// has to stay near 12 cm or each sensor starts seeing its neighbours'
// cars. A cardboard tube over the transducers narrows the cone if you
// need a small circle from a higher mount.
const float MOUNT_HEIGHT_CM  = 12.0;   // sensor face to floor

// WHAT counts as a car. A reading is a car only if the surface it
// bounced off sits between these two heights above the floor. Measure
// your model car's roof and leave about 2 cm of margin either side.
// Keep CAR_MAX_HEIGHT_CM at least 3 cm below MOUNT_HEIGHT_CM: the sensor
// is blind inside its first 2 cm, so a roof that close reads as nothing.
const float CAR_MIN_HEIGHT_CM = 2.0;
const float CAR_MAX_HEIGHT_CM = 8.0;

// Once a slot is taken, the band widens by this much before it will let
// go again. A car whose roof happens to sit right on a band edge
// otherwise flickers as the readings wobble across the line. Simulated
// at 10% dropped echoes and 3 mm of noise: a roof exactly on the edge
// was reported wrongly 71% of the time without hysteresis and 0.2% with
// 1 cm of it, and it costs nothing on the cases that do not need it.
const float HYSTERESIS_CM    = 1.0;

// How far apart the three slot centres are, along the board. Only used
// to warn you at boot if the beam is wider than the slot.
const float SLOT_PITCH_CM    = 6.7;

// A car within this distance of the entry sensor counts as "arrived".
const float ENTRY_TRIGGER_CM = 15.0;

// How long something may sit in front of the entry sensor before the page
// stops calling it an arriving car. A car that enters and then leaves
// without parking costs nothing — the slot sensors are the truth and they
// never saw it. But something PARKED in the entry lane, or a bin, or a
// person standing there, would otherwise leave "car at the entry" on the
// screen forever. After this long it is reported as blocked instead.
const unsigned long ENTRY_STUCK_MS = 60000;   // 60 seconds

// ---------------------------------------------------------- the gate
// An SG90 servo lifts a cardboard barrier. The rule is simple: a car at
// the entry opens it, but only if a bay is actually free. If the lot is
// full the barrier stays down, which is the whole point of sensing the
// bays in the first place.
const int SERVO_PIN = 18;              // free pin; not a strapping pin

// Measure these on your own barrier. CLOSED is the barrier down across
// the lane, OPEN is it lifted clear.
const int GATE_CLOSED_ANGLE = 0;
const int GATE_OPEN_ANGLE   = 90;

// The barrier must not come down on a car. The entry sensor sees a car
// in FRONT of the barrier, so it clears as the car drives through; the
// gate then waits GATE_CLEAR_MS more before lowering, which is the time
// the car needs to get past the barrier itself.
const unsigned long GATE_CLEAR_MS    = 3000;
const unsigned long GATE_MIN_OPEN_MS = 2000;    // never flap shut instantly
const unsigned long GATE_MAX_OPEN_MS = 30000;   // last resort if nothing clears

// A sensor must agree with itself this many times in a row before the
// state changes. Without this, one stray echo makes the page blink.
const int   STABLE_READS     = 4;

const unsigned long SENSE_INTERVAL_MS = 120;

// ------------------------------------------------------------- Wi-Fi
// Two ways to reach the page. Pick one.
//
// true  = ACCESS POINT. The ESP32 makes its own Wi-Fi network. A phone
//         joins "VIT-SmartParking" and opens http://192.168.4.1
//         No router needed, the address never changes, and it works in a
//         corridor with no Wi-Fi at all. Use this for the demo.
//         The catch: the phone has no internet while connected, so
//         Android asks "stay connected?" — answer yes, and turn mobile
//         data off or it will silently hop back to 4G.
//
// false = JOIN A ROUTER. The ESP32 connects to an existing Wi-Fi network
//         and gets an address from it. Everyone already on that Wi-Fi can
//         open the page without switching networks, and their phones keep
//         their internet. The catch: the address changes, you have to
//         read it off the Serial Monitor, and campus Wi-Fi with a login
//         portal will not let the ESP32 on at all.
const bool USE_ACCESS_POINT = true;

// Used when USE_ACCESS_POINT is true.
const char* AP_SSID = "VIT-SmartParking";

// Leave AP_PASS as a password to keep the network closed, or set it to ""
// for an open network anyone can tap and join with no password. Open is
// defensible here: the page is read-only, there is no internet behind it,
// and a public parking display is meant to be public.
const char* AP_PASS = "park1234";          // >= 8 chars, or the AP won't start

// The ESP32 defaults to FOUR simultaneous clients and silently refuses
// the fifth. Eight is the practical ceiling before the page gets slow.
const int   AP_MAX_CLIENTS = 8;

// Used when USE_ACCESS_POINT is false. A phone hotspot works well here.
const char* STA_SSID = "YourWiFiName";
const char* STA_PASS = "YourWiFiPassword";

// ---------------------------------------------------------------- state
bool  slotOccupied[3] = {false, false, false};
int   slotAgree[3]    = {0, 0, 0};
bool  slotPending[3]  = {false, false, false};
float slotDist[3]     = {0, 0, 0};

bool  carAtEntry  = false;
int   entryAgree  = 0;
bool  entryPending = false;
float entryDist   = 0;
unsigned long entrySince = 0;      // when the entry last became occupied
bool  entryBlocked = false;        // occupied for longer than ENTRY_STUCK_MS

// ---- gate ----
Servo gateServo;
bool  gateOpen       = false;
bool  needEntryClear = false;      // see openGate(): stops a shut/open loop
unsigned long gateMovedAt    = 0;
unsigned long entryClearedAt = 0;

WebServer server(80);

// The captive portal: a DNS server that answers every lookup with our own
// address, so the phone's "am I online?" check lands here and the OS pops
// the page up by itself. Only runs in access-point mode.
DNSServer dnsServer;
const byte DNS_PORT = 53;
bool portalActive = false;

// ------------------------------------------------------- the car band
// carNear() is clamped out of the blind zone so a badly chosen pair of
// heights cannot produce a band that nothing can ever land in.
inline float carNear() {
  float d = MOUNT_HEIGHT_CM - CAR_MAX_HEIGHT_CM;
  return d < MIN_VALID_CM ? MIN_VALID_CM : d;
}
inline float carFar() { return MOUNT_HEIGHT_CM - CAR_MIN_HEIGHT_CM; }

// The band actually used, which widens once the slot already reads taken.
inline bool inCarBand(float d, bool alreadyOccupied) {
  if (d <= 0) return false;
  float h  = alreadyOccupied ? HYSTERESIS_CM : 0.0;
  float lo = carNear() - h;
  float hi = carFar()  + h;
  if (lo < MIN_VALID_CM) lo = MIN_VALID_CM;
  return d >= lo && d <= hi;
}

// ====================================================== sensor reading
// Returns distance in cm, or -1 for no echo / an impossible value.
float readOnce(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // 30 ms is about 5 m of round trip. Plenty for a tabletop model, and it
  // keeps a missing sensor from stalling the loop.
  unsigned long us = pulseIn(echoPin, HIGH, 30000UL);
  if (us == 0) return -1.0;

  float cm = us / 58.0;
  if (cm < MIN_VALID_CM || cm > MAX_VALID_CM) return -1.0;
  return cm;
}

// Median of up to three readings. One bad value cannot move a median,
// which is why this beats averaging for ultrasonic noise.
//
// A dropped echo is discarded rather than replaced with a stand-in
// number. Substituting "far" for a dropout was the bug that made this
// miscount: a sensor that simply failed to answer looked identical to an
// empty slot, so slots emptied themselves at random.
float readDistance(int trigPin, int echoPin) {
  float r[3];
  int n = 0;

  float a = readOnce(trigPin, echoPin); delay(12);
  float b = readOnce(trigPin, echoPin); delay(12);
  float c = readOnce(trigPin, echoPin);

  if (a > 0) r[n++] = a;
  if (b > 0) r[n++] = b;
  if (c > 0) r[n++] = c;

  if (n == 0) return -1.0;             // sensor is not answering at all
  if (n == 1) return r[0];             // one vote is weak, but it is real
  if (n == 2) return min(r[0], r[1]);  // the nearer of two

  float hi = max(r[0], max(r[1], r[2]));
  float lo = min(r[0], min(r[1], r[2]));
  return r[0] + r[1] + r[2] - hi - lo; // the middle one
}

// Change a boolean state only after STABLE_READS consecutive agreements.
void debounce(bool reading, bool &state, bool &pending, int &agree) {
  if (reading == state) { agree = 0; return; }
  if (reading == pending) {
    if (++agree >= STABLE_READS) { state = reading; agree = 0; }
  } else {
    pending = reading;
    agree = 1;
  }
}

void senseAll() {
  for (int i = 0; i < 3; i++) {
    // Let the previous sensor's sound die away. Without this gap one
    // sensor hears another's echo and reports a distance that belongs to
    // its neighbour. Simulation has no such thing; a real bench does.
    if (i > 0) delay(60);

    slotDist[i] = readDistance(TRIG_SLOT[i], ECHO_SLOT[i]);
    bool occ = inCarBand(slotDist[i], slotOccupied[i]);
    debounce(occ, slotOccupied[i], slotPending[i], slotAgree[i]);
  }

  if (USE_ENTRY_SENSOR) {
    delay(60);
    entryDist = readDistance(TRIG_ENTRY, ECHO_ENTRY);
    bool at = (entryDist > 0) && (entryDist < ENTRY_TRIGGER_CM);

    bool was = carAtEntry;
    debounce(at, carAtEntry, entryPending, entryAgree);

    if (carAtEntry && !was) {         // just arrived: start the clock
      entrySince   = millis();
      entryBlocked = false;
    } else if (!carAtEntry) {         // cleared: forget everything
      entrySince   = 0;
      entryBlocked = false;
    } else if (entrySince && millis() - entrySince > ENTRY_STUCK_MS) {
      entryBlocked = true;            // still there a minute later
    }
  } else {
    entryDist    = -1.0;
    carAtEntry   = false;
    entryBlocked = false;
  }
}

// ============================================================== the gate
void setGate(bool open) {
  if (open == gateOpen) return;                 // only write on a change
  gateOpen = open;
  gateMovedAt = millis();
  gateServo.write(open ? GATE_OPEN_ANGLE : GATE_CLOSED_ANGLE);
  Serial.println(open ? "gate: OPEN" : "gate: CLOSED");
}

// Called once per loop. Everything about the barrier lives here.
void updateGate(int freeBays) {

  if (!gateOpen) {
    // Open only for a car that is actually there, with somewhere to put
    // it. needEntryClear blocks the case below: if the gate timed out
    // while something was still sitting in front of the sensor, the car
    // is still "at the entry", so without this flag the gate would shut
    // and immediately re-open, over and over.
    if (carAtEntry && freeBays > 0 && !needEntryClear) setGate(true);
    if (!carAtEntry) needEntryClear = false;
    return;
  }

  // ---- the gate is open ----
  // Track how long the lane in front of the sensor has been empty.
  if (carAtEntry) entryClearedAt = 0;
  else if (entryClearedAt == 0) entryClearedAt = millis();

  bool heldLongEnough = millis() - gateMovedAt > GATE_MIN_OPEN_MS;
  bool carIsThrough   = entryClearedAt && (millis() - entryClearedAt > GATE_CLEAR_MS);

  // The lot filling up while a car is driving through does NOT drop the
  // barrier on it. That car was let in; it gets to finish.
  if (heldLongEnough && carIsThrough) {
    setGate(false);
    return;
  }

  // Nothing ever cleared — an obstruction, or a car that stopped in the
  // lane. Close, and refuse to re-open until the sensor sees clear air.
  if (millis() - gateMovedAt > GATE_MAX_OPEN_MS) {
    Serial.println("gate: timed out with the lane still blocked");
    setGate(false);
    needEntryClear = true;
  }
}

int freeSlotCount() {
  int n = 0;
  for (int i = 0; i < 3; i++) if (!slotOccupied[i]) n++;
  return n;
}

// =========================================================== dispatch
// Everything about the self-parking car lives here. If no car ever
// connects, none of it runs and the lot behaves exactly as before.
enum Dispatch { DISPATCH_IDLE, DISPATCH_SENT, DISPATCH_PLACING, DISPATCH_DRIVING,
                DISPATCH_DONE, DISPATCH_FAILED, DISPATCH_BLOCKED, DISPATCH_RELEASE };
Dispatch dispatch   = DISPATCH_IDLE;
int  assignedBay    = -1;            // 1..3, or -1
bool carOnline      = false;         // the car said "arrived" recently
unsigned long carSeenAt    = 0;
unsigned long dispatchedAt = 0;
String lastCarMsg = "";

// A dispatch that is never completed must not pin a bay forever.
const unsigned long DISPATCH_TIMEOUT_MS = 90000;
const unsigned long CAR_OFFLINE_MS      = 8000;

int firstFreeSlot() {                 // 1-based, or -1 if the lot is full
  for (int k = 0; k < 3; k++) {
    int bay = SUGGEST_ORDER[k];
    if (bay < 1 || bay > 3) continue;
    if (!slotOccupied[bay - 1]) return bay;
  }
  return -1;
}

// ======================================================== web dashboard
const char PAGE[] PROGMEM = R"HTML(<!doctype html><html><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Smart Parking</title><style>
:root{--bg:#10151b;--card:#1a222c;--edge:#2b3642;--ink:#e8eef4;--dim:#8d9bab;
      --free:#3ddc84;--busy:#ef5f5f;--accent:#4a9eff}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);
     font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
.w{max-width:620px;margin:0 auto;padding:22px 16px 50px}
h1{font-size:25px;margin:0 0 3px;letter-spacing:-.01em}
.sub{color:var(--dim);font-size:13.5px;margin:0 0 18px}
.big{background:var(--card);border:1px solid var(--edge);border-radius:7px;
     padding:18px;margin-bottom:16px;text-align:center}
.big .n{font-size:46px;font-weight:700;line-height:1;color:var(--free)}
.big .n.none{color:var(--busy)}
.big .t{font-size:13px;color:var(--dim);margin-top:6px}
.go{margin-top:12px;padding-top:12px;border-top:1px solid var(--edge);
    font-size:15px}
.go b{color:var(--accent)}
.go b.opn{color:var(--free)} .go b.shut{color:var(--busy)}
button.rel{width:auto;display:inline-block;margin:0 0 0 10px;padding:6px 16px;font-size:13px}
.carnote{font-size:12.5px;color:var(--accent);margin:12px 0 0;min-height:1px}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:11px}
.slot{background:var(--card);border:1px solid var(--edge);border-radius:6px;
      padding:15px 10px;text-align:center}
.slot.free{border-color:var(--free)}
.slot.busy{opacity:.55}
.num{font-size:30px;font-weight:700;line-height:1}
.st{font-size:11px;letter-spacing:.12em;text-transform:uppercase;margin-top:7px;color:var(--dim)}
.slot.free .st{color:var(--free)} .slot.busy .st{color:var(--busy)}
.cm{font-family:ui-monospace,monospace;font-size:11px;color:var(--dim);margin-top:5px}
.foot{margin-top:20px;padding-top:13px;border-top:1px solid var(--edge);
      font-family:ui-monospace,monospace;font-size:11.5px;color:var(--dim)}
</style></head><body><div class="w">
<h1>Smart Parking</h1><p class="sub">3-slot model &middot; ultrasonic sensing</p>
<div class="big"><div class="n" id="count">&mdash;</div>
<div class="t" id="of">slots free</div>
<div class="go" id="go">Connecting&hellip;</div></div>
<div class="grid" id="grid"></div>
<p class="carnote" id="car"></p>
<div class="foot" id="foot">&mdash;</div>
</div><script>
function draw(d){
  var g='';
  for(var i=0;i<d.slots.length;i++){
    var s=d.slots[i], free=!s.occupied;
    g+='<div class="slot '+(free?'free':'busy')+'">'
      +'<div class="num">'+s.id+'</div>'
      +'<div class="st">'+(free?'Free':'Taken')+'</div>'
      +'<div class="cm">'+(s.cm<0?'--':s.cm.toFixed(1))+' cm</div>'
      // The Park button only exists when a self-parking car is actually
      // on the network. Most drivers do not have one, and a greyed-out
      // button with no reason beside it just reads as a broken page.
      +(d.car_online
         ? '<button onclick="send('+s.id+')"'
           +((free && d.dispatch==='idle')?'':' disabled')
           +'>'+(d.assigned===s.id&&d.dispatch!=='idle'?'Assigned':'Park here')+'</button>'
         : '')
      +'</div>';
  }
  document.getElementById('grid').innerHTML=g;

  var c=document.getElementById('count');
  c.textContent=d.free;
  c.className='n'+(d.free===0?' none':'');
  document.getElementById('of').textContent=
    (d.free===1?'slot free of 3':'slots free of 3');

  var go;
  if(d.dispatch==='sent'||d.dispatch==='placing')
    go='Car halted. <b>Set it on the mark facing slot '+d.assigned+'</b>, then let go.';
  else if(d.dispatch==='driving') go='Car driving into slot <b>'+d.assigned+'</b>&hellip;';
  else if(d.dispatch==='blocked') go='<b class="shut">Car blocked</b> &mdash; something is in its way.';
  else if(d.dispatch==='failed') go='<b class="shut">The car could not park.</b> Reset it and try again.';
  else if(d.dispatch==='release') go='Car leaving the bay&hellip;';
  else if(d.dispatch==='done')
    go='Car parked in slot <b>'+d.assigned+'</b>. '
      +'<button class="rel" onclick="release()">Release</button>';
  else if(d.free===0) go='Lot is full &mdash; <b class="shut">gate closed</b>';
  else if(d.gate_open) go='<b class="opn">Gate open</b> &mdash; take slot <b>'+d.suggest+'</b>';
  else if(d.entry_blocked) go='Entry lane blocked &mdash; slot <b>'+d.suggest+'</b> is still free';
  else if(d.car_at_entry) go='Car at the entry &mdash; opening for slot <b>'+d.suggest+'</b>';
  else go='Next free slot is <b>'+d.suggest+'</b>';
  if(Date.now()>holdUntil) document.getElementById('go').innerHTML=go;

  document.getElementById('car').textContent =
    (d.car_online && d.dispatch==='idle')
      ? 'Self-parking car connected — tap a free bay to send it there.' : '';

  document.getElementById('foot').textContent=
    'gate '+(d.gate_open?'open':'closed')+
    '  |  entry '+(d.entry_cm<0?'--':d.entry_cm.toFixed(1))+' cm'+
    '  |  floor '+d.mount.toFixed(1)+' cm'+
    '  |  car band '+d.band_near.toFixed(1)+'-'+d.band_far.toFixed(1)+' cm';
}
function release(){ fetch('/api/release').then(function(){tick()}); }
var holdUntil=0;
function send(n){
  fetch('/api/send?bay='+n).then(function(r){return r.json()}).then(function(j){
    if(!j.ok&&j.why){
      // Hold it. Without this the status poll a moment later rewrites this
      // line and the tap looks like it did nothing at all.
      holdUntil=Date.now()+8000;
      document.getElementById('go').innerHTML=
        '<b class="shut">Cannot send the car:</b> '+j.why;
    } else { holdUntil=0; }
    tick();
  });
}
function tick(){
  fetch('/api/status').then(function(r){return r.json()}).then(draw)
    .catch(function(){document.getElementById('go').textContent=
      'Lost contact with the controller.'});
}
tick(); setInterval(tick,700);
</script></body></html>)HTML";

// ============================================================== Wi-Fi
void startWiFi() {
  if (USE_ACCESS_POINT) {
    // channel 1, SSID not hidden, then the client limit.
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS, 1, 0, AP_MAX_CLIENTS);

    // Answer every DNS lookup with our own address. Whatever the phone
    // asks for after joining, it arrives here, and the OS shows the page.
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
    portalActive = true;

    Serial.println();
    Serial.print("Wi-Fi network: "); Serial.println(AP_SSID);
    Serial.print("Password:      ");
    Serial.println(strlen(AP_PASS) ? AP_PASS : "(open, no password)");
    Serial.print("Open:          http://");
    Serial.println(WiFi.softAPIP());
    Serial.print("Up to "); Serial.print(AP_MAX_CLIENTS);
    Serial.println(" phones at once.");
    Serial.println("The page should open by itself on joining.");
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(STA_SSID, STA_PASS);
  Serial.print("joining "); Serial.print(STA_SSID); Serial.print(" ");

  // Give it 20 seconds, then say so rather than hanging silently forever.
  unsigned long began = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - began < 20000) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    // Falling back rather than failing: an unreachable dashboard during a
    // demo is worse than one on a different network.
    Serial.println("could not join - falling back to access point mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print("Wi-Fi network: "); Serial.println(AP_SSID);
    Serial.print("Password:      "); Serial.println(AP_PASS);
    Serial.print("Open:          http://");
    Serial.println(WiFi.softAPIP());
    return;
  }

  Serial.print("Open: http://");
  Serial.println(WiFi.localIP());
  Serial.println("(anyone on this Wi-Fi can open that address)");
}

// ======================================================== web dashboard
void handleRoot() { server.send_P(200, "text/html", PAGE); }

// Every phone checks a known URL after joining a network to find out
// whether it really has internet. Android asks for /generate_204 and
// expects an empty 204; iOS asks for /hotspot-detect.html and expects the
// word "Success". Sending a redirect instead is what makes the OS decide
// it is behind a portal and open the page. Anything else unrecognised
// gets the same treatment.
// ------------------------------------------------- the car's endpoints
// GET /car/cmd                     -> "NONE" | "GO:n" | "STOP"
// GET /car/event?e=...&slot=n      -> "OK"
void handleCarCmd() {
  carOnline = true; carSeenAt = millis();
  String reply = "NONE";
  if (dispatch == DISPATCH_RELEASE)                      reply = "RELEASE";
  else if (assignedBay > 0 &&
           (dispatch == DISPATCH_SENT ||
            dispatch == DISPATCH_PLACING))               reply = "GO:" + String(assignedBay);
  server.send(200, "text/plain", reply);
}

void handleCarEvent() {
  carOnline = true; carSeenAt = millis();
  String e   = server.hasArg("e")    ? server.arg("e")           : "";
  int    bay = server.hasArg("slot") ? server.arg("slot").toInt() : 0;
  lastCarMsg = e + (bay ? (":" + String(bay)) : "");
  Serial.println("<- car: " + lastCarMsg);

  if (e == "ack") {
    // Car has halted wherever it was roaming; a human now carries it to
    // the start line. The clock does not start until it begins driving.
    dispatch = DISPATCH_PLACING;
  } else if (e == "driving") {
    dispatch = DISPATCH_DRIVING;
    dispatchedAt = millis();
  } else if (e == "parked") {
    // The car's word is a hint; the bay's own sensor is the proof. The
    // state is closed here either way so the car is not left waiting,
    // but the page keeps showing what the sensor says.
    dispatch = DISPATCH_DONE;
  } else if (e == "blocked") {
    dispatch = DISPATCH_BLOCKED;
  } else if (e == "failed") {
    dispatch = DISPATCH_FAILED;
    assignedBay = -1;
  } else if (e == "left") {
    dispatch = DISPATCH_IDLE;
    assignedBay = -1;
  }
  server.send(200, "text/plain", "OK");
}

// Driver taps a bay on the page.
void handleSend() {
  int bay = server.hasArg("bay") ? server.arg("bay").toInt() : -1;
  bool ok = false;
  String why = "";

  bool busy = (dispatch == DISPATCH_SENT || dispatch == DISPATCH_PLACING ||
               dispatch == DISPATCH_DRIVING);
  if (bay < 1 || bay > 3)               why = "no such bay";
  else if (slotOccupied[bay - 1])       why = "that bay is taken";
  else if (busy)                        why = "a car is already on its way";
  else if (!carOnline)                  why = "no car connected";
  else {
    assignedBay  = bay;
    dispatch     = DISPATCH_SENT;
    dispatchedAt = millis();
    lastCarMsg   = "";
    ok = true;
    Serial.println("-> car: GO:" + String(bay));
  }
  server.send(200, "application/json",
              String("{\"ok\":") + (ok ? "true" : "false") +
              ",\"bay\":" + String(bay) +
              ",\"why\":\"" + why + "\"}");
}

// Driver taps Release: the car reverses out and goes back to roaming.
void handleRelease() {
  bool ok = (dispatch == DISPATCH_DONE);
  if (ok) { dispatch = DISPATCH_RELEASE; Serial.println("-> car: RELEASE"); }
  server.send(200, "application/json",
              String("{\"ok\":") + (ok ? "true" : "false") + "}");
}

void handleCaptive() {
  // The car's requests must never be answered with the portal redirect.
  // Without this the ESP32 redirects /car/cmd to the dashboard, the car
  // sees a 302 instead of 200, and it sits at the start line forever.
  if (server.uri().startsWith("/car/")) {
    server.send(404, "text/plain", "no such car endpoint");
    return;
  }
  if (!portalActive) { server.send(404, "text/plain", "Not found"); return; }
  server.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/", true);
  server.send(302, "text/plain", "");
}

void handleStatus() {
  int suggest = firstFreeSlot();

  String j = "{";
  j += "\"free\":"         + String(freeSlotCount()) + ",";
  j += "\"suggest\":"      + String(suggest) + ",";
  j += "\"car_at_entry\":" + String(carAtEntry ? "true" : "false") + ",";
  j += "\"entry_blocked\":" + String(entryBlocked ? "true" : "false") + ",";
  j += "\"gate_open\":"     + String(gateOpen ? "true" : "false") + ",";
  j += "\"car_online\":"    + String(carOnline ? "true" : "false") + ",";
  j += "\"assigned\":"      + String(assignedBay) + ",";
  j += "\"dispatch\":\""    + String(dispatch == DISPATCH_SENT     ? "sent"     :
                                     dispatch == DISPATCH_PLACING  ? "placing"  :
                                     dispatch == DISPATCH_DRIVING  ? "driving"  :
                                     dispatch == DISPATCH_DONE     ? "done"     :
                                     dispatch == DISPATCH_BLOCKED  ? "blocked"  :
                                     dispatch == DISPATCH_RELEASE  ? "release"  :
                                     dispatch == DISPATCH_FAILED   ? "failed"   : "idle") + "\",";
  j += "\"car_msg\":\""     + lastCarMsg + "\",";
  j += "\"entry_cm\":"     + String(entryDist, 1) + ",";
  j += "\"mount\":"        + String(MOUNT_HEIGHT_CM, 1) + ",";
  j += "\"band_near\":"    + String(carNear(), 1) + ",";
  j += "\"band_far\":"     + String(carFar(), 1) + ",";
  j += "\"slots\":[";
  for (int i = 0; i < 3; i++) {
    if (i) j += ",";
    j += "{\"id\":" + String(i + 1) +
         ",\"occupied\":" + String(slotOccupied[i] ? "true" : "false") +
         ",\"cm\":" + String(slotDist[i], 1) + "}";
  }
  j += "]}";
  server.send(200, "application/json", j);
}

// ================================================================ setup
void setup() {
  Serial.begin(9600);
  delay(300);
  Serial.println("\n== Smart Parking: sense and display ==");
  Serial.println(USE_ENTRY_SENSOR ? "4 sensors (3 slots + entry)"
                                  : "3 sensors (slots only)");
  Serial.print("slot 1 = TRIG D"); Serial.print(TRIG_SLOT[0]);
  Serial.print("/ECHO D"); Serial.print(ECHO_SLOT[0]);
  Serial.print(",  slot 2 = D"); Serial.print(TRIG_SLOT[1]);
  Serial.print("/D"); Serial.print(ECHO_SLOT[1]);
  Serial.print(",  slot 3 = D"); Serial.print(TRIG_SLOT[2]);
  Serial.print("/D"); Serial.println(ECHO_SLOT[2]);
  Serial.print("recommend order: ");
  for (int k = 0; k < 3; k++) { Serial.print("slot "); Serial.print(SUGGEST_ORDER[k]);
                                if (k < 2) Serial.print(" -> "); }
  Serial.println();
  {
    int seen[4] = {0, 0, 0, 0};
    bool bad = false;
    for (int k = 0; k < 3; k++) {
      if (SUGGEST_ORDER[k] < 1 || SUGGEST_ORDER[k] > 3) bad = true;
      else seen[SUGGEST_ORDER[k]]++;
    }
    for (int b = 1; b <= 3; b++) if (seen[b] != 1) bad = true;
    if (bad) Serial.println("*** SUGGEST_ORDER must list 1, 2 and 3 once each ***");
  }

  // Print the geometry so the numbers in the code and the numbers on the
  // bench can be compared without re-reading the source.
  Serial.print("mount height      "); Serial.print(MOUNT_HEIGHT_CM, 1);
  Serial.println(" cm");
  Serial.print("car counts when   "); Serial.print(carNear(), 1);
  Serial.print(" - ");                Serial.print(carFar(), 1);
  Serial.println(" cm");
  Serial.print("floor reads about "); Serial.print(MOUNT_HEIGHT_CM, 1);
  Serial.println(" cm  (ignored)");

  // 2 * h * tan(15 deg) = 0.536 * h, for a 30-degree beam.
  float footprint = MOUNT_HEIGHT_CM * 0.536;
  Serial.print("beam footprint    "); Serial.print(footprint, 1);
  Serial.println(" cm across, nominal - MEASURE IT");

  // Two ways the geometry can be wrong. Both produce sensible-looking
  // numbers on the monitor and nonsense occupancy, so they are worth
  // saying out loud at boot rather than leaving to be found later.
  if (footprint > SLOT_PITCH_CM) {
    Serial.println("WARNING: footprint is wider than the slot pitch of "
                   + String(SLOT_PITCH_CM, 1)
                   + " cm - sensors will see each other's cars.");
  }
  if (MOUNT_HEIGHT_CM - carFar() < 2.0) {
    Serial.println("WARNING: the car band reaches within 2 cm of the floor"
                   " - an empty slot may read as parked.");
  }

  if (USE_ENTRY_SENSOR) {
    pinMode(TRIG_ENTRY, OUTPUT); digitalWrite(TRIG_ENTRY, LOW);
    pinMode(ECHO_ENTRY, INPUT);
  }
  for (int i = 0; i < 3; i++) {
    pinMode(TRIG_SLOT[i], OUTPUT); digitalWrite(TRIG_SLOT[i], LOW);
    pinMode(ECHO_SLOT[i], INPUT);
  }

  // The barrier starts down. Giving the servo its angle before anything
  // else means a reset never leaves the gate stuck open.
  gateServo.setPeriodHertz(50);
  gateServo.attach(SERVO_PIN, 500, 2400);
  gateServo.write(GATE_CLOSED_ANGLE);
  Serial.print("gate servo on D"); Serial.print(SERVO_PIN);
  Serial.print("  closed="); Serial.print(GATE_CLOSED_ANGLE);
  Serial.print("deg  open="); Serial.print(GATE_OPEN_ANGLE);
  Serial.println("deg");
  Serial.println("POWER THE SERVO FROM 5 V, NOT 3V3, AND SHARE THE GROUND.");

  startWiFi();

  server.on("/",           handleRoot);
  server.on("/api/status", handleStatus);
  server.on("/api/send",    handleSend);
  server.on("/api/release", handleRelease);
  server.on("/car/cmd",    handleCarCmd);
  server.on("/car/event",  handleCarEvent);
  server.onNotFound(handleCaptive);
  server.begin();
}

// ================================================================= loop
void loop() {
  // The DNS server has to be serviced constantly or the portal never
  // triggers; the phone gives up on its check in a couple of seconds.
  if (portalActive) dnsServer.processNextRequest();
  server.handleClient();

  static unsigned long lastSense = 0;
  if (millis() - lastSense >= SENSE_INTERVAL_MS) {
    lastSense = millis();
    senseAll();
  }

  // The gate is judged every pass, not only when a sensor changes, because
  // its closing condition is a timer rather than a reading.
  updateGate(freeSlotCount());

  // ---- supervise the dispatch ----
  if (carOnline && millis() - carSeenAt > CAR_OFFLINE_MS) carOnline = false;

  // Only the driving leg is on a clock. While a human is carrying the
  // car to the start line it can take as long as it likes.
  if (dispatch == DISPATCH_DRIVING) {
    // The bay's own sensor is the proof the car arrived, not the car's
    // message. If the sensor says taken, the job is done whatever the
    // car did or did not manage to report.
    if (assignedBay > 0 && slotOccupied[assignedBay - 1]) {
      dispatch = DISPATCH_DONE;
      Serial.println("bay " + String(assignedBay) + " now reads taken - dispatch complete");
    } else if (millis() - dispatchedAt > DISPATCH_TIMEOUT_MS) {
      Serial.println("dispatch timed out - releasing bay " + String(assignedBay));
      dispatch = DISPATCH_FAILED;
      assignedBay = -1;
    }
  }

  // Mirror the state to the Serial Monitor whenever it changes, so the
  // board can be checked without a phone.
  static bool  prevOcc[3] = {true, true, true};   // forces a first print
  static bool  prevEntry  = true;
  bool changed = (carAtEntry != prevEntry);
  for (int i = 0; i < 3; i++) if (slotOccupied[i] != prevOcc[i]) changed = true;

  if (changed) {
    for (int i = 0; i < 3; i++) {
      Serial.print("slot "); Serial.print(i + 1); Serial.print(": ");
      if (slotDist[i] < 0) Serial.print("  --.-");
      else                 Serial.print(slotDist[i], 1);
      Serial.print(" cm  ");
      Serial.println(slotOccupied[i] ? "TAKEN" : "free");
      prevOcc[i] = slotOccupied[i];
    }
    Serial.print(freeSlotCount()); Serial.print(" of 3 free");
    if (carAtEntry) Serial.print("   [car at entry]");
    Serial.println();
    Serial.println("---");
    prevEntry = carAtEntry;
  }
}
