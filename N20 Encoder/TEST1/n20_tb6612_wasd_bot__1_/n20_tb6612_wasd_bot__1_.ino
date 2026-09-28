/*
  Dual N20 Encoder-Motor Bot — ESP8266 (NodeMCU) + TB6612FNG
  ---------------------------------------------------------------------
  Same UDP protocol as before: PC sends W/A/S/D/X packets while keys
  are held; ESP8266 drives continuously, auto-stops if no packet
  arrives within COMMAND_TIMEOUT_MS. A command may optionally carry a
  2-digit speed percent right after the letter (e.g. "W70" = forward
  at 70%); no digits = 100%, so plain single-letter senders
  (wasd_control.py) work unchanged.

  PIN ASSIGNMENTS (NodeMCU silkscreen -> GPIO). Uses your existing
  wiring as-is; only change from before is STBY moving off a GPIO:
    PWMB -> D1 (GPIO5)   — right motor speed
    BIN2 -> D2 (GPIO4)
    [D3 skipped]
    BIN1 -> D4 (GPIO2)   — boot pin, pulled HIGH, shares onboard LED;
                            direction-only, harmless
    Left encoder C1 -> D5 (GPIO14) — interrupt-driven tick count
    AIN1 -> D6 (GPIO12)  — left motor direction
    AIN2 -> D7 (GPIO13)  — left motor direction
    PWMA -> D8 (GPIO15)  — left motor speed
    Right encoder C1 -> D0 (GPIO16) — polled each loop() iteration;
                          GPIO16 can't do interrupts, but a plain
                          digitalRead every loop cycle catches edges
                          fine at this pulse rate.

  STBY: hardwired directly to 3.3V (no longer a GPIO) — this is what
  frees up D5 for the left encoder. Trade-off: the driver is now live
  as soon as power is applied, rather than disabled-until-WiFi-connects
  in software. PWM still starts at 0 in code either way, so this stays
  harmless, just noting the behavior change.

  ENCODERS: only C1 per motor is used — C2 is left unconnected. We
  don't need it for direction, since we already know which way each
  motor was just commanded to spin; C1 alone is enough to measure
  tick-rate and keep the two wheels in sync. Wire each N20's VCC to
  3V3, GND to common GND, and leave C2 unconnected.

  POWER:
    TB6612 VM   -> motor battery, matched to your N20's rated voltage
    TB6612 VCC  -> NodeMCU 3V3 (logic only)
    TB6612 STBY -> NodeMCU 3V3 (hardwired, see above)
    TB6612 GND  -> common with battery GND and NodeMCU GND
    AO1/AO2     -> left N20  M1/M2
    BO1/BO2     -> right N20 M1/M2

  COMMANDS:
    W = both motors forward       S = both motors backward
    A = pivot left                D = pivot right
    X = stop (also sent automatically on timeout)

  SPEED: W/S ramp smoothly up to SPEED_STRAIGHT; A/D ramp up to the
  slower SPEED_TURN, both scaled by the command's speed percent. The
  ramp applies to every transition except the watchdog timeout, which
  cuts power instantly for safety.

  WHEEL SYNC: while driving straight (W/S only — turns are left alone),
  each motor's C1 tick-rate is compared every SYNC_INTERVAL_MS and
  whichever side is lagging gets a small PWM trim (capped at MAX_TRIM)
  to match the other. This is relative (no absolute RPM calibration
  needed) — it just keeps both wheels honest against each other so the
  bot drives straight instead of curving from motor-to-motor variance.
  If it hunts/oscillates, lower SYNC_KP; if correction feels sluggish,
  raise it.
*/

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// ---------- WiFi credentials ----------
const char* WIFI_SSID = "ASUS_1E_NIRO_2.4G";
const char* WIFI_PASS = "niro@2026";

// ---------- UDP ----------
const unsigned int UDP_PORT = 4210;
WiFiUDP udp;

// ---------- TB6612FNG pins (GPIO numbers) ----------
const int PIN_PWMB = 5;  // D1  (right motor speed)
const int PIN_BIN2 = 4;  // D2
const int PIN_BIN1 = 2;  // D4
const int PIN_AIN1 = 12; // D6  (left motor)
const int PIN_AIN2 = 13; // D7
const int PIN_PWMA = 15; // D8  (left motor speed)

// ---------- Encoders (C1 only, direction inferred from command) ----------
const int PIN_LEFT_C1 = 14;  // D5 — interrupt
const int PIN_RIGHT_C1 = 16; // D0 — polled (GPIO16 has no interrupt support)

volatile unsigned long leftTicks = 0;
unsigned long rightTicks = 0;
bool lastRightState = LOW;

void ICACHE_RAM_ATTR onLeftEncoder() {
  leftTicks++;
}

// Called every loop() iteration — cheap enough to just poll rather
// than needing a real interrupt on this pin.
void pollRightEncoder() {
  bool state = digitalRead(PIN_RIGHT_C1);
  if (state != lastRightState) {
    rightTicks++;
    lastRightState = state;
  }
}

// ---------- Tuning ----------
const int SPEED_STRAIGHT = 200;               // W/S target PWM duty, 0-1023
const int SPEED_TURN = 80;                   // A/D target PWM duty — slower than straight
const int RAMP_STEP = 15;                     // PWM units added/removed per ramp tick
const unsigned long RAMP_INTERVAL_MS = 25;    // ~830ms 0->full
const unsigned long COMMAND_TIMEOUT_MS = 300; // auto-stop if no command in this long

const unsigned long SYNC_INTERVAL_MS = 100;   // how often the wheel-sync check runs
const float SYNC_KP = 6.0;                    // trim strength — lower if it hunts, raise if sluggish
const int MAX_TRIM = 150;                     // cap on correction, out of 1023

char currentCommand = 'X';
unsigned long lastCommandMillis = 0;
unsigned long lastRampMillis = 0;
unsigned long lastSyncMillis = 0;

int targetLeft = 0, targetRight = 0;   // where each side is heading, signed PWM
int currentLeft = 0, currentRight = 0; // what's actually being applied right now
int leftTrim = 0, rightTrim = 0;       // wheel-sync correction, straight driving only
unsigned long leftTicksLast = 0, rightTicksLast = 0;

// speed: -1023..1023 (sign = direction, magnitude = PWM duty)
void setMotor(int in1, int in2, int pwmPin, int speed) {
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  analogWrite(pwmPin, abs(speed));
}

// Moves `current` one ramp-step closer to `target`. Works for
// acceleration, deceleration, and reversal alike.
int stepToward(int current, int target, int step) {
  if (current < target) {
    current += step;
    if (current > target) current = target;
  } else if (current > target) {
    current -= step;
    if (current < target) current = target;
  }
  return current;
}

void setTargetsFromCommand(int percent) {
  int straightSpeed = (SPEED_STRAIGHT * percent) / 100;
  int turnSpeed = (SPEED_TURN * percent) / 100;
  switch (currentCommand) {
    case 'W': targetLeft = straightSpeed;  targetRight = straightSpeed;  break;
    case 'S': targetLeft = -straightSpeed; targetRight = -straightSpeed; break;
    case 'A': targetLeft = -turnSpeed;     targetRight = turnSpeed;      break;
    case 'D': targetLeft = turnSpeed;      targetRight = -turnSpeed;     break;
    case 'X':
    default:  targetLeft = 0;              targetRight = 0;              break;
  }
}

// Compares left/right tick-rate and sets a small PWM trim for each
// side. Only active for straight commands (W/S) — turns are left as
// pure open-loop ramp, since the wheels are meant to differ there.
// Both sides spin the "same way" (both forward or both backward)
// whenever this runs, so raw tick-rate magnitude is enough — no need
// to know direction from the encoder itself.
void updateEncoderSync() {
  if (millis() - lastSyncMillis < SYNC_INTERVAL_MS) return;
  lastSyncMillis = millis();

  noInterrupts();
  unsigned long lt = leftTicks;
  interrupts();
  unsigned long rt = rightTicks; // plain read is fine, updated only in loop(), not an ISR

  float leftDelta = lt - leftTicksLast; leftTicksLast = lt;
  float rightDelta = rt - rightTicksLast; rightTicksLast = rt;

  if (currentCommand == 'W' || currentCommand == 'S') {
    float avg = (leftDelta + rightDelta) / 2.0;
    leftTrim = constrain((int)(SYNC_KP * (avg - leftDelta)), -MAX_TRIM, MAX_TRIM);
    rightTrim = constrain((int)(SYNC_KP * (avg - rightDelta)), -MAX_TRIM, MAX_TRIM);
  } else {
    leftTrim = 0;
    rightTrim = 0;
  }
}

int applyTrim(int value, int trim) {
  int sign = (value > 0) - (value < 0);
  int magnitude = constrain(abs(value) + trim, 0, 1023);
  return sign * magnitude;
}

void updateRamp() {
  if (millis() - lastRampMillis < RAMP_INTERVAL_MS) return;
  lastRampMillis = millis();

  currentLeft = stepToward(currentLeft, targetLeft, RAMP_STEP);
  currentRight = stepToward(currentRight, targetRight, RAMP_STEP);

  updateEncoderSync();

  setMotor(PIN_AIN1, PIN_AIN2, PIN_PWMA, applyTrim(currentLeft, leftTrim));
  setMotor(PIN_BIN1, PIN_BIN2, PIN_PWMB, applyTrim(currentRight, rightTrim));
}

// Immediate hard stop, bypassing the ramp — used for the safety
// timeout, where "gently coast to a stop" is the wrong call.
void stopNow() {
  targetLeft = 0;
  targetRight = 0;
  currentLeft = 0;
  currentRight = 0;
  leftTrim = 0;
  rightTrim = 0;
  setMotor(PIN_AIN1, PIN_AIN2, PIN_PWMA, 0);
  setMotor(PIN_BIN1, PIN_BIN2, PIN_PWMB, 0);
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT);

  pinMode(PIN_LEFT_C1, INPUT);
  pinMode(PIN_RIGHT_C1, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_LEFT_C1), onLeftEncoder, CHANGE);
  lastRightState = digitalRead(PIN_RIGHT_C1);

  stopNow(); // motors off (STBY is hardwired, so this is what actually keeps them still)

  analogWriteFreq(20000); // 20kHz PWM — above hearing range, no motor whine

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected. IP address: ");
  Serial.println(WiFi.localIP());

  udp.begin(UDP_PORT);
  Serial.print("Listening for UDP commands on port ");
  Serial.println(UDP_PORT);

  lastCommandMillis = millis();
  lastRampMillis = millis();
  lastSyncMillis = millis();
}

void loop() {
  pollRightEncoder();

  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    char buf[8];
    int len = udp.read(buf, sizeof(buf) - 1);
    if (len > 0) {
      buf[len] = '\0';
      char c = toupper(buf[0]);
      if (c == 'W' || c == 'A' || c == 'S' || c == 'D' || c == 'X') {
        currentCommand = c;
        lastCommandMillis = millis();

        // Optional 2-digit speed percent right after the letter, e.g. "W70".
        int percent = 100;
        if (len > 1 && isDigit(buf[1])) {
          percent = constrain(atoi(buf + 1), 0, 100);
        }

        Serial.print(c);
        Serial.print(' ');
        Serial.println(percent);
        setTargetsFromCommand(percent);
      }
    }
  }

  if (millis() - lastCommandMillis > COMMAND_TIMEOUT_MS) {
    if (currentCommand != 'X') {
      currentCommand = 'X';
      stopNow(); // safety cutoff: instant, not ramped
    }
  }

  updateRamp();
}
