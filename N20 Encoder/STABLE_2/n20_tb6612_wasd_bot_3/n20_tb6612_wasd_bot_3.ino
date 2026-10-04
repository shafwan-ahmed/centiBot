/*
  Dual N20 Encoder-Motor Bot — ESP8266 (NodeMCU) + TB6612FNG
  ---------------------------------------------------------------------
  UDP protocol (port 4210), one packet = one command:
      <letter>[percent][T<ms>]
      "W"        forward, continuous, 100%   (keyboard driving, ramped)
      "W70"      forward, continuous, 70%
      "W45T120"  forward at 45% for exactly 120 ms, then active brake
                 (used by navigate_to_target.py — timing is done HERE so
                 PC/WiFi jitter can't stretch a pulse)
      "X"        stop: glides down through the ramp (keyboard release);
                 aborts a running pulse with an active brake
      "V45,-30"  differential drive: left wheel 45%, right wheel -30%,
                 independently, streamed continuously (no pulse/timer) —
                 for arcing/curved navigation, not just straight+pivot.
                 Same deadband mapping and ramp as W/A/S/D; the 300/500ms
                 watchdog still applies, so this needs resending every
                 frame like the old keyboard commands did.
  Letters: W forward, S backward, A rotate left, D rotate right.

  WHY PULSES: a DC gear motor (unlike a stepper) has a deadband, lags
  behind the camera loop, and keeps moving a bit after power is cut.
  Short timed pulses + braking + letting the camera re-measure between
  pulses converges without overshoot even with a slow camera loop.

  PERCENT -> PWM: percent maps into [PWM_MIN, PWM_MAX], not from 0, so
  even 1% is enough to actually break the motor's stiction. Calibrate
  PWM_MIN_* once: the lowest value where the bot reliably starts moving
  from standstill.

  STOPPING: keyboard driving (X release, watchdog timeout) always glides
  down through the ramp — hard-braking a moving bot can tip it. Only the
  end of a short nav pulse (or aborting one) uses active brake (TB6612
  IN1=IN2=H shorts the motor) for BRAKE_MS. Set BRAKE_MS to 0 to coast
  after pulses instead.

  PIN ASSIGNMENTS (NodeMCU silkscreen -> GPIO):
    PWMB -> D1 (GPIO5)   right motor speed
    BIN2 -> D2 (GPIO4)
    [D3 skipped]
    BIN1 -> D4 (GPIO2)   boot pin, pulled HIGH, shares onboard LED; harmless
    Left encoder C1  -> D5 (GPIO14)  interrupt
    AIN1 -> D6 (GPIO12)  left motor direction
    AIN2 -> D7 (GPIO13)
    PWMA -> D8 (GPIO15)  left motor speed
    Right encoder C1 -> D0 (GPIO16)  polled (no interrupt on this pin)
  STBY hardwired to 3.3V. C2 on both encoders unconnected (unused).

  POWER: TB6612 VM -> motor battery, VCC -> 3V3, STBY -> 3V3, GND common.
         AO1/AO2 -> left N20 M1/M2, BO1/BO2 -> right N20 M1/M2.

  WHEEL SYNC (continuous W/S only, not pulses): compares C1 tick-rate
  and trims the lagging side. OFF by default (SYNC_KP = 0): the right
  encoder is polled so it can undercount, and the resulting trim changes
  motor speed mid-run, which can jolt the bot. Set SYNC_KP to ~3 to enable.
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

// ---------- Encoders (C1 only) ----------
const int PIN_LEFT_C1 = 14;  // D5 — interrupt
const int PIN_RIGHT_C1 = 16; // D0 — polled

volatile unsigned long leftTicks = 0;
unsigned long rightTicks = 0;
bool lastRightState = LOW;

void ICACHE_RAM_ATTR onLeftEncoder() {
  leftTicks++;
}

void pollRightEncoder() {
  bool state = digitalRead(PIN_RIGHT_C1);
  if (state != lastRightState) {
    rightTicks++;
    lastRightState = state;
  }
}

// ---------- Tuning ----------
// Duty range each command type works in (0-1023). MIN = lowest duty that
// reliably starts the bot moving; MAX = top speed at 100%.
const int PWM_MIN_STRAIGHT = 50;
const int PWM_MAX_STRAIGHT = 100;
const int PWM_MIN_TURN = 10;
const int PWM_MAX_TURN = 50;

const int RAMP_STEP = 5;                     // continuous commands only
const unsigned long RAMP_INTERVAL_MS = 25;    // ~830ms 0->full
const unsigned long COMMAND_TIMEOUT_MS = 500; // glide to a stop if no continuous command in this long
const int WATCHDOG_DECEL_STEP = 40;           // faster than RAMP_STEP, still gradual (~300ms from full)
const unsigned long BRAKE_MS = 80;            // active-brake hold after a stop/pulse
const unsigned long MIN_PULSE_MS = 20;
const unsigned long MAX_PULSE_MS = 600;

const unsigned long SYNC_INTERVAL_MS = 100;
const float SYNC_KP = 0.0;                    // 0 = wheel sync off (default); ~3 to enable
const int MAX_TRIM = 150;

char currentCommand = 'X';
unsigned long lastCommandMillis = 0;
unsigned long lastRampMillis = 0;
unsigned long lastSyncMillis = 0;

bool braking = false;
unsigned long brakeEndMillis = 0;
bool pulseActive = false;
unsigned long pulseEndMillis = 0;
int decelStep = RAMP_STEP;             // ramp-down rate; the watchdog temporarily raises it

int targetLeft = 0, targetRight = 0;   // signed PWM
int currentLeft = 0, currentRight = 0; // what's applied right now
int leftTrim = 0, rightTrim = 0;
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

// TB6612 short brake: IN1 = IN2 = HIGH shorts the motor terminals.
void brakeMotor(int in1, int in2, int pwmPin) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
  analogWrite(pwmPin, 1023);
}

// percent 1-100 -> [minPwm, maxPwm]; 0 -> 0 (off)
int pwmFromPercent(int percent, int minPwm, int maxPwm) {
  if (percent <= 0) return 0;
  return minPwm + ((maxPwm - minPwm) * percent) / 100;
}

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
  int straightPwm = pwmFromPercent(percent, PWM_MIN_STRAIGHT, PWM_MAX_STRAIGHT);
  int turnPwm = pwmFromPercent(percent, PWM_MIN_TURN, PWM_MAX_TURN);
  switch (currentCommand) {
    case 'W': targetLeft = straightPwm;  targetRight = straightPwm;  break;
    case 'S': targetLeft = -straightPwm; targetRight = -straightPwm; break;
    case 'A': targetLeft = -turnPwm;     targetRight = turnPwm;      break;
    case 'D': targetLeft = turnPwm;      targetRight = -turnPwm;     break;
    case 'X':
    default:  targetLeft = 0;            targetRight = 0;            break;
  }
}

// Compares left/right tick-rate; trims the lagging side. Continuous
// W/S only — turns and pulses are left alone.
void updateEncoderSync() {
  if (millis() - lastSyncMillis < SYNC_INTERVAL_MS) return;
  lastSyncMillis = millis();

  noInterrupts();
  unsigned long lt = leftTicks;
  interrupts();
  unsigned long rt = rightTicks;

  float leftDelta = lt - leftTicksLast; leftTicksLast = lt;
  float rightDelta = rt - rightTicksLast; rightTicksLast = rt;

  if (SYNC_KP > 0 && (currentCommand == 'W' || currentCommand == 'S')) {
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

void applyMotors() {
  setMotor(PIN_AIN1, PIN_AIN2, PIN_PWMA, applyTrim(currentLeft, leftTrim));
  setMotor(PIN_BIN1, PIN_BIN2, PIN_PWMB, applyTrim(currentRight, rightTrim));
}

// Ramp for continuous (keyboard) driving only. Pulses skip it — a ramp
// would eat most of a short pulse and add lag to the camera loop.
void updateRamp() {
  if (braking || pulseActive) return;
  if (millis() - lastRampMillis < RAMP_INTERVAL_MS) return;
  lastRampMillis = millis();

  int stepL = (abs(targetLeft) < abs(currentLeft)) ? decelStep : RAMP_STEP;
  int stepR = (abs(targetRight) < abs(currentRight)) ? decelStep : RAMP_STEP;
  currentLeft = stepToward(currentLeft, targetLeft, stepL);
  currentRight = stepToward(currentRight, targetRight, stepR);

  updateEncoderSync();
  applyMotors();
}

void releaseMotors() { // coast, driver idle
  braking = false;
  setMotor(PIN_AIN1, PIN_AIN2, PIN_PWMA, 0);
  setMotor(PIN_BIN1, PIN_BIN2, PIN_PWMB, 0);
}

void startBrake() {
  targetLeft = 0;
  targetRight = 0;
  currentLeft = 0;
  currentRight = 0;
  leftTrim = 0;
  rightTrim = 0;
  pulseActive = false;
  if (BRAKE_MS == 0) { // coast instead of braking
    releaseMotors();
    return;
  }
  brakeMotor(PIN_AIN1, PIN_AIN2, PIN_PWMA);
  brakeMotor(PIN_BIN1, PIN_BIN2, PIN_PWMB);
  braking = true;
  brakeEndMillis = millis() + BRAKE_MS;
}

void startPulse(int percent, unsigned long ms) {
  setTargetsFromCommand(percent);
  currentLeft = targetLeft;   // jump straight to speed, no ramp
  currentRight = targetRight;
  leftTrim = 0;
  rightTrim = 0;
  pulseActive = true;
  pulseEndMillis = millis() + ms;
  applyMotors();
}

// Reads an optional '-' then digits starting at p[i]; advances i past
// them. Used by the V command to parse each signed wheel percent.
int readSignedInt(const char* p, int &i) {
  bool neg = false;
  if (p[i] == '-') { neg = true; i++; }
  int v = 0;
  while (isDigit(p[i])) {
    v = v * 10 + (p[i] - '0');
    i++;
  }
  return neg ? -v : v;
}

// V command: "V<leftPercent>,<rightPercent>", each signed -100..100,
// e.g. "V45,-30". Independent per-wheel speed for differential-drive
// navigation (arcing turns, not just symmetric W/A/S/D) — sets targets
// directly and skips the ramp-based W/A/S/D speed tables; the ramp
// itself still applies per side, same as always.
void handleVectorPacket(const char* p) {
  int i = 1;
  // NOTE: constrain() is a macro, so its argument is evaluated up to 3
  // times. readSignedInt() mutates `i` as a side effect, so it must be
  // called into a plain variable FIRST, then clamped — calling it
  // directly inside constrain() re-runs it at the now-advanced (past
  // the digits) position on the macro's other branches, which returns
  // 0 and silently overwrites the real parsed value.
  int rawLeft = readSignedInt(p, i);
  int leftPct = constrain(rawLeft, -100, 100);
  if (p[i] == ',') i++;
  int rawRight = readSignedInt(p, i);
  int rightPct = constrain(rawRight, -100, 100);

  lastCommandMillis = millis();
  currentCommand = 'V';
  braking = false;
  pulseActive = false;
  decelStep = RAMP_STEP;

  targetLeft = (leftPct == 0) ? 0 :
    (leftPct > 0 ? 1 : -1) * pwmFromPercent(abs(leftPct), PWM_MIN_STRAIGHT, PWM_MAX_STRAIGHT);
  targetRight = (rightPct == 0) ? 0 :
    (rightPct > 0 ? 1 : -1) * pwmFromPercent(abs(rightPct), PWM_MIN_STRAIGHT, PWM_MAX_STRAIGHT);

  Serial.print("V ");
  Serial.print(leftPct);
  Serial.print(',');
  Serial.print(rightPct);
  Serial.print(" -> pwm ");
  Serial.print(targetLeft);
  Serial.print(',');
  Serial.println(targetRight);
}

// Packet: <letter>[percent][T<ms>]  e.g. "W", "W70", "W45T120"
//     or: V<leftPercent>,<rightPercent>  e.g. "V45,-30" (differential drive)
void handlePacket(const char* p) {
  char c = toupper(p[0]);
  if (c == 'V') {
    handleVectorPacket(p);
    return;
  }
  if (c != 'W' && c != 'A' && c != 'S' && c != 'D' && c != 'X') return;

  int i = 1;
  int percent = 100;
  if (isDigit(p[i])) {
    percent = 0;
    while (isDigit(p[i])) {
      percent = percent * 10 + (p[i] - '0');
      if (percent > 1000) percent = 1000;
      i++;
    }
    percent = constrain(percent, 0, 100);
  }

  unsigned long pulseMs = 0;
  if (p[i] == 'T' || p[i] == 't') {
    i++;
    while (isDigit(p[i])) {
      pulseMs = pulseMs * 10 + (p[i] - '0');
      if (pulseMs > 100000) pulseMs = 100000;
      i++;
    }
    pulseMs = constrain(pulseMs, MIN_PULSE_MS, MAX_PULSE_MS);
  }

  lastCommandMillis = millis();
  Serial.print(c);
  Serial.print(' ');
  Serial.print(percent);
  if (pulseMs > 0) {
    Serial.print(" T");
    Serial.print(pulseMs);
  }
  Serial.println();

  if (c == 'X') {
    currentCommand = 'X';
    if (pulseActive) {
      startBrake();               // abort a nav pulse right now
    } else {
      setTargetsFromCommand(0);   // keyboard release: glide down via the ramp
    }
    return;
  }

  currentCommand = c;
  braking = false;
  decelStep = RAMP_STEP;
  if (pulseMs > 0) {
    startPulse(percent, pulseMs);
  } else {
    pulseActive = false;
    setTargetsFromCommand(percent);
  }
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

  releaseMotors(); // STBY is hardwired, so PWM=0 is what keeps the bot still

  analogWriteFreq(20000); // above hearing range, no motor whine

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
    char buf[16];
    int len = udp.read(buf, sizeof(buf) - 1);
    if (len > 0) {
      buf[len] = '\0';
      handlePacket(buf);
    }
  }

  // End of a timed pulse -> brake.
  if (pulseActive && millis() >= pulseEndMillis) {
    currentCommand = 'X';
    startBrake();
  }

  // Brake hold finished -> release.
  if (braking && millis() >= brakeEndMillis) {
    releaseMotors();
  }

  // Watchdog for continuous commands (pulses time themselves). Glides down
  // rather than braking: a hard stop from speed can tip the bot.
  if (!pulseActive && currentCommand != 'X' &&
      millis() - lastCommandMillis > COMMAND_TIMEOUT_MS) {
    currentCommand = 'X';
    setTargetsFromCommand(0);
    decelStep = WATCHDOG_DECEL_STEP;
  }

  updateRamp();
}
