/*
  Dual N20 Encoder-Motor Bot — ESP8266 (NodeMCU) + TB6612FNG, WASD UDP
  ---------------------------------------------------------------------
  Same UDP protocol as the stepper version: PC sends W/A/S/D/X packets
  while keys are held; ESP8266 drives continuously, auto-stops if no
  packet arrives within COMMAND_TIMEOUT_MS. A command may optionally
  carry a 2-digit speed percentage right after the letter (e.g. "W70"
  = forward at 70% of that direction's max speed); no digits = 100%,
  so plain single-letter senders (wasd_control.py) work unchanged.

  PIN ASSIGNMENTS (NodeMCU silkscreen -> GPIO). D3 is skipped, so this
  runs as two adjacent groups (D1-D2, D4-D8) with a one-pin gap:
    PWMB -> D1 (GPIO5)   — right motor speed (PWM-capable, no boot quirks)
    BIN2 -> D2 (GPIO4)   — no boot quirks
    [D3 skipped]
    BIN1 -> D4 (GPIO2)   — boot pin, pulled HIGH, shares onboard LED;
                            direction-only, harmless
    STBY -> D5 (GPIO14)  — no defined pull, floats briefly at boot;
                            harmless since direction/PWM lines aren't
                            driven yet either at that point
    AIN1 -> D6 (GPIO12)  — left motor direction
    AIN2 -> D7 (GPIO13)  — left motor direction
    PWMA -> D8 (GPIO15)  — left motor speed (PWM-capable, pulled LOW at
                            boot = 0% duty anyway, ideal)

    Free: D0 (GPIO16) — no PWM or interrupt support, least useful pin;
    fine only for a simple digital read/output if ever needed.

  POWER:
    TB6612 VM   -> motor battery, matched to your N20's rated voltage
    TB6612 VCC  -> NodeMCU 3V3 (logic only)
    TB6612 GND  -> common with battery GND and NodeMCU GND
    AO1/AO2     -> left N20  M1/M2
    BO1/BO2     -> right N20 M1/M2

  COMMANDS:
    W = both motors forward
    S = both motors backward
    A = pivot left  (left motor backward, right motor forward)
    D = pivot right (left motor forward, right motor backward)
    X = stop (also sent automatically on timeout)

  If a direction is mirrored on your build, swap the two digitalWrite
  values inside setMotor() for that side, or flip the sign in
  setTargetsFromCommand() — no rewiring needed.

  SPEED: W/S ramp smoothly up to SPEED_STRAIGHT; A/D ramp up to the
  slower SPEED_TURN. The ramp (RAMP_STEP / RAMP_INTERVAL_MS) applies to
  every transition — including reversals, which just pass through 0 —
  except the watchdog timeout, which cuts power instantly for safety.

  Encoders (C1/C2 on each N20) are NOT read yet — this sketch is
  WASD-only, matching where the stepper version left off. Only 2 GPIOs
  are free (D6, D7), enough for one pulse channel per motor but not
  full quadrature; when you get to navigate-to-target, either accept
  single-channel pulse counting (direction inferred from the commanded
  direction) or add an I2C GPIO expander (e.g. PCF8574) for full
  quadrature on both motors.
*/

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// ---------- WiFi credentials ----------
const char* WIFI_SSID = "ASUS_1E_NIRO_2.4G";
const char* WIFI_PASS = "niro@2026";

// ---------- UDP ----------
const unsigned int UDP_PORT = 4210;
WiFiUDP udp;

// ---------- TB6612FNG pins (GPIO numbers), D1-D2 + D4-D8 (D3 skipped) ----------
const int PIN_PWMB = 5;  // D1  (right motor speed)
const int PIN_BIN2 = 4;  // D2
const int PIN_BIN1 = 2;  // D4
const int PIN_STBY = 14; // D5
const int PIN_AIN1 = 12; // D6  (left motor)
const int PIN_AIN2 = 13; // D7
const int PIN_PWMA = 15; // D8  (left motor speed)

// ---------- Tuning ----------
const int SPEED_STRAIGHT = 500;               // W/S target PWM duty, 0-1023 (was 900 — tamed further)
const int SPEED_TURN = 280;                   // A/D target PWM duty — slower than straight (was 500)
const int RAMP_STEP = 15;                     // PWM units added/removed per ramp tick (was 30 — gentler)
const unsigned long RAMP_INTERVAL_MS = 25;    // how often the ramp advances (500/15 * 25ms = ~830ms 0->full)
const unsigned long COMMAND_TIMEOUT_MS = 300; // auto-stop if no command in this long

char currentCommand = 'X';
unsigned long lastCommandMillis = 0;
unsigned long lastRampMillis = 0;

int targetLeft = 0, targetRight = 0;   // where each side is heading, signed PWM
int currentLeft = 0, currentRight = 0; // what's actually being applied right now

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
// acceleration, deceleration, and reversal alike (a reversal just
// passes through 0 on the way), so no special-casing is needed.
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

// Sets the ramp targets for the current command, scaled by percent
// (0-100) of that direction's max speed. The actual PWM values are
// approached gradually by updateRamp(), not applied instantly here.
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

// Advances the ramp toward the current targets and applies it to the
// motors. Called on a fixed tick from loop().
void updateRamp() {
  if (millis() - lastRampMillis < RAMP_INTERVAL_MS) return;
  lastRampMillis = millis();

  currentLeft = stepToward(currentLeft, targetLeft, RAMP_STEP);
  currentRight = stepToward(currentRight, targetRight, RAMP_STEP);

  setMotor(PIN_AIN1, PIN_AIN2, PIN_PWMA, currentLeft);
  setMotor(PIN_BIN1, PIN_BIN2, PIN_PWMB, currentRight);
}

// Immediate hard stop, bypassing the ramp — used for the safety
// timeout, where "gently coast to a stop" is the wrong call.
void stopNow() {
  targetLeft = 0;
  targetRight = 0;
  currentLeft = 0;
  currentRight = 0;
  setMotor(PIN_AIN1, PIN_AIN2, PIN_PWMA, 0);
  setMotor(PIN_BIN1, PIN_BIN2, PIN_PWMB, 0);
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_STBY, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT);

  digitalWrite(PIN_STBY, LOW); // keep driver disabled until WiFi is up
  stopNow();                   // motors off

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

  digitalWrite(PIN_STBY, HIGH); // enable driver now that we're ready
  lastCommandMillis = millis();
  lastRampMillis = millis();
}

void loop() {
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
        setTargetsFromCommand(percent); // ramp eases toward this, doesn't jump
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
