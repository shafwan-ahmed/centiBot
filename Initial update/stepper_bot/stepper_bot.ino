/*
  Dual 28BYJ-48 Stepper Bot — ESP8266 (NodeMCU) WASD Remote Control (UDP)
  ------------------------------------------------------------
  A Python script on your PC sends single-character UDP commands
  (W/A/S/D/X) while you hold keys down; the ESP8266 drives the
  motors continuously and non-blocking, with a watchdog that
  auto-stops if no command arrives for a bit (WiFi drop, script
  closed, etc).

  PIN ASSIGNMENTS (NodeMCU silkscreen labels -> GPIO):
    Left motor  ULN2003 IN1-IN4 -> D1, D2, D5, D6  (GPIO5, GPIO4, GPIO14, GPIO12)
    Right motor ULN2003 IN1-IN4 -> D7, D0, D8, D4  (GPIO13, GPIO16, GPIO15, GPIO2)

  BOOT-PIN NOTES (only matters at power-on/reset/flash, not while running):
    D0 (GPIO16) has no interrupt/PWM support but works fine as a plain output.
    D4 (GPIO2)  is pulled HIGH at boot and shares the onboard LED — it'll
                blink briefly on reset, harmless for a stepper coil.
    D8 (GPIO15) is pulled LOW at boot — avoid anything that forces it HIGH
                externally during power-on/flashing.
  If your board uses different silkscreen labels, match GPIO numbers instead.

  COMMANDS (single ASCII char per UDP packet):
    W = both motors forward
    S = both motors backward
    A = pivot left  (left motor backward, right motor forward)
    D = pivot right (left motor forward, right motor backward)
    X = stop (also sent automatically if no command arrives)

  If a direction is mirrored on your build (e.g. W goes backward,
  or A/D are swapped), flip the +1/-1 in the switch statement in
  applyCommand() below rather than rewiring anything.
*/

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// ---------- WiFi credentials ----------
const char* WIFI_SSID = "ASUS_1E_NIRO_2.4G";
const char* WIFI_PASS = "niro@2026";

// ---------- UDP ----------
const unsigned int UDP_PORT = 4210;
WiFiUDP udp;

// ---------- Motor pin assignments (NodeMCU GPIO numbers) ----------
//const int leftPins[4]  = {5, 4, 14, 12};  // D1, D2, D5, D6 -> IN1, IN2, IN3, IN4
//const int rightPins[4] = {13, 16, 15, 2}; // D7, D0, D8, D4 -> IN1, IN2, IN3, IN4

const int leftPins[4]  = {16, 5, 4, 2};  // D0, D1, D2, D4 -> IN1, IN2, IN3, IN4
const int rightPins[4] = {14, 12, 13, 15}; // D5, D6, D7, D8 -> IN1, IN2, IN3, IN4

// ---------- Stepper timing ----------
const unsigned long STEP_DELAY_US = 1000;     // delay between half-steps (tune for speed/torque)
const unsigned long COMMAND_TIMEOUT_MS = 300; // auto-stop if no command received in this long

const int halfStepSeq[8][4] = {
  {1,0,0,0},
  {1,1,0,0},
  {0,1,0,0},
  {0,1,1,0},
  {0,0,1,0},
  {0,0,1,1},
  {0,0,0,1},
  {1,0,0,1}
};

int leftSeqIndex = 0;
int rightSeqIndex = 0;

char currentCommand = 'X';
unsigned long lastCommandMillis = 0;
unsigned long lastStepMicros = 0;
bool coilsOff = true;

void setPins(const int pins[4], const int state[4]) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(pins[i], state[i]);
  }
}

void stepOnce(const int pins[4], int &seqIndex, int dir) {
  seqIndex = (seqIndex + dir + 8) % 8;
  setPins(pins, halfStepSeq[seqIndex]);
}

void motorsOff() {
  const int off[4] = {0,0,0,0};
  setPins(leftPins, off);
  setPins(rightPins, off);
  coilsOff = true;
}

// One non-blocking tick: advance motors by one half-step according
// to currentCommand, called at most once every STEP_DELAY_US.
void applyCommand() {
  switch (currentCommand) {
    case 'W':
      stepOnce(leftPins,  leftSeqIndex,  +1);
      stepOnce(rightPins, rightSeqIndex, +1);
      coilsOff = false;
      break;
    case 'S':
      stepOnce(leftPins,  leftSeqIndex,  -1);
      stepOnce(rightPins, rightSeqIndex, -1);
      coilsOff = false;
      break;
    case 'A':
      stepOnce(leftPins,  leftSeqIndex,  -1);
      stepOnce(rightPins, rightSeqIndex, +1);
      coilsOff = false;
      break;
    case 'D':
      stepOnce(leftPins,  leftSeqIndex,  +1);
      stepOnce(rightPins, rightSeqIndex, -1);
      coilsOff = false;
      break;
    case 'X':
    default:
      if (!coilsOff) motorsOff();
      break;
  }
}

void setup() {
  Serial.begin(115200);

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
  
  for (int i = 0; i < 4; i++) {
    pinMode(leftPins[i], OUTPUT);
    pinMode(rightPins[i], OUTPUT);
  }
  motorsOff();
  lastCommandMillis = millis();
}

void loop() {
  // Non-blocking check for an incoming UDP packet
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
        Serial.println(c);
      }
    }
  }

  // Watchdog: stop if no command received recently
  if (millis() - lastCommandMillis > COMMAND_TIMEOUT_MS) {
    currentCommand = 'X';
  }

  // Non-blocking stepping tick
  if (micros() - lastStepMicros >= STEP_DELAY_US) {
    lastStepMicros = micros();
    applyCommand();
  }
}
