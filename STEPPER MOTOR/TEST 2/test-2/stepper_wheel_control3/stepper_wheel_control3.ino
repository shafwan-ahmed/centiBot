/*
  Two-wheel 28BYJ-48 stepper control over WiFi (ESP32-S3 Super Mini)
  Left driver  IN1-IN4 -> GPIO1, GPIO2, GPIO3, GPIO4
  Right driver IN1-IN4 -> GPIO9, GPIO10, GPIO11, GPIO12
  Web UI: 4 buttons per wheel (90/180 fwd, 90/180 rev) + 4 "both together" buttons
*/

#include <WiFi.h>
#include <WebServer.h>
#include "esp_wifi.h"

const char* WIFI_SSID     = "ASUS_1E_NIRO_2.4G";
const char* WIFI_PASSWORD = "niro@2026";

// ---- Pins ----
const int LEFT_PINS[4]  = {1, 2, 3, 4};
const int RIGHT_PINS[4] = {9, 10, 11, 12};

// If the right wheel is mounted mirrored (common in diff-drive builds),
// set this to true so "forward" on both wheels actually drives the robot straight.
const bool INVERT_RIGHT = false;

// ---- Stepper timing ----
const int STEPS_PER_REV = 4096;      // 28BYJ-48, half-step mode
const int STEP_DELAY_US = 1200;      // delay between half-steps; raise if motor stalls/skips

// Half-step sequence (8 steps)
const int HALF_STEP_SEQ[8][4] = {
  {1,0,0,0},
  {1,1,0,0},
  {0,1,0,0},
  {0,1,1,0},
  {0,0,1,0},
  {0,0,1,1},
  {0,0,0,1},
  {1,0,0,1}
};

WebServer server(80);

void setupMotorPins(const int pins[4]) {
  for (int i = 0; i < 4; i++) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
  }
}

void writeStep(const int pins[4], int stepIndex) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(pins[i], HALF_STEP_SEQ[stepIndex][i]);
  }
}

void releaseCoils(const int pins[4]) {
  for (int i = 0; i < 4; i++) digitalWrite(pins[i], LOW);
}

// Move a single wheel. forward=true increases the step index direction.
void moveWheel(const int pins[4], long steps, bool forward) {
  static int stepIndex = 0; // not persisted per-wheel on purpose; each call is a discrete move
  int idx = 0;
  for (long s = 0; s < steps; s++) {
    idx = forward ? (idx + 1) % 8 : (idx + 7) % 8;
    writeStep(pins, idx);
    delayMicroseconds(STEP_DELAY_US);
  }
  releaseCoils(pins);
}

// Move both wheels together, one half-step at a time, so they stay in sync.
void moveBoth(long steps, bool forward) {
  bool rightForward = INVERT_RIGHT ? !forward : forward;
  int idxL = 0, idxR = 0;
  for (long s = 0; s < steps; s++) {
    idxL = forward ? (idxL + 1) % 8 : (idxL + 7) % 8;
    idxR = rightForward ? (idxR + 1) % 8 : (idxR + 7) % 8;
    writeStep(LEFT_PINS, idxL);
    writeStep(RIGHT_PINS, idxR);
    delayMicroseconds(STEP_DELAY_US);
  }
  releaseCoils(LEFT_PINS);
  releaseCoils(RIGHT_PINS);
}

long stepsForDegrees(int deg) {
  return (long)STEPS_PER_REV * deg / 360;
}

// ---- Web UI ----
const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Wheel Control</title>
<style>
body{font-family:sans-serif;text-align:center;background:#111;color:#eee;margin:0;padding:16px}
h2{margin:18px 0 8px}
.row{display:flex;justify-content:center;gap:10px;flex-wrap:wrap;margin-bottom:14px}
button{font-size:16px;padding:14px 18px;border:none;border-radius:8px;background:#2b6cff;color:#fff;min-width:90px}
button:active{background:#1a4fcc}
.rev{background:#ff5a3c}
.rev:active{background:#cc4630}
.both{background:#20b358}
.both:active{background:#178a43}
</style></head><body>
<h2>Left Wheel</h2>
<div class="row">
<button onclick="go('left/fwd/90')">90&deg; Fwd</button>
<button onclick="go('left/fwd/180')">180&deg; Fwd</button>
<button class="rev" onclick="go('left/rev/90')">90&deg; Rev</button>
<button class="rev" onclick="go('left/rev/180')">180&deg; Rev</button>
</div>
<h2>Right Wheel</h2>
<div class="row">
<button onclick="go('right/fwd/90')">90&deg; Fwd</button>
<button onclick="go('right/fwd/180')">180&deg; Fwd</button>
<button class="rev" onclick="go('right/rev/90')">90&deg; Rev</button>
<button class="rev" onclick="go('right/rev/180')">180&deg; Rev</button>
</div>
<h2>Both Together</h2>
<div class="row">
<button class="both" onclick="go('both/fwd/90')">90&deg; Fwd</button>
<button class="both" onclick="go('both/fwd/180')">180&deg; Fwd</button>
<button class="both" onclick="go('both/rev/90')">90&deg; Rev</button>
<button class="both" onclick="go('both/rev/180')">180&deg; Rev</button>
</div>
<p id="status">Ready</p>
<script>
function go(path){
  document.getElementById('status').innerText = 'Moving...';
  fetch('/' + path).then(r=>r.text()).then(t=>{
    document.getElementById('status').innerText = t;
  }).catch(()=>{ document.getElementById('status').innerText = 'Error'; });
}
</script></body></html>
)HTML";

void handleRoot() {
  server.send_P(200, "text/html", PAGE);
}

// path like /left/fwd/90
void handleMove() {
  String uri = server.uri(); // e.g. /left/fwd/90
  int p1 = uri.indexOf('/', 1);
  int p2 = uri.indexOf('/', p1 + 1);
  String who = uri.substring(1, p1);         // left | right | both
  String dir = uri.substring(p1 + 1, p2);    // fwd | rev
  int deg = uri.substring(p2 + 1).toInt();   // 90 | 180

  bool forward = (dir == "fwd");
  long steps = stepsForDegrees(deg);

  if (who == "left") {
    moveWheel(LEFT_PINS, steps, forward);
  } else if (who == "right") {
    moveWheel(RIGHT_PINS, steps, forward);
  } else if (who == "both") {
    moveBoth(steps, forward);
  } else {
    server.send(400, "text/plain", "Unknown wheel");
    return;
  }

  server.send(200, "text/plain", who + " " + dir + " " + String(deg) + " done");
}

void setup() {
  Serial.begin(115200);
  setupMotorPins(LEFT_PINS);
  setupMotorPins(RIGHT_PINS);

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(200);
  esp_wifi_set_ps(WIFI_PS_NONE); // disable WiFi power-save, helps some S3 boards handshake reliably
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.printf(" [status=%d]", WiFi.status());
    attempts++;
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.printf("WiFi FAILED to connect. Final status code: %d\n", WiFi.status());
    Serial.println("0=IDLE 1=NO_SSID_AVAIL 3=CONNECTED 4=CONNECT_FAILED 6=DISCONNECTED");
  }

  server.on("/", handleRoot);
  server.on("/left/fwd/90", handleMove);
  server.on("/left/fwd/180", handleMove);
  server.on("/left/rev/90", handleMove);
  server.on("/left/rev/180", handleMove);
  server.on("/right/fwd/90", handleMove);
  server.on("/right/fwd/180", handleMove);
  server.on("/right/rev/90", handleMove);
  server.on("/right/rev/180", handleMove);
  server.on("/both/fwd/90", handleMove);
  server.on("/both/fwd/180", handleMove);
  server.on("/both/rev/90", handleMove);
  server.on("/both/rev/180", handleMove);
  server.begin();
}

void loop() {
  server.handleClient();
}
