/*
  Dual 28BYJ-48 Stepper Bot — ESP32-C3 Web Dashboard Control
  ------------------------------------------------------------
  Two 28BYJ-48 5V steppers, each via a ULN2003 driver board.
  Web dashboard: 4 buttons per motor (180, 90, -90, -180 deg)
  plus 4 buttons to move both motors together by the same amount.

  PIN ASSIGNMENTS:
    Left motor  ULN2003 IN1-IN4 -> GPIO 7, 8, 9, 20
    Right motor ULN2003 IN1-IN4 -> GPIO 0, 1, 2, 3

  NOTE: GPIO9 is the BOOT/strapping pin on most ESP32-C3 boards.
  It's pulled up internally and only sampled at reset, so driving
  it as a plain output after boot is normally fine — just don't
  short it low externally while resetting/flashing the board.

  NOTE: motor moves are executed synchronously (blocking) while
  handling a request. For a simple single-user dashboard like this
  that's fine — the page just waits until the move finishes before
  the button re-enables.
*/

#include <WiFi.h>
#include <WebServer.h>

// ---------- WiFi credentials ----------
const char* WIFI_SSID = "ASUS_1E_NIRO_2.4G";
const char* WIFI_PASS = "niro@2026";

// ---------- Motor pin assignments ----------
const int leftPins[4]  = {7, 8, 9, 20};  // IN1, IN2, IN3, IN4
const int rightPins[4] = {0, 1, 2, 3};   // IN1, IN2, IN3, IN4

// ---------- Stepper constants ----------
// 28BYJ-48 half-step sequence (8 steps), ~4096 steps per full
// output-shaft revolution (approx, gear ratio ~64:1).
const int STEPS_PER_REV = 4096;
const int STEP_DELAY_US = 1500; // delay between half-steps (tune for speed/torque)

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

WebServer server(80);

// per-motor sequence position, so consecutive moves keep phase
int leftSeqIndex = 0;
int rightSeqIndex = 0;

void setPins(const int pins[4], const int state[4]) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(pins[i], state[i]);
  }
}

// Advance one motor by one half-step. dir = +1 or -1.
void stepOnce(const int pins[4], int &seqIndex, int dir) {
  seqIndex = (seqIndex + dir + 8) % 8;
  setPins(pins, halfStepSeq[seqIndex]);
}

// Move left and/or right motor by a number of half-steps (signed).
// Pass stepsLeft = 0 to skip the left motor, stepsRight = 0 to skip the right.
void moveSteps(long stepsLeft, long stepsRight) {
  long remainingLeft  = abs(stepsLeft);
  long remainingRight = abs(stepsRight);
  int dirLeft  = (stepsLeft  >= 0) ? 1 : -1;
  int dirRight = (stepsRight >= 0) ? 1 : -1;

  while (remainingLeft > 0 || remainingRight > 0) {
    if (remainingLeft > 0) {
      stepOnce(leftPins, leftSeqIndex, dirLeft);
      remainingLeft--;
    }
    if (remainingRight > 0) {
      stepOnce(rightPins, rightSeqIndex, dirRight);
      remainingRight--;
    }
    delayMicroseconds(STEP_DELAY_US);
  }

  // de-energize coils when done (avoid overheating/holding current)
  const int off[4] = {0,0,0,0};
  if (stepsLeft  != 0) setPins(leftPins, off);
  if (stepsRight != 0) setPins(rightPins, off);
}

long degToSteps(float deg) {
  return lround((STEPS_PER_REV * deg) / 360.0);
}

// ---------- Web dashboard ----------
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Stepper Bot Control</title>
<style>
  :root{
    --bg:#0f1115; --card:#171a21; --text:#e8eaed; --muted:#9aa0a6;
    --accent:#4f8cff; --accent2:#2e7d32;
  }
  *{box-sizing:border-box;}
  body{
    margin:0; padding:20px; background:var(--bg); color:var(--text);
    font-family:-apple-system,Segoe UI,Roboto,sans-serif;
  }
  h1{font-size:1.3rem; margin:0 0 4px;}
  p.sub{color:var(--muted); margin:0 0 20px; font-size:0.85rem;}
  .card{
    background:var(--card); border-radius:12px; padding:16px;
    margin-bottom:16px;
  }
  .card h2{margin:0 0 12px; font-size:1rem; color:var(--muted); font-weight:600;}
  .grid{
    display:grid; grid-template-columns:repeat(4,1fr); gap:8px;
  }
  button{
    background:#232732; color:var(--text); border:1px solid #333947;
    border-radius:8px; padding:14px 4px; font-size:0.95rem; font-weight:600;
    cursor:pointer;
  }
  button:active{background:var(--accent); border-color:var(--accent);}
  .both button:active{background:var(--accent2); border-color:var(--accent2);}
  #status{margin-top:14px; color:var(--muted); font-size:0.85rem; min-height:1.2em;}
</style>
</head>
<body>
  <h1>Stepper Bot Control</h1>
  <p class="sub">28BYJ-48 dual stepper dashboard</p>

  <div class="card">
    <h2>Left Motor</h2>
    <div class="grid">
      <button onclick="move('left',180)">180</button>
      <button onclick="move('left',90)">90</button>
      <button onclick="move('left',-90)">-90</button>
      <button onclick="move('left',-180)">-180</button>
    </div>
  </div>

  <div class="card">
    <h2>Right Motor</h2>
    <div class="grid">
      <button onclick="move('right',180)">180</button>
      <button onclick="move('right',90)">90</button>
      <button onclick="move('right',-90)">-90</button>
      <button onclick="move('right',-180)">-180</button>
    </div>
  </div>

  <div class="card both">
    <h2>Both Motors</h2>
    <div class="grid">
      <button onclick="move('both',180)">180</button>
      <button onclick="move('both',90)">90</button>
      <button onclick="move('both',-90)">-90</button>
      <button onclick="move('both',-180)">-180</button>
    </div>
  </div>

  <div id="status"></div>

<script>
async function move(motor, deg) {
  const status = document.getElementById('status');
  status.textContent = motor + ' ' + deg + ' deg...';
  const buttons = document.querySelectorAll('button');
  buttons.forEach(b => b.disabled = true);
  try {
    const res = await fetch(`/move?motor=${motor}&deg=${deg}`);
    const text = await res.text();
    status.textContent = text;
  } catch (e) {
    status.textContent = 'Error: ' + e;
  }
  buttons.forEach(b => b.disabled = false);
}
</script>
</body>
</html>
)HTML";

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleMove() {
  if (!server.hasArg("motor") || !server.hasArg("deg")) {
    server.send(400, "text/plain", "missing motor or deg");
    return;
  }
  String motor = server.arg("motor");
  float deg = server.arg("deg").toFloat();
  long steps = degToSteps(deg);

  if (motor == "left") {
    moveSteps(steps, 0);
  } else if (motor == "right") {
    moveSteps(0, steps);
  } else if (motor == "both") {
    moveSteps(steps, steps);
  } else {
    server.send(400, "text/plain", "unknown motor");
    return;
  }

  server.send(200, "text/plain", "OK: " + motor + " moved " + String(deg) + " deg");
}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    pinMode(leftPins[i], OUTPUT);
    pinMode(rightPins[i], OUTPUT);
  }

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

  server.on("/", handleRoot);
  server.on("/move", handleMove);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}
