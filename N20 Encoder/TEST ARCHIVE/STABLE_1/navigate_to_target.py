"""
Click-to-navigate for the N20 DC-motor bot, using short timed pulses.

Controls (video window):
    Left click   = add a waypoint
    Right click  = undo last waypoint
    c            = clear all waypoints
    o            = set/clear a final orientation (applied after the last waypoint)
    s            = start navigating through the waypoints
    q            = quit / abort (stops the bot)

WHY PULSES (vs. the stepper-era "stream commands until the camera says
stop"): a DC gear motor has a deadband, lags behind the camera loop, and
doesn't stop the instant power is cut. So instead the loop is:

    measure (bot stopped) -> send ONE timed pulse -> bot brakes itself
    -> wait for it to settle -> measure again -> repeat

The pulse length is timed on the ESP itself, so PC/WiFi jitter can't
stretch it, and if the marker is lost or this script dies, the bot is
already braked — nothing is left running. Pulse lengths are sized from a
learned turn-rate (deg/ms) and move-rate (px/ms), and the pulse power
bumps itself up if a pulse produces no motion (stiction), so there's
little to hand-tune.

Packet format: <letter><percent>T<ms>, e.g. "D50T120" = rotate right at
50% for 120 ms.  W forward, S backward, A rotate left, D rotate right,
X stop.

Setup:
    pip install opencv-contrib-python numpy
    Set ESP_IP below to your board's IP address.

Calibration (confirmed via testing, no offset/sign flip needed):
current_angle from calculate_angle() IS the robot's forward-facing
heading directly — 0 deg = facing/moving toward +x (rightward in the
camera frame), 180 deg = toward -x. Forward (W) moves along that heading,
D increases the angle, A decreases it (ROTATE_SIGN = 1).
"""

import math
import time
import socket

import cv2
import numpy as np

# ---------- Bot connection ----------
ESP_IP = "192.168.50.100"   # <-- set to your board's IP address
ESP_PORT = 4210

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send_raw(msg: str):
    sock.sendto(msg.encode(), (ESP_IP, ESP_PORT))


def send_stop():
    send_raw('X')


def send_pulse(cmd: str, percent: int, ms: int):
    send_raw(f"{cmd}{int(percent)}T{int(ms)}")


# ---------- ArUco setup ----------
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

cam = cv2.VideoCapture(0)  # change index if the wrong camera opens
cam.set(3, 700)
cam.set(4, 505)
cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # keep frames fresh; stale frames = wrong measurements

if not cam.isOpened():
    raise RuntimeError("Could not open camera. Try a different index (0, 1, 2...).")

# ---------- Tolerances ----------
POSITION_TOLERANCE_PX = 5      # final waypoint
WAYPOINT_TOLERANCE_PX = 10      # intermediate waypoints (no need to be precise)
ANGLE_TOLERANCE_DEG = 8
ROTATE_SIGN = 1                 # confirmed correct via testing

# ---------- Pulse control ----------
BRAKE_MS = 80                   # keep equal to BRAKE_MS in the firmware
SETTLE_MS = 200                 # extra wait for camera/bot to settle before measuring
AVG_FRAMES = 3                  # frames averaged per measurement (noise filter)
PULSE_AIM = 0.7                 # aim for 70% of the estimated pulse, finish in the next one
MIN_PULSE_MS = 40
MAX_TURN_PULSE_MS = 250
MAX_MOVE_PULSE_MS = 400

START_TURN_PERCENT = 50
START_MOVE_PERCENT = 50
PERCENT_BUMP = 15               # added when a pulse produced no motion (stiction)
MAX_PULSE_PERCENT = 90

TURN_RATE_INIT = 0.10           # deg per ms — learned after the first pulses
MOVE_RATE_INIT = 0.10           # px per ms  — learned after the first pulses
RATE_MIN, RATE_MAX = 0.02, 0.5
MIN_TURN_MOTION_DEG = 1.0       # less than this = "didn't move"
MIN_MOVE_MOTION_PX = 3.0

REVERSE_MAX_PX = 60             # target this close and behind us -> back up instead of spinning around
REVERSE_ANGLE_DEG = 150

WINDOW_NAME = "Navigation"


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def calculate_angle(corners):
    top_left, top_right, bottom_right, bottom_left = corners
    vector = top_right - top_left
    return math.degrees(math.atan2(vector[1], vector[0]))


def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def average_pose(samples):
    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    sin_sum = sum(math.sin(math.radians(s[2])) for s in samples)
    cos_sum = sum(math.cos(math.radians(s[2])) for s in samples)
    return (sum(xs) / len(xs), sum(ys) / len(ys), math.degrees(math.atan2(sin_sum, cos_sum)))


class NavApp:
    def __init__(self):
        self.waypoints = []            # list of (x, y)
        self.final_orientation = None  # degrees, or None to skip
        self.running = False
        self.current_index = 0

        self.phase = "SETTLE"          # SETTLE (wait) -> SAMPLE (average frames) -> decide -> SETTLE
        self.ready_at = 0.0
        self.samples = []
        self.pending = None            # last pulse, so we can learn from what it did
        self.last_kind = "turn"

        self.turn_rate = TURN_RATE_INIT
        self.move_rate = MOVE_RATE_INIT
        self.turn_percent = START_TURN_PERCENT
        self.move_percent = START_MOVE_PERCENT

        self.debug = ""
        self.status = ""

    # ----- UI -----
    def on_mouse(self, event, x, y, flags, param):
        if self.running:
            return  # no editing waypoints once navigation has started
        if event == cv2.EVENT_LBUTTONDOWN:
            self.waypoints.append((x, y))
        elif event == cv2.EVENT_RBUTTONDOWN:
            if self.waypoints:
                self.waypoints.pop()

    def draw_overlay(self, img, pose):
        for i, (wx, wy) in enumerate(self.waypoints):
            color = (0, 165, 255) if (self.running and i == self.current_index) else (255, 0, 0)
            cv2.circle(img, (int(wx), int(wy)), 6, color, -1)
            cv2.putText(img, str(i + 1), (int(wx) + 8, int(wy) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        for i in range(len(self.waypoints) - 1):
            cv2.line(img, self.waypoints[i], self.waypoints[i + 1], (255, 0, 0), 1)

        if not self.running:
            ori = f"{self.final_orientation:.0f} deg" if self.final_orientation is not None else "none"
            msg = (f"SETUP | {len(self.waypoints)} waypoint(s) | final orientation: {ori} | "
                   f"L-click: add  R-click: undo  c: clear  o: orientation  s: start  q: quit")
        else:
            msg = (f"RUNNING | waypoint {self.current_index + 1}/{len(self.waypoints)} | "
                   f"{self.phase}  q: abort")
        cv2.putText(img, msg, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        line2 = f"heading:{pose[2]:.0f}" if pose else "marker not visible"
        if self.running and self.debug:
            line2 = f"{self.debug}  |  {self.status}" if pose else "marker not visible"
        cv2.putText(img, line2, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    # ----- run control -----
    def start(self):
        if not self.waypoints:
            return
        self.running = True
        self.current_index = 0
        self.phase = "SETTLE"
        self.ready_at = time.time() + 0.3
        self.samples = []
        self.pending = None
        self.last_kind = "turn"
        self.turn_percent = START_TURN_PERCENT
        self.move_percent = START_MOVE_PERCENT

    def stop(self):
        self.running = False
        send_stop()

    # ----- per-frame step -----
    def step(self, pose, now):
        if self.phase == "SETTLE":
            if now >= self.ready_at:
                self.phase = "SAMPLE"
                self.samples = []
            return
        if pose is None:
            return  # marker lost: nothing is in flight (pulses brake themselves), just wait
        self.samples.append(pose)
        if len(self.samples) < AVG_FRAMES:
            return
        self.decide(average_pose(self.samples), now)

    def decide(self, pose, now):
        self.learn(pose)
        cx, cy, ang = pose

        tx, ty = self.waypoints[self.current_index]
        distance = math.hypot(tx - cx, ty - cy)
        desired = math.degrees(math.atan2(ty - cy, tx - cx))
        herr = ROTATE_SIGN * normalize_angle(desired - ang)
        is_last = self.current_index == len(self.waypoints) - 1
        tol = POSITION_TOLERANCE_PX if is_last else WAYPOINT_TOLERANCE_PX
        self.debug = f"dist:{distance:.0f} herr:{herr:.0f} heading:{ang:.0f}"

        # 1) at the waypoint?
        if distance <= tol:
            if is_last and self.final_orientation is not None:
                oerr = ROTATE_SIGN * normalize_angle(self.final_orientation - ang)
                if abs(oerr) > ANGLE_TOLERANCE_DEG:
                    self.pulse_turn(oerr, pose, now)
                    return
            self.advance()
            return

        # 2) target just behind us: back up instead of spinning around
        if distance < REVERSE_MAX_PX and abs(herr) > REVERSE_ANGLE_DEG:
            self.pulse_move('S', distance, pose, now)
            return

        # 3) not facing it: turn (looser tolerance while already driving)
        turn_tol = ANGLE_TOLERANCE_DEG if self.last_kind == "turn" else ANGLE_TOLERANCE_DEG * 2
        if abs(herr) > turn_tol:
            self.pulse_turn(herr, pose, now)
            return

        # 4) facing it: drive
        self.pulse_move('W', distance, pose, now)

    def advance(self):
        self.current_index += 1
        self.last_kind = "turn"
        if self.current_index >= len(self.waypoints):
            print("All waypoints reached.")
            self.stop()
        else:
            self.phase = "SAMPLE"   # bot is already still, measure right away
            self.samples = []

    # ----- pulses -----
    @staticmethod
    def pulse_ms(raw_ms, cap):
        return clamp(PULSE_AIM * raw_ms, MIN_PULSE_MS, cap)

    def pulse_turn(self, err, pose, now):
        cmd = 'D' if err > 0 else 'A'
        ms = self.pulse_ms(abs(err) / self.turn_rate, MAX_TURN_PULSE_MS)
        self.fire(cmd, self.turn_percent, ms, "turn", pose, now)

    def pulse_move(self, cmd, distance, pose, now):
        ms = self.pulse_ms(distance / self.move_rate, MAX_MOVE_PULSE_MS)
        self.fire(cmd, self.move_percent, ms, "move", pose, now)

    def fire(self, cmd, percent, ms, kind, pose, now):
        send_pulse(cmd, percent, ms)
        self.pending = {"kind": kind, "cmd": cmd, "ms": ms, "pose": pose}
        self.last_kind = kind
        self.status = f"{cmd} {percent}% {ms:.0f}ms"
        self.ready_at = now + (ms + BRAKE_MS + SETTLE_MS) / 1000.0
        self.phase = "SETTLE"

    def learn(self, pose):
        """Compare where the last pulse left the bot with where it started:
        update the rate estimates, or push harder if it didn't move at all."""
        p = self.pending
        self.pending = None
        if not p:
            return
        x0, y0, a0 = p["pose"]
        x1, y1, a1 = pose

        if p["kind"] == "turn":
            d = ROTATE_SIGN * normalize_angle(a1 - a0)
            moved = d if p["cmd"] == 'D' else -d
            if moved < MIN_TURN_MOTION_DEG:
                self.turn_percent = min(MAX_PULSE_PERCENT, self.turn_percent + PERCENT_BUMP)
            elif p["ms"] >= 50:
                sample = moved / p["ms"]
                self.turn_rate = clamp(0.5 * self.turn_rate + 0.5 * sample, RATE_MIN, RATE_MAX)
        else:
            moved = math.hypot(x1 - x0, y1 - y0)
            if moved < MIN_MOVE_MOTION_PX:
                self.move_percent = min(MAX_PULSE_PERCENT, self.move_percent + PERCENT_BUMP)
            elif p["ms"] >= 50:
                sample = moved / p["ms"]
                self.move_rate = clamp(0.5 * self.move_rate + 0.5 * sample, RATE_MIN, RATE_MAX)


def main():
    app = NavApp()
    cv2.namedWindow(WINDOW_NAME)
    cv2.setMouseCallback(WINDOW_NAME, app.on_mouse)

    print("Click on the video window to set waypoints.")
    print("Left click: add | Right click: undo | c: clear | o: final orientation | s: start | q: quit")

    try:
        while True:
            success, img = cam.read()
            if not success:
                print("Failed to grab frame")
                break

            corners, ids, _ = detector.detectMarkers(img)
            pose = None
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(img, corners)
                marker_corners = corners[0][0]  # tracks the first marker seen
                center = marker_corners.mean(axis=0)
                pose = (float(center[0]), float(center[1]), calculate_angle(marker_corners))

            if app.running:
                app.step(pose, time.time())

            app.draw_overlay(img, pose)
            cv2.imshow(WINDOW_NAME, img)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                app.stop()
                print("Stopped.")
                break
            elif key == ord('c') and not app.running:
                app.waypoints.clear()
            elif key == ord('o') and not app.running:
                try:
                    val = input("Final orientation in degrees (blank to clear): ").strip()
                    app.final_orientation = float(val) if val else None
                except ValueError:
                    print("Invalid input, orientation unchanged.")
            elif key == ord('s') and not app.running:
                app.start()
                print(f"Starting navigation through {len(app.waypoints)} waypoint(s).")

    finally:
        send_stop()
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
