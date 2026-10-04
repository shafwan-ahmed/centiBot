"""
Click-to-navigate for the N20 DC-motor bot, using continuous
differential-drive streaming (not stop-and-sample pulses).

Controls (video window):
    Left click   = add a waypoint
    Right click  = undo last waypoint
    c            = clear all waypoints
    o            = set/clear a final orientation (applied after the last waypoint)
    s            = start navigating through the waypoints
    q            = quit / abort (stops the bot)

WHY NOT PULSES: the sample-then-pulse approach (measure with the bot
stopped, send one timed pulse, wait, measure again) worked for the slow
stepper build, but an N20 gear motor is fast enough that the constant
stop-move-stop made the run choppy and slow overall. This version goes
back to a continuous per-frame loop like the stepper build used, but
drives BOTH wheels independently every frame (differential drive)
instead of only ever going straight or pivoting in place.

MOVEMENT MODE (decided every frame, with hysteresis so it doesn't flap):
    ROTATE — heading error > ROTATE_ENTER_DEG: pivot in place (wheels
             opposite directions) until the error drops below
             ROTATE_EXIT_DEG, then switch to DRIVE.
    DRIVE  — heading error is small: drive forward with a differential
             offset between the wheels (one side faster than the other)
             proportional to the heading error, so it curves toward the
             target instead of needing to stop and re-pivot for small
             corrections.
Command sent every frame is "V<leftPercent>,<rightPercent>" (see the
firmware's handleVectorPacket) — independent per-wheel speed, streamed
continuously like the old W/A/S/D keyboard commands (same watchdog:
resend or it stops).

LOOKAHEAD (your "look at the point after next" idea): within
LOOKAHEAD_RADIUS_PX of the current waypoint, the steering target blends
toward the NEXT waypoint after that, so the bot starts curving into the
next leg before it's fully reached the corner, instead of sharply
re-aiming exactly at the waypoint. Arrival is still checked against the
actual waypoint, not the blended aim point, so it doesn't skip waypoints.

Setup:
    pip install opencv-contrib-python numpy
    Set ESP_IP below to your board's IP address.

Calibration (confirmed via testing, no offset/sign flip needed):
current_angle from calculate_angle() IS the robot's forward-facing
heading directly — 0 deg = facing/moving toward +x (rightward in the
camera frame), 180 deg = toward -x. A positive heading error means
"target is to the right" — D (right wheel forward more / speeds up
left) turns that way; ROTATE_SIGN = 1 confirmed correct.
"""

import math
import time
import socket

import cv2
import numpy as np

# ---------- Bot connection ----------
ESP_IP = "192.168.50.100"   # <-- set to your board's IP address
ESP_PORT = 4210
SEND_INTERVAL_S = 0.05      # matches the firmware's watchdog cadence

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send_raw(msg: str):
    sock.sendto(msg.encode(), (ESP_IP, ESP_PORT))


def send_stop():
    send_raw('X')


def send_vector(left_percent: float, right_percent: float):
    l = int(clamp(left_percent, -100, 100))
    r = int(clamp(right_percent, -100, 100))
    msg = f"V{l},{r}"
    print(f"-> {msg}  (raw left={left_percent:.1f} right={right_percent:.1f})")
    send_raw(msg)


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
POSITION_TOLERANCE_PX = 5       # final waypoint
WAYPOINT_TOLERANCE_PX = 10      # intermediate waypoints
ANGLE_TOLERANCE_DEG = 30         # final-orientation alignment tolerance
ROTATE_SIGN = 1                 # confirmed correct via testing

# ---------- Mode switching (hysteresis avoids flapping between modes) ----------
ROTATE_ENTER_DEG = 70           # heading error beyond this -> pivot in place
ROTATE_EXIT_DEG = 40            # heading error below this -> back to driving

# ---------- Rotate-in-place speed (scales with how far off we are) ----------
# The firmware's PWM_MIN_STRAIGHT/PWM_MAX_STRAIGHT window is narrow (your
# tuning), so low percentages here can land below the motors' stall
# torque even though 100% (confirmed via plain W/A/S/D) works. Keeping
# these close to 100 avoids that; if the bot still stalls, raise the
# minimums further before touching anything else.
ROTATE_MIN_PERCENT = 40
ROTATE_MAX_PERCENT = 70

# ---------- Drive speed (scales down approaching the target) ----------
BASE_MOVE_MIN_PERCENT = 70
BASE_MOVE_MAX_PERCENT = 100
SLOWDOWN_DISTANCE_PX = 160

# ---------- Differential steering while driving ----------
TURN_GAIN = 1.0                 # percent of wheel-speed offset per degree of heading error

# ---------- Lookahead / corner cutting ----------
LOOKAHEAD_RADIUS_PX = 60

WINDOW_NAME = "Navigation"


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def scaled_percent(error_magnitude, slowdown_threshold, min_percent, max_percent):
    error_magnitude = abs(error_magnitude)
    if error_magnitude >= slowdown_threshold:
        return max_percent
    fraction = error_magnitude / slowdown_threshold
    return min_percent + (max_percent - min_percent) * fraction


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


class NavApp:
    def __init__(self):
        self.waypoints = []            # list of (x, y)
        self.final_orientation = None  # degrees, or None to skip
        self.running = False
        self.current_index = 0
        self.mode = "ROTATE"           # ROTATE or DRIVE, with hysteresis between them
        self.aligning_final = False    # doing the final-orientation spin after the last waypoint
        self.debug = ""

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
                   f"{self.mode}  q: abort")
        cv2.putText(img, msg, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        line2 = self.debug if (self.running and pose) else ("marker not visible" if self.running else "")
        if line2:
            cv2.putText(img, line2, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    # ----- run control -----
    def start(self):
        if not self.waypoints:
            return
        self.running = True
        self.current_index = 0
        self.mode = "ROTATE"
        self.aligning_final = False

    def stop(self):
        self.running = False
        send_stop()

    # ----- per-frame step -----
    def step(self, pose):
        if pose is None:
            send_stop()  # marker lost: stop rather than drive blind
            return
        cx, cy, ang = pose

        if self.aligning_final:
            self.drive_toward_heading(self.final_orientation_error(ang))
            return

        tx, ty = self.waypoints[self.current_index]
        distance = math.hypot(tx - cx, ty - cy)
        is_last = self.current_index == len(self.waypoints) - 1
        tol = POSITION_TOLERANCE_PX if is_last else WAYPOINT_TOLERANCE_PX

        if distance <= tol:
            self.advance(ang)
            return

        aim_x, aim_y = self.lookahead_aim(cx, cy, tx, ty, distance)
        desired = math.degrees(math.atan2(aim_y - cy, aim_x - cx))
        herr = ROTATE_SIGN * normalize_angle(desired - ang)
        self.debug = f"dist:{distance:.0f} herr:{herr:.0f} heading:{ang:.0f} mode:{self.mode}"

        self.update_mode(herr)
        if self.mode == "ROTATE":
            self.send_rotate(herr)
        else:
            self.send_drive(herr, distance)

    def final_orientation_error(self, ang):
        return ROTATE_SIGN * normalize_angle(self.final_orientation - ang)

    def drive_toward_heading(self, herr):
        # Used only for the final-orientation spin: always rotates in place.
        self.debug = f"orientation err:{herr:.0f}"
        if abs(herr) <= ANGLE_TOLERANCE_DEG:
            print("Target reached.")
            self.stop()
            return
        self.send_rotate(herr)

    def lookahead_aim(self, cx, cy, tx, ty, distance):
        """Within LOOKAHEAD_RADIUS_PX of the current waypoint, blend the
        steering target toward the *next* waypoint so the bot starts
        curving into the next leg early instead of sharply re-aiming
        right at the corner. Arrival is still judged on the real
        waypoint (tx, ty), not this blended point."""
        has_next = self.current_index < len(self.waypoints) - 1
        if not has_next or distance >= LOOKAHEAD_RADIUS_PX:
            return tx, ty
        nx, ny = self.waypoints[self.current_index + 1]
        blend = 1.0 - (distance / LOOKAHEAD_RADIUS_PX)  # 0 at radius edge -> 1 at the waypoint
        return tx + blend * (nx - tx), ty + blend * (ny - ty)

    def update_mode(self, herr):
        if self.mode == "DRIVE" and abs(herr) > ROTATE_ENTER_DEG:
            self.mode = "ROTATE"
        elif self.mode == "ROTATE" and abs(herr) < ROTATE_EXIT_DEG:
            self.mode = "DRIVE"

    def send_rotate(self, herr):
        speed = scaled_percent(herr, ROTATE_ENTER_DEG, ROTATE_MIN_PERCENT, ROTATE_MAX_PERCENT)
        if herr > 0:  # target to the right -> left wheel forward, right wheel back
            send_vector(speed, -speed)
        else:
            send_vector(-speed, speed)

    def send_drive(self, herr, distance):
        base = scaled_percent(distance, SLOWDOWN_DISTANCE_PX, BASE_MOVE_MIN_PERCENT, BASE_MOVE_MAX_PERCENT)
        offset = clamp(TURN_GAIN * herr, -base, base)  # capped so a wheel never has to reverse to steer
        send_vector(base + offset, base - offset)

    def advance(self, ang):
        self.current_index += 1
        self.mode = "ROTATE"
        if self.current_index >= len(self.waypoints):
            if self.final_orientation is not None:
                self.aligning_final = True
            else:
                print("Target reached.")
                self.stop()


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
                app.step(pose)

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

            time.sleep(SEND_INTERVAL_S)

    finally:
        send_stop()
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
