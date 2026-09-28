"""
Navigate the bot to a target (x, y, orientation) using ArUco tracking.

Enter a target pixel coordinate + orientation in the terminal; the
script watches the ArUco marker via camera, computes the needed
turn/drive, and sends speed-scaled W/A/S/D/X UDP commands to the bot
until it arrives.

Command meaning (matches the current N20/TB6612 firmware directly —
no reversal):
    W = forward
    S = backward   (not used by this script's state machine, but valid)
    A = rotate left
    D = rotate right
    X = stop

Each command (except X) carries a 2-digit speed percentage, e.g. "W70"
= forward at 70%. Speed scales down as distance/angle error shrinks —
plenty of speed while far from the target, a crawl right before it
needs to stop, so the bot doesn't sail past the tolerance window.

Setup:
    pip install opencv-contrib-python numpy
    Set ESP_IP below to your board's IP address.

Calibration (confirmed via testing, no offset/sign flip needed):
current_angle from calculate_angle() IS the robot's forward-facing
heading directly — 0 deg = facing/moving toward +x (rightward in the
camera frame), 180 deg = facing/moving toward -x (leftward). Forward
(W) moves it toward +x, backward (S) toward -x, matching that heading
exactly. Rotation direction is confirmed correct too (ROTATE_SIGN = 1).
The live heading is shown in the on-screen overlay so you can visually
re-confirm this holds if you ever remount the marker.
"""

import math
import time
import socket

import cv2
import numpy as np

# ---------- Bot connection ----------
ESP_IP = "192.168.50.100"   # <-- set to your board's IP address
ESP_PORT = 4210
SEND_INTERVAL_S = 0.05      # matches the board's 300ms watchdog

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send_cmd(cmd: str, percent: int = 100):
    """Send a command, optionally with a 0-100 speed percentage."""
    if cmd == 'X':
        sock.sendto(b'X', (ESP_IP, ESP_PORT))
    else:
        percent = max(0, min(100, int(percent)))
        sock.sendto(f"{cmd}{percent:02d}".encode(), (ESP_IP, ESP_PORT))


# ---------- ArUco setup ----------
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

cam = cv2.VideoCapture(0)  # change index if the wrong camera opens
cam.set(3, 700)
cam.set(4, 505)

if not cam.isOpened():
    raise RuntimeError("Could not open camera. Try a different index (0, 1, 2...).")

# ---------- Navigation tuning ----------
POSITION_TOLERANCE_PX = 15
ANGLE_TOLERANCE_DEG = 8
ROTATE_SIGN = 1  # confirmed correct via testing — rotation direction matches, no flip needed

# Speed scaling: full speed once error exceeds the "slowdown" threshold,
# tapering linearly down to the minimum as error approaches zero. The
# minimum should be just above your motors' stall point — if the bot
# stutters/stalls near the target, raise it a bit; if it still
# overshoots, lower the maximum instead.
MAX_MOVE_PERCENT = 45
MIN_MOVE_PERCENT = 25
SLOWDOWN_DISTANCE_PX = 160

MAX_TURN_PERCENT = 35
MIN_TURN_PERCENT = 20
SLOWDOWN_ANGLE_DEG = 45


def scaled_percent(error_magnitude, slowdown_threshold, min_percent, max_percent):
    error_magnitude = abs(error_magnitude)
    if error_magnitude >= slowdown_threshold:
        return max_percent
    fraction = error_magnitude / slowdown_threshold
    return int(min_percent + (max_percent - min_percent) * fraction)


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


def get_target():
    x = float(input("Target X (pixels): "))
    y = float(input("Target Y (pixels): "))
    orientation = float(input("Target orientation (degrees): "))
    return x, y, orientation


def main():
    target_x, target_y, target_orientation = get_target()
    print(f"Navigating to ({target_x:.0f}, {target_y:.0f}) at {target_orientation:.0f} deg. "
          f"Press 'q' in the video window to cancel.")

    state = "ROTATE_TO_HEADING"

    try:
        while True:
            success, img = cam.read()
            if not success:
                print("Failed to grab frame")
                break

            corners, ids, _ = detector.detectMarkers(img)
            cmd = 'X'
            percent = 0

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(img, corners)
                marker_corners = corners[0][0]  # tracks the first marker seen
                current_angle = calculate_angle(marker_corners)
                center = marker_corners.mean(axis=0)
                cx, cy = center[0], center[1]

                dx = target_x - cx
                dy = target_y - cy
                distance = math.hypot(dx, dy)
                desired_heading = math.degrees(math.atan2(dy, dx))
                heading_error = ROTATE_SIGN * normalize_angle(desired_heading - current_angle)
                orientation_error = ROTATE_SIGN * normalize_angle(target_orientation - current_angle)

                cv2.circle(img, (int(target_x), int(target_y)), 6, (255, 0, 0), -1)
                cv2.putText(img,
                            f"dist:{distance:.0f} herr:{heading_error:.0f} "
                            f"heading:{current_angle:.0f} state:{state}",
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

                if state == "ROTATE_TO_HEADING":
                    if distance <= POSITION_TOLERANCE_PX:
                        state = "ROTATE_TO_ORIENTATION"
                    elif abs(heading_error) > ANGLE_TOLERANCE_DEG:
                        cmd = 'D' if heading_error > 0 else 'A'
                        percent = scaled_percent(heading_error, SLOWDOWN_ANGLE_DEG,
                                                  MIN_TURN_PERCENT, MAX_TURN_PERCENT)
                    else:
                        state = "MOVE"

                if state == "MOVE":
                    if distance <= POSITION_TOLERANCE_PX:
                        state = "ROTATE_TO_ORIENTATION"
                    elif abs(heading_error) > ANGLE_TOLERANCE_DEG * 2:
                        state = "ROTATE_TO_HEADING"
                    else:
                        cmd = 'W'
                        percent = scaled_percent(distance, SLOWDOWN_DISTANCE_PX,
                                                  MIN_MOVE_PERCENT, MAX_MOVE_PERCENT)

                if state == "ROTATE_TO_ORIENTATION":
                    if abs(orientation_error) <= ANGLE_TOLERANCE_DEG:
                        state = "DONE"
                    else:
                        cmd = 'D' if orientation_error > 0 else 'A'
                        percent = scaled_percent(orientation_error, SLOWDOWN_ANGLE_DEG,
                                                  MIN_TURN_PERCENT, MAX_TURN_PERCENT)

                if state == "DONE":
                    send_cmd('X')
                    print("Target reached.")
                    cv2.imshow("Navigation", img)
                    cv2.waitKey(500)
                    break
            else:
                cmd = 'X'  # marker not visible, stop for safety

            send_cmd(cmd, percent)

            cv2.imshow("Navigation", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Cancelled.")
                break

            time.sleep(SEND_INTERVAL_S)

    finally:
        send_cmd('X')
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()