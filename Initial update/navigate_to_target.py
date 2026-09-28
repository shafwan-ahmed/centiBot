"""
Navigate the bot to a target (x, y, orientation) using ArUco tracking.

Enter a target pixel coordinate + orientation in the terminal; the
script watches the ArUco marker via camera, computes the needed
turn/drive, and sends W/A/S/D/X UDP commands to the bot until it
arrives.

Command meaning (as wired/observed on your bot):
    D = forward   (translate)
    A = backward  (translate)
    W = rotate right
    S = rotate left
    X = stop

Setup:
    pip install opencv-contrib-python numpy
    Set ESP_IP below to your board's IP address.

If the bot turns the wrong way (overshoots/oscillates instead of
converging), flip ROTATE_SIGN from 1 to -1 below — that's the only
thing likely to need calibrating for your camera/marker orientation.
"""

import math
import time
import socket

import cv2
import numpy as np

# ---------- Bot connection ----------
ESP_IP = "192.168.50.174"   # <-- set to your board's IP address
ESP_PORT = 4210
SEND_INTERVAL_S = 0.05      # matches the board's 300ms watchdog

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send(cmd: str):
    sock.sendto(cmd.encode(), (ESP_IP, ESP_PORT))


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
ROTATE_SIGN = 1  # flip to -1 if the bot turns the wrong direction


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
                cv2.putText(img, f"dist:{distance:.0f} herr:{heading_error:.0f} state:{state}",
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

                if state == "ROTATE_TO_HEADING":
                    if distance <= POSITION_TOLERANCE_PX:
                        state = "ROTATE_TO_ORIENTATION"
                    elif abs(heading_error) > ANGLE_TOLERANCE_DEG:
                        cmd = 'W' if heading_error > 0 else 'S'
                    else:
                        state = "MOVE"

                if state == "MOVE":
                    if distance <= POSITION_TOLERANCE_PX:
                        state = "ROTATE_TO_ORIENTATION"
                    elif abs(heading_error) > ANGLE_TOLERANCE_DEG * 2:
                        state = "ROTATE_TO_HEADING"
                    else:
                        cmd = 'D'

                if state == "ROTATE_TO_ORIENTATION":
                    if abs(orientation_error) <= ANGLE_TOLERANCE_DEG:
                        state = "DONE"
                    else:
                        cmd = 'W' if orientation_error > 0 else 'S'

                if state == "DONE":
                    send('X')
                    print("Target reached.")
                    cv2.imshow("Navigation", img)
                    cv2.waitKey(500)
                    break
            else:
                cmd = 'X'  # marker not visible, stop for safety

            send(cmd)

            cv2.imshow("Navigation", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Cancelled.")
                break

            time.sleep(SEND_INTERVAL_S)

    finally:
        send('X')
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
