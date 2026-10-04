"""
Click-to-navigate: set waypoints by clicking on the camera window,
then start the bot driving through them in order using ArUco tracking.

Controls (video window):
    Left click   = add a waypoint
    Right click  = undo last waypoint
    c            = clear all waypoints
    s            = start navigating through the waypoints
    q            = quit / abort (stops the bot)

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
converging), flip ROTATE_SIGN from 1 to -1 below.
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
POSITION_TOLERANCE_PX = 10
ANGLE_TOLERANCE_DEG = 1
ROTATE_SIGN = 1  # flip to -1 if the bot turns the wrong direction

WINDOW_NAME = "Navigation"


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
        self.waypoints = []       # list of (x, y)
        self.running = False
        self.current_index = 0
        self.state = "ROTATE_TO_HEADING"

    def on_mouse(self, event, x, y, flags, param):
        if self.running:
            return  # no editing waypoints once navigation has started
        if event == cv2.EVENT_LBUTTONDOWN:
            self.waypoints.append((x, y))
        elif event == cv2.EVENT_RBUTTONDOWN:
            if self.waypoints:
                self.waypoints.pop()

    def draw_overlay(self, img):
        # draw waypoints and connecting path
        for i, (wx, wy) in enumerate(self.waypoints):
            color = (0, 165, 255) if (self.running and i == self.current_index) else (255, 0, 0)
            cv2.circle(img, (int(wx), int(wy)), 6, color, -1)
            cv2.putText(img, str(i + 1), (int(wx) + 8, int(wy) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        for i in range(len(self.waypoints) - 1):
            cv2.line(img, self.waypoints[i], self.waypoints[i + 1], (255, 0, 0), 1)

        if not self.running:
            msg = f"SETUP  |  {len(self.waypoints)} waypoint(s)  |  L-click: add  R-click: undo  c: clear  s: start  q: quit"
        else:
            msg = f"RUNNING  |  waypoint {self.current_index + 1}/{len(self.waypoints)}  |  state:{self.state}  q: abort"
        cv2.putText(img, msg, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    def start(self):
        if not self.waypoints:
            return
        self.running = True
        self.current_index = 0
        self.state = "ROTATE_TO_HEADING"

    def stop(self):
        self.running = False
        send('X')


def main():
    app = NavApp()
    cv2.namedWindow(WINDOW_NAME)
    cv2.setMouseCallback(WINDOW_NAME, app.on_mouse)

    print("Click on the video window to set waypoints.")
    print("Left click: add | Right click: undo | c: clear | s: start | q: quit")

    try:
        while True:
            success, img = cam.read()
            if not success:
                print("Failed to grab frame")
                break

            corners, ids, _ = detector.detectMarkers(img)
            cmd = 'X'

            if app.running and ids is not None and app.waypoints:
                cv2.aruco.drawDetectedMarkers(img, corners)
                marker_corners = corners[0][0]  # tracks the first marker seen
                current_angle = calculate_angle(marker_corners)
                center = marker_corners.mean(axis=0)
                cx, cy = center[0], center[1]

                target_x, target_y = app.waypoints[app.current_index]
                dx = target_x - cx
                dy = target_y - cy
                distance = math.hypot(dx, dy)
                desired_heading = math.degrees(math.atan2(dy, dx))
                heading_error = ROTATE_SIGN * normalize_angle(desired_heading - current_angle)

                if app.state == "ROTATE_TO_HEADING":
                    if distance <= POSITION_TOLERANCE_PX:
                        app.state = "ARRIVED"
                    elif abs(heading_error) > ANGLE_TOLERANCE_DEG:
                        cmd = 'W' if heading_error > 0 else 'S'
                    else:
                        app.state = "MOVE"

                if app.state == "MOVE":
                    if distance <= POSITION_TOLERANCE_PX:
                        app.state = "ARRIVED"
                    elif abs(heading_error) > ANGLE_TOLERANCE_DEG * 2:
                        app.state = "ROTATE_TO_HEADING"
                    else:
                        cmd = 'D'

                if app.state == "ARRIVED":
                    app.current_index += 1
                    if app.current_index >= len(app.waypoints):
                        print("All waypoints reached.")
                        app.stop()
                    else:
                        app.state = "ROTATE_TO_HEADING"

            elif app.running:
                cmd = 'X'  # marker not visible or no waypoints left, stop for safety

            if app.running:
                send(cmd)

            app.draw_overlay(img)
            cv2.imshow(WINDOW_NAME, img)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                app.stop()
                print("Stopped.")
                break
            elif key == ord('c') and not app.running:
                app.waypoints.clear()
            elif key == ord('s') and not app.running:
                app.start()
                print(f"Starting navigation through {len(app.waypoints)} waypoint(s).")

            time.sleep(SEND_INTERVAL_S)

    finally:
        send('X')
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
