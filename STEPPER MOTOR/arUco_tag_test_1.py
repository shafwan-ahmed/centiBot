import cv2
import numpy as np

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

cam = cv2.VideoCapture(0)  # try 0 first; change if wrong camera opens
cam.set(3, 700)
cam.set(4, 505)

if not cam.isOpened():
    raise RuntimeError("Could not open camera. Try a different index (0, 1, 2...).")


def calculate_angle(corners):
    top_left, top_right, bottom_right, bottom_left = corners
    vector = top_right - top_left
    angle = np.arctan2(vector[1], vector[0]) * 180 / np.pi
    return angle


while True:
    success, img = cam.read()
    if not success:
        print("Failed to grab frame")
        break

    corners, ids, rejectedImgPoints = detector.detectMarkers(img)

    if ids is not None:
        ids = ids.flatten()  # ensures ids is always 1D, e.g. [0, 1, 2]

        cv2.aruco.drawDetectedMarkers(img, corners)

        for i in range(len(ids)):
            marker_corners = corners[i][0]
            angle = calculate_angle(marker_corners)

            center = marker_corners.mean(axis=0)
            bots_cordinate = (int(center[0]) - 20, int(center[1]) + 13)
            data = f"{ids[i]}"  # no more [0] indexing needed

            cv2.putText(img, f"ID: {data}", (int(center[0]), int(center[1]) - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            cv2.putText(img, f"{angle:.2f}", (int(center[0]) - 10, int(center[1]) + 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            cv2.putText(img, "+", (int(center[0]) - 6, int(center[1]) + 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            print(f"ID: {data}, Coordinate: {bots_cordinate}, Angle: {angle:.2f}")

    cv2.imshow("Result", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam.release()
cv2.destroyAllWindows()