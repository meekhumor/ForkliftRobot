
import cv2
from pathlib import Path

# Use the ArUco dictionary chosen for this project
ARUCO_DICT = cv2.aruco.Dictionary_get(
    cv2.aruco.DICT_4X4_50
)

PARAMETERS = cv2.aruco.DetectorParameters_create()

def main():
    pipeline = (
        "nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
        "nvvidconv ! video/x-raw,format=BGRx ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )

    camera = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    if not camera.isOpened():
        print("ERROR: Could not open camera sensor 0.")
        return

    print("Camera started. Show marker ID 7 to the camera.")
    print("Press Q in the camera window to quit.")

    while True:
        ret, frame = camera.read()

        if not ret:
            print("Could not read camera frame.")
            break

        corners, ids, _ = cv2.aruco.detectMarkers(
            frame, ARUCO_DICT, parameters=PARAMETERS
        )

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for marker_id in ids.flatten():
                print(f"Detected ArUco ID: {marker_id}", end="\r")

        else:
            cv2.putText(
                frame,
                "No ArUco marker detected",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

        cv2.imshow("ArUco Detection Test", frame)

        if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q")):
            break

    camera.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
