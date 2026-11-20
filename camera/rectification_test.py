
from pathlib import Path
import cv2
import numpy as np

PARAMS = Path("camera/calibration/stereo_params.npz")

data = np.load(PARAMS)

map1x = data["map1x"]
map1y = data["map1y"]
map2x = data["map2x"]
map2y = data["map2y"]

print("Calibration loaded.")
print("Image size:", tuple(data["image_size"]))
print("Estimated baseline:", round(float(np.linalg.norm(data["T"])), 2), "mm")

def open_camera(sensor_id):
    pipeline = (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
        "nvvidconv ! video/x-raw,format=BGRx ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=1 max-buffers=1"
    )

    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera sensor {sensor_id}")
    return cap

left = None
right = None

try:
    left = open_camera(0)
    right = open_camera(1)

    print("Press Q in the image window to quit.")

    while True:
        ok_l, frame_l = left.read()
        ok_r, frame_r = right.read()

        if not ok_l or not ok_r:
            print("Could not read both camera frames.")
            break

        # Match the calibration image dimensions.
        size = (map1x.shape[1], map1x.shape[0])
        frame_l = cv2.resize(frame_l, size)
        frame_r = cv2.resize(frame_r, size)

        rect_l = cv2.remap(frame_l, map1x, map1y, cv2.INTER_LINEAR)
        rect_r = cv2.remap(frame_r, map2x, map2y, cv2.INTER_LINEAR)

        # Horizontal guides make vertical misalignment visible.
        combined = np.hstack((rect_l, rect_r))
        for y in range(50, combined.shape[0], 50):
            cv2.line(combined, (0, y), (combined.shape[1], y),
                     (0, 255, 0), 1)

        cv2.putText(combined, "LEFT", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(combined, "RIGHT", (size[0] + 20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.imshow("Stereo Rectification Test", combined)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    if left is not None:
        left.release()
    if right is not None:
        right.release()
    cv2.destroyAllWindows()
    print("Camera resources released.")
