
import cv2
import os
import glob

WIDTH = 1280
HEIGHT = 720

LEFT_DIR = "camera/calibration/left"
RIGHT_DIR = "camera/calibration/right"

os.makedirs(LEFT_DIR, exist_ok=True)
os.makedirs(RIGHT_DIR, exist_ok=True)


def open_camera(sensor_id):
    pipeline = (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM),width={WIDTH},height={HEIGHT},framerate=30/1 ! "
        "nvvidconv ! "
        "video/x-raw,format=BGRx ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=1 max-buffers=1"
    )

    camera = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    if not camera.isOpened():
        raise RuntimeError(f"Could not open camera sensor {sensor_id}")

    return camera


def next_pair_number():
    existing = glob.glob(os.path.join(LEFT_DIR, "left_*.jpg"))
    numbers = []

    for path in existing:
        name = os.path.basename(path)
        try:
            numbers.append(int(name.removeprefix("left_").removesuffix(".jpg")))
        except ValueError:
            pass

    return max(numbers, default=0) + 1


left = None
right = None

try:
    print("Opening camera sensor 0...")
    left = open_camera(0)

    print("Opening camera sensor 1...")
    right = open_camera(1)

    pair_number = next_pair_number()

    print("\nKeep the checkerboard still for each capture.")
    print("SPACE = capture a pair | Q = quit")
    print(f"Next pair number: {pair_number}\n")

    while True:
        ok_l, frame_l = left.read()
        ok_r, frame_r = right.read()

        if not ok_l or not ok_r:
            print("Could not read from both cameras.")
            break

        left_view = frame_l.copy()
        right_view = frame_r.copy()

        cv2.putText(left_view, "LEFT / sensor 0", (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(right_view, "RIGHT / sensor 1", (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.imshow("Left Camera", left_view)
        cv2.imshow("Right Camera", right_view)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

        elif key == ord(" "):
            print("Hold still — capturing pair...")

            ok_l, frame_l = left.read()
            ok_r, frame_r = right.read()

            if not ok_l or not ok_r:
                print("Pair not saved: camera frame unavailable.")
                continue

            left_path = os.path.join(
                LEFT_DIR, f"left_{pair_number:02d}.jpg"
            )
            right_path = os.path.join(
                RIGHT_DIR, f"right_{pair_number:02d}.jpg"
            )

            saved_l = cv2.imwrite(left_path, frame_l)
            saved_r = cv2.imwrite(right_path, frame_r)

            if saved_l and saved_r:
                print(f"Saved pair {pair_number}:")
                print(f"  {left_path}")
                print(f"  {right_path}")
                pair_number += 1
            else:
                for path in (left_path, right_path):
                    if os.path.exists(path):
                        os.remove(path)
                print("Pair not saved: image write failed.")

finally:
    if left is not None:
        left.release()
    if right is not None:
        right.release()

    cv2.destroyAllWindows()
    print("Camera resources released.")
