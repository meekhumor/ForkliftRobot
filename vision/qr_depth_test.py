
import cv2
import numpy as np
import time
from pathlib import Path

# ---------- SETTINGS ----------
ROOT = Path(__file__).resolve().parent.parent
CALIBRATION_FILE = ROOT / "camera" / "calibration" / "stereo_params.npz"

WIDTH = 1280
HEIGHT = 720
SCALE = 0.5
NUM_DISPARITIES = 256
BLOCK_SIZE = 5

MIN_DEPTH_CM = 10
MAX_DEPTH_CM = 300

# ---------- LOAD CALIBRATION ----------
if not CALIBRATION_FILE.exists():
    raise FileNotFoundError(f"Calibration not found: {CALIBRATION_FILE}")

cal = np.load(str(CALIBRATION_FILE))

required = ["map1x", "map1y", "map2x", "map2y", "P1", "T"]
missing = [key for key in required if key not in cal]
if missing:
    raise KeyError(
        f"Calibration file is missing: {missing}. "
        "Check camera/calibration/stereo_params.npz."
    )

map1x, map1y = cal["map1x"], cal["map1y"]
map2x, map2y = cal["map2x"], cal["map2y"]
P1 = cal["P1"]
T = cal["T"]

focal_px = float(P1[0, 0])
baseline_mm = float(np.linalg.norm(T))
focal_small = focal_px * SCALE

# ---------- STEREO MATCHER ----------
matcher = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=NUM_DISPARITIES,
    blockSize=BLOCK_SIZE,
    P1=8 * BLOCK_SIZE * BLOCK_SIZE,
    P2=32 * BLOCK_SIZE * BLOCK_SIZE,
    disp12MaxDiff=1,
    uniquenessRatio=12,
    speckleWindowSize=100,
    speckleRange=2,
    preFilterCap=31,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
)

# ---------- CAMERA SETUP ----------
def open_camera(sensor_id):
    pipeline = (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM),width={WIDTH},height={HEIGHT},framerate=30/1 ! "
        "nvvidconv ! video/x-raw,format=BGRx ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )

    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        cap.release()
        raise RuntimeError(f"Could not open camera sensor {sensor_id}")

    return cap


def get_depth(left_gray, right_gray, x, y):
    """Estimate depth near a point. Return None if the match is unreliable."""
    left_small = cv2.resize(
        left_gray, None, fx=SCALE, fy=SCALE,
        interpolation=cv2.INTER_AREA
    )
    right_small = cv2.resize(
        right_gray, None, fx=SCALE, fy=SCALE,
        interpolation=cv2.INTER_AREA
    )

    disparity = matcher.compute(
        left_small, right_small
    ).astype(np.float32) / 16.0

    sx = int(round(x * SCALE))
    sy = int(round(y * SCALE))
    radius = 4

    if (
        sx < radius or sy < radius
        or sx >= disparity.shape[1] - radius
        or sy >= disparity.shape[0] - radius
    ):
        return None

    patch = disparity[
        sy-radius:sy+radius+1,
        sx-radius:sx+radius+1
    ]

    valid = patch[np.isfinite(patch) & (patch > 1.0)]

    if valid.size < 15:
        return None

    median = float(np.median(valid))
    variation = float(np.median(np.abs(valid - median)))

    if variation > max(3.0, median * 0.08):
        return None

    depth_cm = (focal_small * baseline_mm / median) / 10.0

    if not MIN_DEPTH_CM <= depth_cm <= MAX_DEPTH_CM:
        return None

    return depth_cm


def detect_qr(frame, detector):
    text, points, _ = detector.detectAndDecode(frame)

    if points is None:
        return None, None

    points = points.reshape(-1, 2).astype(np.int32)
    center = np.mean(points, axis=0).astype(int)

    return text, (points, (int(center[0]), int(center[1])))


def main():
    print("=== Automatic QR + Stereo Depth Test ===")
    print(f"Calibration: {CALIBRATION_FILE}")
    print(f"Baseline: {baseline_mm:.2f} mm")
    print(f"Original focal length: {focal_px:.2f} px")
    print("No mouse clicks needed. Press Q to quit.")

    left_cap = None
    right_cap = None

    try:
        left_cap = open_camera(0)
        right_cap = open_camera(1)

        # Reuse one detector rather than recreating it for every frame.
        detector = cv2.QRCodeDetector()

        smoothed_depth = None
        last_print_time = 0.0

        # FPS measurement
        fps_start_time = time.time()
        fps_frames = 0
        fps = 0.0

        while True:
            left_ok, left_frame = left_cap.read()
            right_ok, right_frame = right_cap.read()

            if not left_ok or not right_ok:
                print("Warning: camera frame unavailable.")
                time.sleep(0.05)
                continue

            left_rect = cv2.remap(
                left_frame, map1x, map1y, cv2.INTER_LINEAR
            )
            right_rect = cv2.remap(
                right_frame, map2x, map2y, cv2.INTER_LINEAR
            )

            qr_text, qr_info = detect_qr(left_rect, detector)
            display = left_rect.copy()

            if qr_info is not None:
                points, (cx, cy) = qr_info

                cv2.polylines(
                    display, [points], True, (0, 255, 0), 2
                )
                cv2.circle(display, (cx, cy), 5, (0, 0, 255), -1)

                left_gray = cv2.cvtColor(
                    left_rect, cv2.COLOR_BGR2GRAY
                )
                right_gray = cv2.cvtColor(
                    right_rect, cv2.COLOR_BGR2GRAY
                )

                depth_cm = get_depth(
                    left_gray, right_gray, cx, cy
                )

                if depth_cm is not None:
                    if (
                        smoothed_depth is None
                        or abs(depth_cm - smoothed_depth) > 25
                    ):
                        smoothed_depth = depth_cm
                    else:
                        smoothed_depth = (
                            0.7 * smoothed_depth + 0.3 * depth_cm
                        )

                    cv2.putText(
                        display,
                        f"Depth: {smoothed_depth:.1f} cm",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.9,
                        (0, 255, 0),
                        2
                    )

                    now = time.time()
                    if now - last_print_time >= 1.0:
                        shown_text = (
                            qr_text if qr_text
                            else "(QR detected; text unreadable)"
                        )
                        print(
                            f"QR: {shown_text} | "
                            f"Center: ({cx}, {cy}) | "
                            f"Depth: {smoothed_depth:.1f} cm"
                        )
                        last_print_time = now
                else:
                    cv2.putText(
                        display,
                        "Depth: unreliable",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.9,
                        (0, 165, 255),
                        2
                    )
                    smoothed_depth = None

                if qr_text:
                    cv2.putText(
                        display,
                        qr_text[:45],
                        (20, 75),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 0),
                        2
                    )
            else:
                cv2.putText(
                    display,
                    "QR not detected",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (0, 0, 255),
                    2
                )
                smoothed_depth = None

            # Count processed frames and update FPS once per second.
            fps_frames += 1
            now = time.time()
            elapsed = now - fps_start_time

            if elapsed >= 1.0:
                fps = fps_frames / elapsed
                fps_frames = 0
                fps_start_time = now

            cv2.putText(
                display,
                f"FPS: {fps:.1f}",
                (20, 110),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2
            )

            cv2.imshow("Automatic QR + Depth", display)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")):
                break

    finally:
        if left_cap is not None:
            left_cap.release()
        if right_cap is not None:
            right_cap.release()
        cv2.destroyAllWindows()
        print("Camera resources released.")


if __name__ == "__main__":
    main()
