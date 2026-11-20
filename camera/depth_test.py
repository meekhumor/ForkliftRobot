
from pathlib import Path

import cv2
import numpy as np

# --------------------------------------------------
# Configuration
# --------------------------------------------------
PARAMS = Path("camera/calibration/stereo_params.npz")

# Process at half resolution for better performance.
SCALE = 0.5

# At half resolution, 256 disparity pixels correspond to
# 512 pixels at the original calibration resolution.
NUM_DISPARITIES = 256
BLOCK_SIZE = 5

# Ignore depths closer than this approximate distance.
MIN_DEPTH_CM = 15.0

# --------------------------------------------------
# Load calibration
# --------------------------------------------------
if not PARAMS.exists():
    raise FileNotFoundError(
        f"Calibration file not found: {PARAMS}"
    )

data = np.load(PARAMS)

map1x, map1y = data["map1x"], data["map1y"]
map2x, map2y = data["map2x"], data["map2y"]

focal_px = float(data["P1"][0, 0])
baseline_mm = float(np.linalg.norm(data["T"]))

if focal_px <= 0 or baseline_mm <= 0:
    raise ValueError("Invalid stereo calibration values.")

# Scale the focal length to match the processed images.
focal_small = focal_px * SCALE

original_size = (map1x.shape[1], map1x.shape[0])
small_size = (
    round(original_size[0] * SCALE),
    round(original_size[1] * SCALE),
)

# --------------------------------------------------
# Stereo matcher
# --------------------------------------------------
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

state = {"disparity": None}


def open_camera(sensor_id):
    pipeline = (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        "video/x-raw(memory:NVMM),"
        "width=1280,height=720,framerate=30/1 ! "
        "nvvidconv ! video/x-raw,format=BGRx ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=1 max-buffers=1"
    )

    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        cap.release()
        raise RuntimeError(
            f"Could not open camera sensor {sensor_id}"
        )

    return cap


def on_click(event, x, y, flags, userdata):
    if event != cv2.EVENT_LBUTTONDOWN:
        return

    disp = state["disparity"]

    if disp is None:
        print("Waiting for a disparity frame.")
        return

    h, w = disp.shape

    if not (0 <= x < w and 0 <= y < h):
        return

    # Use a small neighborhood around the selected point.
    patch = disp[
        max(0, y - 4):min(h, y + 5),
        max(0, x - 4):min(w, x + 5),
    ]

    valid = patch[
        np.isfinite(patch)
        & (patch > 1.0)
        & (patch < NUM_DISPARITIES - 1)
    ]

    if valid.size < 5:
        print(
            f"Pixel ({x}, {y}): unreliable match. "
            "Click a textured area inside the box."
        )
        return

    d = float(np.median(valid))

    # Reject patches whose disparity values vary greatly.
    mad = float(np.median(np.abs(valid - d)))

    if mad > max(3.0, d * 0.08):
        print(
            f"Pixel ({x}, {y}): inconsistent disparity "
            f"(median={d:.1f}px, variation={mad:.1f}px). "
            "Try another textured area."
        )
        return

    depth_mm = focal_small * baseline_mm / d
    depth_cm = depth_mm / 10.0

    if depth_cm < MIN_DEPTH_CM:
        print(
            f"Pixel ({x}, {y}): {depth_cm:.1f} cm is "
            "inside the configured minimum depth."
        )
        return

    print(
        f"Pixel ({x}, {y}) | "
        f"disparity={d:.2f}px at half resolution | "
        f"depth={depth_cm:.1f} cm | "
        f"patch variation={mad:.2f}px"
    )


left = None
right = None

try:
    max_depth_from_search_cm = (
        focal_small * baseline_mm / 1.0 / 10.0
    )
    min_depth_from_search_cm = (
        focal_small * baseline_mm
        / (NUM_DISPARITIES - 1)
        / 10.0
    )

    print("=== Stereo Depth Test ===")
    print(f"Calibration: {PARAMS}")
    print(f"Baseline: {baseline_mm:.2f} mm")
    print(f"Original focal length: {focal_px:.2f} px")
    print(f"Processing resolution: {small_size}")
    print(f"Disparity search: 1-{NUM_DISPARITIES - 1} px")
    print(
        "Approximate geometric search span: "
        f"{min_depth_from_search_cm:.1f} cm to "
        f"{max_depth_from_search_cm:.1f} cm"
    )
    print(
        "This is a theoretical search span, not a "
        "guaranteed accurate operating range."
    )
    print("Click a textured area inside the box.")
    print("Press Q in either window to quit.")

    left = open_camera(0)
    right = open_camera(1)

    cv2.namedWindow("Left Rectified - Click Box")
    cv2.namedWindow("Disparity")
    cv2.setMouseCallback(
        "Left Rectified - Click Box", on_click
    )

    while True:
        ok_l, frame_l = left.read()
        ok_r, frame_r = right.read()

        if not ok_l or not ok_r:
            print("Could not read both camera frames.")
            break

        # Rectify at the calibration resolution first.
        rect_l = cv2.remap(
            frame_l, map1x, map1y, cv2.INTER_LINEAR
        )
        rect_r = cv2.remap(
            frame_r, map2x, map2y, cv2.INTER_LINEAR
        )

        # Then resize both views identically.
        rect_l = cv2.resize(rect_l, small_size)
        rect_r = cv2.resize(rect_r, small_size)

        gray_l = cv2.cvtColor(rect_l, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(rect_r, cv2.COLOR_BGR2GRAY)

        raw = matcher.compute(gray_l, gray_r).astype(np.float32)
        disp = raw / 16.0

        state["disparity"] = disp.copy()

        valid = (
            np.isfinite(disp)
            & (disp > 0)
            & (disp < NUM_DISPARITIES - 1)
        )

        display = np.zeros(disp.shape, dtype=np.uint8)
        display[valid] = np.clip(
            disp[valid] * 255.0 / NUM_DISPARITIES,
            0,
            255,
        ).astype(np.uint8)

        colored = cv2.applyColorMap(
            display, cv2.COLORMAP_TURBO
        )
        colored[~valid] = 0

        cv2.imshow("Left Rectified - Click Box", rect_l)
        cv2.imshow("Disparity", colored)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

finally:
    if left is not None:
        left.release()

    if right is not None:
        right.release()

    cv2.destroyAllWindows()
    print("Camera resources released.")
