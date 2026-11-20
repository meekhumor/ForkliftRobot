
from pathlib import Path
import cv2
import numpy as np

LEFT_DIR = Path("camera/calibration/left")
RIGHT_DIR = Path("camera/calibration/right")
OUTPUT_FILE = Path("camera/calibration/stereo_params.npz")

BOARD_SIZE = (7, 7)   # Internal corners: 7 x 7
SQUARE_SIZE_MM = 26.0 # Measured checkerboard square size

# Prepare 3D checkerboard corner positions in millimetres.
objp = np.zeros(
    (BOARD_SIZE[0] * BOARD_SIZE[1], 3), np.float32
)
objp[:, :2] = np.mgrid[
    0:BOARD_SIZE[0], 0:BOARD_SIZE[1]
].T.reshape(-1, 2)
objp *= SQUARE_SIZE_MM

object_points = []
left_points = []
right_points = []

image_size = None
used_pairs = []

criteria = (
    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
    30,
    0.001,
)

left_files = sorted(LEFT_DIR.glob("left_*.jpg"))

for left_path in left_files:
    number = left_path.stem.replace("left_", "")
    right_path = RIGHT_DIR / f"right_{number}.jpg"

    if not right_path.exists():
        continue

    left_img = cv2.imread(str(left_path))
    right_img = cv2.imread(str(right_path))

    if left_img is None or right_img is None:
        continue

    if left_img.shape[:2] != right_img.shape[:2]:
        print(f"Skipping {number}: image sizes differ")
        continue

    gray_left = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

    image_size = (gray_left.shape[1], gray_left.shape[0])

    # Detect corners in both images.
    found_left, corners_left = cv2.findChessboardCornersSB(
        gray_left, BOARD_SIZE
    )
    found_right, corners_right = cv2.findChessboardCornersSB(
        gray_right, BOARD_SIZE
    )

    if not (found_left and found_right):
        print(f"Skipping pair {number}: corners not found in both")
        continue

    # Refine corner locations.
    corners_left = cv2.cornerSubPix(
        gray_left, corners_left, (5, 5), (-1, -1), criteria
    )
    corners_right = cv2.cornerSubPix(
        gray_right, corners_right, (5, 5), (-1, -1), criteria
    )

    object_points.append(objp.copy())
    left_points.append(corners_left)
    right_points.append(corners_right)
    used_pairs.append(number)

    print(f"Using pair {number}")

if len(object_points) < 10:
    raise RuntimeError(
        f"Only {len(object_points)} valid pairs. "
        "Need at least 10 to attempt calibration."
    )

print(f"\nCalibrating with {len(object_points)} pairs...")

# Calibrate each camera individually first.
left_rms, K1, D1, _, _ = cv2.calibrateCamera(
    object_points, left_points, image_size, None, None
)
right_rms, K2, D2, _, _ = cv2.calibrateCamera(
    object_points, right_points, image_size, None, None
)

# Estimate the relative rotation and translation between cameras.
stereo_rms, K1, D1, K2, D2, R, T, E, F = (
    cv2.stereoCalibrate(
        object_points,
        left_points,
        right_points,
        K1, D1, K2, D2,
        image_size,
        criteria=criteria,
        flags=cv2.CALIB_FIX_INTRINSIC,
    )
)

# Rectification maps make corresponding points line up horizontally.
R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
    K1, D1, K2, D2,
    image_size, R, T,
    alpha=0
)

map1x, map1y = cv2.initUndistortRectifyMap(
    K1, D1, R1, P1, image_size, cv2.CV_32FC1
)
map2x, map2y = cv2.initUndistortRectifyMap(
    K2, D2, R2, P2, image_size, cv2.CV_32FC1
)

OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)

np.savez(
    OUTPUT_FILE,
    image_size=np.array(image_size),
    square_size_mm=SQUARE_SIZE_MM,
    K1=K1, D1=D1, K2=K2, D2=D2,
    R=R, T=T, E=E, F=F,
    R1=R1, R2=R2, P1=P1, P2=P2, Q=Q,
    map1x=map1x, map1y=map1y,
    map2x=map2x, map2y=map2y,
    used_pairs=np.array(used_pairs),
)

print("\n--- CALIBRATION RESULTS ---")
print(f"Left camera RMS:  {left_rms:.4f} pixels")
print(f"Right camera RMS: {right_rms:.4f} pixels")
print(f"Stereo RMS:       {stereo_rms:.4f} pixels")
print(f"Estimated baseline: {np.linalg.norm(T):.2f} mm")
print(f"Saved parameters to: {OUTPUT_FILE}")
print("Pairs used:", ", ".join(used_pairs))
