
import cv2
import numpy as np
import time
from pathlib import Path

# ---------------- SETTINGS ----------------
ROOT = Path(__file__).resolve().parent.parent
CALIBRATION_FILE = ROOT / "camera" / "calibration" / "stereo_params.npz"

WIDTH = 1280
HEIGHT = 720
SCALE = 0.5

NUM_DISPARITIES = 256  # must be divisible by 16
BLOCK_SIZE = 5
MIN_DEPTH_CM = 10
MAX_DEPTH_CM = 300

# ArUco dictionary selected for this project
ARUCO_DICTIONARY = cv2.aruco.Dictionary_get(
    cv2.aruco.DICT_4X4_50
)
ARUCO_PARAMETERS = cv2.aruco.DetectorParameters_create()


def open_camera(sensor_id):
    pipeline = (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM),width={WIDTH},height={HEIGHT},framerate=30/1 ! "
        "nvvidconv ! video/x-raw,format=BGRx ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )
    camera = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    if not camera.isOpened():
        raise RuntimeError(f"Could not open camera sensor {sensor_id}")

    return camera


def main():
    if not CALIBRATION_FILE.exists():
        print("ERROR: Stereo calibration file not found:")
        print(CALIBRATION_FILE)
        return

    calibration = np.load(CALIBRATION_FILE)

    required = ["map1x", "map1y", "map2x", "map2y", "P1", "T"]
    missing = [key for key in required if key not in calibration]
    if missing:
        print("ERROR: Calibration file missing:", ", ".join(missing))
        print("Available keys:", calibration.files)
        return

    map1x = calibration["map1x"]
    map1y = calibration["map1y"]
    map2x = calibration["map2x"]
    map2y = calibration["map2y"]
    P1 = calibration["P1"]
    T = calibration["T"]

    baseline_mm = float(np.linalg.norm(T))
    focal_small = float(P1[0, 0]) * SCALE

    print(f"Stereo baseline: {baseline_mm:.2f} mm")
    print(f"Scaled focal length: {focal_small:.2f} pixels")

    matcher = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=NUM_DISPARITIES,
        blockSize=BLOCK_SIZE,
        P1=8 * BLOCK_SIZE**2,
        P2=32 * BLOCK_SIZE**2,
        disp12MaxDiff=1,
        uniquenessRatio=12,
        speckleWindowSize=100,
        speckleRange=2,
        preFilterCap=31,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )

    left_camera = None
    right_camera = None

    try:
        left_camera = open_camera(0)
        right_camera = open_camera(1)

        print("ArUco + stereo depth test started.")
        print("Show your printed/displayed marker to both cameras.")
        print("Press Q in the camera window to quit.")

        previous_depth = {}
        last_print = 0
        frame_count = 0
        fps_start = time.time()

        while True:
            ok_left, left_frame = left_camera.read()
            ok_right, right_frame = right_camera.read()

            if not ok_left or not ok_right:
                print("ERROR: Failed to read a camera frame.")
                break

            # Rectify full-size images using saved stereo calibration
            left_rect = cv2.remap(
                left_frame, map1x, map1y, cv2.INTER_LINEAR
            )
            right_rect = cv2.remap(
                right_frame, map2x, map2y, cv2.INTER_LINEAR
            )

            # Run detection on the rectified left camera
            corners, ids, _ = cv2.aruco.detectMarkers(
                left_rect, ARUCO_DICTIONARY,
                parameters=ARUCO_PARAMETERS
            )

            # Work at half resolution for stereo depth
            left_small = cv2.resize(
                left_rect, None, fx=SCALE, fy=SCALE
            )
            right_small = cv2.resize(
                right_rect, None, fx=SCALE, fy=SCALE
            )

            display = left_small.copy()
            gray_left = cv2.cvtColor(left_small, cv2.COLOR_BGR2GRAY)
            gray_right = cv2.cvtColor(right_small, cv2.COLOR_BGR2GRAY)

            # Compute disparity in the half-resolution rectified images
            disparity = matcher.compute(
                gray_left, gray_right
            ).astype(np.float32) / 16.0

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(
                    display,
                    [c * SCALE for c in corners],
                    ids
                )

                for marker_corners, marker_id_array in zip(corners, ids):
                    marker_id = int(marker_id_array[0])

                    # Marker corners are detected at full resolution.
                    points = marker_corners[0] * SCALE
                    center_x = int(np.mean(points[:, 0]))
                    center_y = int(np.mean(points[:, 1]))

                    cv2.circle(
                        display, (center_x, center_y),
                        4, (0, 0, 255), -1
                    )

                    # Sample a small patch slightly inside the marker.
                    x1 = max(0, center_x - 4)
                    x2 = min(disparity.shape[1], center_x + 5)
                    y1 = max(0, center_y - 4)
                    y2 = min(disparity.shape[0], center_y + 5)

                    patch = disparity[y1:y2, x1:x2]
                    valid = patch[
                        np.isfinite(patch) & (patch > 1.0)
                    ]

                    depth_cm = None

                    if valid.size >= 15:
                        median_disp = float(np.median(valid))
                        mad = float(
                            np.median(np.abs(valid - median_disp))
                        )

                        if mad <= max(3.0, median_disp * 0.08):
                            depth_cm = (
                                focal_small * baseline_mm / median_disp
                            ) / 10.0

                            if not (
                                MIN_DEPTH_CM <= depth_cm <= MAX_DEPTH_CM
                            ):
                                depth_cm = None

                    if depth_cm is not None:
                        # Smooth modest fluctuations; reset after big jumps.
                        old_depth = previous_depth.get(marker_id)

                        if old_depth is not None and abs(depth_cm - old_depth) < 25:
                            depth_cm = 0.65 * old_depth + 0.35 * depth_cm

                        previous_depth[marker_id] = depth_cm
                        label = f"ID {marker_id}: {depth_cm:.1f} cm"
                    else:
                        label = f"ID {marker_id}: depth unavailable"

                    text_y = max(25, center_y - 15)
                    cv2.putText(
                        display, label,
                        (max(5, center_x - 100), text_y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 255, 0), 2
                    )

                    now = time.time()
                    if now - last_print >= 1.0:
                        if depth_cm is not None:
                            print(
                                f"Marker ID {marker_id} | "
                                f"Distance: {depth_cm:.1f} cm"
                            )
                        else:
                            print(
                                f"Marker ID {marker_id} | "
                                "Depth unavailable"
                            )
                        last_print = now

            else:
                cv2.putText(
                    display, "No ArUco marker detected",
                    (15, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (0, 0, 255), 2
                )

            frame_count += 1
            elapsed = time.time() - fps_start

            if elapsed >= 1.0:
                fps = frame_count / elapsed
                print(f"Processing FPS: {fps:.1f}")
                frame_count = 0
                fps_start = time.time()

            cv2.imshow("ArUco Stereo Depth", display)

            if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q")):
                break

    except Exception as exc:
        print("ERROR:", exc)

    finally:
        if left_camera is not None:
            left_camera.release()
        if right_camera is not None:
            right_camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
