
from pathlib import Path
import cv2

LEFT_DIR = Path("camera/calibration/left")
RIGHT_DIR = Path("camera/calibration/right")
BOARD_SIZE = (7, 7)

left_files = sorted(LEFT_DIR.glob("left_*.jpg"))
right_files = {
    p.name.replace("right_", "left_"): p
    for p in RIGHT_DIR.glob("right_*.jpg")
}

good_pairs = 0
bad_pairs = []

print(f"Left images: {len(left_files)}")
print(f"Right images: {len(right_files)}")
print("\nChecking pairs...\n", flush=True)

for index, left_path in enumerate(left_files, start=1):
    right_path = right_files.get(left_path.name)

    if right_path is None:
        print(f"{left_path.stem}: MISSING RIGHT IMAGE", flush=True)
        bad_pairs.append(left_path.name)
        continue

    pair_results = []

    for path in (left_path, right_path):
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)

        if image is None:
            pair_results.append(False)
            continue

        # Reduce image size to speed up detection.
        small = cv2.resize(
            image, None, fx=0.5, fy=0.5,
            interpolation=cv2.INTER_AREA
        )

        found = False

        if hasattr(cv2, "findChessboardCornersSB"):
            try:
                found, _ = cv2.findChessboardCornersSB(
                    small, BOARD_SIZE
                )
            except cv2.error:
                found = False

        pair_results.append(bool(found))

    left_ok, right_ok = pair_results

    if left_ok and right_ok:
        good_pairs += 1
        status = "OK"
    else:
        bad_pairs.append(left_path.name)
        status = (
            f"CHECK (left={'OK' if left_ok else 'FAIL'}, "
            f"right={'OK' if right_ok else 'FAIL'})"
        )

    print(f"[{index}/{len(left_files)}] {left_path.stem}: {status}",
          flush=True)

print("\n--- SUMMARY ---")
print(f"Valid pairs: {good_pairs}/{len(left_files)}")
print(f"Pairs needing attention: {len(bad_pairs)}")
