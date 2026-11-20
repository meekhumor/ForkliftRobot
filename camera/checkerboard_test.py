
import cv2

PATTERN_SIZE = (7, 7)  # Internal corners in your 8x8 checkerboard

pipeline = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
    "nvvidconv ! "
    "video/x-raw,format=BGRx ! "
    "videoconvert ! "
    "video/x-raw,format=BGR ! "
    "appsink drop=1 max-buffers=1"
)

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Could not open camera sensor 0.")
    raise SystemExit(1)

print("Camera opened. Hold the checkerboard flat and fully visible.")
print("Press Q to quit.")

frame_count = 0
found = False
corners = None

while True:
    ret, frame = cap.read()

    if not ret:
        print("Could not read camera frame.")
        break

    frame_count += 1

    # Run checkerboard detection only every 5th frame,
    # using a smaller image to reduce CPU load.
    if frame_count % 5 == 0:
        small = cv2.resize(frame, (640, 360))
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

        found, small_corners = cv2.findChessboardCorners(
            gray, PATTERN_SIZE, None
        )

        if found:
            # Convert corner positions back to full-frame coordinates.
            corners = small_corners * 2.0
        else:
            corners = None

    if found and corners is not None:
        cv2.drawChessboardCorners(
            frame, PATTERN_SIZE, corners, True
        )
        message = "CHECKERBOARD DETECTED"
        color = (0, 255, 0)
    else:
        message = "Searching for checkerboard..."
        color = (0, 0, 255)

    cv2.putText(
        frame, message, (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2
    )

    cv2.imshow("Checkerboard Test", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
