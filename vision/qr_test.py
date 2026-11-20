
import cv2
import sys

# Jetson camera pipeline: left sensor (sensor-id=0)
pipeline = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
    "nvvidconv ! "
    "video/x-raw,format=BGRx ! "
    "videoconvert ! "
    "video/x-raw,format=BGR ! "
    "appsink drop=1"
)

# Start the camera
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("ERROR: Could not open camera.")
    print("Check the camera connection and GStreamer support.")
    sys.exit(1)

detector = cv2.QRCodeDetector()

print("Forklift QR scanner started!")
print("Show a QR code to the camera.")
print("Press Q to quit.")

last_qr = None

while True:
    ret, frame = cap.read()

    if not ret:
        print("ERROR: Could not read camera frame.")
        break

    # Detect and decode a QR code
    data, points, _ = detector.detectAndDecode(frame)

    if points is not None:
        points = points.astype(int).reshape(-1, 2)

        # Draw green outline around the QR code
        for i in range(4):
            p1 = tuple(points[i])
            p2 = tuple(points[(i + 1) % 4])
            cv2.line(frame, p1, p2, (0, 255, 0), 3)

        # Calculate the center of the QR code
        center_x = int(points[:, 0].mean())
        center_y = int(points[:, 1].mean())

        # Draw a red dot at the center
        cv2.circle(
            frame, (center_x, center_y), 7, (0, 0, 255), -1
        )

        # Show the center coordinates
        cv2.putText(
            frame,
            f"Center: ({center_x}, {center_y})",
            (max(10, center_x - 100), max(25, center_y - 15)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 255),
            2,
        )

        # Print QR text only when it changes
        if data and data != last_qr:
            print("QR content:", data)
            last_qr = data

        if data:
            cv2.putText(
                frame,
                data[:50],
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

    else:
        last_qr = None

    cv2.imshow("Forklift Robot - QR Vision", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
