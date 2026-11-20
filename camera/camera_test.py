
import cv2

sensor = input("Choose camera sensor (0 or 1): ").strip()

if sensor not in ("0", "1"):
    print("Please enter 0 or 1.")
    raise SystemExit(1)

pipeline = (
    f"nvarguscamerasrc sensor-id={sensor} ! "
    "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
    "nvvidconv ! "
    "video/x-raw,format=BGRx ! "
    "videoconvert ! "
    "video/x-raw,format=BGR ! "
    "appsink drop=1"
)

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print(f"Could not open sensor {sensor}.")
    raise SystemExit(1)

print(f"Showing sensor {sensor}. Press Q to close.")

while True:
    ret, frame = cap.read()

    if not ret:
        print("Could not read frame.")
        break

    cv2.putText(
        frame,
        f"Camera sensor {sensor}",
        (30, 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        2,
    )

    cv2.imshow(f"Sensor {sensor} Test", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
