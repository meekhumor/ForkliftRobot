
import cv2
import sys


def make_pipeline(sensor_id):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        "video/x-raw(memory:NVMM),width=1280,height=720,"
        "framerate=30/1 ! "
        "nvvidconv ! "
        "video/x-raw,format=BGRx ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=1 max-buffers=1"
    )


def open_camera(sensor_id):
    pipeline = make_pipeline(sensor_id)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        cap.release()
        return None

    return cap


def detect_qr(frame, detector, sensor_name):
    data, points, _ = detector.detectAndDecode(frame)

    if points is not None:
        points = points.astype(int).reshape(-1, 2)

        # Outline the detected QR code
        for i in range(4):
            p1 = tuple(points[i])
            p2 = tuple(points[(i + 1) % 4])
            cv2.line(frame, p1, p2, (0, 255, 0), 3)

        # Mark the QR center
        center_x = int(points[:, 0].mean())
        center_y = int(points[:, 1].mean())

        cv2.circle(
            frame, (center_x, center_y), 7, (0, 0, 255), -1
        )

        cv2.putText(
            frame,
            f"Center: ({center_x}, {center_y})",
            (20, 85),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 255),
            2,
        )

        if data:
            cv2.putText(
                frame,
                "QR: " + data[:35],
                (20, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2,
            )

    cv2.putText(
        frame,
        sensor_name,
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (255, 255, 0),
        2,
    )

    return frame, data


def main():
    print("Opening left camera sensor 0...")
    left_cap = open_camera(0)

    if left_cap is None:
        print("ERROR: Could not open sensor 0.")
        sys.exit(1)

    print("Opening right camera sensor 1...")
    right_cap = open_camera(1)

    if right_cap is None:
        left_cap.release()
        print("ERROR: Could not open sensor 1 at the same time.")
        print("The sensors work individually; simultaneous streaming failed.")
        sys.exit(1)

    left_detector = cv2.QRCodeDetector()
    right_detector = cv2.QRCodeDetector()

    last_left = None
    last_right = None

    print("Both cameras are running.")
    print("Show the same QR code to both cameras.")
    print("Press Q to quit.")

    try:
        while True:
            left_ok, left_frame = left_cap.read()
            right_ok, right_frame = right_cap.read()

            if not left_ok or not right_ok:
                print("ERROR: Could not read both camera feeds.")
                break

            left_frame, left_data = detect_qr(
                left_frame, left_detector, "SENSOR 0"
            )
            right_frame, right_data = detect_qr(
                right_frame, right_detector, "SENSOR 1"
            )

            # Print QR contents only when they change
            if left_data and left_data != last_left:
                print("Sensor 0 QR:", left_data)
            if right_data and right_data != last_right:
                print("Sensor 1 QR:", right_data)

            last_left = left_data
            last_right = right_data

            # Resize for a side-by-side display
            left_small = cv2.resize(left_frame, (640, 360))
            right_small = cv2.resize(right_frame, (640, 360))

            combined = cv2.hconcat([left_small, right_small])

            cv2.imshow("Forklift Stereo Vision", combined)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        left_cap.release()
        right_cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
