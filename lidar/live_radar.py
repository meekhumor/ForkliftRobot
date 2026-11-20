
import subprocess
import threading
import re
import queue
import math
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Path to the compiled SLAMTEC SDK program
PROGRAM = (
    "/home/vishwa/Desktop/ForkliftRobot/"
    "lidar/rplidar_sdk/output/Linux/Release/ultra_simple"
)

PORT = "/dev/ttyUSB0"
BAUDRATE = "460800"

# Keep recent valid measurements
scan_points = deque(maxlen=1500)
data_lock = threading.Lock()

pattern = re.compile(
    r"theta:\s*([\d.]+)\s+Dist:\s*([\d.]+)\s+Q:\s*(\d+)"
)

def read_lidar():
    command = [
        "stdbuf", "-oL",
        PROGRAM,
        "--channel", "--serial", PORT, BAUDRATE
    ]

    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1
    )

    print("Connecting to RPLIDAR C1...")

    try:
        for line in process.stdout:
            match = pattern.search(line)

            if not match:
                continue

            angle = float(match.group(1))
            distance = float(match.group(2)) / 1000.0
            quality = int(match.group(3))

            # Ignore invalid readings
            if distance <= 0 or quality <= 0:
                continue

            with data_lock:
                scan_points.append((angle, distance))

    except Exception as error:
        print("LiDAR reader error:", error)

    finally:
        process.terminate()


# Start LiDAR reading in background
thread = threading.Thread(target=read_lidar, daemon=True)
thread.start()

# Create radar window
fig = plt.figure(figsize=(9, 9))
ax = fig.add_subplot(111, projection="polar")

ax.set_theta_zero_location("N")
ax.set_theta_direction(-1)
ax.set_ylim(0, 6)

ax.set_title(
    "RPLIDAR C1 - Live 360 Degree Scan",
    pad=25
)

ax.set_rmax(6)
ax.set_rticks([1, 2, 3, 4, 5, 6])
ax.grid(True)

points, = ax.plot(
    [], [],
    "o",
    markersize=3
)

def update(frame):
    with data_lock:
        current_points = list(scan_points)

    if current_points:
        angles = [
            math.radians(point[0])
            for point in current_points
        ]

        distances = [
            point[1]
            for point in current_points
        ]

        points.set_data(angles, distances)

    return points,

ani = FuncAnimation(
    fig,
    update,
    interval=100,
    blit=False,
    cache_frame_data=False
)

print("Live radar started. Close the window to stop.")

try:
    plt.show()
finally:
    print("Radar stopped.")
