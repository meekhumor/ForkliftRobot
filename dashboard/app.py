import os
import signal
import subprocess
import threading
import shlex
import json
import math

from pathlib import Path

from flask import Flask, jsonify, render_template, request

PROJECT = Path.home() / "Desktop" / "ForkliftRobot"
LOG_DIR = PROJECT / "logs"
MAP_DIR = PROJECT / "maps"
ROS_SETUP = "/opt/ros/jazzy/setup.bash"
WS_SETUP = str(Path.home() / "ros2_ws" / "install" / "setup.bash")

LOG_DIR.mkdir(parents=True, exist_ok=True)
MAP_DIR.mkdir(parents=True, exist_ok=True)

app = Flask(__name__, template_folder="templates", static_folder="static")

# These are fixed commands; the browser cannot submit shell commands.
COMMANDS = {
    "lidar": (
        f"source {ROS_SETUP} && source {WS_SETUP} && "
        "exec ros2 launch sllidar_ros2 sllidar_c1_launch.py"
    ),
    "transform": (
        f"source {ROS_SETUP} && "
        "exec ros2 run tf2_ros static_transform_publisher "
        "0 0 0 0 0 0 base_link laser"
    ),
    "odometry": (
        f"source {ROS_SETUP} && source {WS_SETUP} && "
        "exec ros2 launch rf2o_laser_odometry "
        "rf2o_laser_odometry.launch.py"
    ),
    "slam": (
        f"source {ROS_SETUP} && source {WS_SETUP} && "
        "exec ros2 launch slam_toolbox online_async_launch.py "
        f"slam_params_file:={PROJECT}/config/forklift_slam.yaml "
        "use_sim_time:=false"
    ),
}

processes = {}
log_handles = {}
lock = threading.RLock()


def process_running(name):
    proc = processes.get(name)
    return proc is not None and proc.poll() is None


def start_process(name):
    if process_running(name):
        return

    log_file = open(LOG_DIR / f"{name}.log", "a", buffering=1)

    proc = subprocess.Popen(
        ["bash", "-lc", COMMANDS[name]],
        cwd=str(PROJECT),
        stdout=log_file,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )

    processes[name] = proc
    log_handles[name] = log_file


def stop_process(name):
    proc = processes.get(name)

    if proc is None:
        return

    if proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass

    handle = log_handles.pop(name, None)
    if handle:
        handle.close()

    processes.pop(name, None)


def mapping_status():
    return {
        name: process_running(name)
        for name in ("lidar", "transform", "odometry", "slam")
    }


@app.route("/")
def home():
    return render_template("index.html")


@app.route("/health")
def health():
    return jsonify(status="online", project="Forklift Robot Control Center")


@app.route("/api/status")
def api_status():
    with lock:
        state = mapping_status()

    return jsonify(
        status="online",
        mapping=any(state.values()),
        processes=state,
    )


@app.route("/api/mapping/start", methods=["POST"])
def mapping_start():
    order = ("lidar", "transform", "odometry", "slam")

    with lock:
        try:
            for name in order:
                start_process(name)
                # Give each ROS process a moment to initialize.
                if processes[name].poll() is not None:
                    raise RuntimeError(f"{name} exited during startup")

                threading.Event().wait(2)

            return jsonify(
                success=True,
                message="Mapping processes started",
                processes=mapping_status(),
            )

        except Exception as exc:
            for name in reversed(order):
                stop_process(name)

            return jsonify(success=False, error=str(exc)), 500


@app.route("/api/mapping/stop", methods=["POST"])
def mapping_stop():
    with lock:
        for name in ("slam", "odometry", "transform", "lidar"):
            stop_process(name)

        return jsonify(
            success=True,
            message="Dashboard-managed mapping processes stopped",
            processes=mapping_status(),
        )


@app.route("/api/map/save", methods=["POST"])
def map_save():
    output = MAP_DIR / "forklift_map"

    command = (
        f"source {ROS_SETUP} && source {WS_SETUP} && "
        f"ros2 run nav2_map_server map_saver_cli -f {output}"
    )

    try:
        result = subprocess.run(
            ["bash", "-lc", command],
            cwd=str(PROJECT),
            capture_output=True,
            text=True,
            timeout=90,
        )

        if result.returncode != 0:
            return jsonify(
                success=False,
                error=(result.stderr or result.stdout)[-3000:],
            ), 500

        return jsonify(
            success=True,
            message="Map saved",
            files=[
                str(output) + ".yaml",
                str(output) + ".pgm",
            ],
        )

    except subprocess.TimeoutExpired:
        return jsonify(success=False, error="Map save timed out"), 504


@app.route("/api/navigate", methods=["POST"])
def api_navigate():
    """Securely dispatch navigation goals to Nav2 via ros2 action send_goal."""
    try:
        data = request.get_json() or {}
        x = float(data.get("x", 0.0))
        y = float(data.get("y", 0.0))
        yaw = float(data.get("yaw", 0.0))

        # Convert yaw to quaternion (qz, qw) for 2D planar rotation
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        # Construct the Nav2 goal YAML structure safely using json
        goal_msg = {
            "pose": {
                "header": {
                    "frame_id": "map"
                },
                "pose": {
                    "position": {"x": x, "y": y, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw}
                }
            }
        }
        
        goal_yaml = json.dumps(goal_msg)
        safe_goal_arg = shlex.quote(goal_yaml)

        command = (
            f"source {ROS_SETUP} && source {WS_SETUP} && "
            f"ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose {safe_goal_arg}"
        )

        # Dispatch asynchronously so the web request doesn't block waiting for navigation to complete
        subprocess.Popen(
            ["bash", "-lc", command],
            cwd=str(PROJECT),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )

        return jsonify(success=True, message=f"Navigation goal sent to X: {x}, Y: {y}, Yaw: {yaw}")

    except Exception as exc:
        return jsonify(success=False, error=str(exc)), 500


if __name__ == "__main__":
    print("Forklift Robot Dashboard starting on port 8000")
    app.run(host="0.0.0.0", port=8000, debug=False)