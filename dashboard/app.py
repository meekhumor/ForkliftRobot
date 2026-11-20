import os
import signal
import subprocess
import threading
import shlex
import json
import math
import time
import urllib.request
import urllib.error
import urllib.parse
import collections
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

# ESP32 Motor Controller Configuration (communicates over phone hotspot)
esp32_config = {
    "ip": os.environ.get("ESP32_IP", "172.19.93.232"),
    "port": int(os.environ.get("ESP32_PORT", 80)),
    "online": False,
    "last_check": 0,
    "telemetry": {}
}

# Fixed ROS processes
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
    "localization": (
        f"source {ROS_SETUP} && source {WS_SETUP} && "
        f"exec ros2 launch {PROJECT}/lidar/launch/localization.launch.py "
        f"map:={PROJECT}/maps/forklift_map.yaml "
        f"params_file:={PROJECT}/config/navigation/nav2_params.yaml"
    ),
    "navigation": (
        f"source {ROS_SETUP} && source {WS_SETUP} && "
        f"exec ros2 launch {PROJECT}/lidar/launch/navigation.launch.py"
    ),
    "bridge": (
        f"source {ROS_SETUP} && source {WS_SETUP} && "
        f"exec python3 {PROJECT}/forklift_bridge/forklift_bridge/cmd_vel_to_esp32.py --ros-args -p esp32_ip:={esp32_config['ip']}"
    ),
    "rosbridge": (
        f"source {ROS_SETUP} && "
        "exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
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

    # Open in write mode so each run starts with clean logs (avoiding stale errors from past runs)
    log_file = open(LOG_DIR / f"{name}.log", "w", buffering=1)

    # Dynamic command adjustment for bridge with current ESP32 IP
    cmd = COMMANDS[name]
    if name == "bridge":
        cmd = f"source {ROS_SETUP} && source {WS_SETUP} && exec python3 {PROJECT}/forklift_bridge/forklift_bridge/cmd_vel_to_esp32.py --ros-args -p esp32_ip:={esp32_config['ip']}"
    elif name == "localization":
        # Clean any orphaned map_server or amcl nodes from previous crashed runs to prevent lifecycle conflicts
        try:
            subprocess.run(["pkill", "-9", "-f", "nav2_map_server|nav2_amcl|lifecycle_manager_localization"], stderr=subprocess.DEVNULL)
            time.sleep(0.3)
        except Exception:
            pass

    proc = subprocess.Popen(
        ["bash", "-lc", cmd],
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
        for name in ("rosbridge", "lidar", "transform", "odometry", "slam")
    }


def nav2_status():
    return {
        name: process_running(name)
        for name in ("rosbridge", "lidar", "transform", "odometry", "localization", "navigation", "bridge")
    }


# ---------------- ESP32 Communication Helpers ----------------

def send_esp32_cmd(action, speed=120, timeout=0.35):
    """Send manual movement / lift action to the ESP32."""
    url = f"http://{esp32_config['ip']}:{esp32_config['port']}/cmd?a={urllib.parse.quote(str(action))}&s={int(speed)}"
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "JetsonDashboard/1.0"})
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            esp32_config["online"] = True
            return True, resp.read().decode("utf-8", errors="ignore")
    except Exception as e:
        esp32_config["online"] = False
        return False, str(e)


def send_esp32_drive(left, right, timeout=0.35):
    """Send continuous left/right speed to the ESP32."""
    url = f"http://{esp32_config['ip']}:{esp32_config['port']}/drive?l={int(left)}&r={int(right)}"
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "JetsonDashboard/1.0"})
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            esp32_config["online"] = True
            return True, resp.read().decode("utf-8", errors="ignore")
    except Exception as e:
        esp32_config["online"] = False
        return False, str(e)


def poll_esp32_status(timeout=0.4):
    """Check ESP32 connection and retrieve telemetry."""
    now = time.time()
    if now - esp32_config["last_check"] < 2.0:
        return esp32_config["online"]

    esp32_config["last_check"] = now
    url = f"http://{esp32_config['ip']}:{esp32_config['port']}/status"
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "JetsonDashboard/1.0"})
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            data = json.loads(resp.read().decode("utf-8", errors="ignore"))
            esp32_config["online"] = True
            esp32_config["telemetry"] = data
            return True
    except Exception:
        esp32_config["online"] = False
        return False


# ---------------- Web Routes ----------------

@app.route("/")
def home():
    return render_template("index.html")


@app.route("/health")
def health():
    return jsonify(status="online", project="Forklift Robot Control Center")


@app.route("/api/config", methods=["GET", "POST"])
def api_config():
    """Get or update ESP32 controller IP address."""
    if request.method == "POST":
        data = request.get_json() or {}
        new_ip = data.get("ip") or request.form.get("ip")
        new_port = data.get("port") or request.form.get("port")
        if new_ip:
            esp32_config["ip"] = str(new_ip).strip()
        if new_port:
            esp32_config["port"] = int(new_port)
        poll_esp32_status(timeout=0.6)

    return jsonify(
        ip=esp32_config["ip"],
        port=esp32_config["port"],
        online=esp32_config["online"],
        telemetry=esp32_config["telemetry"]
    )


@app.route("/api/status")
def api_status():
    with lock:
        if not process_running("rosbridge"):
            try:
                start_process("rosbridge")
            except Exception:
                pass
        m_state = mapping_status()
        n_state = nav2_status()
        r_running = process_running("rosbridge")

    # Async check ESP32 status
    poll_esp32_status()

    return jsonify(
        status="online",
        mapping=m_state["slam"],
        navigating=n_state["navigation"],
        rosbridge=r_running,
        esp32_online=esp32_config["online"],
        esp32_ip=esp32_config["ip"],
        processes={**m_state, **n_state, "rosbridge": r_running},
        telemetry=esp32_config["telemetry"],
    )


@app.route("/api/motor", methods=["GET", "POST"])
def api_motor():
    """Relay RC buttons / speed adjustments from the Jetson web UI to the ESP32."""
    action = request.args.get("a")
    speed = request.args.get("s", 120)
    left = request.args.get("l")
    right = request.args.get("r")

    if request.is_json:
        body = request.get_json()
        action = action or body.get("action") or body.get("a")
        speed = body.get("speed") or body.get("s") or speed
        left = left or body.get("left") or body.get("l")
        right = right or body.get("right") or body.get("r")

    if left is not None and right is not None:
        ok, msg = send_esp32_drive(left, right)
        return jsonify(success=ok, message=msg)

    if not action:
        return jsonify(success=False, error="Missing action 'a' or drive values 'l','r'"), 400

    ok, msg = send_esp32_cmd(action, speed)
    return jsonify(success=ok, message=msg)


@app.route("/api/estop", methods=["POST"])
def api_estop():
    """Emergency Stop: immediately halts all ESP32 motors and stops ROS movement."""
    # 1. Direct hardware stop on ESP32
    ok, msg = send_esp32_cmd("X")

    # 2. Publish zero cmd_vel in ROS
    stop_cmd = (
        f"source {ROS_SETUP} && "
        "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'"
    )
    subprocess.Popen(["bash", "-lc", stop_cmd], start_new_session=True)

    return jsonify(success=True, message="EMERGENCY STOP dispatched to all systems")


# ---------------- Mapping Controls ----------------

@app.route("/api/mapping/start", methods=["POST"])
def mapping_start():
    order = ("rosbridge", "lidar", "transform", "odometry", "slam")

    with lock:
        try:
            for name in order:
                start_process(name)
                if processes[name].poll() is not None:
                    raise RuntimeError(f"{name} exited during startup")
                threading.Event().wait(1.5)

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
            message="Mapping processes stopped",
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


# ---------------- System Logs Viewer API ----------------

@app.route("/api/logs", methods=["GET"])
def api_logs():
    """Retrieve the latest lines of a process log file."""
    name = request.args.get("name", "slam").strip()
    lines_count = int(request.args.get("lines", 120))
    lines_count = min(max(lines_count, 10), 1000)

    # Sanitize process name
    safe_name = os.path.basename(name).replace("..", "").replace(".log", "")
    log_file = LOG_DIR / f"{safe_name}.log"

    if not log_file.exists():
        return jsonify(
            success=False,
            name=safe_name,
            error=f"Log file '{safe_name}.log' not found.",
            content=f"Log file '{safe_name}.log' does not exist yet. It will be generated when '{safe_name}' is started.",
            lines=0
        ), 404

    try:
        with open(log_file, "r", encoding="utf-8", errors="replace") as f:
            last_lines = collections.deque(f, maxlen=lines_count)
        return jsonify(
            success=True,
            name=safe_name,
            content="".join(last_lines),
            lines=len(last_lines)
        )
    except Exception as exc:
        return jsonify(success=False, name=safe_name, error=str(exc), content=""), 500


@app.route("/api/logs/list", methods=["GET"])
def api_logs_list():
    """List available log files."""
    files = sorted([f.stem for f in LOG_DIR.glob("*.log")])
    return jsonify(success=True, logs=files)


# ---------------- Nav2 Autonomous Navigation ----------------

@app.route("/api/nav2/start", methods=["POST"])
def nav2_start():
    order = ("rosbridge", "lidar", "transform", "odometry", "localization", "navigation", "bridge")

    with lock:
        if process_running("slam"):
            stop_process("slam")

        try:
            for name in order:
                start_process(name)
                if processes[name].poll() is not None:
                    raise RuntimeError(f"{name} exited during startup")
                threading.Event().wait(1.5)

            return jsonify(
                success=True,
                message="Autonomous navigation stack started successfully",
                processes=nav2_status(),
            )
        except Exception as exc:
            for name in reversed(order):
                stop_process(name)
            return jsonify(success=False, error=str(exc)), 500


@app.route("/api/nav2/stop", methods=["POST"])
def nav2_stop():
    with lock:
        for name in ("bridge", "navigation", "localization"):
            stop_process(name)

        return jsonify(
            success=True,
            message="Autonomous navigation stack stopped",
            processes=nav2_status(),
        )


@app.route("/api/navigate", methods=["POST"])
def api_navigate():
    """Dispatch navigation goals to Nav2 via NavigateToPose action."""
    try:
        data = request.get_json() or {}
        x = float(data.get("x", 0.0))
        y = float(data.get("y", 0.0))
        yaw = float(data.get("yaw", 0.0))

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        goal_msg = {
            "pose": {
                "header": {"frame_id": "map"},
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

        subprocess.Popen(
            ["bash", "-lc", command],
            cwd=str(PROJECT),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )

        return jsonify(success=True, message=f"Navigation goal dispatched to X: {x:.2f}, Y: {y:.2f}, Yaw: {yaw:.2f}")

    except Exception as exc:
        return jsonify(success=False, error=str(exc)), 500


@app.route("/api/navigate/cancel", methods=["POST"])
def api_navigate_cancel():
    """Halt navigation and stop motors."""
    send_esp32_cmd("S")

    stop_cmd = (
        f"source {ROS_SETUP} && "
        "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'"
    )
    return jsonify(success=True, message="Navigation cancelled")


@app.route("/api/initialpose", methods=["POST"])
def api_initialpose():
    """Publish an initial pose estimate to /initialpose for AMCL relocalization."""
    try:
        data = request.get_json() or {}
        x = float(data.get("x", 0.0))
        y = float(data.get("y", 0.0))
        yaw = float(data.get("yaw", 0.0))

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        msg = {
            "header": {"frame_id": "map"},
            "pose": {
                "pose": {
                    "position": {"x": x, "y": y, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw}
                },
                "covariance": [
                    0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.06
                ]
            }
        }
        yaml_arg = shlex.quote(json.dumps(msg))
        cmd = f"source {ROS_SETUP} && ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped {yaml_arg}"
        subprocess.Popen(["bash", "-lc", cmd], start_new_session=True)
        return jsonify(success=True, message=f"Initial pose broadcast to X: {x:.2f}, Y: {y:.2f}")
    except Exception as exc:
        return jsonify(success=False, error=str(exc)), 500


if __name__ == "__main__":
    try:
        start_process("rosbridge")
    except Exception as exc:
        print(f"Notice: rosbridge auto-start: {exc}")
    print("Forklift Robot Complete Control Center starting on port 8000...")
    app.run(host="0.0.0.0", port=8000, debug=False)