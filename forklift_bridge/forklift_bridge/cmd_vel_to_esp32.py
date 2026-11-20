#!/usr/bin/env python3
"""
ROS 2 to ESP32 Velocity Bridge Node
Subscribes to /cmd_vel and sends differential wheel speeds to the ESP32
over the phone hotspot via HTTP.
"""

import time
import threading
import requests
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class CmdVelToESP32(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_esp32')

        self.declare_parameter('esp32_ip', '172.19.93.232')
        self.declare_parameter('esp32_port', 80)
        self.declare_parameter('wheel_separation', 0.20)  # track width in meters
        self.declare_parameter('max_linear_speed', 0.30)  # m/s
        self.declare_parameter('max_angular_speed', 1.00) # rad/s
        self.declare_parameter('min_pwm', 40)             # deadband threshold
        self.declare_parameter('max_pwm', 255)            # max motor pwm
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('timeout_sec', 0.40)       # failsafe timeout

        self.esp32_ip = self.get_parameter('esp32_ip').value
        self.esp32_port = self.get_parameter('esp32_port').value
        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.max_lin = self.get_parameter('max_linear_speed').value
        self.min_pwm = self.get_parameter('min_pwm').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.rate_hz = self.get_parameter('rate_hz').value
        self.timeout_sec = self.get_parameter('timeout_sec').value

        self.drive_endpoint = f"http://{self.esp32_ip}:{self.esp32_port}/drive"
        self.cmd_endpoint = f"http://{self.esp32_ip}:{self.esp32_port}/cmd"

        self.get_logger().info(f"ESP32 Bridge initialized -> {self.drive_endpoint}")

        self.target_vx = 0.0
        self.target_wz = 0.0
        self.last_cmd_time = time.time()
        self.last_sent_l = 0
        self.last_sent_r = 0
        self.last_send_time = 0.0
        self.last_error_time = 0.0
        self.is_stopped = True

        self.session = requests.Session()
        adapter = requests.adapters.HTTPAdapter(pool_connections=2, pool_maxsize=4, max_retries=0)
        self.session.mount("http://", adapter)
        self.http_pending = False
        self.last_config_check = 0.0

        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.estop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.estop_callback,
            10
        )

        timer_period = 1.0 / self.rate_hz
        self.timer = self.create_timer(timer_period, self.control_loop)

    def sync_esp32_ip(self):
        """Check if user updated ESP32 IP in dashboard web UI."""
        try:
            r = requests.get("http://127.0.0.1:8000/api/config", timeout=0.25)
            if r.status_code == 200:
                cfg = r.json()
                new_ip = cfg.get("ip")
                if new_ip and new_ip != self.esp32_ip:
                    self.esp32_ip = new_ip
                    self.drive_endpoint = f"http://{self.esp32_ip}:{self.esp32_port}/drive"
                    self.cmd_endpoint = f"http://{self.esp32_ip}:{self.esp32_port}/cmd"
                    self.get_logger().info(f"Updated ESP32 IP from dashboard -> {self.drive_endpoint}")
        except Exception:
            pass

    def cmd_vel_callback(self, msg: Twist):
        self.target_vx = msg.linear.x
        self.target_wz = msg.angular.z
        self.last_cmd_time = time.time()

    def estop_callback(self, msg: Bool):
        if msg.data:
            self.target_vx = 0.0
            self.target_wz = 0.0
            try:
                self.session.get(f"{self.cmd_endpoint}?a=X", timeout=0.3)
            except Exception:
                pass

    def vel_to_pwm(self, vel: float) -> int:
        if abs(vel) < 0.005:
            return 0
        sign = 1 if vel > 0 else -1
        ratio = min(1.0, abs(vel) / max(self.max_lin, 0.05))
        pwm = self.min_pwm + ratio * (self.max_pwm - self.min_pwm)
        return int(sign * round(max(self.min_pwm, min(self.max_pwm, pwm))))

    def _async_send(self, url, pwm_l, pwm_r, send_time):
        try:
            resp = self.session.get(url, timeout=(0.35, 0.35))
            if resp.status_code == 200:
                self.last_sent_l = pwm_l
                self.last_sent_r = pwm_r
                self.last_send_time = send_time
                self.is_stopped = (pwm_l == 0 and pwm_r == 0)
        except Exception as e:
            now = time.time()
            if (now - self.last_error_time) > 2.0:
                self.get_logger().warn(f"ESP32 ({self.esp32_ip}) communication timeout: {e}")
                self.last_error_time = now
                self.sync_esp32_ip()
        finally:
            self.http_pending = False

    def control_loop(self):
        now = time.time()

        # Periodically re-sync ESP32 IP if there were errors
        if (now - self.last_config_check) > 4.0:
            self.last_config_check = now
            if (now - self.last_error_time) < 10.0:
                self.sync_esp32_ip()

        if (now - self.last_cmd_time) > self.timeout_sec:
            vx = 0.0
            wz = 0.0
        else:
            vx = self.target_vx
            wz = self.target_wz

        # Differential kinematics
        v_left = vx - (wz * self.wheel_sep / 2.0)
        v_right = vx + (wz * self.wheel_sep / 2.0)

        pwm_l = self.vel_to_pwm(v_left)
        pwm_r = self.vel_to_pwm(v_right)

        speed_changed = (pwm_l != self.last_sent_l) or (pwm_r != self.last_sent_r)
        heartbeat_needed = (pwm_l != 0 or pwm_r != 0) and ((now - self.last_send_time) > 0.25)
        stop_confirmation = (pwm_l == 0 and pwm_r == 0) and not self.is_stopped

        if (speed_changed or heartbeat_needed or stop_confirmation) and not self.http_pending:
            url = f"{self.drive_endpoint}?l={pwm_l}&r={pwm_r}"
            self.http_pending = True
            threading.Thread(target=self._async_send, args=(url, pwm_l, pwm_r, now), daemon=True).start()

    def destroy_node(self):
        try:
            self.session.get(f"{self.drive_endpoint}?l=0&r=0", timeout=0.3)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToESP32()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
