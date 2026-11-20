from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    project_dir = Path.home() / "Desktop" / "ForkliftRobot"
    config_file = project_dir / "config" / "odometry" / "ekf.yaml"

    return LaunchDescription([
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[str(config_file)],
        )
    ])