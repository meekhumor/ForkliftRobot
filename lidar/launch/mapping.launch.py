from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    project_dir = Path.home() / "Desktop" / "ForkliftRobot"
    config_file = project_dir / "config" / "lidar" / "mapper_params.yaml"

    return LaunchDescription([
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[str(config_file)],
        )
    ])