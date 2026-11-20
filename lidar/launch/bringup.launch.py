from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    project_dir = Path.home() / "Desktop" / "ForkliftRobot"

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            "/home/vishwa/ros2_ws/install/sllidar_ros2/share/"
            "sllidar_ros2/launch/sllidar_c1_launch.py"
        )
    )

    transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laser_static_transform",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "base_link",
            "--child-frame-id", "laser",
        ],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            str(project_dir / "config" / "lidar" / "lidar_view.rviz"),
        ],
        output="screen",
    )

    return LaunchDescription([
        lidar_launch,
        transform_node,
        rviz_node,
    ])