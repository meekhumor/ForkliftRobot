from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    project_dir = Path.home() / "Desktop" / "ForkliftRobot"
    default_params = str(project_dir / "config" / "navigation" / "nav2_params.yaml")

    params_arg = DeclareLaunchArgument("params_file", default_value=default_params)
    autostart_arg = DeclareLaunchArgument("autostart", default_value="true")

    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key="",
            param_rewrites={"use_sim_time": "false"},
            convert_types=True,
        ),
        allow_substs=True,
    )

    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[configured_params],
        remappings=[("cmd_vel", "cmd_vel_nav")],
    )

    smoother = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[configured_params],
    )

    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[configured_params],
    )

    behaviors = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[configured_params],
        remappings=[("cmd_vel", "cmd_vel_nav")],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[configured_params],
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[configured_params],
    )

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[configured_params],
        remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "/cmd_vel")],
    )

    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"autostart": autostart},
            {"node_names": lifecycle_nodes},
            {"bond_timeout": 8.0},
            {"attempt_respawn_reconnection": True},
        ],
    )

    return LaunchDescription([
        params_arg,
        autostart_arg,
        controller,
        smoother,
        planner,
        behaviors,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle,
    ])
