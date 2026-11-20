from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    project_dir = Path.home() / "Desktop" / "ForkliftRobot"
    default_map = str(project_dir / "maps" / "forklift_map.yaml")
    default_params = str(project_dir / "config" / "navigation" / "nav2_params.yaml")

    map_arg = DeclareLaunchArgument("map", default_value=default_map)
    params_arg = DeclareLaunchArgument("params_file", default_value=default_params)
    autostart_arg = DeclareLaunchArgument("autostart", default_value="true")

    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={"yaml_filename": map_file, "use_sim_time": "false"},
        convert_types=True,
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            configured_params,
            {"yaml_filename": map_file},
        ],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[configured_params],
        remappings=[("scan", "/scan")],
    )

    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"autostart": autostart},
            {"node_names": ["map_server", "amcl"]},
        ],
    )

    return LaunchDescription([map_arg, params_arg, autostart_arg, map_server, amcl, lifecycle])
