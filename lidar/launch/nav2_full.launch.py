from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    project_dir = Path.home() / "Desktop" / "ForkliftRobot"
    launch_dir = project_dir / "lidar" / "launch"
    default_map = str(project_dir / "maps" / "forklift_map.yaml")
    default_params = str(project_dir / "config" / "navigation" / "nav2_params.yaml")

    map_arg = DeclareLaunchArgument("map", default_value=default_map)
    params_arg = DeclareLaunchArgument("params_file", default_value=default_params)
    autostart_arg = DeclareLaunchArgument("autostart", default_value="true")

    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "localization.launch.py")),
        launch_arguments={
            "map": map_file,
            "params_file": params_file,
            "autostart": autostart,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "navigation.launch.py")),
        launch_arguments={
            "params_file": params_file,
            "autostart": autostart,
        }.items(),
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        autostart_arg,
        localization,
        navigation,
    ])
