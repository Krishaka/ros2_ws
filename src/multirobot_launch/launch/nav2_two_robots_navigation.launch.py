from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def nav2_for(ns: str, params_file):
    nav2_launch = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
    )

    # Remap controller output to Clearpath driver topic inside each namespace:
    #   /a200_0000/cmd_vel -> /a200_0000/platform/cmd_vel_unstamped
    return GroupAction([
        SetRemap(src="cmd_vel", dst="platform/cmd_vel_unstamped"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                "namespace": ns,
                "use_namespace": "true",
                "use_sim_time": "true",
                "autostart": "true",
                "params_file": params_file,
            }.items(),
        ),
    ])


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value="/home/sathvik/ros2_ws/src/multirobot_launch/config/nav2_common.yaml",
            description="Params file with a200_0000: and a200_0001: root keys",
        ),
        nav2_for("a200_0000", params_file),
        nav2_for("a200_0001", params_file),
    ])

