# two_clearpath_robots.launch.py  (RTAB-Map + Nav2, 2 robots)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ----- inputs -----
    setup_path_r1 = LaunchConfiguration("setup_path_r1")
    setup_path_r2 = LaunchConfiguration("setup_path_r2")
    world = LaunchConfiguration("world")

    use_sim_time = LaunchConfiguration("use_sim_time")

    ns1 = LaunchConfiguration("ns1")
    ns2 = LaunchConfiguration("ns2")

    db1 = LaunchConfiguration("db1")
    db2 = LaunchConfiguration("db2")

    localization = LaunchConfiguration("localization")
    rtabmap_viz1 = LaunchConfiguration("rtabmap_viz1")
    rtabmap_viz2 = LaunchConfiguration("rtabmap_viz2")

    x1 = LaunchConfiguration("x1")
    y1 = LaunchConfiguration("y1")
    yaw1 = LaunchConfiguration("yaw1")

    x2 = LaunchConfiguration("x2")
    y2 = LaunchConfiguration("y2")
    yaw2 = LaunchConfiguration("yaw2")

    # ----- Clearpath Gazebo -----
    sim_launch = PathJoinSubstitution([FindPackageShare("clearpath_gz"), "launch", "simulation.launch.py"])
    spawn_launch = PathJoinSubstitution([FindPackageShare("clearpath_gz"), "launch", "robot_spawn.launch.py"])

    sim_r1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            "setup_path": setup_path_r1,
            "world": world,
            "x": x1,
            "y": y1,
            "yaw": yaw1,
        }.items(),
    )

    spawn_r2 = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(spawn_launch),
                launch_arguments={
                    "setup_path": setup_path_r2,
                    "x": x2,
                    "y": y2,
                    "yaw": yaw2,
                }.items(),
            )
        ],
    )

    # ----- RTAB-Map (your patched clone that supports database_path) -----
    rtabmap_launch = PathJoinSubstitution(
        [FindPackageShare("multirobot_launch"), "launch", "husky_slam2d_clone.launch.py"]
    )

    rtabmap_r1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch),
        launch_arguments={
            "rtabmap_viz": rtabmap_viz1,
            "localization": localization,
            "use_sim_time": use_sim_time,
            "robot_ns": ns1,
            "database_path": db1,
        }.items(),
    )

    rtabmap_r2 = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rtabmap_launch),
                launch_arguments={
                    "rtabmap_viz": rtabmap_viz2,
                    "localization": localization,
                    "use_sim_time": use_sim_time,
                    "robot_ns": ns2,
                    "database_path": db2,
                }.items(),
            )
        ],
    )

    # ----- Nav2 (Clearpath wrapper over nav2_bringup/navigation_launch.py) -----
    # This launch reads setup_path/robot.yaml to determine namespace internally. :contentReference[oaicite:1]{index=1}
    nav2_launch = PathJoinSubstitution(
        [FindPackageShare("clearpath_nav2_demos"), "launch", "nav2.launch.py"]
    )

    nav2_r1 = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    "setup_path": setup_path_r1,
                    "use_sim_time": use_sim_time,
                }.items(),
            )
        ],
    )

    nav2_r2 = TimerAction(
        period=16.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    "setup_path": setup_path_r2,
                    "use_sim_time": use_sim_time,
                }.items(),
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("setup_path_r1", default_value="/home/$USER/clearpath_r1"),
            DeclareLaunchArgument("setup_path_r2", default_value="/home/$USER/clearpath_r2"),
            DeclareLaunchArgument("world", default_value="warehouse_world"),

            DeclareLaunchArgument("use_sim_time", default_value="true"),

            DeclareLaunchArgument("ns1", default_value="a200_0000"),
            DeclareLaunchArgument("ns2", default_value="a200_0001"),

            DeclareLaunchArgument("db1", default_value="/home/$USER/ros2_ws/maps/warehouse_r1.db"),
            DeclareLaunchArgument("db2", default_value="/home/$USER/ros2_ws/maps/warehouse_r2.db"),

            DeclareLaunchArgument("localization", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("rtabmap_viz1", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("rtabmap_viz2", default_value="false", choices=["true", "false"]),

            DeclareLaunchArgument("x1", default_value="0.0"),
            DeclareLaunchArgument("y1", default_value="0.0"),
            DeclareLaunchArgument("yaw1", default_value="0.0"),

            DeclareLaunchArgument("x2", default_value="2.0"),
            DeclareLaunchArgument("y2", default_value="0.0"),
            DeclareLaunchArgument("yaw2", default_value="0.0"),

            sim_r1,
            spawn_r2,
            rtabmap_r1,
            rtabmap_r2,
            nav2_r1,
            nav2_r2,
        ]
    )

