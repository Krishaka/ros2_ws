import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, AppendEnvironmentVariable
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('warehouse_sim')

    set_gz_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_share, 'models')
    )

    world_path = PathJoinSubstitution([
        FindPackageShare('warehouse_sim'),
        'worlds',
        'warehouse_world.sdf',
    ])

    gz_cmd = [
        TextSubstitution(text='ign'),
        TextSubstitution(text='gazebo'),
        TextSubstitution(text='-r'),
        world_path,
    ]

    # Parameter bridge arguments to connect Ignition<->ROS for diff_robot
    bridge_args = [
        '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
        '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
        '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',

        # camera (image + camera_info) and depth (adjusted to your SDF/world paths)
        '/world/world_demo/model/diff_robot/link/camera/sensor/camera_front_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
        '/world/world_demo/model/diff_robot/link/camera/sensor/camera_front_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
        '/world/world_demo/model/diff_robot/link/camera/sensor/camera_front_camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',

        # LIDAR — your model2.sdf sets <topic>/scan</topic>, so bridge the short name '/scan'
        '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
    ]

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_parameter_bridge',
        output='screen',
        arguments=bridge_args
    )

    return LaunchDescription([
        set_gz_resources,
        ExecuteProcess(cmd=gz_cmd, output='screen'),
        bridge_node,
    ])
