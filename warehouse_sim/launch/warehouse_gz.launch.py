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
    '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
    '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
    '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',

        # NEW: bridge simulation clock
        '/world/world_demo/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',

        # camera (image + camera_info + depth)
        '/world/world_demo/model/diff_robot/link/camera/sensor/camera_front_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
        '/world/world_demo/model/diff_robot/link/camera/sensor/camera_front_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        '/world/world_demo/model/diff_robot/link/camera/sensor/camera_front_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
        # lidar (2D)
        #'/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
        #'/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',

        # NEW: 3D lidar point cloud (Gazebo -> ROS PointCloud2)
        #'/lidar3d/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        "/lidar3d/points/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked",
       
        # IMU manual-bridge style (one-way)
        '/world/world_demo/model/diff_robot/link/imu_sensor/sensor/imu/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
    ]


    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_parameter_bridge',
        output='screen',
        arguments=bridge_args,
        remappings=[
            ('/scan', '/diff_robot/scan'),
            ('/scan/points', '/diff_robot/scan/points'),
            ('/lidar3d/points', '/diff_robot/lidar3d/points'),
            ('/world/world_demo/clock', '/clock')
        ]
    )

    return LaunchDescription([
        set_gz_resources,
        ExecuteProcess(cmd=gz_cmd, output='screen'),
        bridge_node,
    ])