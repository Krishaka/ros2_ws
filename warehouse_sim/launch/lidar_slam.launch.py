import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('warehouse_sim')

    # Start Gazebo + parameter bridge (your existing launch)
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'warehouse_gz.launch.py')
        )
    )

    # Convert 3D point cloud -> 2D LaserScan
    pcd_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pcd_to_scan',
        remappings=[
            ('cloud_in', '/diff_robot/lidar3d/points'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'output_frame': 'base_link',   # or 'lidar3d_link' if you prefer
            'range_min': 0.1,
            'range_max': 15.0,
            'angle_min': -3.14159,
            'angle_max':  3.14159,
            'use_inf': True,
        }]
    )

    # SLAM Toolbox node
    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')],
        output='screen'
    )

    return LaunchDescription([
        gz_launch,
        pcd_to_scan,
        slam,
    ])

