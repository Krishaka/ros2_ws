from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    base_to_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_basefoot_to_baselink",
        arguments=["0", "0", "0.5", "0", "0", "0", "base_footprint", "base_link"],
    )

    link_to_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_baselink_to_lidar3d",
        arguments=["0", "0", "0.8569669", "0", "0", "0", "base_link", "lidar3d_link"],
    )

    return LaunchDescription([base_to_link, link_to_lidar])

