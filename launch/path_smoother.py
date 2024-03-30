import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'path_smoother'

    path_smoother_node = Node(
        package=package_name,
        executable='cubic_spline_node',
        output="screen",
    )

    path_publisher_node = Node(
        package=package_name,
        executable='path_publisher',
        output="screen",
    )

    nodes = [
        path_smoother_node,
        #path_publisher_node,
    ]

    return LaunchDescription(nodes)
