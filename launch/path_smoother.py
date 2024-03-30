import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'path_smoother'
    # simulator_package = 'arcanain_simulator'
    # rviz_file_name = "path_smoother.rviz"

    # file_path = os.path.expanduser('~/ros2_ws/src/arcanain_simulator/urdf/mobile_robot.urdf.xml')

    # with open(file_path, 'r') as file:
    #     robot_description = file.read()

    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare(package_name), "rviz", rviz_file_name]
    # )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    # )

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
        # rviz_node,
        #path_smoother_node,
        path_publisher_node,
    ]

    return LaunchDescription(nodes)
