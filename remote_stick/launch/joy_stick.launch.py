
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    joy_stick_publisher_node = Node(
        package="remote_stick",
        executable="joy_stick",
        output="screen"
    )
    nodes = [
        joy_stick_publisher_node
    ]

    return LaunchDescription(declared_arguments + nodes)
