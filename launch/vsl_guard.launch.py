"""Launch vsl_guard with a parameters file (default: the package's nav2_guard.yaml).

    ros2 launch vsl_ros2 vsl_guard.launch.py params_file:=/etc/robot/vsl_guard.yaml

Include it from your Nav2 bringup so the guard starts with the stack it guards.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory("vsl_ros2"), "config", "nav2_guard.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params, description="vsl_guard parameters (YAML)"),
            DeclareLaunchArgument("namespace", default_value="", description="namespace for the guard node"),
            Node(
                package="vsl_ros2",
                executable="vsl_guard",
                name="vsl_guard",
                namespace=LaunchConfiguration("namespace"),
                parameters=[LaunchConfiguration("params_file")],
                output="screen",
                respawn=True,  # a restart keeps the Terminal State: the latch is on disk
                respawn_delay=2.0,
            ),
        ]
    )
