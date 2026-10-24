from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("odrive_ros2"),
        "config",
        "odrive_params.yaml",
    )

    return LaunchDescription([
        Node(
            package="odrive_ros2",
            executable="odrive_node.py",
            name="odrive_node",
            output="screen",
            parameters=[config],
        )
    ])
