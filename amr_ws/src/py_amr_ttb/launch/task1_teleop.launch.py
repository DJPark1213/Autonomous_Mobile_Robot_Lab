"""Launch task1_teleop."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_amr_ttb',
            executable='task1_teleop',
            name='task1_teleop',
            output='screen',
        ),
    ])