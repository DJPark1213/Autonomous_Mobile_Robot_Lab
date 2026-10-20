"""Launch the square navigation task."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_amr_ttb',
            executable='lab2_square',
            name='lab2_square',
            output='screen',
        ),
    ])
