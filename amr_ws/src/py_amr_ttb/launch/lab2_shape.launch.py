"""Launch the shape navigation task."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_amr_ttb',
            executable='lab2_shape',
            name='lab2_shape',
            output='screen',
        ),
    ])
