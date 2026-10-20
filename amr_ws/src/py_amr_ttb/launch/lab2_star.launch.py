"""Launch the star navigation task."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_amr_ttb',
            executable='lab2_star',
            name='lab2_star',
            output='screen',
        ),
    ])
