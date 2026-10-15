"""Launch task2_wander."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_amr_ttb',
            executable='task2_wander',
            name='task2_wander',
            output='screen',
        ),
    ])