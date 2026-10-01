"""Launch task4_state_machine."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_amr_ttb',
            executable='task4_state_machine',
            name='task4_state_machine',
            output='screen',
        ),
    ])