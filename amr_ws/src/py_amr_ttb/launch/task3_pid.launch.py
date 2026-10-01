"""Launch task3_pid."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_amr_ttb',
            executable='task3_pid',
            name='task3_pid',
            output='screen',
        ),
    ])