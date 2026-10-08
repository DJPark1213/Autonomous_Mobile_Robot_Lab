"""Launch the odometry-relative go-to-goal task."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_amr_ttb',
            executable='lab2_go_to_goal',
            name='lab2_go_to_goal',
            output='screen',
        ),
    ])
