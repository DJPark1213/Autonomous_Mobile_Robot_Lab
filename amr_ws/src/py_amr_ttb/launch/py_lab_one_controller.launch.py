"""Launch the combined Lab 1 controller in STOP mode."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start sensor subscriptions and the combined command publisher."""
    return LaunchDescription([
        Node(
            package='py_amr_ttb',
            executable='lab_one_controller',
            name='lab_one_controller_node',
            output='screen',
        ),
    ])
