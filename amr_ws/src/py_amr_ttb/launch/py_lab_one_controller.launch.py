"""Launch the intentionally inert Task 4 integration scaffold."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start the scaffold without enabling any robot movement."""
    return LaunchDescription([
        Node(
            package='py_amr_ttb',
            executable='lab_one_controller',
            name='lab_one_controller_node',
            output='screen',
        ),
    ])
