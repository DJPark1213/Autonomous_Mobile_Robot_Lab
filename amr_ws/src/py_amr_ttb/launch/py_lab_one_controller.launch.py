"""Launch the idle Task 4 controller template."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start the controller using its registered console executable."""
    return LaunchDescription([
        Node(
            package='py_amr_ttb',
            executable='lab_one_controller',
            name='lab_one_controller_node',
            output='screen',
        ),
    ])
