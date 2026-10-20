"""Launch the heart task with defaults from LabConfig."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start one heart controller; it begins after valid odometry arrives."""
    return LaunchDescription([
        Node(package='py_amr_ttb', executable='lab2_heart',
             name='lab2_heart', output='screen'),
    ])
