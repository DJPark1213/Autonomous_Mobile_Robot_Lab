"""Start exploration with configurable robot wiring and a saved-run destination.

Learning gathers map evidence and a trajectory for later replay. Completion is
determined by the configured goal, and run status is stored with the evidence.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the learning controller with consistent namespace, base frame and run file."""
    return LaunchDescription([
        DeclareLaunchArgument('robot_namespace', default_value='/TTB10'),
        DeclareLaunchArgument('run_file', default_value='~/amr_lab3_runs/course.npz'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        Node(package='py_amr_ttb', executable='lab3_controller', output='screen',
             parameters=[{'run_mode': 'learn',
                          'robot_namespace': LaunchConfiguration('robot_namespace'),
                          'base_frame': LaunchConfiguration('base_frame'),
                          'run_file': LaunchConfiguration('run_file')}]),
    ])
