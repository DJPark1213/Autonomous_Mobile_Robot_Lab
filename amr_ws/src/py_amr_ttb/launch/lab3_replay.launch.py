"""Use a completed learned course to attempt a more efficient second run.

The controller verifies saved-route clearance and requires alignment near the
original start before following replay targets with current scan checks.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch replay against the chosen learned file and robot frame configuration."""
    return LaunchDescription([
        DeclareLaunchArgument('robot_namespace', default_value='/TTB10'),
        DeclareLaunchArgument('sensor_clock_offset', default_value='0.0',
                              description='Controller clock minus sensor clock in seconds'),
        DeclareLaunchArgument('run_file', default_value='~/amr_lab3_runs/course.npz'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        Node(package='py_amr_ttb', executable='lab3_controller', output='screen',
             parameters=[{'run_mode': 'replay',
                          'robot_namespace': LaunchConfiguration('robot_namespace'),
                          'sensor_clock_offset': ParameterValue(
                              LaunchConfiguration('sensor_clock_offset'), value_type=float),
                          'base_frame': LaunchConfiguration('base_frame'),
                          'run_file': LaunchConfiguration('run_file')}]),
    ])
