"""Use a completed learned course to attempt a more efficient second run.

The controller verifies saved-route clearance and requires alignment near the
original start before following replay targets with current scan checks.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch replay against the chosen learned file and robot frame configuration."""
    return LaunchDescription([
        DeclareLaunchArgument('robot_namespace', default_value='/TTB10'),
        DeclareLaunchArgument('run_file', default_value='~/amr_lab3_runs/course.npz'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('max_linear', default_value='0.46'),
        DeclareLaunchArgument('safety_override', default_value='full'),
        Node(package='py_amr_ttb', executable='lab3_controller', output='screen',
             remappings=[
                 ('/tf', PathJoinSubstitution([LaunchConfiguration('robot_namespace'), 'tf'])),
                 ('/tf_static', PathJoinSubstitution([
                     LaunchConfiguration('robot_namespace'), 'tf_static'])),
             ],
             parameters=[{'run_mode': 'replay',
                          'robot_namespace': LaunchConfiguration('robot_namespace'),
                          'base_frame': LaunchConfiguration('base_frame'),
                          'max_linear': ParameterValue(LaunchConfiguration('max_linear'), value_type=float),
                          'safety_override': LaunchConfiguration('safety_override'),
                          'run_file': LaunchConfiguration('run_file')}]),
    ])
