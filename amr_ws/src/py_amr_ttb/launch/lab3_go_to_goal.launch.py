"""Start exploration with configurable robot wiring and a saved-run destination.

Learning gathers map evidence and a trajectory for later replay. Completion is
determined by the configured goal, and run status is stored with the evidence.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch the learning controller with consistent namespace, base frame and run file."""
    return LaunchDescription([
        DeclareLaunchArgument('robot_namespace', default_value='/TTB10'),
        DeclareLaunchArgument('sensor_clock_offset', default_value='0.0',
                              description='Controller clock minus sensor clock in seconds'),
        DeclareLaunchArgument('run_file', default_value='~/amr_lab3_runs/course.npz'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        Node(package='py_amr_ttb', executable='lab3_controller', output='screen',
             remappings=[
                 ('/tf', PathJoinSubstitution([LaunchConfiguration('robot_namespace'), 'tf'])),
                 ('/tf_static', PathJoinSubstitution([
                     LaunchConfiguration('robot_namespace'), 'tf_static'])),
             ],
             parameters=[{'run_mode': 'learn',
                          'robot_namespace': LaunchConfiguration('robot_namespace'),
                          'sensor_clock_offset': ParameterValue(
                              LaunchConfiguration('sensor_clock_offset'), value_type=float),
                          'base_frame': LaunchConfiguration('base_frame'),
                          'run_file': LaunchConfiguration('run_file')}]),
    ])
