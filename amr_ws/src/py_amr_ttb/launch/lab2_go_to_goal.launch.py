"""Launch the odometry-relative go-to-goal task."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        'linear_gain': '1.0', 'angular_gain': '2.0',
        'linear_ki': '0.01', 'linear_kd': '0.02',
        'angular_ki': '0.01', 'angular_kd': '0.03', 'align_enter': '0.20',
        'max_linear_speed': '0.4', 'max_angular_speed': '0.4',
        'position_tolerance': '0.05', 'yaw_tolerance': '0.10',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value)
          for name, value in defaults.items()],
        Node(
            package='py_amr_ttb',
            executable='lab2_go_to_goal',
            name='lab2_go_to_goal',
            output='screen',
            parameters=[{
                name: ParameterValue(LaunchConfiguration(name), value_type=float)
                for name in defaults
            }],
        ),
    ])
