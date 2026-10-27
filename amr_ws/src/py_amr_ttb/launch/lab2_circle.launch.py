"""Run the circle module without changing shared setup.py console scripts."""

import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

from py_amr_ttb.lab2.circle_config import CircleConfig as C


def generate_launch_description():
    """Launch python -m with ordinary ROS parameter arguments."""
    settings = {
        'radius': C.CIRCLE_RADIUS,
        'speed': C.CIRCLE_SPEED,
        'target_speed': C.CIRCLE_TARGET_SPEED,
        'duration': C.CIRCLE_DURATION,
        'lookahead': C.CIRCLE_LOOKAHEAD,
        'laps': C.CIRCLE_LAPS,
        'direction': C.CIRCLE_DIRECTION,
        'robot_namespace': C.ROBOT_NAMESPACE,
    }
    arguments = [DeclareLaunchArgument(name, default_value=str(default))
                 for name, default in settings.items()]
    command = [sys.executable, '-m', 'py_amr_ttb.lab2.lab2_circle', '--ros-args']
    for name in settings:
        command.extend(['-p', [name + ':=', LaunchConfiguration(name)]])
    return LaunchDescription(arguments + [ExecuteProcess(
        cmd=command, name='lab2_circle', output='screen', emulate_tty=True,
    )])
