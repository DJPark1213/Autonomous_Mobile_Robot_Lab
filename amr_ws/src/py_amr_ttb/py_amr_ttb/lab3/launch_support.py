"""Launch Lab 3 with the same python -m pattern as Lab 2 circle."""

from dataclasses import fields
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

from .core import Config


def launch_description(mode):
    """Pass ordinary ROS parameters; keep the node name out of robot namespaces."""
    if mode not in ('goal', 'wander', 'follow'):
        raise ValueError('mode must be goal, wander or follow')
    defaults = Config()
    settings = {field.name: getattr(defaults, field.name) for field in fields(Config)}
    settings.update(robot_namespace=Config.ROBOT_NAMESPACE,
                    base_frame='base_link', log_dir='~/amr_lab3_runs')
    arguments = [DeclareLaunchArgument(
        name, default_value=str(value).lower() if isinstance(value, bool) else str(value))
        for name, value in settings.items()]
    command = [sys.executable, '-m', 'py_amr_ttb.lab3.node', '--ros-args', '-p', 'mode:='+mode]
    for name in settings:
        command.extend(['-p', [name+':=', LaunchConfiguration(name)]])
    # Absolute sensor topics are set in the node. Only the standard TF
    # listener's global topics are remapped, entirely inside this helper.
    for topic in ('tf', 'tf_static'):
        command.extend(['-r', ['/'+topic+':=', LaunchConfiguration('robot_namespace'), '/'+topic]])
    return LaunchDescription(arguments + [ExecuteProcess(
        cmd=command, name='lab3_'+mode, output='screen', emulate_tty=True)])
