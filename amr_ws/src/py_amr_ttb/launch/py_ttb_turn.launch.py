from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_amr_ttb',
            namespace='',
            executable='ttb_turn',
            name='ttb_turn_node',
        ),
    ])

