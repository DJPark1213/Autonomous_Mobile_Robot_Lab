from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_amr_ttb',
            namespace='',
            executable='lab_one_joystick',
            name='lab_one_joystick_node',
        ),
    ])

