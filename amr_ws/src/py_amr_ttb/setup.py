"""Install the robot laboratory Python package and launch files."""

from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'py_amr_ttb'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join(
                'share',
                package_name,
                'launch',
            ),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='BrodyBrody41@gmail.com',
    description='ROS 2 controllers for AMR Labs 1 and 2.',
    license='TODO',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'task1_teleop = py_amr_ttb.demo.task1_teleop:main',
            'task2_wander = py_amr_ttb.demo.task2_wander:main',
            'task3_pid = py_amr_ttb.demo.task3_pid:main',
            'task4_state_machine = py_amr_ttb.demo.task4_state_machine:main',
            'lab_one_controller = py_amr_ttb.lab_one_controller:main',
            'lab2_go_to_goal = py_amr_ttb.lab2.lab2_go_to_goal:main',
        ],
    },
)
