"""Install the Lab 1 Python package and launch files."""

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
            glob('launch/task*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='BrodyBrody41@gmail.com',
    description='ROS 2 controllers for AMR Lab 1.',
    license='TODO',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'task1_teleop = py_amr_ttb.task1_teleop:main',
            'task2_wander = py_amr_ttb.task2_wander:main',
            'task3_pid = py_amr_ttb.task3_pid:main',
            'task4_state_machine = py_amr_ttb.task4_state_machine:main',
        ],
    },
)
