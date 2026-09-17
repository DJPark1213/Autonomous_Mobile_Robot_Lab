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
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='BrodyBrody41@gmail.com',
    description='ECE 4060 Turtlebot 4 laboratory nodes.',
    license='TODO',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'ttb_turn = py_amr_ttb.ttb_turn:main',
            'lab_one_joystick = py_amr_ttb.lab_one_joystick:main',
            'lab_one_controller = py_amr_ttb.lab_one_controller:main',
        ],
    },
)
