import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hailey',
    maintainer_email='ci@bluebot.local',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'health_monitor = robot_bringup.health_monitor_node:main',
            'apriltag_nav_behavior_tree = robot_bringup.apriltag_nav_behavior_tree_node:main',
            'apriltag_map_recorder = robot_bringup.apriltag_map_recorder_node:main',
        ],
    },
)
