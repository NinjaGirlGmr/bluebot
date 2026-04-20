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
            (
                'straight_line_compensator_node = '
                'robot_bringup.straight_line_compensator_node:main'
            ),
            (
                'apriltag_landmark_tf_publisher = '
                'robot_bringup.apriltag_landmark_tf_publisher_node:main'
            ),
            (
                'apriltag_map_localization = '
                'robot_bringup.apriltag_map_localization_node:main'
            ),
            (
                'goal_pose_sanitizer = '
                'robot_bringup.goal_pose_sanitizer_node:main'
            ),
            (
                'grid_localization_trigger = '
                'robot_bringup.grid_localization_trigger_node:main'
            ),
            (
                'dock_command = '
                'robot_bringup.dock_command_node:main'
            ),
            (
                'dock_detector = '
                'robot_bringup.dock_detector_node:main'
            ),
        ],
    },
)
