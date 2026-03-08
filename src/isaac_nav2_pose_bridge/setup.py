import os
from glob import glob

from setuptools import setup

package_name = 'isaac_nav2_pose_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hailey',
    maintainer_email='hailey@todo.todo',
    description='Bridge Isaac ROS pose outputs to Nav2 /initialpose',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'isaac_to_nav2_pose = isaac_nav2_pose_bridge.isaac_to_nav2_pose:main',
        ],
    },
)
