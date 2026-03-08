from glob import glob

from setuptools import setup

package_name = 'serial_diff_drive_hw'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/urdf', ['urdf/serial_diff_drive_hw.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hailey',
    maintainer_email='NinjaGirlGmr@users.noreply.github.com',
    description='Serial diff drive hardware interface for ros2_control',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'serial_diff_drive_hw = serial_diff_drive_hw.serial_diff_drive_hw:main',
            'drop_detector_node = serial_diff_drive_hw.drop_detector_node:main',
            'cmd_vel_safety_gate = serial_diff_drive_hw.cmd_vel_safety_gate:main',
            'foxglove_waypoint_bridge = serial_diff_drive_hw.foxglove_waypoint_bridge:main',
        ],
    },
)
