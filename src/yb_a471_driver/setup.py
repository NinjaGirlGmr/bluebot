from setuptools import setup

package_name = 'yb_a471_driver'

setup(
  name=package_name,
  version='0.1.0',
  packages=[package_name],
  package_data={package_name: ['resource/' + package_name]},
  data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
  ],
  install_requires=['setuptools'],
  zip_safe=True,
  maintainer='hailey',
  maintainer_email='you@example.com',
  description='Recreated IMU bridge driver for Bluebot stack.',
  license='TODO: License declaration',
  entry_points={
    'console_scripts': [
      'imu_node = yb_a471_driver.imu_node:main',
    ],
  },
)
