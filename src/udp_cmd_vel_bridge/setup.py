from setuptools import find_packages, setup


package_name = "udp_cmd_vel_bridge"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/udp_cmd_vel_bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hailey",
    maintainer_email="hailey@example.com",
    description="UDP to cmd_vel bridge for ROS 2 robots",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "udp_cmd_vel_bridge = udp_cmd_vel_bridge.udp_cmd_vel_bridge_node:main",
        ],
    },
)
