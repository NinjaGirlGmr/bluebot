from glob import glob

from setuptools import setup

package_name = "bluebot_v2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hailey",
    maintainer_email="hailey@todo.todo",
    description="Minimal Bluebot v2 mapping stack with wheel odometry and LiDAR.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "drive_command_node = bluebot_v2.drive_command_node:main",
        ],
    },
)
