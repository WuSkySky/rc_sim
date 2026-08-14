from glob import glob
import os
from setuptools import find_packages, setup

package_name = "robot_r2_aruco"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="skysky",
    maintainer_email="skysky@todo.todo",
    description="ArUco marker detection and 6-DOF pose estimation node.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "aruco_detect = robot_r2_aruco.aruco_detect:main",
            "usb_camera_bridge = robot_r2_aruco.usb_camera_bridge:main",
            "debug_saver = robot_r2_aruco.debug_saver:main",
            "debug_viewer = robot_r2_aruco.debug_viewer:main",
            "aruco_distance = robot_r2_aruco.aruco_distance:main",
        ],
    },
)
