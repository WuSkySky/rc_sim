from glob import glob
import os

from setuptools import find_packages, setup


package_name = "robot_r2_target_alignment"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml", "README.md", "requirements.txt"],
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "model"),
            glob("model/*.pt")
            + glob("model/*.onnx")
            + glob("model/*.engine"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="skysky",
    maintainer_email="skysky@todo.todo",
    description="YOLO11 target detection and chassis alignment nodes.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "yolo_target_detector = "
            "robot_r2_target_alignment.yolo_target_detector:main",
            "target_alignment_controller = "
            "robot_r2_target_alignment.target_alignment_controller:main",
        ],
    },
)
