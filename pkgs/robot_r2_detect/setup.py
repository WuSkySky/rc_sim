from glob import glob
import os

from setuptools import find_packages, setup

package_name = "robot_r2_detect"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "model"), glob("model/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="skysky",
    maintainer_email="skysky@todo.todo",
    description="YOLO-based KFS detection node.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "kfs_detect = robot_r2_detect.kfs_detect:main",
        ],
    },
)
