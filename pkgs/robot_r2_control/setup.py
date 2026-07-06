import os

from setuptools import find_packages, setup

package_name = 'robot_r2_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            ['config/pose_servo.yaml']),
    ],
    install_requires=['setuptools', 'pynput'],
    zip_safe=True,
    maintainer='skysky',
    maintainer_email='skysky@example.com',
    description='Control utilities for Robot R2.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wasd_teleop = robot_r2_control.wasd_teleop:main',
            'pose_servo = robot_r2_control.pose_servo:main',
            'lift_service_controller = robot_r2_control.lift_service_controller:main',
            'step_traverse_service = robot_r2_control.step_traverse_service:main',
        ],
    },
)
