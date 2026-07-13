import os

from setuptools import find_packages, setup

package_name = 'robot_r2_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            [
                'config/chassis_pose_servo.yaml',
                'config/chassis_lift.yaml',
                'config/kfs_alignment.yaml',
                'config/kfs_gripper_lift.yaml',
                'config/kfs_gripper_rotate.yaml',
                'config/kfs_gripper_tip_rotate.yaml',
                'config/kfs_gripper_grip.yaml',
            ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='skysky',
    maintainer_email='skysky@example.com',
    description='Low-level hardware control nodes for Robot R2.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'chassis_pose_servo = robot_r2_controller.chassis_pose_servo:main',
            'chassis_lift = robot_r2_controller.chassis_lift:main',
            'kfs_alignment = robot_r2_controller.kfs_alignment:main',
            'kfs_gripper_lift = robot_r2_controller.kfs_gripper_lift:main',
            'kfs_gripper_rotate = robot_r2_controller.kfs_gripper_rotate:main',
            'kfs_gripper_tip_rotate = robot_r2_controller.kfs_gripper_tip_rotate:main',
            'kfs_gripper_grip = robot_r2_controller.kfs_gripper_grip:main',
        ],
    },
)
