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
            [
                'config/stage_two.yaml',
                'config/stage_two_point_one.yaml',
                'config/stage_two_point_two.yaml',
                'config/kfs_loader.yaml',
                'config/step_traverse.yaml',
            ]),
    ],
    install_requires=['setuptools', 'pynput'],
    zip_safe=True,
    maintainer='skysky',
    maintainer_email='skysky@example.com',
    description='High-level control utilities for Robot R2.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_control = robot_r2_control.teleop_control:main',
            'stage_two_control = robot_r2_control.stage_two_control:main',
            'stage_two_point_one = robot_r2_control.stage_two_point_one:main',
            'stage_two_point_two = robot_r2_control.stage_two_point_two:main',
            'kfs_loader_control = robot_r2_control.kfs_loader:main',
            'step_traverse = robot_r2_control.step_traverse:main',
        ],
    },
)
