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
                'config/step_traverse_service.yaml',
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
            'teleop_controller = robot_r2_control.teleop_controller:main',
            'step_traverse_controller = robot_r2_control.step_traverse_controller:main',
        ],
    },
)
