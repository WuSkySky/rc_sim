import os

from setuptools import find_packages, setup


package_name = 'serial_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            ['config/serial_bridge.yaml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='skysky',
    maintainer_email='skysky@example.com',
    description='Serial bridge for Robot R2 real hardware.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge = serial_pkg.serial_bridge:main',
        ],
    },
)
