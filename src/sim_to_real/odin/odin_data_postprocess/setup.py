import os

from setuptools import find_packages, setup


package_name = 'odin_data_postprocess'

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
            ['config/odometry_pose_republisher.yaml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='skysky',
    maintainer_email='skysky@example.com',
    description='Post-processing nodes for Odin sensor data.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'odometry_pose_republisher = '
            'odin_data_postprocess.odometry_pose_republisher:main',
        ],
    },
)
