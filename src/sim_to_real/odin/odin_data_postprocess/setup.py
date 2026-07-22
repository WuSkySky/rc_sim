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
        ('share/' + package_name, ['package.xml', 'README.md']),
        (
            os.path.join('share', package_name, 'config'),
            [
                'config/map_odom_tf_publisher.yaml',
                'config/odometry_pose_republisher.yaml',
                'config/odometry_tf_publisher.yaml',
            ],
        ),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='skysky',
    maintainer_email='skysky@example.com',
    description='Post-processing nodes for Odin sensor data.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'map_odom_tf_publisher = '
            'odin_data_postprocess.map_odom_tf_publisher:main',
            'odometry_pose_republisher = '
            'odin_data_postprocess.odometry_pose_republisher:main',
            'odometry_tf_publisher = '
            'odin_data_postprocess.odometry_tf_publisher:main',
        ],
    },
)
