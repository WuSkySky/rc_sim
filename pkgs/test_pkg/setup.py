from setuptools import find_packages, setup


package_name = 'test_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='skysky',
    maintainer_email='skysky@example.com',
    description='Standalone executable tests for Robot R2 control services.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'move_to_center_test = robot_r2_tests.move_to_center:main',
            'step_traverse_test = robot_r2_tests.step_traverse:main',
        ],
    },
)
