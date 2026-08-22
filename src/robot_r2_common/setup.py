from setuptools import find_packages, setup


package_name = 'robot_r2_common'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='skysky',
    maintainer_email='skysky@example.com',
    description='Shared runtime helpers for Robot R2 nodes.',
    license='TODO: License declaration',
    tests_require=['pytest'],
)
