import os
from setuptools import find_packages, setup

package_name = 'robot_r2_description'


def package_files(data_files, directory_list):
    paths_dict = {}
    for directory in directory_list:
        for path, _, filenames in os.walk(directory):
            if '__pycache__' in path.split(os.sep):
                continue
            for filename in filenames:
                if filename.endswith('.pyc'):
                    continue
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                paths_dict.setdefault(install_path, []).append(file_path)

    data_files.extend(paths_dict.items())
    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=package_files([
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ], ['resource', 'models']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='skysky',
    maintainer_email='skysky@example.com',
    description='Robot model descriptions for RC2026.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
