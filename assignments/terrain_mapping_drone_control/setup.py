from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'terrain_mapping_drone_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py')),
        ('share/' + package_name + '/config',
            glob('config/*')),
        ('share/' + package_name + '/models/terrain',
            glob('models/terrain/*.*')),
        ('share/' + package_name + '/models/terrain/meshes',
            glob('models/terrain/meshes/*.*')),
        ('share/' + package_name + '/models/cylinder',
            glob('models/cylinder/*.*')),
        ('share/' + package_name + '/models/cylinder/materials',
            glob('models/cylinder/materials/*.*')),
        ('share/' + package_name + '/scripts',
            glob('scripts/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A ROS2 package for executing spiral trajectory with PX4',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'px4_odom_converter = terrain_mapping_drone_control.px4_odom_converter:main',
            'cylinder_landing_node = terrain_mapping_drone_control.cylinder_landing_node:main',
        ],
    },
    python_requires='>=3.8'
) 