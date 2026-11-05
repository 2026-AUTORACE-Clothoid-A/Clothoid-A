import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'clothoid_lidar'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Clothoid-A',
    maintainer_email='todo@example.com',
    description='Lidar missions: obstacle extraction and the rubber cone course',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_extractor = clothoid_lidar.obstacle_extractor_node:main',
            'cone_sorter = clothoid_lidar.cone_sorter_node:main',
            'cone_follower = clothoid_lidar.cone_follower_node:main',
            'lidar_debug = clothoid_lidar.lidar_debug_node:main',
        ],
    },
)
