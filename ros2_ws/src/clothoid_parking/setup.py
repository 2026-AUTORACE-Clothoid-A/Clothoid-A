import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'clothoid_parking'

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
    description='T-parking 2026: lidar empty-bay search and one forward arc',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'parking = clothoid_parking.parking_node:main',
        ],
    },
)
