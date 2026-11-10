import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'clothoid_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Clothoid-A',
    maintainer_email='todo@example.com',
    description='Mission state logic',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission = clothoid_mission.mission_node:main',
            'speed_bump = clothoid_mission.speed_bump_node:main',
        ],
    },
)
