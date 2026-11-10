import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'clothoid_debug'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Clothoid-A',
    maintainer_email='todo@example.com',
    description='RViz view of what the stack sees and where it steers, live or on a bag',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'decision_viz = clothoid_debug.decision_viz:main',
            'lane_viewer = clothoid_debug.lane_viewer:main',
            'stop_viz = clothoid_debug.stop_viz:main',
        ],
    },
)
