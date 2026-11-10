import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'clothoid_finish'

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
    description='After parking: follow the yellow line to the end line and stop',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'finish = clothoid_finish.finish_node:main',
        ],
    },
)
