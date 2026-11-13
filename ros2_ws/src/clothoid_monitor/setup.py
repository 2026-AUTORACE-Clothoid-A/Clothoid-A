from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'clothoid_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: [
            'templates/*.html',
            'static/css/*.css',
            'static/js/*.js',
        ],
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
    ],
    install_requires=['setuptools', 'flask'],
    zip_safe=False,
    maintainer='esem5377',
    maintainer_email='esem5377@gmail.com',
    description='Flask-based Web Monitoring Dashboard for Clothoid-A',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monitor_node = clothoid_monitor.monitor_node:main',
        ],
    },
)
