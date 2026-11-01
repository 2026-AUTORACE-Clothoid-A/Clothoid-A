import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    lidar_share = get_package_share_directory('clothoid_lidar')
    bringup_share = get_package_share_directory('clothoid_bringup')
    parameters = [os.path.join(bringup_share, 'config', 'topics.yaml'),
                  os.path.join(lidar_share, 'config', 'lidar.yaml')]

    return LaunchDescription([
        Node(package='clothoid_lidar', executable=name, name=name, output='screen',
             parameters=parameters)
        for name in ('obstacle_extractor', 'cone_sorter', 'cone_follower')
    ])
