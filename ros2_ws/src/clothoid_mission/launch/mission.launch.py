import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mission_share = get_package_share_directory('clothoid_mission')
    bringup_share = get_package_share_directory('clothoid_bringup')

    return LaunchDescription([
        Node(
            package='clothoid_mission',
            executable='mission',
            name='mission',
            output='screen',
            parameters=[
                os.path.join(bringup_share, 'config', 'topics.yaml'),
                os.path.join(mission_share, 'config', 'mission.yaml'),
            ],
        ),
    ])
