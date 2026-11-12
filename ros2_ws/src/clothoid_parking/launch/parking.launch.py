import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    parking_share = get_package_share_directory('clothoid_parking')
    bringup_share = get_package_share_directory('clothoid_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('six_bays', default_value='false',
                             description='Use verified grid targets, including outside bays'),
        DeclareLaunchArgument('target_bay', default_value='0',
                             description='0: automatic, 1..3: lane-side, 4..6: inner row'),
        Node(
            package='clothoid_parking',
            executable='parking',
            name='parking',
            output='screen',
            parameters=[
                os.path.join(bringup_share, 'config', 'topics.yaml'),
                os.path.join(parking_share, 'config', 'parking.yaml'),
                {'debug': LaunchConfiguration('debug')},
                {'layout.control_enabled': LaunchConfiguration('six_bays'),
                 'layout.target_bay': LaunchConfiguration('target_bay')},
            ],
        ),
    ])
