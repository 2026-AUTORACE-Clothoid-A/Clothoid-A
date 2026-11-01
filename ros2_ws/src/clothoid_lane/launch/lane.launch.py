import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lane_share = get_package_share_directory('clothoid_lane')
    bringup_share = get_package_share_directory('clothoid_bringup')
    topics = os.path.join(bringup_share, 'config', 'topics.yaml')

    def node(executable, name, yaml_name, extra=None):
        return Node(
            package='clothoid_lane',
            executable=executable,
            name=name,
            output='screen',
            parameters=[topics, os.path.join(lane_share, 'config', yaml_name),
                        {'debug': LaunchConfiguration('debug')}] + (extra or []),
        )

    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='false'),
        node('lane_main', 'lane_main', 'lane.yaml'),
        node('speed_zone', 'speed_zone', 'speed_zone.yaml'),
        node('stopline', 'stopline', 'stopline.yaml'),
    ])
