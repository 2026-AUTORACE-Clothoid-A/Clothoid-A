"""Mission node with the car's camera profile (clothoid_lane camera_<camera>.yaml): the light /
pedestrian line stop runs clothoid_finish's FinishCore on the camera, as finish does."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    mission_share = get_package_share_directory('clothoid_mission')
    bringup_share = get_package_share_directory('clothoid_bringup')
    lane_share = get_package_share_directory('clothoid_lane')
    camera = LaunchConfiguration('camera').perform(context)
    return [Node(
        package='clothoid_mission', executable='mission', name='mission', output='screen',
        parameters=[
            os.path.join(bringup_share, 'config', 'topics.yaml'),
            os.path.join(mission_share, 'config', 'mission.yaml'),
            os.path.join(lane_share, 'config', f'camera_{camera}.yaml'),
            {'debug': LaunchConfiguration('debug'), 'parking_enabled': LaunchConfiguration('parking_enabled')},
        ])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='limo2'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('parking_enabled', default_value='true'),
        OpaqueFunction(function=nodes),
    ])
