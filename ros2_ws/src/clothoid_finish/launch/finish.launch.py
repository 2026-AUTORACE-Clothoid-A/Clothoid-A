"""Finish node with the car's camera profile (clothoid_lane camera_<camera>.yaml)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    lane = get_package_share_directory('clothoid_lane')
    finish = get_package_share_directory('clothoid_finish')
    camera = LaunchConfiguration('camera').perform(context)
    return [Node(
        package='clothoid_finish', executable='finish', name='finish', output='screen',
        parameters=[os.path.join(finish, 'config', 'finish.yaml'),
                    os.path.join(lane, 'config', f'camera_{camera}.yaml'),
                    {'debug': LaunchConfiguration('debug'),
                     'use_sim_time': LaunchConfiguration('use_sim_time')}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='limo2'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=nodes),
    ])
