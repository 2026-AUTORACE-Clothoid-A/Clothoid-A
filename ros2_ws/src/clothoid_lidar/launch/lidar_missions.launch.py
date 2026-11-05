"""Lidar missions with the tuning view: cone course nodes + lidar_view (rviz on by default).

    ros2 launch clothoid_lidar lidar_missions.launch.py            # on the car, rviz:=false if headless
The static obstacle avoidance node joins here once it exists. full.launch.py keeps including
cone.launch.py directly, so the competition run never starts rviz.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('clothoid_lidar'), 'launch')
    view_args = {n: LaunchConfiguration(n) for n in
                 ('use_sim_time', 'rviz', 'gap_b', 'min_points_b')}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gap_b', default_value='0.06'),
        DeclareLaunchArgument('min_points_b', default_value='3'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'cone.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'lidar_view.launch.py')),
            launch_arguments=view_args.items()),
    ])
