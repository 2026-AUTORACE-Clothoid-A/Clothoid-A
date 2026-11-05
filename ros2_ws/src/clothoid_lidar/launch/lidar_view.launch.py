"""Lidar tuning view: lidar_debug node (clusters A/B) + rviz with lidar.rviz.

Set A = obstacle_extractor values in lidar.yaml (what the running stack uses).
Set B = candidate values from the launch arguments (default cluster_gap 0.06, min_points 3).
Bag on the dev PC:
    ros2 launch clothoid_lidar lidar_view.launch.py use_sim_time:=true
    ros2 bag play <bag> --clock --topics /scan /tf /tf_static /camera/color/image_raw
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    share = get_package_share_directory('clothoid_lidar')
    with open(os.path.join(share, 'config', 'lidar.yaml')) as f:
        cfg = yaml.safe_load(f)
    ex = cfg['obstacle_extractor']['ros__parameters']
    so = cfg['cone_sorter']['ros__parameters']
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    sim = arg('use_sim_time').lower() == 'true'

    params = {k: float(ex[k]) for k in ('range_min', 'range_max', 'angle_min_deg', 'angle_max_deg')}
    params.update({
        'use_sim_time': sim,
        'a_cluster_gap': float(ex['cluster_gap']), 'a_min_points': int(ex['min_points']),
        'a_max_width': float(ex['max_width']), 'a_cone_radius': float(ex['cone_radius']),
        'b_cluster_gap': float(arg('gap_b')), 'b_min_points': int(arg('min_points_b')),
        'b_max_width': float(arg('max_width_b') or ex['max_width']),
        'b_cone_radius': float(arg('cone_radius_b') or ex['cone_radius']),
    })
    params.update({k: float(so[k]) for k in so if k.startswith(('start_', 'ing_')) and 'count' not in k})

    return [
        Node(package='clothoid_lidar', executable='lidar_debug', name='lidar_debug',
             output='screen', parameters=[params]),
        Node(package='rviz2', executable='rviz2', name='rviz_lidar', output='log',
             arguments=['-d', os.path.join(share, 'rviz', 'lidar.rviz')],
             parameters=[{'use_sim_time': sim}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true when playing a bag with --clock'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gap_b', default_value='0.06', description='set B cluster_gap'),
        DeclareLaunchArgument('min_points_b', default_value='3', description='set B min_points'),
        DeclareLaunchArgument('max_width_b', default_value='',
                              description='set B max_width (empty = same as lidar.yaml)'),
        DeclareLaunchArgument('cone_radius_b', default_value='',
                              description='set B cone_radius (empty = same as lidar.yaml)'),
        OpaqueFunction(function=_nodes),
    ])
