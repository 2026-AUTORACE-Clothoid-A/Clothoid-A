"""Observation-only lane/parking processing and a dedicated RViz view.

Camera/scan/imu/odom must already be supplied by sensors or a bag. start_nodes=false
opens just the view alongside an existing stack; never start duplicate processors.
"""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    parking = get_package_share_directory('clothoid_parking')
    lane = get_package_share_directory('clothoid_lane')

    def load(folder, filename, scope):
        with open(os.path.join(folder, 'config', filename)) as stream:
            return yaml.safe_load(stream)[scope]['ros__parameters']

    def processors(context):
        if not IfCondition(LaunchConfiguration('start_nodes')).evaluate(context):
            return []
        camera = LaunchConfiguration('camera').perform(context)
        if camera not in ('limo1', 'limo2'):
            raise ValueError('camera must be limo1 or limo2')
        clock = {'use_sim_time': LaunchConfiguration('use_sim_time')}
        return [
            Node(package='clothoid_lane', executable='lane_main', name='parking_inspect_lane',
                 parameters=[load(lane, 'lane.yaml', 'lane_main'),
                             load(lane, f'camera_{camera}.yaml', '/**'), clock,
                             {'parking_landmark.always_observe': True, 'debug': False}],
                 remappings=[('/cmd_vel_lane', '/parking_inspect/cmd_vel_lane'),
                             ('/parking_flag', '/parking_inspect/parking_flag'),
                             ('/parking_done', '/parking_inspect/parking_done')], output='screen'),
            Node(package='clothoid_parking', executable='parking', name='parking_inspect',
                 parameters=[load(parking, 'parking.yaml', 'parking'), clock,
                             {'inspection_only': True}],
                 remappings=[('/cmd_vel_parking', '/parking_inspect/cmd_vel_parking'),
                             ('/parking/bay_candidate', '/parking_inspect/bay_candidate'),
                             ('/parking_done', '/parking_inspect/parking_done')], output='screen'),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('camera', default_value='limo2'),
        DeclareLaunchArgument('start_nodes', default_value='true',
                              description='false: RViz only alongside an existing stack'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('compressed', default_value='false',
                              description='decode a bag compressed image topic into the raw camera topic'),
        Node(package='image_transport', executable='republish', name='parking_image_decode',
             condition=IfCondition(LaunchConfiguration('compressed')),
             arguments=['compressed', 'raw'],
             remappings=[('in/compressed', '/camera/color/image_raw/compressed'),
                         ('out', '/camera/color/image_raw')],
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'in_transport': 'compressed', 'out_transport': 'raw'}]),
        OpaqueFunction(function=processors),
        Node(package='rviz2', executable='rviz2', name='parking_inspect_rviz',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(parking, 'rviz', 'parking_inspect.rviz')],
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
