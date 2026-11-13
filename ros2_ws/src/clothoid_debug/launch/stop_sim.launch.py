"""Stop at the stop line in the simulator, with YOLO, shown in RViz (rviz/stop.rviz).

Start the simulator first (docker/README.md), e.g. `--mission crosswalk` or `--mission signal`, then

    ros2 launch clothoid_debug stop_sim.launch.py model_path:=/path/best.pt

The real lane, YOLO and mission nodes (mission runs finish's FinishCore for the stop) drive the car
through twist_mux;
stop_viz draws the YOLO boxes, the mission state and the stop line / stop point. In the signal scene the simulator
window's T / G / Y keys switch the light.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def share(package, *path):
    return os.path.join(get_package_share_directory(package), *path)


def generate_launch_description():
    def include(package, launch_file, args=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share(package, 'launch', launch_file)),
            launch_arguments=(args or {}).items())

    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value='',
                              description='YOLO weights (.pt on the PC, .engine on the Jetson)'),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_stopline', default_value='false'),
        DeclareLaunchArgument('share_lane_camera', default_value='true'),
        GroupAction([
            SetParameter(name='use_sim_time', value=True),
            include('clothoid_lane', 'lane.launch.py', {
                'debug': 'true', 'use_stopline': LaunchConfiguration('use_stopline'),
                'share_lane_camera': LaunchConfiguration('share_lane_camera')}),
            include('clothoid_mission', 'mission.launch.py', {'debug': 'true'}),
            Node(package='clothoid_yolo', executable='yolo', name='yolo', output='screen',
                 parameters=[share('clothoid_bringup', 'config', 'topics.yaml'),
                             share('clothoid_yolo', 'config', 'yolo.yaml'),
                             {'backend': 'ultralytics',
                              'model_path': LaunchConfiguration('model_path'),
                              'device': LaunchConfiguration('device')}]),
            Node(package='twist_mux', executable='twist_mux', name='twist_mux', output='screen',
                 parameters=[share('clothoid_bringup', 'config', 'twist_mux.yaml')],
                 remappings=[('cmd_vel_out', '/cmd_vel')]),
            include('clothoid_debug', 'stop_viz.launch.py',
                    {'rviz': LaunchConfiguration('rviz'), 'use_sim_time': 'true'}),
        ]),
    ])
