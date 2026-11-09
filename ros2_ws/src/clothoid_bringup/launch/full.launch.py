"""Whole competition stack: platform bring-up plus every mission package.

    ros2 launch clothoid_bringup full.launch.py
    ros2 launch clothoid_bringup full.launch.py use_yolo:=false use_parking:=false
    ros2 launch clothoid_bringup full.launch.py debug:=true backend:=ultralytics model_path:=/path/best.pt
    ros2 launch clothoid_bringup full.launch.py camera:=limo2      # per car warp, see clothoid_lane/config

`debug:=true` also starts clothoid_debug/decision_viz; open clothoid_debug/rviz/decision.rviz to watch it.

Each part can be switched off so one person can run their package against a bag or a partial stack.
`debug`, `backend` and `model_path` are passed on as launch arguments (never put them in a package yaml,
see docs/PACKAGES.md). Platform arguments (`camera_pitch`, `use_camera`, `use_lidar`, `use_ekf`,
`require_arm`, `mux_output`, `steering_scale`, `max_inner_angle`, `cmd_timeout`) are forwarded to
`bringup.launch.py`.

Off by default: `require_arm:=true` locks autonomy until /estop receives `false` (config/twist_mux_arm.yaml).
Shadow run (nothing drives, see what the stack would send): `mux_output:=/cmd_vel_dry`.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    def include(package, launch_file, args=None, condition=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])),
            launch_arguments=(args or {}).items(),
            condition=condition)

    def enabled(name):
        return IfCondition(LaunchConfiguration(name))

    switches = ['use_lane', 'use_yolo', 'use_lidar_missions', 'use_parking', 'use_mission']
    platform_args = ['camera_pitch', 'use_camera', 'use_lidar', 'use_ekf', 'steering_scale',
                     'max_inner_angle']
    debug = LaunchConfiguration('debug')
    # defaults must match bringup.launch.py
    mux_args = {'require_arm': 'false', 'mux_output': '/cmd_vel', 'cmd_timeout': '0.5'}

    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value='true') for name in switches],
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('camera', default_value='limo2',
                              description='camera profile: limo1 or limo2 (clothoid_lane/config)'),
        DeclareLaunchArgument('backend', default_value='ultralytics',
                              description='yolo backend: ultralytics (default) | none'),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('show', default_value='false',
                              description='show yolo detection window'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_ekf', default_value='false'),
        DeclareLaunchArgument('steering_scale', default_value='1.06'),
        DeclareLaunchArgument('max_inner_angle', default_value='0.48869'),
        *[DeclareLaunchArgument(name, default_value=value) for name, value in mux_args.items()],

        # sensors, TF, limo_base, twist_mux
        include('clothoid_bringup', 'bringup.launch.py',
                args={name: LaunchConfiguration(name) for name in [*platform_args, *mux_args]}),

        include('clothoid_lane', 'lane.launch.py',
                {'debug': debug, 'camera': LaunchConfiguration('camera')}, enabled('use_lane')),
        include('clothoid_yolo', 'yolo.launch.py',
                {'backend': LaunchConfiguration('backend'),
                 'model_path': LaunchConfiguration('model_path'),
                 'show': LaunchConfiguration('show')}, enabled('use_yolo')),
        include('clothoid_lidar', 'cone.launch.py', condition=enabled('use_lidar_missions')),
        include('clothoid_parking', 'parking.launch.py', {'debug': debug}, enabled('use_parking')),
        include('clothoid_mission', 'mission.launch.py', condition=enabled('use_mission')),

        # read-only rate / latency / mux-winner log (clothoid_bringup/latency_monitor.py)
        DeclareLaunchArgument('use_latency_monitor', default_value='false'),
        DeclareLaunchArgument('latency_csv', default_value='',
                              description='append every report to this CSV, e.g. ~/bags/latency.csv'),
        Node(package='clothoid_bringup', executable='latency_monitor', name='latency_monitor',
             output='screen', parameters=[{'csv_path': LaunchConfiguration('latency_csv')}],
             condition=enabled('use_latency_monitor')),

        # with debug on, also publish what rviz/decision.rviz shows
        include('clothoid_debug', 'viz.launch.py', {'camera': LaunchConfiguration('camera')},
                enabled('debug')),
    ])
