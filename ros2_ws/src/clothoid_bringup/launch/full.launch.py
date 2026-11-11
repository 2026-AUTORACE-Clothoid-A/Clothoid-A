"""Whole competition stack: platform bring-up plus every mission package.

    ros2 launch clothoid_bringup full.launch.py
    ros2 launch clothoid_bringup full.launch.py use_yolo:=false use_parking:=false
    ros2 launch clothoid_bringup full.launch.py debug:=true backend:=ultralytics model_path:=/path/best.pt
    ros2 launch clothoid_bringup full.launch.py camera:=limo2      # per car warp, see clothoid_lane/config

`viz:=true` starts clothoid_debug/decision_viz (off by default, even with debug:=true: ~50 % of a core
on limo2); open clothoid_debug/rviz/decision.rviz to watch it. `use_stopline:=true` / `use_speed_zone:=true`
start the stopline / speed_zone nodes (off by default, see clothoid_lane/launch/lane.launch.py).

Each part can be switched off so one person can run their package against a bag or a partial stack.
`debug`, `backend` and `model_path` are passed on as launch arguments (never put them in a package yaml,
see docs/PACKAGES.md). Platform arguments (`camera_pitch`, `use_camera`, `use_lidar`, `use_ekf`, `use_joint_states`,
`require_arm`, `mux_output`, `steering_scale`, `max_inner_angle`, `cmd_timeout`) are forwarded to
`bringup.launch.py`.

Off by default: `require_arm:=true` locks autonomy until /estop receives `false` (config/twist_mux_arm.yaml).
Shadow run (nothing drives, see what the stack would send): `mux_output:=/cmd_vel_dry`.
Every node gets `OMP_NUM_THREADS=1` and `OPENBLAS_NUM_THREADS=1` (docs/PIPELINE_LOAD_PLAN.md 6.0).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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

    switches = ['use_lane', 'use_yolo', 'use_lidar_missions', 'use_parking', 'use_finish', 'use_mission']
    platform_args = ['camera_pitch', 'use_camera', 'use_lidar', 'use_ekf', 'use_joint_states',
                     'steering_scale', 'max_inner_angle']
    debug = LaunchConfiguration('debug')
    # defaults must match bringup.launch.py
    mux_args = {'require_arm': 'false', 'mux_output': '/cmd_vel', 'cmd_timeout': '0.5'}

    return LaunchDescription([
        # one BLAS/OpenMP thread per process: numpy/torch would otherwise start one per core in
        # every node and fight over the Jetson's cores (docs/PIPELINE_LOAD_PLAN.md 6.0)
        SetEnvironmentVariable('OMP_NUM_THREADS', '1'),
        SetEnvironmentVariable('OPENBLAS_NUM_THREADS', '1'),
        *[DeclareLaunchArgument(name, default_value='true') for name in switches],
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('viz', default_value='false',
                              description='start decision_viz (clothoid_debug)'),
        DeclareLaunchArgument('stop_viz', default_value='false',
                              description='start the stop line view: stop_viz + RViz rviz/stop.rviz'),
        DeclareLaunchArgument('use_stopline', default_value='false'),
        DeclareLaunchArgument('share_lane_camera', default_value='true'),
        DeclareLaunchArgument('wait_for_yolo', default_value='true',
                              description='hold automatic driving until YOLO completes its first image'),
        DeclareLaunchArgument('use_speed_zone', default_value='false'),
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
        DeclareLaunchArgument('use_joint_states', default_value='false'),
        DeclareLaunchArgument('steering_scale', default_value='1.06'),
        DeclareLaunchArgument('max_inner_angle', default_value='0.48869'),
        *[DeclareLaunchArgument(name, default_value=value) for name, value in mux_args.items()],

        # sensors, TF, limo_base, twist_mux
        include('clothoid_bringup', 'bringup.launch.py',
                args={**{name: LaunchConfiguration(name) for name in [*platform_args, *mux_args]},
                      'require_yolo_ready': PythonExpression([
                          "'", LaunchConfiguration('use_yolo'), "'.lower() == 'true' and '",
                          LaunchConfiguration('wait_for_yolo'), "'.lower() == 'true'"])}),

        include('clothoid_lane', 'lane.launch.py',
                {'debug': debug, 'camera': LaunchConfiguration('camera'),
                 'use_stopline': LaunchConfiguration('use_stopline'),
                 'share_lane_camera': LaunchConfiguration('share_lane_camera'),
                 'use_speed_zone': LaunchConfiguration('use_speed_zone')}, enabled('use_lane')),
        include('clothoid_yolo', 'yolo.launch.py',
                {'backend': LaunchConfiguration('backend'),
                 'model_path': LaunchConfiguration('model_path'),
                 'show': LaunchConfiguration('show')}, enabled('use_yolo')),
        include('clothoid_lidar', 'cone.launch.py', condition=enabled('use_lidar_missions')),
        include('clothoid_parking', 'parking.launch.py', {'debug': debug}, enabled('use_parking')),
        # after /parking_done: left yellow line to the end line (needs the camera)
        include('clothoid_finish', 'finish.launch.py',
                {'debug': debug, 'camera': LaunchConfiguration('camera')}, enabled('use_finish')),
        # mission also runs the light / pedestrian line stop (FinishCore, camera only while armed)
        include('clothoid_mission', 'mission.launch.py',
                {'debug': debug, 'camera': LaunchConfiguration('camera'),
                 'parking_enabled': LaunchConfiguration('use_parking')}, enabled('use_mission')),
        # YOLO boxes, mission state, stop line / stop point in RViz (rviz/stop.rviz), stop_viz:=true
        include('clothoid_debug', 'stop_viz.launch.py', condition=enabled('stop_viz')),

        # read-only rate / latency / mux-winner log (clothoid_bringup/latency_monitor.py)
        DeclareLaunchArgument('use_latency_monitor', default_value='false'),
        DeclareLaunchArgument('latency_csv', default_value='',
                              description='append every report to this CSV, e.g. ~/bags/latency.csv'),
        Node(package='clothoid_bringup', executable='latency_monitor', name='latency_monitor',
             output='screen', parameters=[{'csv_path': LaunchConfiguration('latency_csv')}],
             condition=enabled('use_latency_monitor')),

        # what rviz/decision.rviz shows, only with viz:=true
        include('clothoid_debug', 'viz.launch.py', {'camera': LaunchConfiguration('camera')},
                enabled('viz')),
    ])
