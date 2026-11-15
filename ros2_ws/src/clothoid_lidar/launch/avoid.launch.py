"""Static obstacle mission: LIMO extractor and static_avoid. full.launch.py should include this next
to cone.launch.py. The camera red share (red_ratio node) is not started: off by default, see
config/static_avoid.yaml.

The camera geometry comes from the lane package's per-car file (camera_<camera>.yaml) and the white
threshold from lane.yaml's lane_main section, so the entry-line detector uses the lane's values;
static_avoid also follows lane_main's white_* when they are changed at run time.

    ros2 launch clothoid_lidar avoid.launch.py                     # on the car (camera:=limo2 = car 7)
    ros2 launch clothoid_lidar avoid.launch.py use_sim_time:=true  # with `ros2 bag play --clock`
"""
import os

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _lane_files(camera):
    """(camera profile path or None, {white_*, yellow_*} from lane.yaml or {})."""
    try:
        lane = get_package_share_directory('clothoid_lane')
    except PackageNotFoundError:
        return None, {}
    cam = os.path.join(lane, 'config', f'camera_{camera}.yaml')
    white = {}
    try:
        with open(os.path.join(lane, 'config', 'lane.yaml')) as f:
            main = (yaml.safe_load(f) or {}).get('lane_main', {}).get('ros__parameters', {})
        white = {k: int(main[k]) for k in ('white_s_max', 'white_v_min') if k in main}
        # the node refuses main's 2025 yellow (S >= 215) with a warning
        white.update({new: [int(x) for x in main[old]] for old, new in
                      (('hsv_lower', 'yellow_lower'), ('hsv_upper', 'yellow_upper')) if old in main})
    except OSError:
        pass
    return (cam if os.path.exists(cam) else None), white


def _nodes(context):
    lidar = get_package_share_directory('clothoid_lidar')
    topics = os.path.join(get_package_share_directory('clothoid_bringup'), 'config', 'topics.yaml')
    lidar_yaml = os.path.join(lidar, 'config', 'lidar.yaml')
    avoid_yaml = os.path.join(lidar, 'config', 'static_avoid.yaml')
    sim = {'use_sim_time': LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'}
    camera = LaunchConfiguration('camera').perform(context)
    flags = {k: LaunchConfiguration(k).perform(context).lower() == 'true'
             for k in ('ignore_cone', 'debug_heavy', 'course_map')}
    flags['recovery_reverse'] = LaunchConfiguration('reverse').perform(context).lower() == 'true'
    flags['escape'] = LaunchConfiguration('escape').perform(context).lower() == 'true'
    cam_file, white = _lane_files(camera)
    extra = LaunchConfiguration('params').perform(context)
    flags['image_source'] = LaunchConfiguration('image').perform(context)
    avoid_params = ([topics, avoid_yaml] + ([cam_file] if cam_file else []) + [white, sim, flags]
                    + ([extra] if extra else []))
    note = (f'static_avoid: camera file {cam_file}' if cam_file else
            f'static_avoid: no camera_{camera}.yaml in clothoid_lane, built-in limo2 values')
    return [
        LogInfo(msg=note + (f', lane white {white}' if white else ', built-in white threshold')),
        Node(package='clothoid_lidar', executable='obstacle_extractor', name='limo_extractor',
             output='screen', parameters=[topics, lidar_yaml, sim]),
        Node(package='clothoid_lidar', executable='static_avoid', name='static_avoid',
             output='screen', parameters=avoid_params),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('camera', default_value='limo2',
                              description='lane camera profile: limo1 | limo2 (limo2 = car 7)'),
        DeclareLaunchArgument('ignore_cone', default_value='false',
                              description='test track only: its cone wall beside the section ends the avoidance'),
        DeclareLaunchArgument('debug_heavy', default_value='false',
                              description='also publish /avoid/candidates and /avoid/costmap (slow on the car)'),
        DeclareLaunchArgument('image', default_value='10hz', choices=['10hz', 'raw'],
                              description='10hz: /camera/color/image_10hz (bringup use_image_10hz:=true), '
                                          'every frame; raw: /camera/color/image_raw, every 3rd frame'),
        DeclareLaunchArgument('params', default_value='',
                              description='extra static_avoid parameter file, last (e.g. the simulator)'),
        DeclareLaunchArgument('course_map', default_value='true',
                              description='/imu + /odom course map from where the node starts: at the race '
                                          'start the section start line starts the mission; elsewhere '
                                          '(judged from the first turn) a blocker does. false: always '
                                          'a blocker (an avoid-only test started in the section itself)'),
        DeclareLaunchArgument('reverse', default_value='false',
                              description='back up 0.2 m when every path stays blocked (recovery)'),
        DeclareLaunchArgument('escape', default_value='false',
                              description='stuck: drive a free curvature-limited three-arc escape path '
                                          '(AvoidParams.escape; not yet seen on the car)'),
        OpaqueFunction(function=_nodes),
    ])
