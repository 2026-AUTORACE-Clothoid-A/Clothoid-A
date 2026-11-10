import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    lane_share = get_package_share_directory('clothoid_lane')
    bringup_share = get_package_share_directory('clothoid_bringup')
    topics = os.path.join(bringup_share, 'config', 'topics.yaml')
    # per car warp (camera_limo1.yaml / camera_limo2.yaml), loaded last so it wins
    camera = PathJoinSubstitution(
        [lane_share, 'config', ['camera_', LaunchConfiguration('camera'), '.yaml']])

    def node(executable, name, yaml_name, extra=None, condition=None):
        return Node(
            condition=condition,
            package='clothoid_lane',
            executable=executable,
            name=name,
            output='screen',
            parameters=[topics, os.path.join(lane_share, 'config', yaml_name),
                        camera, {'debug': LaunchConfiguration('debug')}] + (extra or []),
        )

    def camera_nodes(context):
        stopline = IfCondition(LaunchConfiguration('use_stopline')).evaluate(context)
        shared = IfCondition(LaunchConfiguration('share_lane_camera')).evaluate(context)
        if stopline and shared:
            # No global __node remapping: retain distinct lane_main/stopline parameter scopes.
            return [Node(package='clothoid_lane', executable='lane_shared', output='screen',
                         parameters=[topics, os.path.join(lane_share, 'config', 'lane.yaml'),
                                     os.path.join(lane_share, 'config', 'stopline.yaml'),
                                     camera, {'debug': LaunchConfiguration('debug')}])]
        nodes = [node('lane_main', 'lane_main', 'lane.yaml')]
        if stopline:
            nodes.append(node('stopline', 'stopline', 'stopline.yaml'))
        return nodes

    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('camera', default_value='limo2',
                              description='camera profile: limo1 or limo2 (config/camera_*.yaml)'),
        # off by default to save CPU (limo2 2026-11-09: stopline ~84 % of a core). Off means no
        # /stopline_detected or /stopline_near: mission then never raises /parking_flag at a stop line
        # and never stops for the pedestrian / traffic light, so start parking with /parking_flag.
        DeclareLaunchArgument('use_stopline', default_value='false'),
        DeclareLaunchArgument('share_lane_camera', default_value='true',
                              description='reuse the lane image subscription for stopline'),
        # speed_zone: floor colour -> /speed_limit (2025 red zone). The 2026 slow zone is a YOLO sign,
        # so it is off by default.
        DeclareLaunchArgument('use_speed_zone', default_value='false'),
        OpaqueFunction(function=camera_nodes),
        node('speed_zone', 'speed_zone', 'speed_zone.yaml',
             condition=IfCondition(LaunchConfiguration('use_speed_zone'))),
    ])
