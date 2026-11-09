import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('camera', default_value='limo2',
                              description='camera profile: limo1 or limo2 (config/camera_*.yaml)'),
        # off by default to save CPU (limo2 2026-11-09: stopline ~84 % of a core). Off means no
        # /stopline_detected or /stopline_near: mission then never raises /parking_flag at a stop line
        # and never stops for the pedestrian / traffic light, so start parking with /parking_flag.
        DeclareLaunchArgument('use_stopline', default_value='false'),
        node('lane_main', 'lane_main', 'lane.yaml'),
        # speed_zone (floor colour -> /speed_limit) is not started any more: not needed on the 2026 track
        node('stopline', 'stopline', 'stopline.yaml',
             condition=IfCondition(LaunchConfiguration('use_stopline'))),
    ])
