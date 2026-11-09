"""decision_viz alone. full.launch.py starts it with `viz:=true`, bag.launch.py always."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # the same per car warp and camera pitch as lane_main (clothoid_lane/config/camera_*.yaml)
    camera = PathJoinSubstitution([FindPackageShare('clothoid_lane'), 'config',
                                   ['camera_', LaunchConfiguration('camera'), '.yaml']])
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='limo2',
                              description='camera profile: limo1 or limo2'),
        Node(package='clothoid_debug', executable='decision_viz', name='decision_viz',
             output='screen', parameters=[camera]),
    ])
