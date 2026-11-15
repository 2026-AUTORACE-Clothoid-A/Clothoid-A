"""Stop line view alone: stop_viz + RViz (rviz/stop.rviz). Works on the car and in the simulator.

    ros2 launch clothoid_debug stop_viz.launch.py                 # with RViz
    ros2 launch clothoid_debug stop_viz.launch.py rviz:=false     # stop_viz only (RViz elsewhere)

Only listens (camera, /detections, /mission/state, /mission/line_debug_image with mission debug:=true),
so it can be started and stopped at any time while the car drives. full.launch.py starts it with
stop_viz:=true.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution([FindPackageShare('clothoid_debug'), 'rviz', 'stop.rviz'])
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='clothoid_debug', executable='stop_viz', name='stop_viz', output='screen',
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        Node(package='rviz2', executable='rviz2', name='stop_rviz', output='log',
             arguments=['-d', rviz_config],
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
