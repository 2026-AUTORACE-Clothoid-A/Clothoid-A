"""RViz with rviz/decision.rviz. An .rviz file cannot hold use_sim_time, it is a node parameter.

    ros2 launch clothoid_debug rviz.launch.py                        # bag with `ros2 bag play --clock`
    ros2 launch clothoid_debug rviz.launch.py use_sim_time:=false    # on the car, or a bag played from rqt
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('clothoid_debug'), 'rviz', 'decision.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', config],
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
