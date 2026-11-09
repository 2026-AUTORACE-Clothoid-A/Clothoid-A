"""RViz for the parking manoeuvre (rviz/parking.rviz, fixed frame base_link).

    ros2 launch clothoid_parking rviz.launch.py                       # simulator
    ros2 launch clothoid_parking rviz.launch.py use_sim_time:=false   # on the car
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('clothoid_parking'), 'rviz', 'parking.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='rviz2', executable='rviz2', name='parking_rviz', arguments=['-d', config],
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
