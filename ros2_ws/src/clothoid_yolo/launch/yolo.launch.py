import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    yolo_share = get_package_share_directory('clothoid_yolo')

    parameters = [
        os.path.join(yolo_share, 'config', 'yolo.yaml'),
        {'backend': LaunchConfiguration('backend'),
         'model_path': LaunchConfiguration('model_path'),
         'show': LaunchConfiguration('show'),
         'debug': LaunchConfiguration('debug')},
    ]

    try:
        bringup_share = get_package_share_directory('clothoid_bringup')
        topics_cfg = os.path.join(bringup_share, 'config', 'topics.yaml')
        if os.path.isfile(topics_cfg):
            parameters.insert(0, topics_cfg)
    except PackageNotFoundError:
        pass

    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='ultralytics'),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('show', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),
        Node(
            package='clothoid_yolo',
            executable='yolo',
            name='yolo',
            output='screen',
            parameters=parameters,
        ),
    ])
