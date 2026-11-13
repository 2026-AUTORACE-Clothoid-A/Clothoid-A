import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('clothoid_monitor')
    default_config = os.path.join(pkg_share, 'config', 'monitor.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to monitor config yaml file'
    )
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='5000',
        description='Web port for dashboard'
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/color/image_raw',
        description='Image topic to display'
    )
    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/scan',
        description='LaserScan topic to display in BEV radar'
    )

    monitor_node = Node(
        package='clothoid_monitor',
        executable='monitor_node',
        name='clothoid_monitor_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config'),
            {
                'web_port': LaunchConfiguration('port'),
                'image_topic': LaunchConfiguration('image_topic'),
                'lidar_topic': LaunchConfiguration('lidar_topic'),
            }
        ]
    )

    return LaunchDescription([
        config_arg,
        port_arg,
        image_topic_arg,
        lidar_topic_arg,
        monitor_node,
    ])
