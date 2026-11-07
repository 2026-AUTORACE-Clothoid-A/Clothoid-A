"""The real perception and mission nodes for a bag, which holds raw sensors only.

    ros2 launch clothoid_debug bag.launch.py
    ros2 bag play <bag_dir> --clock
    rviz2 -d <repo>/ros2_ws/src/clothoid_debug/rviz/decision.rviz --ros-args -p use_sim_time:=true

No limo_base and no twist_mux here, so nothing drives /cmd_vel. On the car use
`full.launch.py debug:=true` instead of this file.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    def include(package, launch_file, args=None):
        path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                        launch_arguments=(args or {}).items())

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='true needs `ros2 bag play --clock`'),
        DeclareLaunchArgument('camera', default_value='limo2',
                              description='camera profile of the car that recorded the bag'),
        GroupAction([
            SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
            include('clothoid_lane', 'lane.launch.py',
                    {'debug': 'true', 'camera': LaunchConfiguration('camera')}),
            # limo2 bags hold only the compressed camera image, lane_main reads raw
            Node(package='image_transport', executable='republish', name='camera_decompress',
                 arguments=['compressed', 'raw'],
                 remappings=[('in/compressed', '/camera/color/image_raw/compressed'),
                             ('out', '/camera/color/image_raw')]),
            include('clothoid_lidar', 'cone.launch.py'),
            include('clothoid_mission', 'mission.launch.py'),
            # the bag has this TF only in its first moment, a start offset would skip it (URDF value)
            Node(package='tf2_ros', executable='static_transform_publisher',
                 name='base_link_to_laser_link',
                 arguments=['--x', '0.103', '--z', '-0.034',
                            '--frame-id', 'base_link', '--child-frame-id', 'laser_link']),
            include('clothoid_debug', 'viz.launch.py', {'camera': LaunchConfiguration('camera')}),
        ]),
    ])
