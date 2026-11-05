"""Bring up the LIMO: description/TF, base driver, camera, lidar, twist_mux.

Based on the platform's `wego/launch/teleop_launch.py`, without rviz and with only the
color stream enabled (the DaBai DC1 is on USB2.0 and only reaches ~12 Hz with every stream on).
Run on the LIMO (Jetson) after sourcing ~/wego_ws and the clothoid workspace.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def static_tf(name, parent, child, xyz, rpy=('0', '0', '0'), pitch=None):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            '--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
            '--roll', rpy[0], '--pitch', pitch if pitch is not None else rpy[1],
            '--yaw', rpy[2],
            '--frame-id', parent, '--child-frame-id', child,
        ],
    )


def generate_launch_description():
    bringup_share = get_package_share_directory('clothoid_bringup')
    twist_mux_params = os.path.join(bringup_share, 'config', 'twist_mux.yaml')

    def include(package, *path, args=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(package), *path])),
            launch_arguments=(args or {}).items())

    camera_pitch = LaunchConfiguration('camera_pitch')

    return LaunchDescription([
        DeclareLaunchArgument('camera_pitch', default_value='0.0',
                              description='camera pitch in rad, measure the real mounting angle'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_ekf', default_value='false',
                              description='fuse /odom and /imu into /odometry/filtered'),
        DeclareLaunchArgument('steering_scale', default_value='2.47',
                              description='limo_base: inner wheel angle per body value (official 2.47)'),
        DeclareLaunchArgument('max_inner_angle', default_value='0.48869',
                              description='limo_base: inner wheel limit in rad (official 28 deg)'),

        # URDF: base_link -> laser_link, imu_link, wheels
        include('limo_description', 'launch', 'load_urdf.launch.py'),

        # camera mounting (same nominal values as wego/camera_tilt_launch.py; measure and update)
        static_tf('base_link_to_camera_link', 'base_link', 'camera_link',
                  xyz=(0.23, 0.05, 0.06), pitch=camera_pitch),
        # optical frame (z forward, x right, y down)
        static_tf('camera_link_to_optical', 'camera_link', 'camera_color_optical_frame',
                  xyz=(0, 0, 0), rpy=('-1.5708', '0', '-1.5708')),

        include('limo_base', 'launch', 'limo_base.launch.py', args={
            name: LaunchConfiguration(name) for name in ('steering_scale', 'max_inner_angle')}),

        # DaBai DC1 runs on astra_stereo_u3.launch.py. Color only; the driver's own TF has a
        # unit error (29 m), so publish_tf is off and we publish the TF above.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('orbbec_camera'), 'launch', 'astra_stereo_u3.launch.py'])),
            launch_arguments={
                'enable_depth': 'false',
                'enable_ir': 'false',
                'enable_point_cloud': 'false',
                'enable_colored_point_cloud': 'false',
                'publish_tf': 'false',
            }.items(),
            condition=IfCondition(LaunchConfiguration('use_camera'))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('ydlidar_ros2_driver'), 'launch', 'ydlidar.launch.py'])),
            condition=IfCondition(LaunchConfiguration('use_lidar'))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('robot_localization'), 'launch', 'limo_ekf_launch.py'])),
            condition=IfCondition(LaunchConfiguration('use_ekf'))),

        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[twist_mux_params],
            remappings=[('cmd_vel_out', '/cmd_vel')],
        ),
    ])
