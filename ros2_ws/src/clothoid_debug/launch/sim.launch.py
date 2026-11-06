"""The real perception, mission and command nodes driving the simulator in simul/.

Start the simulator first, see docker/README.md. It stands in for the sensors and limo_base, and its
/cmd_vel comes from twist_mux, as on the car.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter


def share(package, *path):
    return os.path.join(get_package_share_directory(package), *path)


def generate_launch_description():
    def include(package, launch_file, args=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share(package, 'launch', launch_file)),
            launch_arguments=(args or {}).items())

    # the simulator camera uses limo1's warp
    warp = share('clothoid_lane', 'config', 'camera_limo1.yaml')

    return LaunchDescription([
        GroupAction([
            SetParameter(name='use_sim_time', value=True),
            include('clothoid_lane', 'lane.launch.py', {'debug': 'true'}),
            include('clothoid_lidar', 'cone.launch.py'),
            include('clothoid_mission', 'mission.launch.py'),
            Node(package='twist_mux', executable='twist_mux', name='twist_mux', output='screen',
                 parameters=[share('clothoid_bringup', 'config', 'twist_mux.yaml')],
                 remappings=[('cmd_vel_out', '/cmd_vel')]),
            Node(package='clothoid_debug', executable='decision_viz', name='decision_viz',
                 output='screen', parameters=[warp, {'use_odom_pose': True}]),
        ]),
    ])
