"""decision_viz alone. full.launch.py starts it with `debug:=true`, bag.launch.py always."""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # lane.yaml is keyed to lane_main, so hand decision_viz the same warp values from it
    path = os.path.join(get_package_share_directory('clothoid_lane'), 'config', 'lane.yaml')
    with open(path) as f:
        lane = yaml.safe_load(f)['lane_main']['ros__parameters']
    warp = {k: lane[k] for k in ('warp_x_h', 'warp_x_l', 'warp_y_h', 'warp_y_l')}

    return LaunchDescription([
        Node(package='clothoid_debug', executable='decision_viz', name='decision_viz',
             output='screen', parameters=[warp]),
    ])
