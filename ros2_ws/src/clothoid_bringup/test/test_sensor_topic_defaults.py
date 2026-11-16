"""Runtime nodes use the averaged sensors with or without the launch YAML files."""
import importlib
from pathlib import Path

import pytest
import rclpy


ROOT = Path(__file__).parents[2]
CASES = [
    ('clothoid_lane.lane_main', 'LaneMainNode', 'clothoid_lane/config/lane.yaml'),
    ('clothoid_mission.mission_node', 'MissionNode', 'clothoid_mission/config/mission.yaml'),
    ('clothoid_parking.parking_node', 'ParkingNode', 'clothoid_parking/config/parking.yaml'),
    ('clothoid_finish.finish_node', 'FinishNode', 'clothoid_finish/config/finish.yaml'),
    ('clothoid_lidar.static_avoid_node', 'StaticAvoidNode', 'clothoid_lidar/config/static_avoid.yaml'),
    ('clothoid_lidar.cone_follower_node', 'ConeFollowerNode', 'clothoid_lidar/config/lidar.yaml'),
    ('clothoid_debug.decision_viz', 'DecisionViz', None),
]


@pytest.mark.parametrize('configured', [False, True], ids=['standalone', 'launch-config'])
@pytest.mark.parametrize('module,cls,config', CASES, ids=[c[1] for c in CASES])
def test_nodes_subscribe_only_to_25hz_sensors(module, cls, config, configured):
    args = ['--ros-args', '-p', 'overlay:=false']
    if configured:
        args += ['--params-file', str(ROOT/'clothoid_bringup/config/topics.yaml')]
        if config:
            args += ['--params-file', str(ROOT/config)]
    rclpy.init(args=args)
    node = getattr(importlib.import_module(module), cls)()
    try:
        if cls in ('ParkingNode', 'FinishNode'):
            node._sensors_on(True)
        topics = {s.topic_name for s in node.subscriptions}
        assert {'/imu_25hz', '/odom_25hz'} <= topics
        assert not {'/imu', '/odom'} & topics
        if cls in ('ParkingNode', 'FinishNode'):
            node._sensors_on(False)
            assert not {'/imu_25hz', '/odom_25hz'} & {s.topic_name for s in node.subscriptions}
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('module,cls', [(c[0], c[1]) for c in CASES if c[1] in ('ConeFollowerNode', 'DecisionViz')])
def test_explicit_raw_topic_overrides_remain_available_for_old_bags(module, cls):
    rclpy.init(args=['--ros-args', '-p', 'overlay:=false',
                    '-p', 'imu_topic:=/imu', '-p', 'odom_topic:=/odom'])
    node = getattr(importlib.import_module(module), cls)()
    try:
        topics = {s.topic_name for s in node.subscriptions}
        assert {'/imu', '/odom'} <= topics
        assert not {'/imu_25hz', '/odom_25hz'} & topics
    finally:
        node.destroy_node()
        rclpy.shutdown()
