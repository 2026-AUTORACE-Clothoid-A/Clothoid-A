from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage
from clothoid_interfaces.msg import Detection, DetectionArray
from clothoid_mission.mission_node import MissionNode
from clothoid_mission.mission_logic import STOPPED_LIGHT, DRIVE


@pytest.fixture
def node():
    config = Path(__file__).parents[1] / 'config/mission.yaml'
    # These tests exercise identity and floor evidence with small synthetic boxes.
    # The configured proximity thresholds are tested in test_bump_sign_policy.py.
    args = ['--ros-args', '--params-file', str(config)]
    for group in ('light', 'sign', 'fork'):
        args += ['-p', group + '_max_offset_height_ratio:=0.0']
    for kind in ('bump', 'slow', 'parking', 'pedestrian'):
        args += ['-p', kind + '_min_area:=0.0', '-p', kind + '_min_height:=0.0']
    rclpy.init(args=args)
    node = MissionNode()
    clock = [1000.]
    node.now_sec = lambda: clock[0]
    yield node, clock
    node.destroy_node()
    rclpy.shutdown()


def detections(node, clock, now, name, x=100):
    clock[0] = now
    msg = DetectionArray(detections=[Detection(class_name=name, confidence=.9,
                                              x_min=x, x_max=x + 20, y_min=200, y_max=220)])
    msg.header.stamp.sec = int(now)
    msg.header.stamp.nanosec = int(round((now - int(now)) * 1e9))
    node.on_detections(msg)
    return msg


def test_selected_red_is_not_released_by_another_green(node):
    n, clock = node
    for now in (1000., 1000.1, 1000.2):
        detections(n, clock, now, 'traffic_light_red')
    n.on_timer()
    chosen = n.light_target_id
    n.logic.state = STOPPED_LIGHT
    for now in (1000.3, 1000.4, 1000.5):
        detections(n, clock, now, 'traffic_light_green', x=450)
        n.on_timer()
    assert n.logic.state == STOPPED_LIGHT and n.light_target_id == chosen
    for now in (1000.6, 1000.7, 1000.8):
        detections(n, clock, now, 'traffic_light_green')
        n.on_timer()
    assert n.logic.state == DRIVE


def test_delayed_source_data_and_duplicates_do_not_refresh_evidence(node):
    n, clock = node
    message = detections(n, clock, 1000., 'sign_parking')
    n.on_detections(message)
    n.on_detections(message)
    assert not n.seen
    clock[0] = 1001.
    n.on_detections(message)
    assert not n.seen


def test_parking_handover_requires_fresh_bay_and_reached_separate_entry(node):
    n, clock = node
    for now in (1000., 1000.1, 1000.2):
        detections(n, clock, now, 'sign_parking')
    n.on_timer()
    assert n.logic.parking_armed and not n.logic.parking_flag
    scene = PoseStamped()
    scene.header.frame_id, scene.header.stamp.sec = 'base_link', 1000
    scene.header.stamp.nanosec = 200_000_000
    scene.pose.position.x, scene.pose.position.y = 1., -.4
    n.on_parking_scene(scene)
    n.on_timer()
    assert not n.logic.parking_flag
    n.entry_evidence.entry_at = n.travel + n.finish_params.front
    n.on_timer()
    assert n.logic.parking_flag


def test_parking_scene_from_wrong_frame_or_expired_sensor_is_not_used(node):
    n, clock = node
    for now in (1000., 1000.1, 1000.2):
        detections(n, clock, now, 'sign_parking')
    n.on_timer()
    scene = PoseStamped()
    scene.pose.position.x = 1.
    scene.header.frame_id, scene.header.stamp.sec = 'laser_link', 1000
    n.on_parking_scene(scene)
    assert n.parking_scene_time is None
    scene.header.frame_id = 'base_link'
    clock[0] = 1001.
    n.on_parking_scene(scene)
    assert n.parking_scene_time is None


def test_bump_approach_is_published_refreshed_and_cleared_on_timeout(node):
    n, clock = node
    flags = []
    n.bump_approach_pub = SimpleNamespace(publish=lambda msg: flags.append(msg.data))
    for now in (1000., 1000.1, 1000.2):
        detections(n, clock, now, 'sign_bump')
    n.on_timer()
    assert flags == [True]
    clock[0] += 1.1
    n.on_timer()
    assert flags == [True, True]
    clock[0] += n.logic.p.bump_arm_timeout_sec
    n.on_timer()
    assert flags == [True, True, False]
