"""Configured proximity gates retain the roadside bump warning and reject distant boxes."""
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32
from clothoid_interfaces.msg import Detection, DetectionArray
from clothoid_mission.mission_logic import classes_seen
from clothoid_mission.mission_node import MissionNode


@pytest.fixture
def mission():
    config = Path(__file__).parents[1]/'config/mission.yaml'
    rclpy.init(args=['--ros-args', '--params-file', str(config)])
    node = MissionNode()
    probe = Node('bump_policy_probe')
    limits = []
    probe.create_subscription(Float32, '/speed_limit', lambda m: limits.append(m.data), 10)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(probe)
    yield node, probe, ex, limits
    ex.shutdown()
    node.destroy_node()
    probe.destroy_node()
    rclpy.shutdown()


def detection(x, size, confidence=.9):
    return Detection(class_name='sign_bump', confidence=confidence,
                     x_min=x, y_min=180, x_max=x+size, y_max=180+size)


@pytest.mark.parametrize('x,size', [(8,28), (100,32), (300,36), (600,28)])
def test_configured_node_confirms_roadside_sign_and_requests_slow_speed(mission, x, size):
    import time
    node, _, ex, limits = mission
    for index in range(2):
        node.on_detections(DetectionArray(detections=[detection(x+index, size)]))
        assert not node.seen
    node.on_detections(DetectionArray())  # one missing frame is tolerated
    node.on_detections(DetectionArray(detections=[detection(x+2,size)]))
    assert node.seen == {'sign_bump'}
    node.on_timer()
    assert node.logic.bump_state == 'approaching'
    deadline = time.monotonic()+2.
    while not limits and time.monotonic()<deadline:
        ex.spin_once(timeout_sec=.01)
    assert limits and limits[-1] == pytest.approx(.2)


@pytest.mark.parametrize('x,size,confidence', [(20,30,.59), (0,30,.9), (20,27,.9)])
def test_low_confidence_clipped_and_unresolvable_signs_still_fail(mission, x, size, confidence):
    node, _, _, _ = mission
    for _ in range(5):
        node.on_detections(DetectionArray(detections=[detection(x,size,confidence)]))
    assert not node.seen
    node.on_timer()
    assert node.logic.bump_state == 'idle'


def test_proximity_filter_is_applied_to_each_box_before_tracking(mission):
    node, _, _, _ = mission
    for _ in range(3):
        node.on_detections(DetectionArray(detections=[detection(300,30), detection(400,16)]))
    assert node.seen == {'sign_bump'}
    assert len(node.sign_tracker.tracks) == 1
    assert node.confirmed_tracks[0].box.height == 30


def test_bump_height_can_be_tuned_without_restarting_the_node(mission):
    from rclpy.parameter import Parameter
    node, _, _, _ = mission
    node.set_parameters([Parameter('bump_min_height', value=32.)])
    for _ in range(3):
        node.on_detections(DetectionArray(detections=[detection(300,30)]))
    assert not node.seen
    node.set_parameters([Parameter('bump_min_height', value=28.)])
    for _ in range(3):
        node.on_detections(DetectionArray(detections=[detection(300,30)]))
    assert node.seen == {'sign_bump'}


@pytest.mark.parametrize('name,small,large', [('sign_parking',27,28),
                                          ('sign_pedestrian',27,28), ('sign_slow',30,50)])
def test_other_signs_keep_their_own_size_gate(mission, name, small, large):
    node, _, _, _ = mission
    for _ in range(3):
        msg = detection(300,small)
        msg.class_name = name
        node.on_detections(DetectionArray(detections=[msg]))
    assert not node.seen
    for _ in range(3):
        msg = detection(300,large)
        msg.class_name = name
        node.on_detections(DetectionArray(detections=[msg]))
    assert node.seen == {name}


def test_bump_override_does_not_relax_other_signs_or_lights():
    def box(name):
        return SimpleNamespace(class_name=name,x_min=10,y_min=200,x_max=90,y_max=240)
    boxes = [box(k) for k in ('sign_bump','sign_parking','sign_pedestrian','sign_slow',
                             'sign_left','traffic_light_red')]
    seen = classes_seen(boxes, bump_max_offset_height_ratio=0.,
        min_sizes={k:(2500.,40.) for k in ('sign_parking','sign_pedestrian','sign_slow')})
    assert seen == {'sign_bump','sign_left'}
    assert classes_seen([box('sign_bump')]) == set()  # API callers can still inherit the shared limit
