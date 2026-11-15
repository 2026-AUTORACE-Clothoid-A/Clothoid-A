from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
from clothoid_finish.bev import CameraModel
from clothoid_finish.finish_core import Phase
from clothoid_interfaces.msg import Detection, DetectionArray
from clothoid_mission.mission_node import MissionNode
from clothoid_mission.mission_logic import STOPPED_LIGHT, DRIVE


@pytest.fixture
def node():
    config = Path(__file__).parents[1] / 'config/mission.yaml'
    # These tests exercise identity and floor evidence with small synthetic boxes.
    # The configured proximity thresholds are tested in test_bump_sign_policy.py.
    args = ['--ros-args', '--params-file', str(config),
            '-p', 'require_cone_for_parking:=false']
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
    cam = CameraModel()
    n.on_info(CameraInfo(width=640, height=480,
                         k=[cam.fx, 0., cam.cx, 0., cam.fy, cam.cy, 0., 0., 1.]))
    for now in (1000., 1000.1, 1000.2):
        detections(n, clock, now, 'sign_bump')
    n.on_timer()
    assert flags == [False]
    n.core.phase = Phase.STOP
    n.on_timer()
    assert flags == [False, True]
    clock[0] += 1.1
    n.on_timer()
    assert flags == [False, True, True]
    clock[0] += n.logic.p.bump_arm_timeout_sec
    n.on_timer()
    assert flags == [False, True, True, False]


@pytest.mark.parametrize('sign', ['sign_slow', 'sign_bump'])
def test_stopline_mission_uses_camera_distance_and_measured_travel_before_slowing(node, sign):
    from test_line_stop import render

    n, clock = node
    cam = CameraModel()
    n.on_info(CameraInfo(width=640, height=480,
                         k=[cam.fx, 0., cam.cx, 0., cam.fy, cam.cy, 0., 0., 1.]))
    limits = []
    n.limit_pub = SimpleNamespace(publish=lambda msg: limits.append(msg.data))
    for now in (1000., 1000.1, 1000.2):
        detections(n, clock, now, sign)
    n.on_timer()
    core = n.core
    assert core.phase == Phase.FOLLOW and not core.require_crosswalk
    assert n.image_sub is not None and not limits

    image = render(cam, 1.3, 0., 0., crosswalk=False, letters=False)
    for _ in range(3):
        clock[0] += .05
        msg = n.bridge.cv2_to_imgmsg(image, 'bgr8')
        msg.header.stamp.sec = int(clock[0])
        msg.header.stamp.nanosec = int(round((clock[0] - int(clock[0])) * 1e9))
        n.on_image(msg)
        n.on_timer()
        assert not limits
    assert core.phase == Phase.COMMIT and core.remaining() > .4

    # The line can leave the image. Use measured 0.8 m/s, independently of the 0.2 slow setting.
    for _ in range(30):
        clock[0] += .05
        msg = Odometry()
        msg.header.stamp.sec = int(clock[0])
        msg.header.stamp.nanosec = int(round((clock[0] - int(clock[0])) * 1e9))
        msg.twist.twist.linear.x = .8
        n.on_odom(msg)
        n.on_timer()
        if limits:
            break
    assert core.phase == Phase.STOP
    assert limits == [pytest.approx(.2)]
    assert not n.logic.parking_flag and n.image_sub is None


def test_roadside_candidate_does_not_stop_or_park_without_scene(node):
    n,clock=node
    # Restore strict proximity gates: a 20px sign at the side fails the old rule.
    from rclpy.parameter import Parameter
    n.set_parameters([Parameter('scene_sign_candidates',value=True),
                      Parameter('sign_max_offset_height_ratio',value=5.),
                      Parameter('pedestrian_min_height',value=28.),
                      Parameter('parking_min_height',value=28.)])
    for name in ('sign_pedestrian','sign_parking'):
        for k in range(3):
            clock[0]+=.1
            detections(n,clock,clock[0],name,x=10)
            n.on_timer()
        assert name in n.seen
        assert n.logic.state==DRIVE and not n.logic.parking_flag
    assert n.logic.parking_armed


def test_scene_candidate_quality_gates_remain(node):
    n,clock=node
    for k in range(5):
        msg=detections(n,clock,1000.+k*.1,'sign_pedestrian',x=0)
    assert 'sign_pedestrian' not in n.seen


def test_observation_extends_parking_arm_but_skip_does_not_complete_mission():
    from clothoid_mission.mission_logic import MissionLogic,MissionParams,Inputs
    m=MissionLogic(MissionParams(require_cone_for_parking=False))
    m.step(Inputs(now=0.,seen={'sign_parking'}))
    assert m.step(Inputs(now=9.,parking_observing=True)).parking_prepare
    out=m.step(Inputs(now=10.,parking_skipped=True,seen={'sign_parking'}))
    assert not out.parking_flag and not out.parking_prepare and out.state==DRIVE
    assert not m.step(Inputs(now=11.,seen={'sign_parking'})).parking_prepare
    # A missed parking mission cannot block a subsequent independent pedestrian mission.
    out=m.step(Inputs(now=11.1,seen={'sign_pedestrian'},line_phase='arrived'))
    assert out.state=='stopped_pedestrian'


def test_fork_confirms_on_two_frames_while_parking_keeps_three(node):
    n, clock = node
    detections(n, clock, 1000., 'sign_left')
    assert 'sign_left' not in n.seen
    detections(n, clock, 1000.1, 'sign_left')
    assert 'sign_left' in n.seen
    detections(n, clock, 1000.2, 'sign_parking')
    detections(n, clock, 1000.3, 'sign_parking')
    assert 'sign_parking' not in n.seen
    detections(n, clock, 1000.4, 'sign_parking')
    assert 'sign_parking' in n.seen


@pytest.mark.parametrize('name,direction', [('sign_left', 0), ('sign_right', 1)])
def test_configured_fork_sweep_confirms_after_missing_frames(node, name, direction):
    n, clock = node
    first = DetectionArray(detections=[Detection(class_name=name, confidence=.55,
                          x_min=0, x_max=20, y_min=200, y_max=220)])
    first.header.stamp.sec = 1000
    n.on_detections(first)
    assert name not in n.seen
    for k in range(1, 8):
        clock[0] = 1000. + .1 * k
        msg = DetectionArray()
        msg.header.stamp.sec = 1000
        msg.header.stamp.nanosec = k * 100_000_000
        n.on_detections(msg)
    clock[0] = 1000.8
    msg = DetectionArray(detections=[Detection(class_name=name, confidence=.55,
                        x_min=450, x_max=470, y_min=200, y_max=220)])
    msg.header.stamp.sec, msg.header.stamp.nanosec = 1000, 800_000_000
    n.on_detections(msg)
    n.on_timer()
    assert name in n.seen and n.logic.direction == direction
    assert not n.logic.parking_flag and n.logic.state == DRIVE
