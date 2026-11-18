import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from nav_msgs.msg import Odometry
from clothoid_finish.bev import CameraModel
from clothoid_finish.finish_core import Phase
from clothoid_interfaces.msg import Detection, DetectionArray
from clothoid_mission.mission_node import MissionNode
from clothoid_mission.mission_logic import STOPPED_LIGHT, DRIVE
from rclpy.parameter import Parameter


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


def test_live_white_hue_updates_existing_core_and_preserves_the_committed_target(node):
    n, _ = node
    cam = CameraModel()
    n.on_info(CameraInfo(width=640, height=480,
        k=[cam.fx, 0., cam.cx, 0., cam.fy, cam.cy, 0., 0., 1.]))
    core = n.core
    core.phase, core.target = Phase.COMMIT, 1.2
    result = n.set_parameters_atomically([Parameter('white_h_max', value=45)])
    assert result.successful
    assert n.core is core and core.white_h_max == 45
    assert core.phase == Phase.COMMIT and core.target == 1.2
    assert n.new_core().white_h_max == 45
    result = n.set_parameters_atomically([Parameter('white_h_min', value=60)])
    assert not result.successful
    assert core.white_h_min == n.get_parameter('white_h_min').value == 0
    assert core.white_h_max == 45


def test_color_switch_preserves_committed_mission_and_updates_new_cores(node):
    n, _ = node
    cam = CameraModel()
    n.on_info(CameraInfo(width=640, height=480,
        k=[cam.fx, 0., cam.cx, 0., cam.fy, cam.cy, 0., 0., 1.]))
    core = n.core
    core.phase, core.target = Phase.COMMIT, 1.2
    assert n.set_parameters_atomically([Parameter('color_method', value='rgb'),
        Parameter('gamma_enabled', value=True)]).successful
    assert n.core is core and core.target == 1.2 and core.phase == Phase.COMMIT
    assert core.color_pipeline.p.color_method == n.new_core().color_pipeline.p.color_method == 'rgb'
    pipeline = core.color_pipeline
    assert not n.set_parameters_atomically([Parameter('gamma', value=-1.)]).successful
    assert core.color_pipeline is pipeline and n.get_parameter('gamma').value == 1.2


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


def test_overlapping_red_on_selected_signal_holds_until_three_green_only_frames(node):
    n, clock = node
    for now in (1000., 1000.1, 1000.2):
        detections(n, clock, now, 'traffic_light_red')
    n.on_timer()
    n.logic.state = STOPPED_LIGHT
    for k, now in enumerate((1000.3, 1000.4, 1000.5)):
        message = detections(n, clock, now, 'traffic_light_red')
        # Use a new, later source frame containing both overlapping colours.
        clock[0] += .01
        message.header.stamp.nanosec += 10_000_000
        message.detections.append(Detection(class_name='traffic_light_green', confidence=.7,
            x_min=100, x_max=120, y_min=200, y_max=220))
        n.on_detections(message)
        n.on_timer()
        assert n.logic.state == STOPPED_LIGHT
    for k, now in enumerate((1000.6, 1000.7, 1000.8)):
        detections(n, clock, now, 'traffic_light_green')
        n.on_timer()
        assert n.logic.state == (DRIVE if k == 2 else STOPPED_LIGHT)


@pytest.mark.parametrize('stop_observer', [False, True])
def test_entry_analysis_is_limited_to_ten_slots_without_throttling_active_stop(node, stop_observer):
    n, clock = node
    cam = CameraModel()
    n.on_info(CameraInfo(width=640, height=480,
        k=[cam.fx, 0., cam.cx, 0., cam.fy, cam.cy, 0., 0., 1.]))
    detections(n, clock, 1000., 'sign_pedestrian')
    n.on_timer()
    if stop_observer:
        n.core.start(1000.)
    calls = []
    n.core.image = lambda *args: calls.append(n.core.observe_entry)
    for k in range(30):
        clock[0] = 1000.+k/30
        message = Image(height=1, width=1, encoding='bgr8', step=3, data=[0, 0, 0])
        message.header.stamp.sec = 1000
        message.header.stamp.nanosec = round(k/30*1e9)
        n.on_image(message)
    assert len(calls) == (30 if stop_observer else 10)
    assert sum(calls) == 10


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
    limits = []
    n.limit_pub = SimpleNamespace(publish=lambda msg: limits.append(msg.data))
    for now in (1000., 1000.1, 1000.2):
        detections(n, clock, now, 'sign_parking')
    n.on_timer()
    assert n.logic.parking_armed and not n.logic.parking_flag
    assert not limits
    n.entry_evidence.entry_at = n.travel + n.finish_params.front + .25
    n.on_timer()
    assert .2 < limits[-1] < .8 and not n.logic.parking_flag
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
    n.mission_entry.entry_at = n.travel + .1
    n.on_timer()
    assert flags == [False, True]
    clock[0] += 1.1
    n.on_timer()
    assert flags == [False, True, True]
    clock[0] += n.logic.p.bump_arm_timeout_sec
    n.on_timer()
    assert flags == [False, True, True, False]


@pytest.mark.parametrize('sign', ['sign_slow', 'sign_bump'])
def test_mission_start_brakes_before_front_wheel_entry_without_committing_a_stop(node, sign):
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
    assert core.phase == Phase.IDLE and n.mission_entry_active
    assert n.image_sub is not None and not limits

    def odom(speed):
        msg = Odometry()
        msg.header.stamp.sec = int(clock[0])
        msg.header.stamp.nanosec = int(round((clock[0] - int(clock[0])) * 1e9))
        msg.twist.twist.linear.x = speed
        n.on_odom(msg)

    odom(.8)

    image = render(cam, 1.3, 0., 0., crosswalk=False, letters=False)
    for _ in range(3):
        clock[0] += .05
        msg = n.bridge.cv2_to_imgmsg(image, 'bgr8')
        msg.header.stamp.sec = int(clock[0])
        msg.header.stamp.nanosec = int(round((clock[0] - int(clock[0])) * 1e9))
        n.on_image(msg)
        odom(0.)  # fixed synthetic image, then resume measured approach below
        n.latest_speed = .8
        n.on_timer()
        assert not limits
    assert core.phase == Phase.IDLE and n.mission_entry.entry_at is not None

    # The line can leave the image. Use measured 0.8 m/s, independently of the 0.2 slow setting.
    for _ in range(30):
        clock[0] += .05
        odom(.8)
        n.on_timer()
        if limits:
            break
    assert core.phase == Phase.IDLE
    assert n.mission_entry.remaining(n.travel, .1) > .2
    assert limits == [pytest.approx(.2)]
    assert not n.logic.parking_flag and n.logic.state == DRIVE


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


def test_entry_observation_prepares_before_sign_confirmation_without_sending_speed(node):
    from test_line_stop import render

    n, clock = node
    cam = CameraModel()
    n.on_info(CameraInfo(width=640, height=480,
                         k=[cam.fx, 0., cam.cx, 0., cam.fy, cam.cy, 0., 0., 1.]))
    limits = []
    n.limit_pub = SimpleNamespace(publish=lambda m: limits.append(m.data))
    detections(n, clock, 1000., 'sign_pedestrian')  # only one candidate frame
    n.on_timer()
    assert n.mission_entry_active and not n.logic._armed() and not limits
    image = render(cam, 1.3, 0., .1, crosswalk=False, letters=False)
    for _ in range(3):
        clock[0] += .05
        msg = n.bridge.cv2_to_imgmsg(image, 'bgr8')
        msg.header.stamp.sec = int(clock[0])
        msg.header.stamp.nanosec = int(round((clock[0]-int(clock[0]))*1e9))
        n.on_image(msg)
        n.on_timer()
    assert n.mission_entry.entry_at is not None and not limits
    assert n.core.phase == Phase.IDLE and n.logic.state == DRIVE


@pytest.mark.parametrize('sign', ['sign_pedestrian', 'traffic_light_red'])
def test_entry_slowing_does_not_move_or_complete_the_existing_bumper_stop_target(node, sign):
    n, clock = node
    cam = CameraModel()
    n.on_info(CameraInfo(width=640, height=480,
                         k=[cam.fx, 0., cam.cx, 0., cam.fy, cam.cy, 0., 0., 1.]))
    limits, states = [], []
    n.limit_pub = SimpleNamespace(publish=lambda m: limits.append(m.data))
    n.mission_state_pub = SimpleNamespace(publish=lambda m: states.append(json.loads(m.data)))
    for now in (1000., 1000.1, 1000.2):
        detections(n, clock, now, sign)
        n.on_timer()
    assert n.finish_params.front == .161
    assert n.finish_params.stop_gap == .03
    assert n.get_parameter('front_axle_x').value == .1
    # Entry slows the car, but does not commit or complete the final crosswalk stop.
    n.mission_entry.entry_at = n.travel
    n.on_timer()
    assert limits[-1] == pytest.approx(.2) and n.core.phase == Phase.FOLLOW
    assert states[-1]['entry_phase'] == 'entered' and states[-1]['speed_limit'] == .2
    assert states[-1]['front'] == .161 and states[-1]['entry_front'] == .1
    for now in (1000.3, 1000.35, 1000.4):
        n.core._sighting(now, 1.)
    assert n.core.target == pytest.approx(1.-.161-.03)
    clock[0] = 1000.4
    n.on_timer()
    n.on_timer()
    assert limits[-1] == pytest.approx(.2)


@pytest.mark.parametrize('sign', ['sign_slow', 'sign_bump', 'sign_pedestrian', 'traffic_light_red'])
def test_start_line_slows_near_entry_instead_of_starting_a_long_crawl(node, sign):
    n, clock = node
    limits = []
    n.limit_pub = SimpleNamespace(publish=lambda m: limits.append(m.data))
    for now in (1000., 1000.1, 1000.2):
        detections(n, clock, now, sign)
        n.on_timer()
    n.latest_speed, n.latest_speed_time = .8, clock[0]
    n.mission_entry.entry_at = n.travel + .1 + .30
    n.on_timer()
    assert not limits
    assert n.mission_entry.remaining(n.travel, .1) == pytest.approx(.30)
    n.travel += .08
    clock[0] += .05
    n.on_timer()
    assert limits[-1] == pytest.approx(.2)
    assert not n.mission_entry.reached(n.travel, .1)


@pytest.mark.parametrize('speed', [.4, .8])
def test_shorter_entry_lead_reaches_slow_speed_before_crossing_with_actuator_delay(node, speed):
    n, _ = node
    value = lambda name: n.get_parameter(name).value
    n.mission_entry.entry_at = 1.
    distance, target, apply_at, slow_remaining = 0., speed, None, None
    for step in range(1000):
        now = step * .01
        if step % 5 == 0 and n.mission_entry.should_slow(
                distance, value('front_axle_x'), speed, .2,
                value('entry_reaction_sec'), value('entry_deceleration')):
            if apply_at is None:
                apply_at = now + .10  # delayed controller, independent of the gate's response lead
                slow_remaining = n.mission_entry.remaining(distance, value('front_axle_x'))
        if apply_at is not None and now + 1e-9 >= apply_at:
            target = .2
        next_speed = max(target, speed - 2.5 * .01)
        distance += (speed + next_speed) * .5 * .01
        speed = next_speed
        if n.mission_entry.reached(distance, value('front_axle_x')):
            assert slow_remaining is not None and slow_remaining <= .25
            assert speed <= .22
            break
    else:
        pytest.fail('vehicle did not reach the mission boundary')


def test_observation_extends_parking_arm_but_skip_does_not_complete_mission():
    from clothoid_mission.mission_logic import MissionLogic,MissionParams,Inputs
    m=MissionLogic(MissionParams(require_cone_for_parking=False))
    m.step(Inputs(now=0.,seen={'sign_parking'}))
    assert m.step(Inputs(now=9.,parking_observing=True)).parking_prepare
    out=m.step(Inputs(now=10.,parking_skipped=True,seen={'sign_parking'},parking_speed_limit=.2))
    assert not out.parking_flag and not out.parking_prepare and out.state==DRIVE
    assert out.speed_limit is None
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
