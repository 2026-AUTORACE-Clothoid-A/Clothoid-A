"""lazy_sensors: no imu/odom/scan subscriptions before /parking_flag, run starts once they arrive."""
import pytest

rclpy = pytest.importorskip('rclpy')
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import Imu, LaserScan  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402

from clothoid_parking.parking_node import ParkingNode  # noqa: E402
from clothoid_parking.planner import Phase  # noqa: E402


@pytest.fixture
def node():
    rclpy.init()
    n = ParkingNode()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def sensor_topics(n):
    return {s.topic_name for s in n.subscriptions} & {'/imu_25hz', '/odom_25hz', '/scan'}


def feed(n):
    n.on_imu(Imu())
    n.on_odom(Odometry())
    scan = LaserScan()
    scan.angle_increment, scan.range_max, scan.ranges = 0.01, 12.0, [1.0] * 10
    n.on_scan(scan)


def test_no_sensor_subscriptions_before_flag(node):
    assert sensor_topics(node) == set()


def test_registered_layout_prepare_tracks_motion_without_commanding_parking(node):
    node.on_prepare(Bool(data=True))
    assert sensor_topics(node) == {'/imu_25hz', '/odom_25hz', '/scan'}
    assert node.core.phase == Phase.IDLE and not node.core.flag
    node.on_timer()
    assert node.core.phase == Phase.IDLE
    node.on_prepare(Bool(data=False))
    assert sensor_topics(node) == set()


def test_handover_removes_passive_scan_before_high_rate_sensors_start(node):
    node.on_prepare(Bool(data=True))
    node.on_flag(Bool(data=True))
    assert node.probe_sub is None
    assert sum(s.topic_name == '/scan' for s in node.subscriptions) == 1
    assert sensor_topics(node) == {'/imu_25hz', '/odom_25hz', '/scan'}


def test_flag_subscribes_then_starts_when_data_arrives(node):
    node.on_flag(Bool(data=True))
    assert sensor_topics(node) == {'/imu_25hz', '/odom_25hz', '/scan'}
    assert not node.core.flag                 # waiting for the sensors
    feed(node)
    node.on_timer()
    assert node.core.flag and node.core.phase == Phase.SEARCH


def test_flag_without_data_gives_up_after_wait(node):
    node.on_flag(Bool(data=True))
    node.t_armed -= node.arm_wait + 0.1
    node.on_timer()
    assert not node.core.flag and node.t_armed is None
    node.on_flag(Bool(data=True))             # mission repeats the flag: wait again
    assert node.t_armed is not None


def test_always_on_when_lazy_off():
    rclpy.init(args=['--ros-args', '-p', 'lazy_sensors:=false'])
    n = ParkingNode()
    try:
        assert sensor_topics(n) == {'/imu_25hz', '/odom_25hz', '/scan'}
    finally:
        n.destroy_node()
        rclpy.shutdown()


def test_legacy_prepare_remains_scan_only():
    rclpy.init(args=['--ros-args', '-p', 'layout.enabled:=false'])
    n = ParkingNode()
    try:
        n.on_prepare(Bool(data=True))
        assert sensor_topics(n) == {'/scan'}
        assert n.core.command(n.now()) is None
    finally:
        n.destroy_node()
        rclpy.shutdown()


def test_inspection_previews_flag_without_sending_driving_commands(monkeypatch):
    rclpy.init(args=['--ros-args', '-p', 'inspection_only:=true'])
    n = ParkingNode()
    try:
        assert n.core.preparing and sensor_topics(n) == {'/imu_25hz', '/odom_25hz', '/scan'}
        assert not n.core.layout.p.control_enabled
        sent = []
        monkeypatch.setattr(n, 'send', lambda *args: sent.append(args))
        n.on_prepare(Bool(data=False))
        feed(n)
        n.on_flag(Bool(data=True))
        n.on_timer()
        assert n.preview.core.phase == Phase.SEARCH
        assert not sent
        assert n.core.preparing and n.core.phase == Phase.IDLE
        assert n.core.command(n.now()) is None
    finally:
        n.destroy_node()
        rclpy.shutdown()


def test_registered_handover_uses_grid_not_parked_vehicle_gap(node, monkeypatch):
    from types import SimpleNamespace
    from clothoid_parking.layout import LayoutParams, ParkingLayout
    from clothoid_parking.tracker import Pose2D
    node.core.layout = ParkingLayout(LayoutParams(control_enabled=True, target_bay=2))
    for t in (.1,.2,.3):
        node.core.layout.observe(t,(1.,-.25),0.,Pose2D())
    node.core.layout.selected = 1
    sent=[]
    node.candidate_pub = SimpleNamespace(publish=sent.append)
    def forbidden(*args):
        raise AssertionError('legacy gap finder must not gate registered handover')
    monkeypatch.setattr(node.probe,'scan',forbidden)
    scan=LaserScan()
    scan.header.stamp=node.get_clock().now().to_msg()
    node.on_probe_scan(scan)
    assert len(sent)==1
    bay=node.core.layout.bays()[1]
    assert sent[0].pose.position.x == pytest.approx(bay.face_x)
    assert sent[0].pose.position.y == pytest.approx(bay.face_y)
    node.core.layout.selected=None
    node.on_probe_scan(scan)
    assert len(sent)==1


def test_provisional_handover_requires_fresh_observed_near_term_space(node):
    import numpy as np
    from types import SimpleNamespace
    from clothoid_parking.layout import LayoutParams, ParkingLayout
    from clothoid_parking.tracker import Pose2D
    layout=ParkingLayout(LayoutParams(control_enabled=True,target_bay=2))
    for t in (.1,.2,.3):layout.observe(t,(1.,-.25),0.,Pose2D())
    node.core.layout=layout
    node.core.approach_bay=layout.bays()[1]
    layout.support_count=layout.p.confirm_scans
    now=node.now()
    layout.last_scan=now
    node.last={name:now for name in ('imu','odom','scan')}
    sent=[]
    node.candidate_pub=SimpleNamespace(publish=sent.append)
    msg=LaserScan();msg.header.stamp=node.get_clock().now().to_msg()
    node.on_probe_scan(msg)
    assert not sent  # final target geometry alone is insufficient
    for x in np.linspace(0.,.2,30):
        for k in np.floor(layout.footprint(x,0.,0.)/layout.p.grid).astype(int):
            layout.cells[tuple(k)]=(now,False)
    node.on_probe_scan(msg)
    assert len(sent)==1 and layout.selected is None
    layout.cells[(int(.3/layout.p.grid),0)]=(now,True)
    node.on_probe_scan(msg)
    assert len(sent)==1  # known near-term obstacle still vetoes handover
    node.last['imu']=now-1.
    node.on_probe_scan(msg)
    assert len(sent)==1


@pytest.mark.parametrize('missing', ['imu', 'odom', 'scan'])
def test_exit_stops_for_missing_sensors_without_reporting_done(node, monkeypatch, missing):
    from clothoid_parking.layout import LayoutParams
    from clothoid_parking.core import ParkingCore
    from types import SimpleNamespace
    node.core = ParkingCore(layout=LayoutParams(control_enabled=True, target_bay=2))
    node.core.planner._go(Phase.EXIT_TURN, node.now(), 'exiting')
    node.core.planner.bay = SimpleNamespace(row_yaw=2.)
    now = node.now()
    node.core.layout.last_scan = now
    node.last = {name: now for name in node.last}
    node.last[missing] = now-1.
    sent, done = [], []
    monkeypatch.setattr(node, 'send', lambda *args: sent.append(args))
    node.done_pub = SimpleNamespace(publish=done.append)
    node.on_timer()
    assert sent == [(0., 0.)]
    assert not done and node.core.phase == Phase.EXIT_TURN


def test_completed_exit_publishes_done_only_after_lane_heading(node, monkeypatch):
    from types import SimpleNamespace
    from clothoid_parking.core import ParkingCore
    from clothoid_parking.layout import LayoutParams
    node.core = ParkingCore(layout=LayoutParams(control_enabled=True, target_bay=2))
    now = node.now()
    node.core.planner._go(Phase.EXIT_TURN, now, 'exiting')
    node.core.planner.bay = SimpleNamespace(row_yaw=2.)
    node.core.layout.last_scan = now
    node.last = {name: now for name in node.last}
    sent, done = [], []
    monkeypatch.setattr(node, 'send', lambda *args: sent.append(args))
    node.done_pub = SimpleNamespace(publish=done.append)
    node.on_timer()
    assert sent[-1][0] < 0 and not done
    node.core.tracker.rear.yaw = 2.
    node.on_timer()
    assert node.core.phase == Phase.DONE
    assert sent[-1] == (0., 0.)
    assert len(done) == 1 and done[0].data
