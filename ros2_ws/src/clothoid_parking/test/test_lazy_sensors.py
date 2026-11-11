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
    return {s.topic_name for s in n.subscriptions} & {'/imu', '/odom', '/scan'}


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
    assert sensor_topics(node) == {'/imu', '/odom', '/scan'}
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
    assert sensor_topics(node) == {'/imu', '/odom', '/scan'}


def test_flag_subscribes_then_starts_when_data_arrives(node):
    node.on_flag(Bool(data=True))
    assert sensor_topics(node) == {'/imu', '/odom', '/scan'}
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
        assert sensor_topics(n) == {'/imu', '/odom', '/scan'}
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


def test_inspection_auto_observes_and_ignores_driving_flags():
    rclpy.init(args=['--ros-args', '-p', 'inspection_only:=true'])
    n = ParkingNode()
    try:
        assert n.core.preparing and sensor_topics(n) == {'/imu', '/odom', '/scan'}
        assert not n.core.layout.p.control_enabled
        n.on_prepare(Bool(data=False))
        n.on_flag(Bool(data=True))
        n.on_timer()
        assert n.core.preparing and n.core.phase == Phase.IDLE
        assert n.core.command(n.now()) is None
    finally:
        n.destroy_node()
        rclpy.shutdown()
