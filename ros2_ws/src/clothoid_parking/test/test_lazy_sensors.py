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
