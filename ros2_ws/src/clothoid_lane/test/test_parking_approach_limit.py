from types import SimpleNamespace
import rclpy
from rclpy.time import Time
from std_msgs.msg import Float32
from clothoid_lane.lane_main import LaneMainNode


def test_independent_speed_caps_take_min_and_zero_means_stop():
    clock=[10.]
    p={'base_speed':.8,'speed_limit_timeout':.5}
    n=SimpleNamespace(speed_limit=.1,speed_limit_stamp=Time(seconds=10),
                      parking_approach_limit=None,
                      get_parameter=lambda k:SimpleNamespace(value=p[k]),
                      get_clock=lambda:SimpleNamespace(now=lambda:Time(nanoseconds=int(clock[0]*1e9))))
    LaneMainNode.on_parking_approach(n,Float32(data=.2))
    assert LaneMainNode.current_speed(n)==.1
    LaneMainNode.on_parking_approach(n,Float32(data=0.))
    assert LaneMainNode.current_speed(n)==0.
    LaneMainNode.on_parking_approach(n,Float32(data=-1.))
    assert LaneMainNode.current_speed(n)==.1
    LaneMainNode.on_parking_approach(n,Float32(data=.2))
    clock[0]=10.6
    assert LaneMainNode.current_speed(n)==.8
