"""/lane/yaw_hold (mission, first slow zone = the start's hill): keep the IMU yaw, ignore the camera
target (1118 PM: the hill's yellow paint pulled the target sideways)."""
import math

from std_msgs.msg import Bool

from test_bump_gap import frame, lane, sensors  # noqa: F401  (fixture)


def test_holds_the_yaw_of_the_moment_it_turned_on_and_ignores_the_camera(lane):  # noqa: F811
    node, clock, result, commands = lane
    sensors(node, yaw=0.)
    node.on_yaw_hold(Bool(data=True))
    result.target_x = 300.                       # the camera pulls hard to one side
    frame(node)
    assert commands[-1].angular.z == 0.          # yaw unchanged: straight
    clock.t += .1
    sensors(node, yaw=math.radians(3.))          # drifted left
    frame(node)
    assert commands[-1].angular.z < 0.           # steers back right


def test_off_or_stale_returns_to_the_camera(lane):  # noqa: F811
    node, clock, result, commands = lane
    sensors(node)
    node.on_yaw_hold(Bool(data=True))
    node.on_yaw_hold(Bool(data=False))
    result.target_x = 300.
    frame(node)
    assert commands[-1].angular.z != 0.
    node.on_yaw_hold(Bool(data=True))
    clock.t += 2.                                # no heartbeat for longer than steering_lock_timeout
    sensors(node)
    frame(node)
    assert commands[-1].angular.z != 0.
