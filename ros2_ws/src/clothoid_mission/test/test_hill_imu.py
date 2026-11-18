"""IMU-only hill slow (hill_slow_imu): 1118 PM, the start's bump sign became a slow sign whose 4 s hold
ended before the hill (slow off at 2.6 m, hill 2.8~3.5 m)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from clothoid_mission.mission_logic import BUMP_IDLE, Inputs, MissionLogic, MissionParams


def params(**kw):
    return MissionParams(hill_slow_imu=True, bump_pitch_thresh=3.5, bump_level_thresh=1.2,
                         bump_level_duration_sec=0.3, hill_max_sec=6.0, **kw)


def run(logic, profile, t0=0., dt=0.04):
    """profile: list of pitches at 25 Hz; returns the speed limits."""
    return [logic.step(Inputs(now=t0+i*dt, imu_pitch=p)).speed_limit for i, p in enumerate(profile)]


def test_off_by_default_keeps_the_sign_only_bump():
    out = MissionLogic(MissionParams()).step(Inputs(now=1., imu_pitch=6.))
    assert out.speed_limit is None


def test_hill_slows_without_a_sign_through_the_crest_and_releases_when_level():
    logic = MissionLogic(params())
    up, crest, down, flat = [-6.]*25, [-0.5]*10, [5.]*20, [0.2]*20
    limits = run(logic, [0.]*5 + up + crest + down + flat)
    assert limits[:5] == [None]*5
    on = limits[5:5+25+10+20]
    assert all(v == 0.2 for v in on)                 # the crest (level, no opposite tilt yet) holds
    assert limits[-1] is None                        # level 0.3 s after the opposite tilt
    assert logic.bump_state == BUMP_IDLE             # the sign bump state machine is untouched


def test_no_steering_lock():
    logic = MissionLogic(params())
    outs = [logic.step(Inputs(now=i*0.04, imu_pitch=-6.)) for i in range(10)]
    assert all(o.speed_limit == 0.2 and not o.steering_lock for o in outs)


def test_single_ramp_or_silent_imu_releases_after_hill_max_sec():
    logic = MissionLogic(params())
    logic.step(Inputs(now=0., imu_pitch=-6.))
    assert logic.step(Inputs(now=5.9, imu_pitch=None)).speed_limit == 0.2
    assert logic.step(Inputs(now=6.1, imu_pitch=None)).speed_limit is None


def test_below_threshold_disturbance_does_not_slow():
    logic = MissionLogic(params())
    assert all(v is None for v in run(logic, [3.0, -3.2, 2.5, -1.0]*10))


# slow zones (1118 PM): slow at once on the slow sign, 5 s after its last sighting; the zigzag zone keeps
# it until the crosswalk stop; a stop release (crosswalk wait over, green) ends the zone

def zone_params(**kw):
    return MissionParams(slow_hold_sec=5.0, slow_at_stopline=False, slow_until_pedestrian_stop=True,
                         use_stop_line=True, **kw)


def test_slow_sign_slows_at_once_for_five_seconds():
    logic = MissionLogic(zone_params())
    assert logic.step(Inputs(now=0., seen={'sign_slow'})).speed_limit == 0.2
    assert logic.step(Inputs(now=4.9)).speed_limit == 0.2
    assert logic.step(Inputs(now=5.1)).speed_limit is None


def test_zigzag_zone_stays_slow_until_the_crosswalk_stop_then_lane_speed():
    logic = MissionLogic(zone_params())
    logic.step(Inputs(now=0., seen={'sign_slow'}))
    logic.step(Inputs(now=1., seen={'sign_pedestrian'}))
    assert logic.step(Inputs(now=7., seen={'sign_pedestrian'})).speed_limit == 0.2   # past the 5 s
    out = logic.step(Inputs(now=9., line_phase='arrived'))
    assert out.stop
    assert not logic.slow_for_pedestrian
    out = logic.step(Inputs(now=9.+3.6))                     # wait over
    assert not out.stop and out.speed_limit is None


def test_pedestrian_sign_without_a_slow_zone_does_not_slow():
    logic = MissionLogic(zone_params())
    assert logic.step(Inputs(now=0., seen={'sign_pedestrian'})).speed_limit is None


def test_green_after_a_light_stop_ends_the_slow_zone():
    logic = MissionLogic(zone_params())
    logic.step(Inputs(now=0., seen={'sign_slow', 'traffic_light_red'}))
    assert logic.step(Inputs(now=1., line_phase='arrived')).stop
    out = logic.step(Inputs(now=2., seen={'traffic_light_green'}))
    assert not out.stop and out.speed_limit is None          # 5 s hold would still run without the release


# first slow zone yaw hold and the traffic light latch (1118 PM)

def test_yaw_hold_from_the_first_slow_sign_until_the_hill_is_over():
    logic = MissionLogic(MissionParams(hill_slow_imu=True, first_slow_yaw_hold=True, yaw_hold_max_sec=10.))
    assert not logic.step(Inputs(now=0., imu_pitch=0.)).yaw_hold
    assert logic.step(Inputs(now=1., seen={'sign_slow'}, imu_pitch=0.)).yaw_hold
    t = 1.
    for pitch in [-6.]*10 + [5.]*10 + [0.]*5:                # up, down, level (hill over)
        t += .1
        out = logic.step(Inputs(now=t, imu_pitch=pitch))
    assert not out.yaw_hold
    # never again: the second slow sign (crosswalk zone) does not hold the yaw
    assert not logic.step(Inputs(now=20., seen={'sign_slow'}, imu_pitch=0.)).yaw_hold


def test_yaw_hold_ends_by_the_cap_without_a_hill():
    logic = MissionLogic(MissionParams(hill_slow_imu=True, first_slow_yaw_hold=True, yaw_hold_max_sec=10.))
    logic.step(Inputs(now=0., seen={'sign_slow'}, imu_pitch=0.))
    assert logic.step(Inputs(now=9.9, imu_pitch=0.)).yaw_hold
    assert not logic.step(Inputs(now=10.1, imu_pitch=0.)).yaw_hold


def test_light_stop_latches():
    logic = MissionLogic(MissionParams(use_stop_line=True))
    logic.step(Inputs(now=0., seen={'traffic_light_red'}))
    assert not logic.step(Inputs(now=.5)).light_stopped
    assert logic.step(Inputs(now=1., line_phase='arrived')).light_stopped
    out = logic.step(Inputs(now=2., seen={'traffic_light_green'}))
    assert not out.stop and out.light_stopped
    assert not MissionLogic(MissionParams()).step(Inputs(now=0.)).light_stopped


def test_yaw_hold_starts_after_the_configured_distance_past_the_sign():
    logic = MissionLogic(MissionParams(hill_slow_imu=True, first_slow_yaw_hold=True, yaw_hold_after_m=1.0))
    assert not logic.step(Inputs(now=0., seen={'sign_slow'}, imu_pitch=0., travel=1.0)).yaw_hold
    assert not logic.step(Inputs(now=3., imu_pitch=0., travel=1.9)).yaw_hold
    assert logic.step(Inputs(now=4., imu_pitch=0., travel=2.05)).yaw_hold
    assert logic.step(Inputs(now=4.1, imu_pitch=0., travel=None)).yaw_hold      # odometry lost: keep it
