import math

from tools.bump_viewer import triggers


def test_segments_split_on_level_change_and_gaps():
    samples = [(0., 1), (.05, 1), (.1, 2), (.15, 2), (.2, 0), (1., 2), (1.05, 2), (3., 2)]
    assert triggers.segments(samples) == [(0., .05, 1), (.1, .15, 2), (1., 1.05, 2), (3., 3., 2)]


def test_state_lanes_separate_armed_from_acting():
    states = [(0., {'armed': ['slow'], 'entry_phase': 'search', 'state': 'drive'}),
              (.05, {'armed': ['slow'], 'entry_phase': 'braking', 'speed_limit': .2, 'state': 'drive'}),
              (.1, {'armed': [], 'line_phase': 'commit', 'state': 'stopped_light'})]
    lanes = triggers.from_states(states)
    assert lanes['slow'] == [(0., .05, 1)]
    assert lanes['entry'] == [(0., 0., 1), (.05, .05, 2)]
    assert lanes['limit'] == [(.05, .05, 2)]
    assert lanes['line'] == [(.1, .1, 2)] and lanes['stop'] == [(.1, .1, 2)]


def test_signals_and_rc_takeover():
    signals = [(10., '/parking_flag', True), (11., '/parking_flag', True), (12., '/parking_flag', False),
               (5., '/direction_flag', 1)]
    lanes = triggers.from_signals(sorted(signals))
    assert lanes['parking'] == [(10., 11., 2)]
    assert lanes['fork'] == [(5., 5., 2)]
    rows = [{'t': 1., 'message': 'control mode 3 (RC transmitter)'},
            {'t': 4., 'message': 'control mode 1 (ROS commands)'},
            {'t': 9., 'message': 'control mode 3 (RC transmitter)'}, {'t': None, 'message': 'x'}]
    assert triggers.rc_segments(rows, 20.) == [(1., 4., 2), (9., 20., 2)]


def test_path_uses_imu_yaw_and_wheel_speed():
    path = triggers.Path2D()
    t = 0.
    for _ in range(101):                     # quarter circle: 0.5 rad/s for pi s at 1 m/s
        path.imu(t, math.pi / 2 / math.pi)
        path.odom(t, 1.)
        t += math.pi / 100
    x, y, yaw = path.pose()
    assert abs(yaw - math.pi / 2) < .03
    assert abs(x - 2.) < .1 and abs(y - 2.) < .1   # radius 2 m


def test_active_at_reports_highest_level():
    lanes = {'line': [(0., 1., 1), (1., 2., 2)], 'stop': [(5., 6., 2)]}
    assert triggers.active_at(lanes, 1.) == {'line': 2}
    assert triggers.active_at(lanes, 3.) == {}
