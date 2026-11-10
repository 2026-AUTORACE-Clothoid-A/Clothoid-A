import math

from clothoid_bringup.latency_stats import WindowStats, mux_winner, parse_pairs


def test_rate_gap_and_age():
    st = WindowStats()
    for i in range(10):
        st.add(i * 0.1, age=0.02 + 0.001 * i)
    r = st.report(window=1.0, now=0.95)
    assert r['count'] == 10
    assert math.isclose(r['hz'], 10.0)
    assert math.isclose(r['max_gap'], 0.1, abs_tol=1e-9)
    assert math.isclose(r['age_max'], 0.029)
    assert 0.02 <= r['age_p50'] <= 0.029


def test_silence_counts_as_gap_and_survives_reset():
    st = WindowStats()
    st.add(0.0)
    st.add(0.1)
    st.reset()
    r = st.report(window=1.0, now=1.1)       # nothing arrived in this window
    assert r['count'] == 0
    assert math.isclose(r['max_gap'], 1.0)   # 0.1 -> 1.1 of silence
    st.add(1.2)
    assert math.isclose(st.gaps[0], 1.1)     # first gap uses the arrival before the reset


def test_no_ages_gives_nan():
    st = WindowStats()
    st.add(0.0)
    assert math.isnan(st.report(1.0)['age_p50'])


def test_parse_pairs():
    assert parse_pairs(['/a=/img', '/b=', ' /c ', '']) == [('/a', '/img'), ('/b', ''), ('/c', '')]


def test_mux_winner_closest_match():
    history = {'/lane': [((0.3, 0.1), 1.0)], '/state': [((0.0, 0.0), 1.2)],
               '/avoid': [((0.3, 0.1), 1.25)]}
    assert mux_winner((0.3, 0.1), 1.3, history, max_age=0.5, grace=0.1) == '/avoid'
    assert mux_winner((0.0, 0.0), 1.3, history, max_age=0.5, grace=0.1) == '/state'
    assert mux_winner((0.3, 0.1), 2.0, history, max_age=0.5, grace=0.1) == '?'   # all too old
    assert mux_winner((0.9, 0.0), 1.3, history, max_age=0.5, grace=0.1) == '?'


def test_mux_winner_input_arrives_after_output():
    # the monitor saw /cmd_vel 3 ms before the /cmd_vel_lane message that caused it
    history = {'/lane': [((0.2, 0.0), 0.9), ((0.3, 0.1), 1.003)]}
    assert mux_winner((0.3, 0.1), 1.0, history, max_age=0.5, grace=0.1) == '/lane'
    assert mux_winner((0.3, 0.1), 1.0, history, max_age=0.5, grace=0.0) == '?'
