from types import SimpleNamespace

from clothoid_mission.sign_tracker import SignTracker


def box(name='sign_pedestrian', x=100, y=220, size=16, confidence=.9):
    return SimpleNamespace(class_name=name, confidence=confidence,
                           x_min=x, x_max=x + size, y_min=y, y_max=y + size)


def test_small_roadside_and_large_central_signs_both_confirm_independently():
    tracker = SignTracker()
    ds = [box(x=20), box(x=300, y=180, size=100)]
    for t in (0., .1):
        assert tracker.update(t, ds) == []
    tracks = tracker.update(.2, ds)
    assert len(tracks) == 2 and len({t.id for t in tracks}) == 2


def test_distinct_objects_cannot_pool_confirmation():
    tracker = SignTracker()
    for t, x in ((0., 100), (.1, 300), (.2, 100), (.3, 300)):
        assert not tracker.update(t, [box(x=x)])


def test_duplicates_and_reordered_source_images_add_no_hits():
    tracker = SignTracker()
    for stamp in (1., 1., .9, 1.):
        assert not tracker.update(stamp, [box()])
    assert not tracker.update(1.1, [box()])
    assert tracker.update(1.2, [box()])[0].hits == 3


def test_duplicate_boxes_in_one_image_count_once():
    tracker = SignTracker()
    assert not tracker.update(0., [box()] * 10)
    assert len(tracker.tracks) == 1
    assert next(iter(tracker.tracks.values())).hits == 1


def test_dropout_retains_identity_but_is_not_a_current_confirmation():
    tracker = SignTracker()
    for t in (0., .1, .2):
        tracks = tracker.update(t, [box()])
    identity = tracks[0].id
    assert not tracker.update(.3, [])
    assert tracker.update(.4, [box()])[0].id == identity
    assert not tracker.update(3., [box()])
    assert identity not in tracker.tracks


def test_green_needs_its_own_hits_on_the_same_signal():
    tracker = SignTracker()
    for t in (0., .1, .2):
        red = tracker.update(t, [box('traffic_light_red')])
    signal = red[0].id
    tracker.pin(signal)
    assert not tracker.update(.3, [box('traffic_light_green')])
    assert not tracker.update(.4, [box('traffic_light_green')])
    assert tracker.update(.5, [box('traffic_light_green')])[0].id == signal


def test_pinned_signal_survives_long_occlusion_without_reusing_stale_colour():
    tracker = SignTracker()
    for t in (0., .1, .2):
        red = tracker.update(t, [box('traffic_light_red')])
    signal = red[0].id
    tracker.pin(signal)
    assert not tracker.update(10., [])
    assert signal in tracker.tracks
    for t in (10.1, 10.2):
        assert not tracker.update(t, [box('traffic_light_green')])
    assert tracker.update(10.3, [box('traffic_light_green')])[0].id == signal


def test_other_green_does_not_share_the_red_track():
    tracker = SignTracker()
    for t in (0., .1, .2):
        tracks = tracker.update(t, [box('traffic_light_red'), box('traffic_light_green', x=400)])
    assert len({t.id for t in tracks}) == 2


def test_long_occlusion_with_two_possible_signals_does_not_reacquire_arbitrarily():
    tracker = SignTracker()
    for t in (0., .1, .2):
        red = tracker.update(t, [box('traffic_light_red', size=60)])
    identity = red[0].id
    tracker.pin(identity)
    for t in (10., 10.1, 10.2):
        green = tracker.update(t, [box('traffic_light_green', x=60, size=60),
                                   box('traffic_light_green', x=140, size=60)])
        assert all(track.id != identity for track in green)


def test_conflicting_colours_are_unknown_even_when_repeated():
    tracker = SignTracker()
    for t in (0., .1, .2, .3):
        assert not tracker.update(t, [box('traffic_light_red'), box('traffic_light_green')])
    assert len(tracker.tracks) == 1


def test_quality_applies_per_box_instead_of_per_class():
    tracker = SignTracker()
    for t in (0., .1, .2):
        tracks = tracker.update(t, [box(), box(x=300, confidence=.5), box(x=0)])
    assert len(tracks) == 1 and tracks[0].box.x_min == 100


def test_old_hits_expire_and_memory_is_bounded():
    tracker = SignTracker(max_tracks=4)
    for t in (0., 1., 2.):
        assert not tracker.update(t, [box()])
    ds = [box(name='sign_'+str(i), x=30 + i * 30) for i in range(10)]
    tracker.update(2.1, ds)
    assert len(tracker.tracks) <= 4


def fork_tracker(**overrides):
    options = dict(class_hits={'sign_left': 2, 'sign_right': 2},
                   class_max_age={'sign_left': 1., 'sign_right': 1.},
                   class_confidence={'sign_left': .55, 'sign_right': .55},
                   edge_exempt=('sign_left', 'sign_right'), any_x=('sign_left', 'sign_right'))
    return SignTracker(**(options | overrides))


def test_fork_sweep_survives_missed_frames_and_preserves_identity():
    tracker = fork_tracker()
    assert not tracker.update(0., [box('sign_left', x=459, size=20, confidence=.65)])
    identity = next(iter(tracker.tracks))
    for t in (.1, .2, .3, .4, .5, .6, .7):
        assert not tracker.update(t, [])
    tracks = tracker.update(.8, [box('sign_left', x=273, size=20, confidence=.55)])
    assert len(tracks) == 1 and tracks[0].id == identity and tracks[0].hits == 2
    assert not tracker.update(1.9, [box('sign_left', x=100, size=20)])


def test_fork_relaxation_does_not_admit_low_quality_other_signs():
    tracker = fork_tracker()
    for name in ('sign_left', 'sign_right'):
        assert tracker.box_reason(box(name, x=0, confidence=.55)) is None
        assert tracker.box_reason(box(name, confidence=.54)).startswith('conf')
    for name in ('sign_parking', 'sign_pedestrian', 'traffic_light_red'):
        assert tracker.box_reason(box(name, x=0)) == 'edge'
        assert tracker.box_reason(box(name, confidence=.55)).startswith('conf')


def test_fork_any_x_requires_matching_side_height_and_row():
    tracker = fork_tracker()
    assert not tracker.update(0., [box('sign_left', x=100, size=20)])
    assert not tracker.update(.1, [box('sign_right', x=500, size=20)])
    assert not tracker.update(.2, [box('sign_left', x=500, y=330, size=20)])
    assert not tracker.update(.3, [box('sign_left', x=500, size=80)])


def test_fork_relaxation_can_be_disabled():
    tracker = fork_tracker(any_x=())
    assert not tracker.update(0., [box('sign_left', x=100, size=20)])
    assert not tracker.update(.5, [box('sign_left', x=500, size=20)])
