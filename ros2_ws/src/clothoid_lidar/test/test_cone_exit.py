"""Runtime configuration keeps the original forward-only course exit box."""
from pathlib import Path
import numpy as np
import yaml
from clothoid_lidar.cone_sorter import ConeMission, ConeSorter, SorterParams


def entry():
    return np.array([[.2, .4], [.5, .4], [.8, .4], [.2, -.4], [.5, -.4], [.8, -.4]])


def test_runtime_angles_and_exit_box_are_restored():
    config = yaml.safe_load((Path(__file__).parents[1]/'config/lidar.yaml').read_text())
    p = config['cone_sorter']['ros__parameters']
    assert p['angle_from_left_first_cone'] == 45.
    assert p['first_step_angle_deg'] == 90.  # main's normal tracer configuration
    assert p['ing_x_min'] == 0. and p['ing_x_max'] == 1.
    m = ConeMission(ConeSorter(SorterParams(**{k:v for k,v in p.items() if k in SorterParams.__dataclass_fields__})))
    m.update(entry(), 0.)
    assert m.running
    corner = np.array([[.1, .35], [.3, .25], [.5, .1], [-.05, -.4]])
    assert m.update(corner, .1) is None and not m.running


def test_empty_scan_releases_immediately_without_a_time_latch():
    m = ConeMission(ConeSorter(SorterParams()))
    m.update(entry(), 0.)
    assert m.running
    assert m.update(np.zeros((0, 2)), .1) is None and not m.running


def test_rear_cones_do_not_start_or_continue_a_course():
    m = ConeMission(ConeSorter(SorterParams()))
    behind = np.array([[x, y] for x in (-.15, -.1, -.05) for y in (-.4, .4)])
    assert not m.sorter.continue_zone_ok(behind)
    assert m.update(behind, 0.) is None and not m.running
