import numpy as np

from clothoid_lidar.lidar_debug import FEW, OK, WIDE, accepted_centres, classify_clusters, summary
from clothoid_lidar.obstacle_extractor import ExtractorParams, extract_obstacles

ANGLE_MIN = np.deg2rad(-110.0)
INC = np.deg2rad(0.9)
N = int(round(np.deg2rad(220.0) / INC)) + 1


def raycast(circles, walls=()):
    """circles: (x, y, r); walls: ((x0, y0), (x1, y1)) segments. Returns ranges."""
    ang = ANGLE_MIN + INC * np.arange(N)
    d = np.stack((np.cos(ang), np.sin(ang)), axis=1)
    r = np.full(N, np.inf)
    for cx, cy, rad in circles:
        b = d @ np.array([cx, cy])
        disc = b * b - (cx * cx + cy * cy - rad * rad)
        hit = disc >= 0
        t = b - np.sqrt(np.where(hit, disc, 0.0))
        r = np.where(hit & (t > 0), np.minimum(r, t), r)
    for (x0, y0), (x1, y1) in walls:
        e = np.array([x1 - x0, y1 - y0])
        for i in range(N):
            m = np.array([[d[i, 0], -e[0]], [d[i, 1], -e[1]]])
            if abs(np.linalg.det(m)) < 1e-9:
                continue
            t, u = np.linalg.solve(m, np.array([x0, y0]))
            if t > 0 and 0 <= u <= 1:
                r[i] = min(r[i], t)
    return r


def scene():
    rng = np.random.default_rng(3)
    cones = [(0.5, 0.3, 0.04), (0.5, -0.3, 0.04), (0.9, 0.35, 0.04), (0.9, -0.25, 0.04),
             (1.05, 0.35, 0.04), (1.6, 0.1, 0.04)]          # 0.9/1.05 pair: 15 cm apart
    walls = [((0.3, 1.2), (1.8, 1.2)), ((1.2, -0.9), (1.4, -0.5))]
    r = raycast(cones, walls)
    return r + rng.normal(0, 0.003, r.size) * np.isfinite(r)


def test_accepted_matches_extractor_for_several_param_sets():
    r = scene()
    for gap, minp, width in ((0.08, 2, 0.15), (0.06, 2, 0.15), (0.06, 3, 0.12), (0.05, 3, 0.12)):
        p = ExtractorParams(cluster_gap=gap, min_points=minp, max_width=width, range_max=2.0)
        a = accepted_centres(classify_clusters(r, ANGLE_MIN, INC, p))
        b = extract_obstacles(r, ANGLE_MIN, INC, p)
        assert a.shape == b.shape
        np.testing.assert_allclose(a, b)


def test_every_beam_lands_in_exactly_one_cluster():
    r = scene()
    p = ExtractorParams()
    cl = classify_clusters(r, ANGLE_MIN, INC, p)
    valid = np.isfinite(r) & (r >= p.range_min) & (r <= p.range_max)
    assert sum(len(k.points) for k in cl) == int(valid.sum())


def test_reasons():
    r = scene()
    cl = classify_clusters(r, ANGLE_MIN, INC, ExtractorParams())
    s = summary(cl)
    assert s[WIDE] >= 1                       # wall pieces
    assert s[OK] >= 4
    for k in cl:
        if k.kind == WIDE:
            assert k.width > 0.15
        if k.kind == FEW:
            assert len(k.points) < 2
        if k.kind == OK:
            assert k.width <= 0.15


def test_empty_scan():
    assert classify_clusters(np.full(N, np.inf), ANGLE_MIN, INC, ExtractorParams()) == []
    assert accepted_centres([]).shape == (0, 2)
