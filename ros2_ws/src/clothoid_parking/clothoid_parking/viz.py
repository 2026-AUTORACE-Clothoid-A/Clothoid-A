"""RViz markers for the parking manoeuvre, all in base_link so they work on the car (no odom TF)
and in the simulator alike. Open rviz/parking.rviz (`ros2 launch clothoid_parking rviz.launch.py`).

    grey points   lidar points gathered while searching (dead-reckoned into one map)
    green box     the empty bay found (inside of the lines, 500 x 600 mm)
    blue box      where the car body should end (LIMO 322 x 220 mm at the bay centre)
    orange line   planned path: straight to the turn point, then the full-lock arc
    text          phase and reason
"""
import math

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from clothoid_parking.planner import Phase

BAY_W, BAY_D = 0.5, 0.6
CAR_L, CAR_W = 0.322, 0.220


def _to_base(pose, x, y):
    dx, dy = x - pose.x, y - pose.y
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return Point(x=dx * c + dy * s, y=-dx * s + dy * c, z=0.0)


def _rect(pose, cx, cy, yaw, length, width):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    for dl, dw in ((1, 1), (1, -1), (-1, -1), (-1, 1), (1, 1)):
        pts.append(_to_base(pose, cx + dl * length / 2 * c - dw * width / 2 * s,
                            cy + dl * length / 2 * s + dw * width / 2 * c))
    return pts


def _marker(stamp, ns, mid, mtype, rgba, scale):
    m = Marker()
    m.header.frame_id = 'base_link'
    m.header.stamp = stamp
    m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = scale
    m.color.r, m.color.g, m.color.b, m.color.a = rgba
    m.lifetime = Duration(sec=1)
    return m


def markers(core, stamp) -> MarkerArray:
    pose = core.tracker.pose
    planner, p = core.planner, core.planner.p
    out = MarkerArray()

    pts = _marker(stamp, 'points', 0, Marker.POINTS, (0.6, 0.6, 0.6, 0.8), 0.02)
    pts.points = [_to_base(pose, x, y) for x, y in core.points()]
    out.markers.append(pts)

    bay = planner.bay
    if bay is not None:
        box = _marker(stamp, 'bay', 0, Marker.LINE_STRIP, (0.1, 0.9, 0.2, 1.0), 0.015)
        # bay centre is the parked car's centre; the bay itself is centred on it
        box.points = _rect(pose, bay.x, bay.y, bay.entry_yaw, BAY_D, BAY_W)
        out.markers.append(box)
        car = _marker(stamp, 'target', 0, Marker.LINE_STRIP, (0.2, 0.5, 1.0, 1.0), 0.01)
        car.points = _rect(pose, bay.x, bay.y, bay.entry_yaw, CAR_L, CAR_W)
        out.markers.append(car)

        # planned rear-axle path: straight along the row, then the arc
        path = _marker(stamp, 'path', 0, Marker.LINE_STRIP, (1.0, 0.55, 0.0, 1.0), 0.01)
        c, s = math.cos(bay.row_yaw), math.sin(bay.row_yaw)
        along = p.turn_radius + p.turn_lead
        tx, ty = bay.face_x - along * c, bay.face_y - along * s     # on the face line
        # shift onto the line the rear axle is actually driving on
        rx = pose.x - p.rear_offset * math.cos(pose.yaw)
        ry = pose.y - p.rear_offset * math.sin(pose.yaw)
        off = -(rx - bay.face_x) * s + (ry - bay.face_y) * c      # lateral of the rear axle
        tx, ty = tx - off * s, ty + off * c
        side = p.side
        cx, cy = tx - side * p.turn_radius * s, ty + side * p.turn_radius * c
        if planner.phase == Phase.SEARCH:
            path.points.append(_to_base(pose, rx, ry))
        start = math.atan2(ty - cy, tx - cx)
        for k in range(19):
            a = start + side * k * (math.pi / 2) / 18
            path.points.append(_to_base(pose, cx + p.turn_radius * math.cos(a),
                                        cy + p.turn_radius * math.sin(a)))
        out.markers.append(path)

    text = _marker(stamp, 'phase', 0, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0), 0.08)
    text.pose.position.z = 0.35
    text.text = f'{planner.phase.value}: {planner.reason}'
    out.markers.append(text)
    return out


def layout_markers(core, stamp) -> MarkerArray:
    """Six candidate rectangles; grey remains uncorroborated, never imply it is a goal."""
    out = MarkerArray()
    layout = core.layout
    if layout is None:
        return out
    for i, bay in enumerate(layout.bays()):
        state = layout.states[i]
        supported = layout.support_count >= layout.p.confirm_scans
        if not supported:
            rgba = (.5, .5, .5, .7)
        elif state == 'occupied':
            rgba = (1., .2, .2, 1.)
        elif state == 'free':
            rgba = (.1, .9, .2, 1.)
        else:
            rgba = (1., .65, .1, 1.)
        box = _marker(stamp, 'layout', i, Marker.LINE_STRIP, rgba, .012)
        box.points = _rect(core.tracker.pose, bay.x, bay.y, bay.entry_yaw,
                           layout.p.bay_depth, layout.p.bay_width)
        out.markers.append(box)
        label = _marker(stamp, 'layout_state', i, Marker.TEXT_VIEW_FACING, rgba, .06)
        label.pose.position = _to_base(core.tracker.pose, bay.x, bay.y)
        label.pose.position.z = .05
        label.text = f'{i+1}: {state if supported else "candidate"}'
        out.markers.append(label)
    return out


def observation_markers(core, stamp, now) -> MarkerArray:
    """Recent evidence in the vehicle frame, generated only while a viewer subscribes.

    Grey background is unknown, not an observed free floor. Cells use their real
    evidence timestamps, so stale data disappears even when scans stop arriving.
    """
    out = MarkerArray()
    layout, pose = core.layout, core.tracker.pose
    if layout is None:
        return out
    background = _marker(stamp, 'evidence_background', 0, Marker.CUBE,
                         (.24, .24, .28, 1.), 1.)
    background.scale.x = background.scale.y = 2*layout.p.lidar_range
    background.scale.z = .005
    background.pose.position.z = -.03
    out.markers.append(background)
    free = _marker(stamp, 'observed_free', 0, Marker.CUBE_LIST, (.1, .55, .65, .65), layout.p.grid)
    occupied = _marker(stamp, 'observed_occupied', 0, Marker.CUBE_LIST, (1., .15, .12, 1.), layout.p.grid)
    # World-aligned cells rotate as well as translate into base_link.
    for marker in (free, occupied):
        marker.pose.position = _to_base(pose, 0., 0.)
        marker.pose.orientation.z = math.sin(-pose.yaw/2)
        marker.pose.orientation.w = math.cos(-pose.yaw/2)
        marker.scale.z = .01
    for (ix, iy), (seen, hit) in layout.cells.items():
        if not 0 <= now-seen <= layout.p.evidence_age:
            continue
        x, y = (ix+.5)*layout.p.grid, (iy+.5)*layout.p.grid
        relative = _to_base(pose, x, y)
        if max(abs(relative.x), abs(relative.y)) > layout.p.lidar_range:
            continue
        point = Point(x=float(x), y=float(y), z=0.)
        (occupied if hit else free).points.append(point)
    out.markers.extend((free, occupied))
    vehicle = _marker(stamp, 'vehicle', 0, Marker.LINE_STRIP, (.4, .65, 1., 1.), .018)
    vehicle.points = _rect(pose, pose.x, pose.y, pose.yaw, layout.p.body_length, layout.p.body_width)
    out.markers.append(vehicle)
    if layout.origin is not None:
        arrow = _marker(stamp, 'grid_reference', 0, Marker.ARROW, (1., .9, .1, 1.), .02)
        arrow.scale.y, arrow.scale.z = .045, .07
        x, y = map(float, layout.origin)
        arrow.points = [_to_base(pose, x, y),
                        _to_base(pose, x+.25*math.cos(layout.yaw), y+.25*math.sin(layout.yaw))]
        out.markers.append(arrow)
    status = _marker(stamp, 'observation_status', 0, Marker.TEXT_VIEW_FACING, (1., 1., 1., 1.), .075)
    status.pose.position.x, status.pose.position.y, status.pose.position.z = 1., 1., .1
    age = now-layout.last_scan
    freshness = 'waiting for scan' if not math.isfinite(age) else f'scan age {age:.1f}s'
    mode = 'CONTROL' if layout.p.control_enabled else 'OBSERVE'
    status.text = (f'{mode} | {freshness}\n{layout.reason}\n'
                   'MAP: cyan=observed free / red=hit / grey=unknown\n'
                   'BAYS: grey=candidate / orange=unconfirmed route / green=clear')
    out.markers.append(status)
    return out
