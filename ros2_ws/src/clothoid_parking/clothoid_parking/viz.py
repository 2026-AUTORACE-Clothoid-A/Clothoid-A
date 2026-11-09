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
