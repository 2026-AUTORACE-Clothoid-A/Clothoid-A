#!/usr/bin/env python3
"""Repeat the static obstacle mission in the Panda3D simulator (/home/a/simul) with the real nodes.

Per run: a copy of the simulator map with the obstacle LIMOs in the chosen grid cells, the simulator
headless in real time, `ros2 launch clothoid_lidar avoid.launch.py` (LIMO extractor + static_avoid,
sim time, the run's parameters) and a driver in this process standing in for lane_main:
  * before the avoidance it follows lane 2, while /avoid/active it forwards /cmd_vel_avoid, after the
    hand-over it goes to the sign's lane (A = lane 1, B = lane 3);
  * it publishes /direction_flag (from the start, or once the car passes sign_at metres after the
    entry line).
Judged from /sim/ground_truth and /sim/parking_status: a collision, the closest approach to a LIMO,
the painted island (its pointed 0.25 m tip counted apart), the lane at the end, stuck.

    source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
    python3 tools/static_avoid/simul_batch.py --set fixed --conf 'seq={}' --conf 'one={"sequence": false}'
    python3 tools/static_avoid/simul_batch.py --set random --count 20 --conf 'seq={}'

Simulator track (map x points against the driving direction): lane centres y 1.46 (lane 3, B),
0.94 (lane 2), 0.42 (lane 1, A), lane width 0.52 m; white entry line x 8.64; island tip x 4.27
(4.37 m after the line). Obstacle cells: 8 columns of 0.470 m between x 4.268 and 8.030; the rear
of a LIMO in column c is 8.64 - (4.268 + (c - 0.5) 0.470 + 0.161) m after the entry line
(c 8: 0.69, 7: 1.16, 6: 1.63, 5: 2.10, 4: 2.57, 3: 3.04, 2: 3.51, 1: 3.98).
"""
import argparse
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time

import numpy as np
import yaml

DOMAIN = os.environ.setdefault('ROS_DOMAIN_ID', '77')
os.environ['ROS_LOCALHOST_ONLY'] = '1'    # as the simulator and the nodes it starts (a shell's 0 cut them off)

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rcl_interfaces.msg import Log  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import Bool, Int32, String  # noqa: E402

SIM = os.environ.get('SIMUL_DIR', '/home/a/simul')
RELAY = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'image_relay.py')
WS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'ros2_ws'))
ENTRY_X, ISLAND_TIP_X, ISLAND_LEN, TIP = 8.64, 4.27, 1.75, 0.25
LANE_Y = {1: 0.42, 2: 0.94, 3: 1.46}
ROW = {1: 'lower', 2: 'middle', 3: 'upper'}
X_MIN, COL_W, LIMO_L, LIMO_W = 4.268, (8.030 - 4.268) / 8, 0.322, 0.220
CAR_L, CAR_W, WHEELBASE = 0.322, 0.220, 0.2
SPAWN = (9.3, 0.94, math.pi)          # 0.66 m before the entry line (corner exit), lane 2
# --corner: from before the crosswalk on the right-hand road (x 9.58, driving -y), through the 90 deg
# right turn into lane 2 (1112 car runs: the avoidance started in the turn, 25~30 deg before its end,
# and the turn ended after the entry line: lane_main turns late and wide). The road's centre line
# (between the yellow arcs of track_geometry.json, outer r 0.80 about (9.08, 1.44), inner r 0.60
# about (8.73, 1.79)) is r 0.70 about (8.88, 1.64) and ends 0.24 m before the entry line; the driver
# takes r 1.08 about (8.50, 2.02) instead, still on the road, ending 0.14 m after it
CORNER_SPAWN = (9.58, 3.0, -math.pi / 2)
CORNER_C, CORNER_R = (8.50, 2.02), 1.08


def corner_path(step=0.02):
    """Lane 2 centre line from CORNER_SPAWN through the turn and on along lane 2, (N, 2)."""
    x0, y0 = CORNER_SPAWN[0], CORNER_SPAWN[1]
    cx, cy = CORNER_C
    down = np.column_stack((np.full(int((y0 - cy) / step), x0), np.arange(y0, cy, -step)))
    a = np.arange(0.0, -math.pi / 2, -step / CORNER_R)
    arc = np.column_stack((cx + CORNER_R * np.cos(a), cy + CORNER_R * np.sin(a)))
    xs = np.arange(cx, 0.0, -step)
    along = np.column_stack((xs, np.full_like(xs, LANE_Y[2])))
    return np.vstack((down, arc, along))
END_X = 3.3                           # past the island tip by ~1 m: judged here


def cell_x(col):
    return X_MIN + (col - 0.5) * COL_W


def rear_after_entry(col):
    return ENTRY_X - (cell_x(col) + LIMO_L / 2)


def layouts(kind, count, seed):
    if kind == 'fixed':
        # nearest cells to the tape-measured 1108 layouts; the simulator's own 7-car practice layout
        return [('1108 run1-2 (L2 c7, L3 c6)', {2: [7], 3: [6]}),
                ('1108 run3 (L2 c7, L1 c6)', {2: [7], 1: [6]}),
                ('1108 run6-9 (L2 c3, L3 c4)', {2: [3], 3: [4]}),
                ('sim 7 cars', {3: [3, 8], 2: [4, 5, 7], 1: [2, 6]})]
    if kind == 'corner':
        # with --corner: a lane 2 LIMO in column 8 (rear 0.69 m after the entry line) or 7 (1.16 m)
        # is 1.5 m ahead while the car is still in the turn
        return [('corner L2 c8, L3 c7', {2: [8], 3: [7]}),
                ('corner L2 c8, L1 c7', {2: [8], 1: [7]}),
                ('corner L2 c8, L3 c5', {2: [8], 3: [5]}),
                ('corner L2 c8, L1 c5', {2: [8], 1: [5]}),
                ('corner L2 c7, L3 c6', {2: [7], 3: [6]}),
                ('corner L2 c7, L1 c5, L3 c4', {2: [7], 1: [5], 3: [4]})]
    rng = np.random.default_rng(seed)
    seen, out = set(), []
    while len(out) < count:
        c2 = int(rng.integers(3, 8))             # lane 2 rear 1.16 .. 3.04 m
        side = int(rng.choice([1, 3]))
        cs = int(rng.integers(2, 7))             # other lane rear 1.63 .. 3.51 m
        if (c2, side, cs) in seen:
            continue
        seen.add((c2, side, cs))
        out.append((f'L2 c{c2}, L{side} c{cs}', {2: [c2], side: [cs]}))
    return out


def write_map(tmp, cells):
    with open(os.path.join(SIM, 'src/limo_parking_sim/config/parking_map.yaml'), encoding='utf-8') as f:
        world = yaml.safe_load(f)
    lay = world['avoidance_layout']
    lay['occupied'] = {ROW[lane]: cols for lane, cols in cells.items()}
    top = max(c for cols in cells.values() for c in cols)
    if top > lay['columns']:         # column 9: one more 0.470 m column towards the entry line
        lay['columns'], lay['x_max'] = top, X_MIN + top * COL_W
    path = os.path.join(tmp, 'map.yaml')
    with open(path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(world, f)
    return path


def rect(x, y, yaw, length, width):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(length / 2, width / 2), (length / 2, -width / 2), (-length / 2, -width / 2),
           (-length / 2, width / 2)]
    return np.array([(x + c * a - s * b, y + s * a + c * b) for a, b in pts])


def outline(poly, n=12):
    return np.vstack([poly[k] + f * (poly[(k + 1) % 4] - poly[k])
                      for k in range(4) for f in np.linspace(0, 1, n, endpoint=False)])


def gap(car, limos):
    pts = outline(car)
    best = math.inf
    for x, y in limos:
        if ((np.abs(pts[:, 0] - x) < LIMO_L / 2) & (np.abs(pts[:, 1] - y) < LIMO_W / 2)).any():
            return -1.0
        o = outline(rect(x, y, math.pi, LIMO_L, LIMO_W))
        best = min(best, float(np.min(np.linalg.norm(pts[:, None] - o[None], axis=2))))
    return best


def on_island(car):
    """'wide' if the car is over the island body, 'tip' if only over its pointed first 0.25 m."""
    pts = outline(car)
    u = ISLAND_TIP_X - pts[:, 0]                     # metres into the island
    inside = (u >= 0) & (u <= ISLAND_LEN)
    half = np.clip(np.minimum(u, ISLAND_LEN - u) / TIP, 0, 1) * (0.52 / 2 - 0.02)
    hit = inside & (np.abs(pts[:, 1] - LANE_Y[2]) < half)
    if not hit.any():
        return None
    return 'wide' if (u[hit] > TIP).any() else 'tip'


class Driver(Node):
    def __init__(self, direction, sign_at, speed=0.3, corner=False):
        super().__init__('simul_batch_driver', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.direction, self.sign_at, self.speed = direction, sign_at, speed
        self.path = corner_path() if corner else None
        self.pose = None
        self.active = False
        self.was_active = False
        self.avoid_cmd, self.avoid_t = None, -1.0
        self.collisions = 0
        self.collision_names = []
        self.log = []
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.flag_pub = self.create_publisher(Int32, '/direction_flag', 1)
        self.create_subscription(Odometry, '/sim/ground_truth', self.on_truth, 10)
        self.create_subscription(Bool, '/avoid/active', self.on_active, 10)
        self.create_subscription(Twist, '/cmd_vel_avoid', self.on_avoid, 10)
        self.create_subscription(String, '/sim/parking_status', self.on_status, 10)
        self.create_subscription(Log, '/rosout', self.on_log, 50)

        self.flag_sent = 0.0

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_truth(self, msg):
        q = msg.pose.pose.orientation
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def on_active(self, msg):
        self.active = msg.data
        self.was_active |= msg.data

    def on_avoid(self, msg):
        self.avoid_cmd, self.avoid_t = msg, self.now()

    def on_status(self, msg):
        st = json.loads(msg.data)
        self.collisions = st.get('collisions', 0)
        if st.get('collision_names'):
            self.collision_names = st['collision_names']

    def on_log(self, msg):
        if msg.name == 'static_avoid' and 'load' not in msg.msg[:5]:
            self.log.append(f'{self.now():.1f} {msg.msg}')

    def lane_target(self):
        if self.was_active and not self.active:          # handed over: lane_main takes the sign's road
            if self.direction in (0, 1):
                return LANE_Y[{0: 1, 1: 3}[self.direction]]
            return min((LANE_Y[1], LANE_Y[3]), key=lambda y: abs(y - self.pose[1]))
        return LANE_Y[2]

    def step(self):
        """One control step; returns False once the run is over."""
        if self.pose is None:
            return True
        x, y, yaw = self.pose
        passed = ENTRY_X - x
        if self.direction in (0, 1) and (self.sign_at is None or passed >= self.sign_at):
            if self.now() - self.flag_sent > 0.5:
                self.flag_pub.publish(Int32(data=self.direction))
                self.flag_sent = self.now()
        cmd = Twist()
        if self.active:
            if self.avoid_cmd is not None and self.now() - self.avoid_t < 0.3:
                cmd = self.avoid_cmd
        else:
            yt, ld = self.lane_target(), 0.45
            px, py = x - ld, yt                           # look-ahead on the lane, driving to -x
            if self.path is not None and not self.was_active:      # the corner, then lane 2
                k = int(np.argmin(np.hypot(self.path[:, 0] - x, self.path[:, 1] - y)))
                px, py = self.path[min(k + int(ld / 0.02), len(self.path) - 1)]
            dx, dy = px - x, py - y
            lx = math.cos(yaw) * dx + math.sin(yaw) * dy
            ly = -math.sin(yaw) * dx + math.cos(yaw) * dy
            delta = math.atan2(2 * WHEELBASE * ly, lx * lx + ly * ly)
            delta = max(-0.40, min(0.40, delta))
            cmd.linear.x = self.speed
            cmd.angular.z = self.speed * math.tan(delta) / WHEELBASE
        self.cmd_pub.publish(cmd)
        return x > END_X


def start(cmd, logfile, env):
    return subprocess.Popen(['bash', '-c', cmd], stdout=logfile, stderr=subprocess.STDOUT, env=env,
                            start_new_session=True)


def stop(proc):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(8)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()


def run_one(name, cells, direction, conf, sign_at, t_max, keep_dir, spawn=SPAWN, image_every=0,
            record=False, corner=False):
    tmp = tempfile.mkdtemp(prefix='simul_batch_')
    env = dict(os.environ, ROS_DOMAIN_ID=DOMAIN, ROS_LOCALHOST_ONLY='1')
    map_path = write_map(tmp, cells)
    params = os.path.join(tmp, 'avoid.yaml')
    with open(params, 'w') as f:
        # the simulator publishes /imu and /odom only, no limo_base 25 Hz copies (static_avoid.yaml)
        yaml.safe_dump({'static_avoid': {'ros__parameters': dict(
            {'lane_width': 0.52, 'imu_topic': '/imu', 'odom_topic': '/odom'}, **conf)}}, f)
    sim_log, avoid_log = open(os.path.join(tmp, 'sim.log'), 'w'), open(os.path.join(tmp, 'avoid.log'), 'w')
    # headless rendering through EGL without a display (Intel / Mesa on this PC)
    sim = start(f'cd {SIM} && EGL_PLATFORM=surfaceless ./run.sh --map {map_path} --headless --spawn {spawn[0]} '
                f'{spawn[1]} {spawn[2]} --duration {t_max + 30}', sim_log, env)
    rec = None
    if record:                       # raw sensors for replay_bag.py (the run's directory is kept)
        keep_dir = True
        rec = start(f'source /opt/ros/humble/setup.bash && ros2 bag record -o {tmp}/bag '
                    '/camera/color/image_raw /scan /odom /imu /sim/ground_truth /rosout /avoid/active',
                    open(os.path.join(tmp, 'record.log'), 'w'), env)
    avoid = start(f'source /opt/ros/humble/setup.bash && source {WS}/install/setup.bash && '
                  f'ros2 launch clothoid_lidar avoid.launch.py camera:=limo2 use_sim_time:=true '
                  f'ignore_cone:=true course_map:=false params:={params} '
                  f'image:={"10hz" if image_every else "raw"}',
                  avoid_log, env)
    relay = None
    if image_every:
        # the car's bringup throttle (topic_tools is not installed here) in its own process: inside
        # the driver's single thread it passed ~1 frame/s of the 30 Hz raw image
        relay = start(f'source /opt/ros/humble/setup.bash && python3 {RELAY} {image_every}',
                      open(os.path.join(tmp, 'relay.log'), 'w'), env)
    drv = Driver(direction, sign_at, corner=corner)
    limos = [(cell_x(c), LANE_Y[lane]) for lane, cols in cells.items() for c in cols]
    track, min_gap, island, ok_start, arrived = [], math.inf, None, False, False
    t_wall = time.time()
    t0 = None
    try:
        while time.time() - t_wall < t_max * 3 + 60:
            rclpy.spin_once(drv, timeout_sec=0.05)
            if drv.pose is None:
                continue
            if not ok_start:
                # wait for the avoid node (it publishes /avoid/active every scan)
                if drv.active is False and drv.count_publishers('/avoid/active') == 0:
                    drv.cmd_pub.publish(Twist())
                    continue
                ok_start, t0 = True, drv.now()
            if not drv.step():
                arrived = True
                break
            if drv.now() - t0 > t_max:
                break
            car = rect(*drv.pose, CAR_L, CAR_W)
            track.append((drv.now() - t0, *drv.pose, drv.active))
            min_gap = min(min_gap, gap(car, limos))
            where = on_island(car)
            if where == 'wide' or (where == 'tip' and island is None):
                island = where
    finally:
        drv.cmd_pub.publish(Twist())
        stop(avoid)
        if relay is not None:
            stop(relay)
        if rec is not None:
            stop(rec)
        stop(sim)
        sim_log.close()
        avoid_log.close()
    x, y = (track[-1][1], track[-1][2]) if track else (spawn[0], spawn[1])
    want = {0: 1, 1: 3}.get(direction)
    lane = min(LANE_Y, key=lambda k: abs(LANE_Y[k] - y))
    why = []
    if drv.collisions or min_gap <= 0.0:
        why.append('HIT')
    elif min_gap <= 0.03:
        why.append('gap<3')
    if island == 'wide':
        why.append('island')
    if not arrived:
        why.append('stuck')
    elif want is not None and lane != want:
        why.append('lane')
    lanes = []
    for _, tx, ty, _, act in track:
        if tx > ENTRY_X:                                  # the corner (--corner) is no lane
            continue
        k = min(LANE_Y, key=lambda j: abs(LANE_Y[j] - ty))
        if not lanes or lanes[-1] != k:
            lanes.append(k)
    out = dict(name=name, direction=direction, why=why, gap=min_gap, tip=island == 'tip',
               lanes='-'.join(map(str, lanes)), time=track[-1][0] if track else 0.0, end=(x, y),
               log=drv.log[-12:], dir=tmp)
    drv.destroy_node()
    if keep_dir:
        out['track'] = track
    else:
        shutil.rmtree(tmp, ignore_errors=True)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--set', choices=('fixed', 'random', 'corner'), default='fixed')
    ap.add_argument('--count', type=int, default=20, help='random layouts')
    ap.add_argument('--seed', type=int, default=7)
    ap.add_argument('--conf', action='append', default=[], help='name={static_avoid params as JSON}')
    ap.add_argument('--sign', default='LR', help='L, R or LR')
    ap.add_argument('--sign-at', type=float, default=None, help='m after the entry line')
    ap.add_argument('--t-max', type=float, default=45.0, help='simulated s per run')
    ap.add_argument('--only', type=int, default=None, help='just this layout index')
    ap.add_argument('--keep', action='store_true', help='keep logs (printed directory)')
    ap.add_argument('--record', action='store_true', help='also record a bag of the raw sensors (kept)')
    ap.add_argument('--image-every', type=int, default=3,
                    help='republish every n-th raw frame on /camera/color/image_10hz and run avoid on it '
                         '(3 = the car\'s 10 Hz throttle, 5 = 6 Hz); 0 = avoid reads the raw image')
    ap.add_argument('--spawn', type=float, nargs=3, default=list(SPAWN), metavar=('X', 'Y', 'YAW'),
                    help='start pose in map coordinates (default: corner exit, lane 2)')
    ap.add_argument('--corner', action='store_true',
                    help='start before the crosswalk and drive the 90 deg right turn into lane 2 (the '
                         'avoidance may start in the turn, as on the 1112 car runs); overrides --spawn')
    ap.add_argument('--json', default=None, help='write every result here')
    a = ap.parse_args()
    confs = {}
    for c in a.conf or ['default={}']:
        k, js = c.split('=', 1)
        confs[k] = json.loads(js)
    lays = layouts(a.set, a.count, a.seed)
    if a.only is not None:
        lays = [lays[a.only]]
    rclpy.init()
    results = []
    try:
        for cname, conf in confs.items():
            for name, cells in lays:
                for sign in a.sign:
                    d = {'L': 0, 'R': 1}[sign]
                    r = run_one(name, cells, d, conf, a.sign_at, a.t_max, a.keep,
                                CORNER_SPAWN if a.corner else tuple(a.spawn), a.image_every, a.record,
                                a.corner)
                    r['conf'] = cname
                    results.append(r)
                    print(f'[{cname}] {name} sign {sign}: {"ok" if not r["why"] else " ".join(r["why"])}'
                          f'  gap {100 * r["gap"]:.0f} cm  lanes {r["lanes"]}'
                          f'{"  (tip)" if r["tip"] else ""}  {r["time"]:.0f} s, end x {r["end"][0]:.2f} y {r["end"][1]:.2f}'
                          + (f'  {r["dir"]}' if a.keep else ''), flush=True)
                    if r['why']:
                        for line in r['log']:
                            print('      ' + line, flush=True)
    finally:
        rclpy.shutdown()
    print()
    for cname in confs:
        rs = [r for r in results if r['conf'] == cname]
        bad = [r for r in rs if r['why']]
        cnt = {k: sum(k in r['why'] for r in rs) for k in ('HIT', 'gap<3', 'island', 'stuck', 'lane')}
        gaps = [r['gap'] for r in rs if math.isfinite(r['gap']) and r['gap'] > 0]
        print(f'{cname:16s} ok {len(rs) - len(bad)}/{len(rs)} | ' + ' '.join(f'{k} {v}' for k, v in cnt.items())
              + f' | tip {sum(r["tip"] for r in rs)} | gap min {100 * min(gaps or [0]):.0f} cm')
    if a.json:
        with open(a.json, 'w') as f:
            json.dump([{k: v for k, v in r.items() if k != 'track'} for r in results], f, indent=1)


if __name__ == '__main__':
    main()
