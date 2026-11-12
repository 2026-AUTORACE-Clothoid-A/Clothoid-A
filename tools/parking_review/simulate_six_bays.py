#!/usr/bin/env python3
"""Use the actual simulator physics without a window or ROS publishers.

This isolates parking selection/control: a known CAD anchor replaces the camera.
It must not be reported as end-to-end visual localization validation.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'ros2_ws/src/clothoid_parking'), str(ROOT/'simul/src/limo_parking_sim')]
from clothoid_parking.core import ParkingCore
from clothoid_parking.layout import LayoutParams
from clothoid_parking.planner import PlanParams, Phase
from clothoid_parking.tracker import Pose2D
from limo_parking_sim.config import load_config
from limo_parking_sim.model import Simulation


def run(target, mode, show=False, ros_view=False, scenario="nominal"):
    cfg = load_config(str(ROOT/'simul/src/limo_parking_sim/config/limo_pro.yaml'),
                      str(ROOT/'simul/src/limo_parking_sim/config/parking_map.yaml'))
    cfg['simulation']['spawn'] = [5.5, 4.84, 0.]
    world = cfg['world']
    # These are test-scene placements, not controller coordinates.
    xs, ys = [6.360085, 6.880126, 7.400167], [4.270171, 3.649864]
    empty = {target-1}
    if target > 3:
        empty.add(target-4)  # open the front bay on the route to the inner one
    if scenario in ('all_occupied', 'auto_all_occupied'):
        empty.clear()
    if scenario == 'blocked_front' and target > 3:
        empty.discard(target-4)
    world['obstacles'] = [dict(name=f'parked_{r*3+c+1}', pose=[x, y, math.pi/2])
                          for r, y in enumerate(ys) for c, x in enumerate(xs) if r*3+c not in empty]
    for key in ('sign', 'fork', 'traffic_light', 'advertisement', 'speed_bump', 'cone_course'):
        world.pop(key, None)
    world['signs'], world['cones'] = [], []
    if scenario == 'sign_occlusion':
        world['signs'] = [dict(name='occluding_sign', asset='parking.png', pose=[xs[(target-1)%3]-.22, 4.55, 0.], post_width=.04)]
    if scenario == 'lane_blocked':
        world['obstacles'].append(dict(name='lane_obstacle', pose=[5.95, 4.84, 0.]))
    for key in ('noise_stddev', 'noise_per_m', 'dropout_probability', 'spike_probability'):
        if scenario != 'sensor_noise':
            cfg['lidar'][key] = 0.
    sim = Simulation(cfg)
    if scenario == "fast_approach":
        sim.speed = .8
    noise_source = None
    if scenario == "sensor_noise":
        from limo_parking_sim.ros_bridge import RosBridge
        noise_source = lambda: RosBridge._noisy_odom_imu(SimpleNamespace(cfg=cfg, model=sim))
    scene, next_mode = None, [None]
    if show:
        from limo_parking_sim.scene import Scene
        from direct.gui.OnscreenText import OnscreenText
        from panda3d.core import TextNode
        scene = Scene(cfg, sim)
        scene.set_view('parking')
        def restart(value):
            next_mode[0] = value
            scene.closed = True
        scene.base.accept('f5', restart, [mode])
        scene.base.accept('f6', restart, ['controller'])
        scene.base.accept('f7', restart, ['integrated'])
        scene.base.ignore('k')
        scene.base.ignore('e')
        scene.help.setText('P: pause/resume | F5: restart | F6: goal-given control | F7: lidar selection\n'
                           '1..4: views | mouse: pan/orbit/zoom | Esc: close')
        scene.title.destroy()
        scene.title = OnscreenText(text=f'PARKING BAY {target} / {mode.upper()} TEST',
                                   pos=(-1.52, .91), scale=.05, align=TextNode.ALeft,
                                   fg=(1., 1., 1., 1.))
        detail = OnscreenText(text='', pos=(-1.52, .53), scale=.035,
                             align=TextNode.ALeft, mayChange=True,
                             fg=(1., .85, .3, 1.))
    plan = PlanParams(rear_offset=0., odom_scale=.985 if scenario == "sensor_noise" else 1., turn_radius=sim.min_radius, strict_completion=True)
    core = ParkingCore(plan=plan, layout=LayoutParams(control_enabled=True, target_bay=0 if scenario == 'auto_all_occupied' else target))
    core.prepare(True)
    for t in (.01, .02, .03):
        core.layout.observe(t, (6.100085-5.5, 4.580171-4.84), 0., Pose2D())
    goal = core.layout.bays()[target-1]
    if mode == 'controller':
        # Separate manoeuvre feasibility from unobserved-space gating.
        core.layout.p.control_enabled = False
        core.set_flag(.04, True)
        core.planner.bay = goal
    review = None
    if ros_view:
        from sim_review_ros import ReviewPublisher
        review = ReviewPublisher(core, mode, target)
    events, parked, reasons, states = [], None, Counter(), Counter()
    held = None
    exploration_reasons = Counter()
    previous = core.phase
    deadline, paused_once = time.perf_counter(), False
    for i in range(1, 6001):
        if scene:
            while scene.paused and not scene.closed:
                if review:
                    review.update(sim)
                scene.render_frame()
                time.sleep(.02)
                deadline = time.perf_counter()
            if scene.closed:
                break
        now = i/100
        measured_speed, measured_gyro = noise_source() if noise_source else (sim.speed, sim.yaw_rate)
        core.imu(now, measured_gyro)
        if i % 2 == 0:
            core.odom(now, measured_speed)
        if mode == 'integrated' and i % 10 == 0:
            ranges = sim.scan()
            if scenario == "no_returns":
                ranges[:] = float("nan")
            core.scan(now, ranges, float(sim.angles[0]), float(sim.angles[1]-sim.angles[0]),
                      cfg['lidar']['range_min'], cfg['lidar']['range_max'])
            reasons[core.layout.reason] += 1
            states[core.layout.states[target-1]] += 1
            if not core.flag and core.layout.selected is not None:
                core.set_flag(now, True)
        command = core.command(now)
        v, steer = command or (0., 0.)
        if mode == 'integrated' and not core.flag:
            # Actual exploration policy caps lane speed, not a test-only stop point.
            cap = core.exploration_limit(now)
            v = min(.8, cap) if cap is not None else .8
            exploration_reasons[core.exploration.reason] += 1
            if core.exploration.skipped:
                break
        if command is not None and command[0] == 0 and core.phase in (Phase.TURN, Phase.ENTER, Phase.EXIT_BACK, Phase.EXIT_TURN) and held is None:
            held = dict(t=now, phase=core.phase.value, reason=core.layout.reason, pose=[sim.x,sim.y,sim.yaw])
        sim.command(v, v*math.tan(steer)/cfg['vehicle']['wheelbase'])
        sim.step(.01)
        if review:
            review.update(sim)
        if scene:
            detail.setText(f'Known CAD anchor / ideal lidar / camera NOT tested\n'
                           f'Parking: {core.phase.value} | Target: {target}\n'
                           + (core.layout.reason if mode == 'integrated' else
                              'Goal supplied: manoeuvre test, NOT autonomous bay selection'))
            if i % 3 == 0:
                scene.render_frame()
            if i % 10 == 0:
                scene.update_scan(sim.scan())
            if mode == 'integrated' and now > 3 and v == 0 and not core.flag and not paused_once:
                paused_once = True
                scene.paused = True
            deadline += .01
            time.sleep(max(0., deadline-time.perf_counter()))
        if core.phase != previous:
            events.append([round(now, 2), core.phase.value, core.planner.reason])
            previous = core.phase
        if core.phase == Phase.PARKED and parked is None:
            corners = sim.body().corners()
            gx, gy = xs[(target-1)%3], ys[(target-1)//3]
            inside = bool(((abs(corners[:, 0]-gx)<.25)&(abs(corners[:, 1]-gy)<.30)).all())
            parked = dict(inside=inside, position=[sim.x, sim.y], yaw=sim.yaw)
        if sim.collisions or core.phase in (Phase.DONE, Phase.ABORT):
            break
    if scene:
        scene.paused = True
        while not scene.closed:
            if review:
                review.update(sim)
            scene.render_frame()
            time.sleep(.02)
        scene.close()
    if review:
        review.close()
    return dict(target=target, mode=mode, scenario=scenario, first_hold=held, phase=core.phase.value, parked=parked,
                next_mode=next_mode[0],
                collisions=sim.collisions, layout_reason=core.layout.reason,
                states=core.layout.states, events=events, reasons=dict(reasons),
                target_state_counts=dict(states), exploration_reasons=dict(exploration_reasons),
                skipped=core.exploration.skipped,
                returned_to_lane=abs(sim.y-4.84)<.1 and abs(sim.yaw)<.2,
                scope='actual simulator physics; '+('configured lidar/imu/odom noise' if noise_source else 'ideal sensors')+'; supplied CAD anchor, camera not tested')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--cases', action='store_true', help='Also test occlusion, blocked lane/front, occupied and invalid scans')
    p.add_argument('--show', action='store_true', help='Show the same test in the simulator window')
    p.add_argument('--ros-view', action='store_true', help='Publish clock and RViz markers only')
    p.add_argument('--target', type=int, choices=range(1, 7), default=1)
    p.add_argument('--mode', choices=('controller', 'integrated'), default='integrated')
    args = p.parse_args()
    result = []
    if args.show:
        mode = args.mode
        while mode is not None:
            row = run(args.target, mode, show=True, ros_view=args.ros_view)
            result.append(row)
            print(json.dumps(row), flush=True)
            mode = row['next_mode']
    else:
        for mode in ('controller', 'integrated'):
            for target in range(1, 7):
                row = run(target, mode)
                result.append(row)
                print(json.dumps(row), flush=True)
    if args.cases and not args.show:
        for scenario, target in [('fast_approach',1), ('sensor_noise',1), ('sign_occlusion',1), ('blocked_front',4), ('all_occupied',1), ('auto_all_occupied',1), ('lane_blocked',1), ('no_returns',1)]:
            row = run(target, 'integrated', scenario=scenario)
            result.append(row)
            print(json.dumps(row), flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2))
