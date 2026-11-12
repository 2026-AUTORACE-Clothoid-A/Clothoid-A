#!/usr/bin/env python3
"""Closed-loop actual simulator physics/lidar; supplied CAD anchor, not camera validation."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import sys
import time
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'ros2_ws/src/clothoid_parking'), str(ROOT/'simul/src/limo_parking_sim')]
from clothoid_parking.core import ParkingCore
from clothoid_parking.layout import LayoutParams
from clothoid_parking.planner import PlanParams, Phase
from clothoid_parking.tracker import Pose2D
from limo_parking_sim.config import load_config
from limo_parking_sim.model import Simulation

CASES = {'aligned': (0,0,0,0), 'both_10':(10,10,0,0), 'both_20':(20,20,0,0), 'both_minus20':(-20,-20,0,0),
         'opposite_15':(15,-15,0,0), 'opposite_25':(25,-25,0,0),
         'depth_opposite':(0,0,.08,-.08), 'one_forward':(0,0,.10,0)}

def run(case, mode='registered', target=2, seconds=45., show=False):
    cfg = load_config(str(ROOT/'simul/src/limo_parking_sim/config/limo_pro.yaml'), str(ROOT/'simul/src/limo_parking_sim/config/parking_map.yaml'))
    spawn = [5.8,4.84,0.]
    cfg['simulation']['spawn'] = spawn
    xs, ys = [6.360085,6.880126,7.400167], [4.270171,3.649864]
    world = cfg['world']
    for key in ('sign','fork','traffic_light','advertisement','speed_bump','cone_course'):
        world.pop(key,None)
    world['signs'],world['cones'] = [],[]
    empty = {target-1}
    if target>3: empty.add(target-4)
    if case == 'occupied': empty.clear()
    left,right,dl,dr = CASES.get(case,(0,0,0,0))
    world['obstacles'] = []
    for r,y in enumerate(ys):
        for c,x in enumerate(xs):
            if r*3+c in empty: continue
            angle = left if c == 0 else right if c == 2 else 0
            shift = dl if c == 0 else dr if c == 2 else 0
            world['obstacles'].append(dict(name=f'parked_{r*3+c+1}',pose=[x,y+shift,math.pi/2+math.radians(angle)]))
    if case == 'lane_blocked':
        world['obstacles'].append(dict(name='blocking_lane',pose=[6.3,4.84,0.]))
    for key in ('noise_stddev','noise_per_m','dropout_probability','spike_probability'):
        if case != 'sensor_noise': cfg['lidar'][key] = 0.
    if case == 'rear_visible':
        cfg['lidar']['angle_min_deg'],cfg['lidar']['angle_max_deg'] = -180.,180.
    sim = Simulation(cfg)
    plan = PlanParams(rear_offset=0.,odom_scale=1.,turn_radius=sim.min_radius)
    layout = LayoutParams(control_enabled=(mode=='registered'),target_bay=target)
    core = ParkingCore(plan=plan,layout=layout)
    core.prepare(True)
    for t in (.01,.02,.03):
        core.layout.observe(t,(6.100085-spawn[0],4.580171-spawn[1]),0.,Pose2D())
    core.set_flag(.04,True)
    phases,holds,timings,trace = [],Counter(),[],[]
    command_timings=[]
    v,steer=0.,0.
    first_hold=None
    parked = None
    prev = core.phase
    scene = None
    restart = [False]
    if show:
        from limo_parking_sim.scene import Scene
        from direct.gui.OnscreenText import OnscreenText
        from panda3d.core import TextNode
        scene = Scene(cfg, sim)
        scene.set_view('parking')
        scene.paused = True
        scene.base.ignore('k')
        scene.base.ignore('e')
        def again():
            restart[0] = True
            scene.closed = True
        scene.base.accept('f5', again)
        scene.title.destroy()
        scene.title = OnscreenText(text='FRONT MIDDLE BAY 2 / '+case, pos=(-1.5,.91), scale=.05, align=TextNode.ALeft, fg=(1.,1.,1.,1.))
        scene.help.setText('P: start / pause | F5: restart | 1: parking view | Esc: close')
        detail = OnscreenText(text='Current parking code / CAD anchor supplied\nPress P to start',
                             pos=(-1.5,.50), scale=.035, align=TextNode.ALeft,
                             mayChange=True, fg=(1.,.85,.3,1.))
        scene.render_frame()
        screen = ROOT/'ros2_ws/log/parking_pose_audit/live_scene.png'
        screen.parent.mkdir(parents=True, exist_ok=True)
        scene.base.win.getScreenshot().write(str(screen))
        print('GUI_READY: front middle bay 2; paused; P starts, F5 restarts', flush=True)
    deadline = time.perf_counter()
    for i in range(1,int(seconds*100)+1):
        if scene:
            while scene.paused and not scene.closed:
                scene.render_frame()
                time.sleep(.02)
                deadline = time.perf_counter()
            if scene.closed:
                break
        now = i/100
        core.imu(now,sim.yaw_rate)
        if i%2==0: core.odom(now,sim.speed)
        if i%10==0:
            ranges = sim.scan()
            if case == 'no_returns': ranges[:] = np.nan
            start = time.perf_counter()
            core.scan(now,ranges,float(sim.angles[0]),float(sim.angles[1]-sim.angles[0]),cfg['lidar']['range_min'],cfg['lidar']['range_max'])
            timings.append((time.perf_counter()-start)*1000)
        if i%5==0:  # production parking node runs at 20 Hz
            start = time.perf_counter()
            v,steer = core.command(now) or (0.,0.)
            command_timings.append((time.perf_counter()-start)*1000)
            if v == 0 and core.phase in (Phase.TURN,Phase.ENTER,Phase.EXIT_BACK,Phase.EXIT_TURN) and first_hold is None:
                first_hold=dict(t=now,phase=core.phase.value,reason=core.layout.reason)
        if v == 0: holds[core.layout.reason] += 1
        sim.command(v,v*math.tan(steer)/cfg['vehicle']['wheelbase'])
        sim.step(.01)
        if i%10==0: trace.append([now,sim.x,sim.y,sim.yaw,core.phase.value])
        if prev != core.phase:
            phases.append([now,core.phase.value,core.planner.reason]);prev = core.phase
        if core.phase==Phase.PARKED and parked is None:
            corners=sim.body().corners();gx,gy=xs[(target-1)%3],ys[(target-1)//3]
            parked=dict(inside=bool(((abs(corners[:,0]-gx)<.25)&(abs(corners[:,1]-gy)<.30)).all()),pose=[sim.x,sim.y,sim.yaw])
        if scene:
            detail.setText('Current parking code / CAD anchor supplied\n'
                           f'Bay 2 | {core.phase.value} | {core.layout.reason}')
            if i%3==0: scene.render_frame()
            deadline += .01
            time.sleep(max(0.,deadline-time.perf_counter()))
        if sim.collisions or core.phase in (Phase.DONE,Phase.ABORT): break
    if scene:
        scene.paused = True
        while not scene.closed:
            scene.render_frame()
            time.sleep(.02)
        scene.close()
    b=core.planner.bay
    return dict(case=case,mode=mode,target=target,phase=core.phase.value,parked=parked,collisions=sim.collisions,
                goal=None if b is None else [b.x+spawn[0],b.y+spawn[1],b.row_yaw],
                restart=restart[0],events=phases,holds=dict(holds),states=core.layout.states,trace=trace,first_hold=first_hold,
                command_ms=dict(mean=float(np.mean(command_timings)) if command_timings else 0.,p95=float(np.percentile(command_timings,95)) if command_timings else 0.),
                scan_ms=dict(mean=float(np.mean(timings)) if timings else 0.,p95=float(np.percentile(timings,95)) if timings else 0.),
                obstacles=world['obstacles'],scope='actual simulator physics; configured lidar noise only in sensor_noise; rear sector unmasked only in rear_visible; ideal odom/imu; supplied CAD registration; no camera/MUX validation')

if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--case',action='append',choices=list(CASES)+['occupied','lane_blocked','no_returns','sensor_noise','rear_visible']);ap.add_argument('--mode',choices=('legacy','registered','both'),default='both')
    ap.add_argument('--show',action='store_true')
    a=ap.parse_args();rows=[]
    if a.show:
        while True:
            row=run((a.case or ['opposite_15'])[0], 'registered' if a.mode=='both' else a.mode, show=True)
            rows.append(row)
            a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(rows,indent=2))
            if not row['restart']: break
        sys.exit(0)
    for case in a.case or list(CASES)+['occupied','lane_blocked','no_returns','sensor_noise','rear_visible']:
        for mode in ('legacy','registered') if a.mode=='both' else (a.mode,):
            r=run(case,mode);rows.append(r)
            print(json.dumps({k:v for k,v in r.items() if k not in ('trace','obstacles')}),flush=True)
            a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(rows,indent=2))
