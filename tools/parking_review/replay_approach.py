#!/usr/bin/env python3
"""Wire actual mission/lane/parking callbacks on recorded inputs without ROS commands.

Recorded motion never responds to the new commands: this is an integration/evidence
replay, not a claim of closed-loop parking success. No recorded mission flags are fed.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import sqlite3
from types import SimpleNamespace
import time
import cv2
import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.serialization import deserialize_message
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, CompressedImage, Imu, LaserScan
from nav_msgs.msg import Odometry
from clothoid_interfaces.msg import DetectionArray
from clothoid_lane.lane_main import LaneMainNode
from clothoid_mission.mission_node import MissionNode
from clothoid_parking.parking_node import ParkingNode
ROOT=Path(__file__).resolve().parents[2]


def replay(bag, mode):
    configs=['clothoid_lane/config/lane.yaml','clothoid_lane/config/camera_limo2.yaml',
             'clothoid_mission/config/mission.yaml','clothoid_parking/config/parking.yaml']
    args=['--ros-args']
    for path in configs:args+=['--params-file',str(ROOT/'ros2_ws/src'/path)]
    rclpy.init(args=args)
    lane,mission,parking=LaneMainNode(),MissionNode(),ParkingNode()
    mission.set_parameters([Parameter('scene_sign_candidates',value=mode=='new')])
    parking.approach.p.enabled=mode=='new'
    clock=[0.]
    fake=SimpleNamespace(now=lambda:Time(nanoseconds=int(clock[0]*1e9)))
    for n in (lane,mission,parking):n.get_clock=lambda:fake
    mission.now_sec=parking.now=lambda:clock[0]
    pub=lambda f:SimpleNamespace(publish=f)
    for n in (lane,mission,parking):
        for name in list(vars(n)):
            if name.endswith('_pub') and getattr(n,name) is not None:
                setattr(n,name,pub(lambda msg:None))
    mission.prepare_pub=pub(lambda msg:(parking.on_prepare(msg),lane.parking_landmark.on_prepare(msg)))
    mission.parking_pub=pub(lambda msg:(parking.on_flag(msg),lane.on_parking(msg),lane.parking_landmark.on_flag(msg)))
    lane.cmd_pub=pub(parking.on_lane_command)
    mission.limit_pub=pub(lane.on_speed_limit)
    mission.entry_distance_pub=pub(parking.on_entry_distance)
    mission.bump_approach_pub=pub(lane.on_bump_approach)
    parking.candidate_pub=pub(mission.on_parking_scene)
    parking.done_pub=pub(mission.on_parking_done)
    parking.observing_pub=pub(mission.on_parking_observing)
    parking.skipped_pub=pub(mission.on_parking_skipped)
    caps=[]
    def cap(msg):
        lane.on_parking_approach(msg)
        if msg.data>=0:caps.append([clock[0]-t0,msg.data,parking.approach.remaining,parking.approach.reason,lane.current_speed(),parking.core.tracker.speed])
    parking.approach_pub=pub(cap)
    lane.parking_landmark.pub=pub(parking.on_entry)
    kinds={'/detections':DetectionArray,'/camera/color/camera_info':CameraInfo,
           '/camera/color/image_raw/compressed':CompressedImage,'/imu':Imu,'/odom':Odometry,'/scan':LaserScan}
    events=[];prev=None;seen=Counter();raw=Counter();costs=Counter();samples=Counter();registrations=[]
    def tick(t):
        nonlocal prev
        clock[0]=t
        start=time.perf_counter();mission.on_timer();parking.on_timer()
        costs['timers']+=(time.perf_counter()-start)*1000;samples['timers']+=1
        state=(mission.logic.state,mission.logic.parking_armed,mission.logic.parking_flag,
               parking.core.phase.value,parking.approach.skipped)
        if state!=prev:
            events.append([round(t-t0,3),state,parking.core.layout.reason,parking.approach.reason]);prev=state
    try:
        t0=None;next_tick=None
        for db in sorted(bag.glob('*.db3')):
            c=sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True)
            topics={i:(n,kinds[n]) for i,n in c.execute('select id,name from topics') if n in kinds}
            if t0 is None:
                t0=c.execute('select min(timestamp) from messages').fetchone()[0]*1e-9;next_tick=t0
            sql='select topic_id,timestamp,data from messages where topic_id in ('+','.join('?' for _ in topics)+') order by timestamp'
            for tid,ns,b in c.execute(sql,list(topics)):
                t=ns*1e-9
                while next_tick<=t:
                    tick(next_tick);next_tick+=.05
                clock[0]=t;n,kind=topics[tid];msg=deserialize_message(b,kind)
                start=time.perf_counter()
                if n=='/detections':
                    raw.update(d.class_name for d in msg.detections);mission.on_detections(msg);seen.update(mission.seen)
                elif n.endswith('camera_info'):
                    mission.on_info(msg);lane.parking_landmark.on_info(msg)
                elif n=='/imu':
                    mission.on_imu(msg)
                    if parking.sensor_subs:parking.on_imu(msg)
                elif n=='/odom':
                    mission.on_odom(msg)
                    if parking.sensor_subs:parking.on_odom(msg)
                elif n=='/scan':
                    if parking.sensor_subs:parking.on_scan(msg)
                    l=parking.core.layout
                    if l.origin is not None:registrations.append([t-t0,l.support_count,l.selected,l.reason])
                else:
                    im=cv2.imdecode(np.frombuffer(msg.data,np.uint8),cv2.IMREAD_COLOR)
                    result=lane.process_bgr(im,msg.header)
                    lane.parking_landmark.process(im,msg.header,result,lane.detector.p)
                    mission.on_image(msg)
                costs[n]+=(time.perf_counter()-start)*1000;samples[n]+=1
            c.close()
        return dict(bag=bag.name,mode=mode,raw=dict(raw),confirmed_callbacks=dict(seen),events=events,
                    caps=caps,registrations=registrations,
                    mean_ms={k:costs[k]/samples[k] for k in costs},
                    scope='actual callbacks wired, recorded motion unchanged; not driving success')
    finally:
        for n in (parking,mission,lane):n.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('bag',type=Path)
    ap.add_argument('--mode',choices=('old','new'),required=True);ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args();cv2.setNumThreads(1);r=replay(a.bag,a.mode)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(r,indent=2))
    print(json.dumps({k:v for k,v in r.items() if k not in ('caps','registrations')}),flush=True)
