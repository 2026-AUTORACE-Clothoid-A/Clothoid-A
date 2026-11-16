#!/usr/bin/env python3
"""Read-only Humble bag review; no ROS nodes or drive publishers are created."""
import argparse
import bisect
import json
import math
from pathlib import Path
import sys
import subprocess
import cv2
import numpy as np
import yaml
cv2.setNumThreads(1)

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools/offline_pipeline'))
from run_pipeline import build
from bagio import read, stamp
from clothoid_lidar.obstacle_extractor import extract_obstacles
from clothoid_parking.preview import ParkingPreview
from clothoid_parking.planner import PlanParams
from clothoid_parking.bay_finder import BayParams

CAM = '/camera/color/image_raw/compressed'
FIELDS = ['/mission/state','/parking/state','/parking_prepare','/parking_flag','/parking_done',
          '/parking/layout_status','/stopline_detected','/stopline_near','/speed_limit',
          '/bump/approaching','/steering_lock','/finish/state','/yolo/ready']


def matrix(t):
    q=t.rotation; x,y,z,w=q.x,q.y,q.z,q.w
    out=np.eye(4)
    out[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    out[:3,3]=[t.translation.x,t.translation.y,t.translation.z]
    return out


def resolve(edges, source, target='base_link'):
    todo=[(source,np.eye(4))]; seen=set()
    while todo:
        node, mat=todo.pop()
        if node==target: return mat
        if node in seen: continue
        seen.add(node)
        for (a,b),transform in edges.items():
            if a==node: todo.append((b,transform@mat))
    return None


def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def run(bag, output, hz):
    meta=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    t0=meta['starting_time']['nanoseconds_since_epoch']*1e-9
    duration=meta['duration']['nanoseconds']*1e-9
    available={t['topic_metadata']['name']:t['message_count'] for t in meta['topics_with_message_count']}
    folder=output/bag.name; folder.mkdir(parents=True,exist_ok=True)
    (folder/'frames').mkdir(exist_ok=True)
    model=build(); edges={}; detections=[]; events=[]; last_event={}; first={}
    for topic,t,msg in read(str(bag),['/detections','/tf_static']):
        if topic=='/tf_static':
            for tf in msg.transforms:
                a,b=tf.child_frame_id.lstrip('/'),tf.header.frame_id.lstrip('/')
                mat=matrix(tf.transform); edges[a,b]=mat; edges[b,a]=np.linalg.inv(mat)
        else:
            boxes=[[d.class_name,round(float(d.confidence),3),d.x_min,d.y_min,d.x_max,d.y_max] for d in msg.detections]
            detections.append((stamp(msg) or t,t,boxes))
            for cls in sorted({d[0] for d in boxes}):
                if cls not in first:
                    first[cls]=round(t-t0,3); events.append({'t':round(t-t0,3),'label':cls+' 최초 검출','kind':'detection'})
    detections.sort(key=lambda d:d[0]); det_times=[d[0] for d in detections]
    py=yaml.safe_load((ROOT/'ros2_ws/src/clothoid_parking/config/parking.yaml').read_text())['parking']['ros__parameters']
    plan=PlanParams(**{k[5:]:v for k,v in py.items() if k.startswith('plan.') and k[5:] in PlanParams.__dataclass_fields__})
    bayp=BayParams(**{k[4:]:v for k,v in py.items() if k.startswith('bay.') and k[4:] in BayParams.__dataclass_fields__})
    preview=ParkingPreview(plan,bayp,py.get('laser_x',.103))
    state={}; times={}; frames=[]; scan=[]; cones=[]; left=[]; right=[]; center=[]; bays=[]
    scan_time=-math.inf; layout_time=-math.inf; last_frame=-math.inf; last_step=-math.inf
    lane_total=lane_found=0; cone_running=False; tf_ok=False; errors=[]
    topics=FIELDS+[CAM,'/scan','/odom','/imu','/parking/layout','/cmd_vel','/cmd_vel_lane','/cmd_vel_parking']
    for topic,t,msg in read(str(bag),topics):
        rt=t-t0; st=stamp(msg) if hasattr(msg,'header') else t
        st=st or t
        if topic in FIELDS:
            value=msg.data; state[topic]=value; times[topic]=round(rt,3)
            if topic=='/mission/state':
                try:
                    mission=json.loads(value)
                    value=json.dumps({k:mission.get(k) for k in ('state','armed','seen','parking_prepare','parking_scene','bump_state')},ensure_ascii=False)
                except (ValueError,TypeError): pass
            if topic not in ('/parking/layout_status','/speed_limit') and last_event.get(topic)!=value:
                events.append({'t':round(rt,3),'label':topic+' → '+str(value),'kind':'state'})
                last_event[topic]=value
            if topic=='/parking_flag': preview.core.set_flag(t,value)
        elif topic=='/imu': preview.core.imu(st,msg.angular_velocity.z)
        elif topic=='/odom':
            preview.core.odom(st,msg.twist.twist.linear.x)
            state['odom_speed']=round(msg.twist.twist.linear.x,3)
        elif topic.startswith('/cmd_vel'):
            state[topic]=[round(msg.linear.x,3),round(msg.angular.z,3)]; times[topic]=round(rt,3)
        elif topic=='/parking/layout':
            bays=[[p.position.x,p.position.y,yaw(p.orientation)] for p in msg.poses]
            layout_time=rt
        elif topic=='/scan':
            preview.core.scan(st,msg.ranges,msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max)
            ranges=np.asarray(msg.ranges); angles=msg.angle_min+np.arange(len(ranges))*msg.angle_increment
            valid=np.isfinite(ranges)&(ranges>=msg.range_min)&(ranges<=min(msg.range_max,4))
            pts=np.column_stack((ranges[valid]*np.cos(angles[valid]),ranges[valid]*np.sin(angles[valid])))
            trans=resolve(edges,msg.header.frame_id.lstrip('/')); tf_ok=trans is not None
            if tf_ok:
                pts=pts@trans[:2,:2].T+trans[:2,3]
                cc=extract_obstacles(msg.ranges,msg.angle_min,msg.angle_increment,model['ep'])
                cc=cc@trans[:2,:2].T+trans[:2,3]
                cones=cc.round(3).tolist()
                sorted_cones=model['mission'].update(cc)
                left=[]; right=[]; center=[]
                if sorted_cones is not None:
                    left=sorted_cones.left.round(3).tolist(); right=sorted_cones.right.round(3).tolist()
                    follow=model['follower'].decide(sorted_cones.left,sorted_cones.right,bool(left),bool(right))
                    if follow is not None: center=follow.center.round(3).tolist()
                running=model['mission'].running
                if running!=cone_running:
                    events.append({'t':round(rt,3),'label':'콘 재분석 '+('시작' if running else '종료'),'kind':'replay'})
                    cone_running=running
                scan=pts.round(3).tolist()
            else: scan=[]; cones=[]; left=[]; right=[]; center=[]
            scan_time=rt
        elif topic==CAM:
            im=cv2.imdecode(np.frombuffer(msg.data,np.uint8),1)
            if im is None:
                errors.append(round(rt,3)); continue
            render=rt-last_frame>=1/hz
            lr=model['lane'].detect(im,model['side'],model['lane_width'],render)
            lane_total+=1; lane_found+=int(lr.found)
            if not render: continue
            last_frame=rt
            idx=bisect.bisect_right(det_times,st)-1
            boxes=[]; det_age=None
            if idx>=0:
                det_age=st-detections[idx][0]
                if det_age<=.2: boxes=detections[idx][2]
            for name,confidence,x1,y1,x2,y2 in boxes:
                color=(100,235,80) if name=='sign_parking' else (40,195,255)
                cv2.rectangle(im,(x1,y1),(x2,y2),color,2)
                cv2.putText(im,f'{name} {confidence:.2f}',(max(0,x1),max(18,y1-5)),cv2.FONT_HERSHEY_SIMPLEX,.45,color,1,cv2.LINE_AA)
            bev=lr.debug_image
            if bev is None: bev=np.zeros((240,320,3),np.uint8)
            if bev.ndim==2: bev=cv2.cvtColor(bev,cv2.COLOR_GRAY2BGR)
            merged=np.hstack((cv2.resize(im,(640,480)),cv2.resize(bev,(480,480))))
            filename=f'frames/{len(frames):05d}.jpg'
            if not cv2.imwrite(str(folder/filename),merged,[cv2.IMWRITE_JPEG_QUALITY,76]): raise RuntimeError(filename)
            pose=preview.core.tracker.pose; c,s=math.cos(pose.yaw),math.sin(pose.yaw)
            path=[[(x-pose.x)*c+(y-pose.y)*s,-(x-pose.x)*s+(y-pose.y)*c] for x,y in preview.path]
            frames.append(dict(t=round(rt,3),image=filename,lane=bool(lr.found),reference=lr.reference,
                boxes=boxes,det_age=None if det_age is None else round(det_age,3),state=dict(state),ages={k:round(rt-v,3) for k,v in times.items()},
                scan=scan if rt-scan_time<.3 else [],cones=cones if rt-scan_time<.3 else [],
                left=left if rt-scan_time<.3 else [],right=right if rt-scan_time<.3 else [],center=center if rt-scan_time<.3 else [],
                cone_running=cone_running,tf_ok=tf_ok,bays=bays if rt-layout_time<.75 else [],
                path=np.round(path,3).tolist(),preview=preview.core.phase.value))
        if t-last_step>=.05:
            preview.step(t); last_step=t
    events.sort(key=lambda x:x['t'])
    result=dict(name=bag.name,duration=duration,frames=frames,events=events,
                summary=dict(first_detection=first,lane_frames=lane_total,lane_found=lane_found,decode_errors=errors),topics=available)
    (folder/'data.json').write_text(json.dumps(result,ensure_ascii=False,allow_nan=False))
    print(json.dumps({'bag':bag.name,'frames':len(frames),**result['summary']},ensure_ascii=False),flush=True)
    return {k:result[k] for k in ('name','duration','summary')}


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('bags',nargs='+',type=Path); ap.add_argument('--output',type=Path,required=True); ap.add_argument('--hz',type=float,default=10)
    args=ap.parse_args()
    if not 0<args.hz<=30: ap.error('--hz must be in (0,30]')
    args.output.mkdir(parents=True,exist_ok=True)
    results=[run(b,args.output,args.hz) for b in args.bags]
    provenance={'commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
                'dirty':subprocess.check_output(['git','status','--short'],cwd=ROOT,text=True),'hz':args.hz,'bags':results}
    (args.output/'manifest.json').write_text(json.dumps(provenance,ensure_ascii=False,indent=2))
    (args.output/'index.html').write_text((Path(__file__).parent/'index.html').read_text())

if __name__=='__main__': main()
