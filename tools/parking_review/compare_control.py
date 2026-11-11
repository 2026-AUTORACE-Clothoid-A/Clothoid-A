#!/usr/bin/env python3
"""Compare legacy and observe-mode commands on recorded imu/odom/scan/parking flags.
Uses actual recorded flag events; neither branch sends commands to ROS.
"""
import argparse
import json
from pathlib import Path
import sqlite3
import sys
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'ros2_ws/src/clothoid_parking'))
from clothoid_parking.core import ParkingCore
from clothoid_parking.layout import LayoutParams


def run(row):
    old, new = ParkingCore(), ParkingCore(layout=LayoutParams(control_enabled=False))
    new.prepare(True)
    kinds = {'/imu':Imu, '/odom':Odometry, '/scan':LaserScan, '/parking_flag':Bool}
    samples = differences = 0
    last = -float('inf')
    phases = set()
    for db in row['dbs']:
        con = sqlite3.connect('file:'+str(db)+'?mode=ro', uri=True)
        topics = {i:(n,kinds[n]) for i,n in con.execute('select id,name from topics') if n in kinds}
        query = 'select topic_id,timestamp,data from messages where topic_id in ('+','.join('?' for _ in topics)+') order by timestamp'
        for tid, ts, blob in con.execute(query,list(topics)):
            name,kind = topics[tid]
            m = deserialize_message(blob,kind)
            stamp = ts*1e-9 if name == '/parking_flag' else m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            for core in (old,new):
                if name == '/imu': core.imu(stamp,m.angular_velocity.z)
                elif name == '/odom': core.odom(stamp,m.twist.twist.linear.x)
                elif name == '/scan': core.scan(stamp,m.ranges,m.angle_min,m.angle_increment,m.range_min,m.range_max)
                else: core.set_flag(stamp,m.data)
            now = ts*1e-9
            if now-last >= .05:
                a,b = old.command(now),new.command(now)
                differences += int(a != b or old.phase != new.phase)
                samples += 1
                phases.add(old.phase.value)
                last = now
        con.close()
    result = dict(name=row['name'],samples=samples,differences=differences,phases=sorted(phases))
    print(json.dumps(result),flush=True)
    return result


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('--inventory',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();results=[run(row) for row in json.loads(a.inventory.read_text())]
    a.output.write_text(json.dumps(results,indent=2))
    if any(row['differences'] for row in results): raise SystemExit(1)
