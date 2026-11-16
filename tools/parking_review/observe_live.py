#!/usr/bin/env python3
"""Read-only trace of actual ROS commands, parking decisions and simulator motion."""
import argparse
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
from clothoid_parking.layout import LayoutParams, ParkingLayout
from clothoid_parking.tracker import Pose2D


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=100.)
    parser.add_argument('--until-finish', action='store_true')
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('parking_live_observer')
    sim = [None]
    phase = ['']
    deadline = [time.monotonic()+args.seconds]
    node.create_subscription(Clock, '/clock', lambda m: sim.__setitem__(0, m.clock.sec+m.clock.nanosec*1e-9), 1)
    with args.output.open('w') as out:
        def record(topic, value):
            out.write(json.dumps(dict(wall=time.time(), sim=sim[0], topic=topic, value=value))+'\n')
            if topic == '/parking/state':
                phase[0] = value.split(':', 1)[0]
                print(json.dumps(dict(sim=sim[0], state=value)), flush=True)
                if value.startswith('abort:') or (value.startswith('done:') and not args.until_finish):
                    deadline[0] = min(deadline[0], time.monotonic()+2.)
            if topic == '/finish/state':
                print(json.dumps(dict(sim=sim[0], finish=value)), flush=True)
                if value.startswith(('stop:', 'abort:')):
                    deadline[0] = min(deadline[0], time.monotonic()+2.)
        # Counterfactual checks only: never publish these results as a command.
        shadows = [(name, ParkingLayout(LayoutParams(position_tolerance=0., clearance=margin)))
                   for name,margin in (('current_10cm', .10), ('clearance_3cm', .03), ('physical_body', 0.))]
        def observation(msg):
            if phase[0] not in ('turn', 'enter') or sim[0] is None:
                return
            markers = {m.ns:m for m in msg.markers}
            if 'observed_free' not in markers or 'observed_occupied' not in markers:
                return
            ref = markers['observed_free'].pose
            yaw = -2*math.atan2(ref.orientation.z, ref.orientation.w)
            c,s = math.cos(yaw),math.sin(yaw)
            pose = Pose2D(-c*ref.position.x+s*ref.position.y,
                          -s*ref.position.x-c*ref.position.y, yaw)
            cells = {}
            for ns,hit in (('observed_free',False),('observed_occupied',True)):
                for p in markers[ns].points:
                    cells[(math.floor(p.x/.04),math.floor(p.y/.04))]=(sim[0],hit)
            speed,steer,distance = .2,-.41,.20
            if phase[0] == 'enter':
                arrow = markers.get('grid_reference')
                if arrow is None or len(arrow.points) != 2:
                    return
                a,b = arrow.points
                row = math.atan2(b.y-a.y,b.x-a.x)
                entry = row-math.pi/2
                # Designated front middle bay, from this observed grid reference.
                tx = a.x+.78*math.cos(row)+.31*math.cos(entry)
                ty = a.y+.78*math.sin(row)+.31*math.sin(entry)
                depth = tx*math.cos(entry)+ty*math.sin(entry)
                speed = math.copysign(max(.06,min(.1,1.5*abs(depth))),depth)
                steer = max(-.41,min(.41,2*math.atan2(math.sin(entry),math.cos(entry))))
                if speed < 0:
                    steer = -steer
                distance = min(.20,abs(depth))
            result = {'phase':phase[0], 'requested':[speed,steer,distance]}
            for name,layout in shadows:
                layout.cells = cells
                clear = layout.motion_clear(sim[0],pose,speed,steer,.2,distance)
                result[name] = dict(clear=clear,reason=layout.motion_reason)
            if 'observation_status' in markers:
                result['actual_status'] = markers['observation_status'].text
            record('/parking/shadow_clearance', result)
        node.create_subscription(MarkerArray, '/parking/observation_markers', observation, 1)
        for topic in ('/parking/state','/parking/layout_status','/parking/approach_status','/mission/state','/finish/state','/sim/parking_status'):
            node.create_subscription(String, topic, lambda m,t=topic: record(t,m.data), 1)
        for topic in ('/parking_flag','/parking_done','/finish_done','/yolo/ready'):
            node.create_subscription(Bool, topic, lambda m,t=topic: record(t,m.data), 1)
        for topic in ('/cmd_vel_parking','/cmd_vel','/sim/cmd_vel_mux','/cmd_vel_lane','/cmd_vel_state','/cmd_vel_avoid','/cmd_vel_finish'):
            node.create_subscription(Twist, topic, lambda m,t=topic: record(t,[m.linear.x,m.angular.z]), 1)
        node.create_subscription(Odometry, '/sim/ground_truth',
                                 lambda m: record('/sim/ground_truth', [m.pose.pose.position.x,m.pose.pose.position.y,
                                     m.pose.pose.orientation.z,m.pose.pose.orientation.w]), qos_profile_sensor_data)
        node.create_subscription(MarkerArray, '/parking/markers',
                                 lambda m: record('/parking/markers', [dict(ns=a.ns,text=a.text,
                                     points=[[p.x,p.y] for p in a.points]) for a in m.markers if a.ns in ('target','phase')]), 1)
        while rclpy.ok() and time.monotonic()<deadline[0]:
            rclpy.spin_once(node, timeout_sec=.1)
    node.destroy_node();rclpy.shutdown()


if __name__ == '__main__':
    main()
