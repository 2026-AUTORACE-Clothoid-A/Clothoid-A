"""Subscribes to the topics a node listens to and does nothing: the rclpy receive + executor cost."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image, LaserScan
class NullNode(Node):
    def __init__(self):
        super().__init__('null')
        self.declare_parameter('subs', ['imu', 'odom'])
        types = {'imu': (Imu, '/imu', 50), 'odom': (Odometry, '/odom', 20),
                 'image': (Image, '/camera/color/image_raw', qos_profile_sensor_data),
                 'scan': (LaserScan, '/scan', qos_profile_sensor_data)}
        for s in self.get_parameter('subs').value:
            t, topic, q = types[s]
            self.create_subscription(t, topic, getattr(self, 'on_' + s), q)
    def on_imu(self, m): pass
    def on_odom(self, m): pass
    def on_image(self, m): pass
    def on_scan(self, m): pass
