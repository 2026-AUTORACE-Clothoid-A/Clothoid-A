"""QoS for raw camera subscriptions.

`qos_profile_sensor_data` keeps 5 frames. When a node falls behind it then works through up to 5
old frames (~130 ms at 30 Hz) before the newest one; with depth 1 it always takes the newest.
Only for images. IMU / odom keep `qos_profile_sensor_data` (depth 5), never depth 1: in a node that
also takes images, odom arrives in bursts while an image callback runs, and depth 1 keeps only the
last one (measured: 160~180 ms stamp gaps at depth 1, 23~32 ms at depth 5/10; depth 5 still let one
80 ms gap through at start-up, so static_avoid uses 10). A deeper queue adds no delay as long as the
odom / imu callbacks only store the value and control runs from a timer; keep it that way.
"""
from rclpy.qos import HistoryPolicy, QoSProfile, qos_profile_sensor_data

IMAGE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=qos_profile_sensor_data.reliability,
    durability=qos_profile_sensor_data.durability,
)
