"""QoS for raw camera subscriptions.

`qos_profile_sensor_data` keeps 5 frames. When a node falls behind it then works through up to 5
old frames (~130 ms at 30 Hz) before the newest one; with depth 1 it always takes the newest.
Only for images. IMU / odom: a node that integrates velocity or rate needs every sample and keeps
`qos_profile_sensor_data`; a node that uses the accumulated pose or an absolute heading can take depth 1,
a dropped message loses nothing (static_avoid counts distance as the difference of /odom positions).
"""
from rclpy.qos import HistoryPolicy, QoSProfile, qos_profile_sensor_data

IMAGE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=qos_profile_sensor_data.reliability,
    durability=qos_profile_sensor_data.durability,
)
