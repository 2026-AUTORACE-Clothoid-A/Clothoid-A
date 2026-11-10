"""QoS for raw camera subscriptions.

`qos_profile_sensor_data` keeps 5 frames. When a node falls behind it then works through up to 5
old frames (~130 ms at 30 Hz) before the newest one; with depth 1 it always takes the newest.
Only for images: IMU / odom keep `qos_profile_sensor_data`, their integrators want every sample.
"""
from rclpy.qos import HistoryPolicy, QoSProfile, qos_profile_sensor_data

IMAGE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=qos_profile_sensor_data.reliability,
    durability=qos_profile_sensor_data.durability,
)
