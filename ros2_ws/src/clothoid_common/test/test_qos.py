from rclpy.qos import qos_profile_sensor_data

from clothoid_common.qos import IMAGE_QOS


def test_image_qos_is_sensor_data_with_depth_one():
    assert IMAGE_QOS.depth == 1
    assert IMAGE_QOS.reliability == qos_profile_sensor_data.reliability
    assert IMAGE_QOS.durability == qos_profile_sensor_data.durability
