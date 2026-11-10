"""Run lane and parking-line detection with one image subscription and conversion."""
import rclpy
import cv2
from rclpy.executors import SingleThreadedExecutor

from clothoid_lane.lane_main import LaneMainNode
from clothoid_lane.stopline_node import StopLineNode


def main(args=None):
    rclpy.init(args=args)
    lane = LaneMainNode()
    stopline = StopLineNode(subscribe_images=False)
    threads = lane.declare_parameter('shared_opencv_threads', 1).value
    if threads < 0:
        raise ValueError('shared_opencv_threads must be >= 0 (0 keeps OpenCV defaults)')
    if threads:
        cv2.setNumThreads(threads)
    if lane.get_parameter('image_topic').value != stopline.get_parameter('image_topic').value:
        raise ValueError('Shared camera requires matching lane and stopline image topics')
    lane.image_observers.append(stopline.on_bgr)
    executor = SingleThreadedExecutor()
    executor.add_node(lane)
    executor.add_node(stopline)
    lane.get_logger().info(
        f'camera shared with stopline: one subscription, unchanged detectors; '
        f'OpenCV threads={cv2.getNumThreads()}')
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        stopline.destroy_node()
        lane.destroy_node()
        rclpy.try_shutdown()
