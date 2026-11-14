"""Readiness comes from a completed image, including an empty result; no model inference."""
import time
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Header

from clothoid_common.image_utils import bgr_to_image
from clothoid_yolo.yolo_node import YoloNode


@pytest.fixture
def running():
    rclpy.init(args=['--ros-args', '-p', 'backend:=none'])
    yolo = YoloNode()
    probe = Node('readiness_test')
    executor = SingleThreadedExecutor()
    executor.add_node(yolo)
    executor.add_node(probe)
    received = []
    probe.create_subscription(Bool, '/yolo/ready', lambda m: received.append(m.data),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    yield yolo, probe, executor, received
    executor.shutdown()
    probe.destroy_node()
    yolo.destroy_node()
    rclpy.shutdown()


def until(executor, condition):
    deadline = time.monotonic()+3.
    while not condition() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=.01)
    assert condition()


def frame():
    return bgr_to_image(np.zeros((10, 12, 3), dtype=np.uint8), Header())


def test_model_disabled_or_image_invalid_does_not_release(running):
    yolo, _, ex, received = running
    until(ex, lambda: received == [False])
    yolo.on_image(frame())       # backend=none publishes an array, but did not infer
    assert not yolo.ready
    yolo.backend = SimpleNamespace(infer=lambda bgr: [])
    yolo.on_image(Image(encoding='invalid'))
    assert not yolo.ready
    ex.spin_once(timeout_sec=.05)
    assert received == [False]


def test_failed_then_empty_success_and_late_subscriber(running):
    yolo, probe, ex, received = running
    until(ex, lambda: received == [False])

    def fail(bgr):
        raise RuntimeError('model failed before its first successful image')

    yolo.backend = SimpleNamespace(infer=fail)
    with pytest.raises(RuntimeError):
        yolo.on_image(frame())
    assert not yolo.ready
    yolo.backend = SimpleNamespace(infer=lambda bgr: [])
    yolo.on_image(frame())
    until(ex, lambda: received == [False, True])
    late = []
    probe.create_subscription(Bool, '/yolo/ready', lambda m: late.append(m.data),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    until(ex, lambda: late == [True])
    yolo.on_image(frame())
    ex.spin_once(timeout_sec=.1)
    assert received == [False, True]  # one transition, no per-frame readiness traffic


def test_10hz_source_processes_every_frame(running):
    yolo, _, ex, received = running
    assert yolo.image_source == '10hz'
    assert yolo.image_topic == '/camera/color/image_10hz'
    until(ex, lambda: received == [False])
    calls = 0

    def count_infer(bgr):
        nonlocal calls
        calls += 1
        return []

    yolo.backend = SimpleNamespace(infer=count_infer)
    for _ in range(5):
        yolo.on_image(frame())
    assert calls == 5


def test_raw_image_every_skips_frames_after_ready(running):
    yolo, _, ex, received = running
    yolo.image_source = 'raw'
    until(ex, lambda: received == [False])
    calls = 0

    def count_infer(bgr):
        nonlocal calls
        calls += 1
        return []

    yolo.backend = SimpleNamespace(infer=count_infer)
    yolo.on_image(frame())  # first frame runs and marks ready
    until(ex, lambda: received == [False, True])
    for _ in range(6):     # with default image_every=3, skips 4 of 6 subsequent frames
        yolo.on_image(frame())
    assert calls == 3
