"""Exercise the compiled driver against a pseudo-UART, never the vehicle port.

Run after building limo_base on the Jetson. All ROS command/readiness topics are
remapped, with a separate localhost DDS domain. No camera or YOLO model is run.
"""
import os
from pathlib import Path
import select
import signal
import struct
import subprocess
import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool


@pytest.mark.parametrize('required,pre_ready', [(True, False), (False, False), (True, True)])
def test_actual_serial_commands_wait_for_readiness(tmp_path, required, pre_ready):
    root = Path(__file__).resolve().parents[4]
    binary = root/'ros2_ws/install/limo_base/lib/limo_base/limo_base'
    if not binary.is_file():
        pytest.skip('limo_base is built only where the vendor limo_msgs is installed')
    # A dedicated context and remappings prevent reaching any live robot node.
    rclpy.init(domain_id=187)
    node = Node('uart_readiness_probe')
    commands = node.create_publisher(Twist, '/startup_test/cmd', 1)
    readiness = node.create_publisher(Bool, '/startup_test/ready',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    if pre_ready:
        readiness.publish(Bool(data=True))
    master, slave = os.openpty()
    slave_path = os.ttyname(slave)
    assert slave_path.startswith('/dev/pts/')
    os.set_blocking(master, False)
    log = (tmp_path/'driver.log').open('w')
    args = [str(binary), '--ros-args', '-p', 'port_name:='+slave_path,
            '-p', 'require_yolo_ready:='+str(required).lower(), '-p', 'cmd_timeout:=0.3',
            '-r', '/cmd_vel:=/startup_test/cmd', '-r', '/yolo/ready:=/startup_test/ready']
    env = dict(os.environ, ROS_DOMAIN_ID='187', ROS_LOCALHOST_ONLY='1')
    proc = subprocess.Popen(['bash', '-c', 'ulimit -c 0\nexec "$@"', 'driver_test', *args],
                            env=env, stdout=log, stderr=subprocess.STDOUT)
    buffer = bytearray()
    # Body feedback selects Ackermann mode, matching the actual drive configuration.
    payload = bytes([0, 1, 0, 124, 0, 0, 1, 0])
    body = bytes([0x55, 0x0e, 0x02, 0x11])+payload+bytes([0, sum(payload) & 0xff])

    def window(seconds):
        sent = []
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            assert proc.poll() is None, (tmp_path/'driver.log').read_text()
            os.write(master, body)
            msg = Twist()
            msg.linear.x, msg.angular.z, msg.linear.y = .8, .2, .3
            commands.publish(msg)
            rclpy.spin_once(node, timeout_sec=.01)
            if select.select([master], [], [], 0)[0]:
                buffer.extend(os.read(master, 65536))
            while len(buffer) >= 14:
                if buffer[:2] != b'\x55\x0e':
                    del buffer[0]
                    continue
                frame = bytes(buffer[:14])
                del buffer[:14]
                assert frame[13] == sum(frame[4:12]) & 0xff
                if frame[2:4] == b'\x01\x11':
                    sent.append(struct.unpack('>hhhh', frame[4:12]))
        return sent

    try:
        deadline = time.monotonic()+8.
        while commands.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        assert commands.get_subscription_count() > 0
        if required:
            until = time.monotonic()+3.
            while readiness.get_subscription_count() == 0 and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.02)
            assert readiness.get_subscription_count() > 0
        before = window(.6)
        assert before
        if required and not pre_ready:
            assert all(v == (0, 0, 0, 0) for v in before)
            readiness.publish(Bool(data=False))
            assert all(v == (0, 0, 0, 0) for v in window(.3))
            readiness.publish(Bool(data=True))
            after = window(.7)
            assert any(v[0] == 800 and v[3] != 0 for v in after)
            readiness.publish(Bool(data=False))
            window(.2)  # allow the subscriber callback to run
            blocked = window(.3)
            assert blocked and all(v == (0, 0, 0, 0) for v in blocked)
        else:
            after = window(.5)
            assert any(v[0] == 800 and v[3] != 0 for v in after)
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=3.)
        log.close()
        os.close(master)
        os.close(slave)
        node.destroy_node()
        rclpy.shutdown()
