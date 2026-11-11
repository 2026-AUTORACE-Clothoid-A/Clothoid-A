import os
from pathlib import Path
import shutil
import socket

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rosidl_runtime_py import message_to_yaml
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import BatteryState, CompressedImage, Image, LaserScan
import yaml
import cv2
import numpy as np

from .flask_app_factory import FLASK_IMPORT_ERROR, FlaskServerThread, create_app
from .graph_utils import build_graph_snapshot
from .image_utils import extract_jpeg_dimensions
from .monitor_state import MonitorState

PACKAGE_ROOT = Path(__file__).resolve().parent

DEFAULT_PARAM_TARGET_NODES = ['lane_main', 'clothoid_mission', 'limo_base', 'clothoid_finish', 'yolo']

_HIDDEN_PARAM_NAMES = frozenset({'use_sim_time', 'start_type_description_service'})

_EDITABLE_PARAM_TYPES = (
    ParameterType.PARAMETER_BOOL,
    ParameterType.PARAMETER_INTEGER,
    ParameterType.PARAMETER_DOUBLE,
    ParameterType.PARAMETER_STRING,
)


def _param_value_to_json(value):
    if value.type == ParameterType.PARAMETER_BOOL:
        return {'type': value.type, 'value': bool(value.bool_value)}
    if value.type == ParameterType.PARAMETER_INTEGER:
        return {'type': value.type, 'value': int(value.integer_value)}
    if value.type == ParameterType.PARAMETER_DOUBLE:
        return {'type': value.type, 'value': float(value.double_value)}
    if value.type == ParameterType.PARAMETER_STRING:
        return {'type': value.type, 'value': str(value.string_value)}
    return {'type': value.type, 'value': None}


def _json_to_param_value(type_id, raw_value):
    if type_id not in _EDITABLE_PARAM_TYPES:
        raise ValueError(f'parameter type {type_id} is not editable here')
    value = ParameterValue()
    value.type = type_id
    if type_id == ParameterType.PARAMETER_BOOL:
        value.bool_value = (
            raw_value if isinstance(raw_value, bool)
            else str(raw_value).strip().lower() in ('1', 'true', 'yes', 'on')
        )
    elif type_id == ParameterType.PARAMETER_INTEGER:
        value.integer_value = int(raw_value)
    elif type_id == ParameterType.PARAMETER_DOUBLE:
        value.double_value = float(raw_value)
    elif type_id == ParameterType.PARAMETER_STRING:
        value.string_value = str(raw_value)
    return value


def detect_lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        return s.getsockname()[0]
    except OSError:
        return '127.0.0.1'
    finally:
        s.close()


def resolve_resource_path(filename):
    candidates = []
    try:
        share_dir = Path(get_package_share_directory('clothoid_monitor'))
        candidates.append(share_dir / 'resource' / filename)
    except PackageNotFoundError:
        pass

    candidates.append(PACKAGE_ROOT.parent / 'resource' / filename)

    for candidate in candidates:
        if candidate.exists():
            return candidate

    return None


def render_lidar_bev(scan_msg: LaserScan, width: int = 400, height: int = 400, max_range: float = 3.0) -> bytes:
    """Render a 2D Bird's-Eye View (BEV) radar image from sensor_msgs/LaserScan."""
    img = np.full((height, width, 3), (28, 24, 20), dtype=np.uint8)
    cx = width // 2
    cy = int(height * 0.65)
    scale = (cy - 25) / max(0.5, float(max_range))

    # Grid rings
    for dist in (0.5, 1.0, 2.0, 3.0):
        if dist > max_range:
            continue
        r_px = int(dist * scale)
        if r_px < min(cx, cy):
            cv2.circle(img, (cx, cy), r_px, (55, 50, 42), 1, cv2.LINE_AA)
            cv2.putText(img, f'{dist:.1f}m', (cx + 4, cy - r_px + 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 115, 105), 1, cv2.LINE_AA)

    # Cross axes
    cv2.line(img, (cx, 0), (cx, height), (45, 40, 35), 1)
    cv2.line(img, (0, cy), (width, cy), (45, 40, 35), 1)

    # Points projection (ROS base_link: x forward, y left)
    ranges = np.asarray(scan_msg.ranges)
    angles = scan_msg.angle_min + np.arange(ranges.size) * scan_msg.angle_increment
    valid = np.isfinite(ranges) & (ranges > 0.05) & (ranges <= max_range + 0.5)
    r = ranges[valid]
    a = angles[valid]

    if r.size > 0:
        x = r * np.cos(a)
        y = r * np.sin(a)
        px = (cx - y * scale).astype(int)
        py = (cy - x * scale).astype(int)
        in_bounds = (px >= 0) & (px < width) & (py >= 0) & (py < height)
        px, py, r_val = px[in_bounds], py[in_bounds], r[in_bounds]

        for p_x, p_y, dist in zip(px, py, r_val):
            if dist < 0.6:
                color = (40, 40, 255)      # Red (danger)
            elif dist < 1.2:
                color = (0, 210, 255)     # Yellow (caution)
            else:
                color = (255, 230, 70)    # Cyan/Mint (clear)
            cv2.circle(img, (p_x, p_y), 2, color, -1)

    # Car marker (LIMO ~ 0.32m x 0.22m)
    car_w = max(10, int(0.22 * scale))
    car_l = max(14, int(0.32 * scale))
    cv2.rectangle(img, (cx - car_w // 2, cy - car_l // 2), (cx + car_w // 2, cy + car_l // 2),
                  (200, 140, 50), -1)
    arrow_pts = np.array([[cx - car_w // 2, cy - car_l // 2],
                          [cx + car_w // 2, cy - car_l // 2],
                          [cx, cy - car_l // 2 - 8]], dtype=np.int32)
    cv2.fillPoly(img, [arrow_pts], (0, 180, 255))

    # Header labels
    cv2.putText(img, f'LiDAR BEV (max {max_range}m)', (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 220, 220), 1, cv2.LINE_AA)
    cv2.putText(img, f'pts: {r.size}', (width - 75, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (150, 180, 200), 1, cv2.LINE_AA)

    success, encoded = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    return encoded.tobytes() if success else bytes()


class MonitorNode(Node):
    def __init__(self):
        super().__init__('clothoid_monitor_node')

        if FLASK_IMPORT_ERROR is not None:
            raise RuntimeError(
                'Flask is not installed. Install "python3-flask" and try again.'
            ) from FLASK_IMPORT_ERROR

        self.declare_parameter('battery_topic', '/battery_state')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('debug_image', True)
        self.declare_parameter('lane_topic', '/lane/debug_image')
        self.declare_parameter('yolo_topic', '/yolo/debug_image')
        self.declare_parameter('aruco_topic', '/parking/debug_image')
        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('lidar_max_range', 3.0)
        self.declare_parameter('control_topic', '/cmd_vel_lane')
        self.declare_parameter('storage_path', '/')
        self.declare_parameter('storage_poll_interval_sec', 1.0)
        self.declare_parameter('web_host', '0.0.0.0')
        self.declare_parameter('web_port', 5000)
        self.declare_parameter('page_title', 'Clothoid-A Monitor')
        self.declare_parameter('refresh_interval_ms', 500)
        self.declare_parameter('image_refresh_interval_ms', 100)
        self.declare_parameter('stale_timeout_sec', 3.0)
        self.declare_parameter('control_history_window_sec', 20.0)
        self.declare_parameter('image_source_width', 640)
        self.declare_parameter('image_source_height', 480)
        self.declare_parameter('image_display_width', 640)
        self.declare_parameter('image_display_height', 480)
        self.declare_parameter('debug_log', False)
        self.declare_parameter('param_target_nodes', DEFAULT_PARAM_TARGET_NODES)
        self.declare_parameter('auto_enable_debug', True)
        self.declare_parameter('auto_debug_nodes', ['lane_main', 'yolo'])

        self.battery_topic = str(self.get_parameter('battery_topic').value)
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.control_topic = str(self.get_parameter('control_topic').value)
        self.debug_image = bool(self.get_parameter('debug_image').value)
        self.lane_topic = str(self.get_parameter('lane_topic').value)
        self.yolo_topic = str(self.get_parameter('yolo_topic').value)
        self.aruco_topic = str(self.get_parameter('aruco_topic').value)
        self.lidar_topic = str(self.get_parameter('lidar_topic').value)
        self.lidar_max_range = float(self.get_parameter('lidar_max_range').value)

        self.storage_path = str(self.get_parameter('storage_path').value)
        self.storage_poll_interval_sec = float(self.get_parameter('storage_poll_interval_sec').value)
        self.web_host = str(self.get_parameter('web_host').value)
        self.web_port = int(self.get_parameter('web_port').value)
        self.page_title = str(self.get_parameter('page_title').value)
        self.refresh_interval_ms = int(self.get_parameter('refresh_interval_ms').value)
        self.image_refresh_interval_ms = int(self.get_parameter('image_refresh_interval_ms').value)
        self.debug_log = bool(self.get_parameter('debug_log').value)
        stale_timeout_sec = float(self.get_parameter('stale_timeout_sec').value)
        self.control_history_window_sec = float(self.get_parameter('control_history_window_sec').value)
        self.image_source_width = int(self.get_parameter('image_source_width').value)
        self.image_source_height = int(self.get_parameter('image_source_height').value)
        self.image_display_width = int(self.get_parameter('image_display_width').value)
        self.image_display_height = int(self.get_parameter('image_display_height').value)
        self.param_target_nodes = list(self.get_parameter('param_target_nodes').value)
        self.auto_enable_debug = bool(self.get_parameter('auto_enable_debug').value)
        self.auto_debug_nodes = list(self.get_parameter('auto_debug_nodes').value)
        self._auto_enabled_nodes = set()
        self._auto_debug_pending = set()
        self._auto_debug_timer = None

        self.header_logo_path = resolve_resource_path('Telechips-CI-White.png')
        self.telechips_logo_path = resolve_resource_path('Telechips-CI-White.png')
        self.topst_logo_path = resolve_resource_path('TOPST-Logo(White).png')
        self._echo_subscription = None
        self._echo_topic_name = None
        self._param_clients = {}

        self.state = MonitorState(
            stale_timeout_sec,
            self.image_source_width,
            self.image_source_height,
            self.control_history_window_sec,
        )

        self.app = create_app(
            self.state,
            self.page_title,
            self.battery_topic,
            self.image_topic,
            self.control_topic,
            self.storage_path,
            self.refresh_interval_ms,
            self.image_refresh_interval_ms,
            self.header_logo_path,
            self.telechips_logo_path,
            self.topst_logo_path,
            self.image_display_width,
            self.image_display_height,
            self.debug_image,
            self.lane_topic,
            self.yolo_topic,
            self.aruco_topic,
            self.control_history_window_sec,
            graph_snapshot_provider=self.get_graph_snapshot,
            topic_echo_setter=self.set_echo_topic,
            param_target_nodes=self.param_target_nodes,
            param_lister=self.list_node_params,
            param_setter=self.set_node_param,
            lidar_topic=self.lidar_topic,
        )
        self.server_thread = FlaskServerThread(self.app, self.web_host, self.web_port)

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # 1. Main camera stream: subscribe to both Image (raw) and CompressedImage
        self.create_subscription(
            Image,
            self.image_topic,
            self.image_raw_callback,
            sensor_qos,
        )
        if not self.image_topic.endswith('/compressed'):
            self.create_subscription(
                CompressedImage,
                f'{self.image_topic}/compressed',
                self.image_compressed_callback,
                sensor_qos,
            )

        # 2. Debug visual streams (Lane, YOLO, LiDAR BEV / Parking)
        if self.debug_image:
            self.create_subscription(Image, self.lane_topic, self.debug_lane_raw_callback, sensor_qos)
            self.create_subscription(Image, self.yolo_topic, self.debug_yolo_raw_callback, sensor_qos)
            # Subscribe to LaserScan for real-time BEV Radar view
            self.create_subscription(LaserScan, self.lidar_topic, self.scan_callback, sensor_qos)
            # Also subscribe to Image if aruco_topic is set to an image
            if self.aruco_topic:
                self.create_subscription(Image, self.aruco_topic, self.debug_aruco_raw_callback, sensor_qos)

        # 3. Control command (Twist)
        self.create_subscription(Twist, self.control_topic, self.control_callback, 10)

        # 4. Battery state (optional standard BatteryState)
        self.create_subscription(BatteryState, self.battery_topic, self.battery_callback, 10)

        self.storage_timer = self.create_timer(
            self.storage_poll_interval_sec,
            self.storage_timer_callback,
        )
        self.storage_timer_callback()

        self.server_thread.start()

        if self.auto_enable_debug and self.auto_debug_nodes:
            self._auto_debug_timer = self.create_timer(1.0, self._check_and_enable_debug_params)
            self._check_and_enable_debug_params()

        display_host = detect_lan_ip() if self.web_host in ('0.0.0.0', '') else self.web_host
        auto_debug_info = f'{self.auto_debug_nodes}' if self.auto_enable_debug else 'Disabled'
        self.get_logger().info(
            f'\n=======================================================\n'
            f' [Clothoid-A Monitor Started]\n'
            f' Web URL       : http://{display_host}:{self.web_port}\n'
            f' Image Topic   : {self.image_topic}\n'
            f' Control Topic : {self.control_topic}\n'
            f' Lane Debug    : {self.lane_topic}\n'
            f' YOLO Debug    : {self.yolo_topic}\n'
            f' Auto Debug    : {auto_debug_info}\n'
            f' Param Nodes   : {self.param_target_nodes}\n'
            f'======================================================='
        )

    def _convert_raw_image_to_jpeg(self, msg: Image) -> bytes:
        # Fast JPEG encoding from sensor_msgs/Image
        height = msg.height
        width = msg.width
        encoding = msg.encoding.lower()

        if encoding in ('bgr8', '8uc3'):
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
        elif encoding in ('rgb8',):
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        elif encoding in ('mono8', '8uc1'):
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width))
        else:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, -1))

        success, encoded = cv2.imencode('.jpg', arr, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        if success:
            return encoded.tobytes()
        return bytes()

    def image_raw_callback(self, msg: Image):
        try:
            jpeg_bytes = self._convert_raw_image_to_jpeg(msg)
            if jpeg_bytes:
                self.state.update_image(jpeg_bytes, msg.width, msg.height)
        except Exception as exc:
            if self.debug_log:
                self.get_logger().error(f'Failed to process {self.image_topic} raw frame: {exc}')

    def image_compressed_callback(self, msg: CompressedImage):
        try:
            frame_bytes = bytes(msg.data)
            width, height = extract_jpeg_dimensions(frame_bytes)
            if width is None or height is None:
                width = self.image_source_width
                height = self.image_source_height
            self.state.update_image(frame_bytes, width, height)
        except Exception as exc:
            if self.debug_log:
                self.get_logger().error(f'Failed to process compressed frame: {exc}')

    def _debug_image_raw_callback(self, msg: Image, image_key: str):
        try:
            jpeg_bytes = self._convert_raw_image_to_jpeg(msg)
            if jpeg_bytes:
                self.state.update_debug_image(image_key, jpeg_bytes, msg.width, msg.height)
        except Exception as exc:
            if self.debug_log:
                self.get_logger().error(f'Failed to process {image_key} debug frame: {exc}')

    def debug_lane_raw_callback(self, msg: Image):
        self._debug_image_raw_callback(msg, 'lane')

    def debug_yolo_raw_callback(self, msg: Image):
        self._debug_image_raw_callback(msg, 'yolo')

    def scan_callback(self, msg: LaserScan):
        try:
            jpeg_bytes = render_lidar_bev(msg, width=400, height=400, max_range=self.lidar_max_range)
            if jpeg_bytes:
                self.state.update_debug_image('lidar', jpeg_bytes, 400, 400)
        except Exception as exc:
            if self.debug_log:
                self.get_logger().error(f'Failed to render lidar scan: {exc}')

    def debug_aruco_raw_callback(self, msg: Image):
        self._debug_image_raw_callback(msg, 'aruco')
        self._debug_image_raw_callback(msg, 'lidar')

    def control_callback(self, msg: Twist):
        # linear.x: speed/throttle, angular.z: steering
        throttle = msg.linear.x
        steering = msg.angular.z
        self.state.update_control(throttle, steering)

    def battery_callback(self, msg: BatteryState):
        percent = msg.percentage
        if percent > 0.0 and percent <= 1.0:
            percent *= 100.0
        self.state.update_battery(percent)

    def storage_timer_callback(self):
        try:
            usage = shutil.disk_usage(self.storage_path)
            self.state.update_storage(usage.used, usage.total)
        except Exception as exc:
            pass

    def get_graph_snapshot(self):
        return build_graph_snapshot(self)

    def set_echo_topic(self, topic_name):
        topic_name = str(topic_name or '').strip()
        if topic_name == self._echo_topic_name:
            return True

        self._clear_echo_subscription()
        if not topic_name:
            return True

        type_str = self._lookup_topic_type(topic_name)
        if type_str is None:
            return False

        try:
            msg_class = get_message(type_str)
        except (AttributeError, ValueError, ModuleNotFoundError) as exc:
            self.get_logger().warning(f'Cannot echo {topic_name} ({type_str}): {exc}')
            return False

        self.state.reset_topic_echo(topic_name)
        self._echo_topic_name = topic_name
        self._echo_subscription = self.create_subscription(
            msg_class, topic_name, self._echo_callback, 10,
        )
        return True

    def _lookup_topic_type(self, topic_name):
        for name, types in self.get_topic_names_and_types():
            if name == topic_name and types:
                return types[0]
        return None

    def _clear_echo_subscription(self):
        if self._echo_subscription is not None:
            self.destroy_subscription(self._echo_subscription)
            self._echo_subscription = None
        self._echo_topic_name = None

    def _echo_callback(self, msg):
        try:
            yaml_text = message_to_yaml(msg)
        except Exception as exc:
            yaml_text = f'<failed to format message: {exc}>'
        self.state.append_topic_echo(self._echo_topic_name, yaml_text)

    def _param_client_group(self, node_name):
        group = self._param_clients.get(node_name)
        if group is None:
            group = {
                'list': self.create_client(ListParameters, f'/{node_name}/list_parameters'),
                'get': self.create_client(GetParameters, f'/{node_name}/get_parameters'),
                'set': self.create_client(SetParameters, f'/{node_name}/set_parameters'),
            }
            self._param_clients[node_name] = group
        return group

    def list_node_params(self, node_name):
        if node_name not in self.param_target_nodes:
            return None
        clients = self._param_client_group(node_name)
        if not clients['list'].wait_for_service(timeout_sec=1.0):
            return None
        try:
            list_resp = clients['list'].call(ListParameters.Request(), timeout_sec=2.0)
        except TimeoutError:
            return None
        names = sorted(n for n in list_resp.result.names if n not in _HIDDEN_PARAM_NAMES)
        if not names:
            return []
        if not clients['get'].wait_for_service(timeout_sec=1.0):
            return None
        try:
            get_resp = clients['get'].call(GetParameters.Request(names=names), timeout_sec=2.0)
        except TimeoutError:
            return None
        params = []
        for name, value in zip(names, get_resp.values):
            entry = _param_value_to_json(value)
            entry['name'] = name
            params.append(entry)
        return params

    def set_node_param(self, node_name, param_name, type_id, raw_value):
        if node_name not in self.param_target_nodes:
            return False, f'unknown node {node_name!r}'
        clients = self._param_client_group(node_name)
        if not clients['set'].wait_for_service(timeout_sec=1.0):
            return False, f'{node_name} not reachable'
        try:
            value = _json_to_param_value(type_id, raw_value)
        except (ValueError, TypeError) as exc:
            return False, str(exc)
        request = SetParameters.Request(parameters=[Parameter(name=param_name, value=value)])
        try:
            response = clients['set'].call(request, timeout_sec=2.0)
        except TimeoutError:
            return False, 'timeout waiting for set_parameters response'
        result = response.results[0]
        return bool(result.successful), str(result.reason)

    def _check_and_enable_debug_params(self):
        for node_name in self.auto_debug_nodes:
            clients = self._param_client_group(node_name)
            set_client = clients['set']
            ready = set_client.service_is_ready()

            if not ready:
                self._auto_enabled_nodes.discard(node_name)
                self._auto_debug_pending.discard(node_name)
                continue

            if node_name in self._auto_enabled_nodes or node_name in self._auto_debug_pending:
                continue

            param_val = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True)
            param = Parameter(name='debug', value=param_val)
            req = SetParameters.Request(parameters=[param])

            self._auto_debug_pending.add(node_name)
            future = set_client.call_async(req)
            future.add_done_callback(
                lambda f, name=node_name: self._on_auto_debug_param_set(name, f)
            )

    def _on_auto_debug_param_set(self, node_name: str, future):
        self._auto_debug_pending.discard(node_name)
        try:
            resp = future.result()
            if resp and resp.results and resp.results[0].successful:
                self._auto_enabled_nodes.add(node_name)
                self.get_logger().info(f'[clothoid_monitor] Auto-enabled debug parameter for /{node_name}')
            else:
                reason = resp.results[0].reason if (resp and resp.results) else 'unknown'
                self.get_logger().warn(
                    f'[clothoid_monitor] Failed to auto-enable debug for /{node_name}: {reason}'
                )
        except Exception as exc:
            self.get_logger().warn(
                f'[clothoid_monitor] Error setting debug for /{node_name}: {exc}'
            )

    def destroy_node(self):
        if hasattr(self, '_auto_debug_timer') and self._auto_debug_timer is not None:
            self._auto_debug_timer.cancel()

        if hasattr(self, 'server_thread') and self.server_thread is not None:
            self.server_thread.shutdown()
            self.server_thread.join(timeout=2.0)

        # Revert auto-enabled debug parameters back to False on exit to save CPU/battery
        if getattr(self, 'auto_enable_debug', False) and getattr(self, '_auto_enabled_nodes', None):
            for node_name in list(self._auto_enabled_nodes):
                try:
                    clients = self._param_clients.get(node_name)
                    if clients and clients['set'].service_is_ready():
                        param_val = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False)
                        param = Parameter(name='debug', value=param_val)
                        req = SetParameters.Request(parameters=[param])
                        future = clients['set'].call_async(req)
                        if self.executor is not None and rclpy.ok():
                            self.executor.spin_until_future_complete(future, timeout_sec=0.5)
                        self.get_logger().info(f'[clothoid_monitor] Reverted debug parameter for /{node_name} to False')
                except Exception as exc:
                    self.get_logger().debug(f'Could not revert debug param for {node_name}: {exc}')

        super().destroy_node()


def main(args=None):
    node = None
    rclpy.init(args=args)
    try:
        node = MonitorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        if node is not None:
            node.get_logger().info('Shutting down clothoid_monitor node.')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
