"""Small self-contained dashboard for the standalone path experiment."""

from __future__ import annotations

import json
import math
import queue
import threading
import time
import webbrowser
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Optional
from urllib.parse import urlparse

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, Vector3
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty


ODOM_TOPIC = '/model/research_catamaran/odometry'
REFERENCE_POSE_TOPIC = '/asv_path_tracking/reference_pose'
REFERENCE_PATH_TOPIC = '/asv_path_tracking/reference_path'
CONTROLLER_STATUS_TOPIC = '/asv_path_tracking/status'
PATH_STATUS_TOPIC = '/asv_path_tracking/path_status'
CURRENT_TOPIC = '/ocean_current'
RUN_TOPIC = '/asv_path_tracking/run'
RESET_TOPIC = '/asv_path_tracking/reset'
PATH_NODE = '/asv_path_generator'
CONTROLLER_NODE = '/asv_path_tracking_controller'


def finite(value: Any, default: Optional[float] = None) -> Optional[float]:
    """Return a finite float or a supplied default."""
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


class Request:
    """One HTTP request handed back to the ROS executor thread."""

    def __init__(self, method: str, path: str, body: bytes):
        self.method = method
        self.path = path
        self.body = body
        self.done = threading.Event()
        self.status = 500
        self.headers: Dict[str, str] = {}
        self.response = b''


class DashboardHandler(BaseHTTPRequestHandler):
    """Serve dashboard assets and JSON through the ROS-owned request queue."""

    server: 'DashboardServer'

    def log_message(self, _format: str, *_args: Any) -> None:
        return

    def do_GET(self) -> None:
        self._dispatch('GET')

    def do_POST(self) -> None:
        length = int(self.headers.get('Content-Length', '0'))
        self._dispatch('POST', self.rfile.read(length))

    def _dispatch(self, method: str, body: bytes = b'') -> None:
        request = Request(method, urlparse(self.path).path, body)
        self.server.dashboard.requests.put(request)
        request.done.wait(timeout=3.0)
        self.send_response(request.status)
        for key, value in request.headers.items():
            self.send_header(key, value)
        self.end_headers()
        self.wfile.write(request.response)


class DashboardServer(ThreadingHTTPServer):
    """HTTP server carrying a reference to the ROS dashboard node."""

    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, dashboard):
        self.dashboard = dashboard
        super().__init__(address, DashboardHandler)


class PathTrackingDashboard(Node):
    """Expose telemetry and safe experiment controls to a browser."""

    def __init__(self) -> None:
        super().__init__('asv_path_tracking_dashboard')
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 8780)
        self.declare_parameter('open_browser', False)
        self.declare_parameter('history_size', 1200)
        self.host = str(self.get_parameter('host').value).strip() or '0.0.0.0'
        requested_port = int(self.get_parameter('port').value)
        self.history = deque(
            maxlen=max(int(self.get_parameter('history_size').value), 100))
        self.requests: 'queue.Queue[Request]' = queue.Queue()
        self.latest_odom: Optional[Odometry] = None
        self.reference_pose: Optional[PoseStamped] = None
        self.reference_path: Optional[Path] = None
        self.current = (0.0, 0.0, 0.0)
        self.controller_values: Dict[str, str] = {}
        self.controller_message = 'waiting for controller'
        self.path_values: Dict[str, str] = {}
        self.path_message = 'waiting for path reference'
        self.last_odom: Optional[float] = None
        self.last_controller: Optional[float] = None
        self.last_reference: Optional[float] = None
        self._assets = self._resolve_assets()
        self._server = self._make_server(requested_port)
        self.port = int(self._server.server_address[1])
        self._server_thread = threading.Thread(
            target=self._server.serve_forever,
            name='asv-path-tracking-dashboard-http',
            daemon=True,
        )
        self._server_thread.start()

        self.run_pub = self.create_publisher(Bool, RUN_TOPIC, 10)
        self.reset_pub = self.create_publisher(Empty, RESET_TOPIC, 10)
        self.path_client = self.create_client(
            SetParameters, PATH_NODE + '/set_parameters')
        self.controller_client = self.create_client(
            SetParameters, CONTROLLER_NODE + '/set_parameters')
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_callback, 10)
        self.create_subscription(
            PoseStamped, REFERENCE_POSE_TOPIC, self.reference_callback, 10)
        self.create_subscription(
            Path,
            REFERENCE_PATH_TOPIC,
            self.reference_path_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_subscription(
            Vector3, CURRENT_TOPIC, self.current_callback, 10)
        self.create_subscription(
            DiagnosticArray,
            CONTROLLER_STATUS_TOPIC,
            self.controller_callback,
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            PATH_STATUS_TOPIC,
            self.path_callback,
            10,
        )
        self.create_timer(0.02, self.process_requests)
        self.create_timer(0.5, self.record_history)

        if bool(self.get_parameter('open_browser').value):
            threading.Timer(1.0, self.open_browser).start()
        self.get_logger().info(
            f'Path dashboard listening on {self.host}:{self.port}; '
            f'open http://127.0.0.1:{self.port}')

    def _make_server(self, requested_port: int) -> DashboardServer:
        try:
            return DashboardServer((self.host, requested_port), self)
        except OSError as error:
            self.get_logger().warning(
                f'Path dashboard port {requested_port} unavailable ({error}); '
                'selecting a free port.')
            return DashboardServer((self.host, 0), self)

    def _resolve_assets(self) -> Path:
        try:
            from ament_index_python.packages import get_package_share_directory
            installed = Path(get_package_share_directory('asv_path_tracking'))
            assets = installed / 'dashboard'
            if assets.is_dir():
                return assets
        except Exception:
            pass
        return Path(__file__).resolve().parents[1] / 'dashboard'

    def open_browser(self) -> None:
        try:
            webbrowser.open(f'http://127.0.0.1:{self.port}')
        except Exception as error:
            self.get_logger().warning(f'Could not open dashboard browser: {error}')

    def odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message
        self.last_odom = time.monotonic()

    def reference_callback(self, message: PoseStamped) -> None:
        self.reference_pose = message
        self.last_reference = time.monotonic()

    def reference_path_callback(self, message: Path) -> None:
        self.reference_path = message

    def current_callback(self, message: Vector3) -> None:
        self.current = (float(message.x), float(message.y), float(message.z))

    def controller_callback(self, message) -> None:
        if not message.status:
            return
        self.controller_values = {
            item.key: item.value for item in message.status[0].values}
        self.controller_message = message.status[0].message
        self.last_controller = time.monotonic()

    def path_callback(self, message) -> None:
        if not message.status:
            return
        self.path_values = {
            item.key: item.value for item in message.status[0].values}
        self.path_message = message.status[0].message

    def process_requests(self) -> None:
        while True:
            try:
                request = self.requests.get_nowait()
            except queue.Empty:
                return
            try:
                self._handle(request)
            except Exception as error:
                request.status = 500
                request.response = json.dumps({'error': str(error)}).encode()
                request.headers = {'Content-Type': 'application/json'}
            finally:
                request.done.set()

    def _handle(self, request: Request) -> None:
        if request.method == 'GET' and request.path in {
                '/', '/index.html', '/app.js', '/style.css'}:
            name = 'index.html' if request.path == '/' else request.path[1:]
            path = self._assets / name
            if not path.is_file():
                request.status = 404
                request.response = b'not found'
                request.headers = {'Content-Type': 'text/plain'}
                return
            request.status = 200
            request.response = path.read_bytes()
            content_types = {
                'index.html': 'text/html; charset=utf-8',
                'app.js': 'text/javascript; charset=utf-8',
                'style.css': 'text/css; charset=utf-8',
            }
            request.headers = {
                'Content-Type': content_types[name],
                'Cache-Control': 'no-store',
            }
            return
        if request.method == 'GET' and request.path == '/api/health':
            self._json(request, self._health())
            return
        if request.method == 'GET' and request.path == '/api/state':
            self._json(request, self._state())
            return
        if request.method == 'POST' and request.path == '/api/run':
            body = json.loads(request.body or b'{}')
            message = Bool()
            message.data = bool(body.get('running', True))
            self.run_pub.publish(message)
            self._json(request, {'ok': True, 'running': message.data})
            return
        if request.method == 'POST' and request.path == '/api/reset':
            self.history.clear()
            self.reset_pub.publish(Empty())
            self._json(request, {'ok': True})
            return
        if request.method == 'POST' and request.path == '/api/parameters':
            self._set_parameters(json.loads(request.body or b'{}'))
            self._json(request, {'ok': True})
            return
        request.status = 404
        request.response = b'not found'
        request.headers = {'Content-Type': 'text/plain'}

    def _set_parameters(self, values: Dict[str, Any]) -> None:
        path_names = {
            'path_type', 'path_width', 'path_height', 'speed',
            'path_center_x', 'path_center_y', 'corner_roundness',
        }
        controller_names = {
            'max_thrust', 'kp_x', 'ki_x', 'kd_x', 'kp_y', 'ki_y', 'kd_y',
            'kp_yaw', 'ki_yaw', 'kd_yaw', 'slew_rate',
            'enable_current_compensation', 'enable_yaw_control',
        }
        path_values = {key: value for key, value in values.items()
                       if key in path_names}
        controller_values = {key: value for key, value in values.items()
                             if key in controller_names}
        if path_values:
            self._call_parameters(self.path_client, path_values)
        if controller_values:
            self._call_parameters(self.controller_client, controller_values)

    def _call_parameters(self, client, values: Dict[str, Any]) -> None:
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=0.0)
        if not client.service_is_ready():
            return
        request = SetParameters.Request()
        request.parameters = [
            self._parameter(name, value) for name, value in values.items()]
        client.call_async(request)

    @staticmethod
    def _parameter(name: str, value: Any) -> Parameter:
        parameter = Parameter()
        parameter.name = name
        parameter.value = ParameterValue()
        if isinstance(value, bool):
            parameter.value.type = ParameterType.PARAMETER_BOOL
            parameter.value.bool_value = value
        elif isinstance(value, (int, float)):
            parameter.value.type = ParameterType.PARAMETER_DOUBLE
            parameter.value.double_value = float(value)
        else:
            parameter.value.type = ParameterType.PARAMETER_STRING
            parameter.value.string_value = str(value)
        return parameter

    def record_history(self) -> None:
        state = self._state()
        self.history.append({
            't': state['simulation']['dashboard_time_s'],
            'x': state['pose']['x'],
            'y': state['pose']['y'],
            'rx': state['reference']['x'],
            'ry': state['reference']['y'],
            'cross': state['errors']['cross_track_m'],
        })

    def _health(self) -> Dict[str, Any]:
        now = time.monotonic()
        ages = {
            'odom_age_s': (
                None if self.last_odom is None else now - self.last_odom),
            'reference_age_s': (
                None if self.last_reference is None else now - self.last_reference),
            'controller_age_s': (
                None if self.last_controller is None else now - self.last_controller),
        }
        stale = any(
            age is None or age > 2.0 for age in ages.values())
        return {
            'level': 'STALE' if stale else 'OK',
            'message': self.controller_message,
            **ages,
        }

    def _state(self) -> Dict[str, Any]:
        pose = self.latest_odom.pose.pose if self.latest_odom else None
        reference = self.reference_pose.pose if self.reference_pose else None
        return {
            'health': self._health(),
            'simulation': {
                'dashboard_time_s': time.time(),
            },
            'path': self.path_values,
            'reference': {
                'x': 0.0 if reference is None else reference.position.x,
                'y': 0.0 if reference is None else reference.position.y,
            },
            'reference_path': [
                {'x': pose.pose.position.x, 'y': pose.pose.position.y}
                for pose in (self.reference_path.poses
                             if self.reference_path is not None else [])
            ],
            'pose': {
                'x': 0.0 if pose is None else pose.position.x,
                'y': 0.0 if pose is None else pose.position.y,
                'z': 0.0 if pose is None else pose.position.z,
            },
            'errors': {
                'cross_track_m': finite(
                    self.controller_values.get('cross_track_error_m'), 0.0),
                'along_track_m': finite(
                    self.controller_values.get('along_track_error_m'), 0.0),
                'heading_rad': finite(
                    self.controller_values.get('heading_error_rad'), 0.0),
            },
            'current': {
                'x': self.current[0],
                'y': self.current[1],
                'z': self.current[2],
            },
            'controller': self.controller_values,
            'history': list(self.history),
        }

    @staticmethod
    def _json(request: Request, value: Any) -> None:
        request.status = 200
        request.response = json.dumps(value, allow_nan=False).encode()
        request.headers = {
            'Content-Type': 'application/json',
            'Cache-Control': 'no-store',
        }

    def destroy_node(self) -> bool:
        self._server.shutdown()
        self._server.server_close()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the browser dashboard."""
    rclpy.init(args=args)
    node = PathTrackingDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
