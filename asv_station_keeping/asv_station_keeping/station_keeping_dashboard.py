"""Local research dashboard for the ROS 2 station-keeping experiment.

The dashboard deliberately uses only the Python standard library for its HTTP
server and browser assets.  This keeps the experiment reproducible on a ROS 2
Humble installation without adding a plotting framework to the ROS package.
The browser renders the live plots with its own Canvas implementation.
"""

import json
import math
import queue
import threading
import time
import webbrowser
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Deque, Dict, Optional
from urllib.parse import urlparse

import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Empty

from asv_station_keeping.visualization_model import (
    level_as_int,
    parse_diagnostic_values,
    quaternion_to_yaw,
)


STATUS_TOPIC = '/research_catamaran/station_keeping/status'
ODOMETRY_TOPIC = '/model/research_catamaran/odometry'
CURRENT_TOPIC = '/ocean_current'
CLOCK_TOPIC = '/clock'
RESET_TOPIC = '/research_catamaran/station_keeping/reset_integrators'
START_DEMO_TOPIC = '/research_catamaran/station_keeping/start_demo'
CONTROLLER_NODE = '/research_station_keeping_pid'
CURRENT_NODE = '/research_current_disturbance'


PARAMETER_STATUS_KEYS = {
    'target_x': 'target_x_m',
    'target_y': 'target_y_m',
    'target_yaw': 'target_yaw_rad',
    'target_slew_rate': 'target_slew_rate_mps',
    'target_yaw_slew_rate': 'target_yaw_slew_rate_radps',
    'max_thrust': 'max_thrust_N',
    'kp_x': 'kp_x',
    'ki_x': 'ki_x',
    'kd_x': 'kd_x',
    'kp_y': 'kp_y',
    'ki_y': 'ki_y',
    'kd_y': 'kd_y',
    'kp_yaw': 'kp_yaw',
    'ki_yaw': 'ki_yaw',
    'kd_yaw': 'kd_yaw',
    'integral_limit': 'integral_limit',
    'integral_leak': 'integral_leak',
    'slew_rate': 'slew_rate_Nps',
    'velocity_filter_tau': 'velocity_filter_tau',
    'enable_yaw_control': 'enable_yaw_control',
    'enable_tilt_guard': 'enable_tilt_guard',
    'tilt_soft_limit': 'tilt_soft_limit_rad',
    'tilt_hard_limit': 'tilt_hard_limit_rad',
    'position_tolerance': 'position_tolerance_m',
    'yaw_tolerance': 'yaw_tolerance_rad',
    'current_x': 'current_x_mps',
    'current_y': 'current_y_mps',
    'current_z': 'current_z_mps',
}

CONTROLLER_PARAMETER_NAMES = set(PARAMETER_STATUS_KEYS) - {
    'current_x',
    'current_y',
    'current_z',
}
CURRENT_PARAMETER_NAMES = {'current_x', 'current_y', 'current_z'}
BOOLEAN_PARAMETERS = {'enable_yaw_control', 'enable_tilt_guard'}


def _finite(value: Any, default: float = 0.0) -> float:
    """Return a finite float for browser-safe JSON."""
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _bool_value(value: Any, default: bool = False) -> bool:
    """Parse ROS diagnostic boolean values without Python truthiness traps."""
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    text = str(value).strip().lower()
    if text in {'true', '1', 'yes', 'on'}:
        return True
    if text in {'false', '0', 'no', 'off'}:
        return False
    return default


def _rpy_from_quaternion(x: float, y: float, z: float, w: float):
    """Return roll, pitch, yaw from an x/y/z/w quaternion."""
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = (
        math.copysign(math.pi / 2.0, sin_pitch)
        if abs(sin_pitch) >= 1.0 else math.asin(sin_pitch)
    )
    return roll, pitch, quaternion_to_yaw(x, y, z, w)


def _json_safe(value):
    """Return value with NaN/inf replaced by None so it is valid JSON."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


class _HttpRequest:
    """A browser request waiting for work on the ROS executor thread."""

    def __init__(self, kind: str, payload: Optional[Dict[str, Any]] = None):
        self.kind = kind
        self.payload = payload or {}
        self.event = threading.Event()
        self.result: Dict[str, Any] = {
            'ok': False,
            'message': 'request did not complete',
        }


class DashboardHttpHandler(BaseHTTPRequestHandler):
    """Serve the dashboard assets and small JSON API."""

    server: 'DashboardHttpServer'

    def log_message(self, format_string, *args):
        """Keep HTTP access logs out of the ROS experiment terminal."""
        return

    @property
    def dashboard(self):
        return self.server.dashboard_node

    def _send_bytes(self, body: bytes, content_type: str, status: int = 200):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, value: Dict[str, Any], status: int = 200):
        body = json.dumps(value, allow_nan=False).encode('utf-8')
        self._send_bytes(body, 'application/json; charset=utf-8', status)

    def _asset(self, name: str):
        asset = self.dashboard.asset_path(name)
        if asset is None or not asset.is_file():
            self._send_json({'error': 'asset not found'}, 404)
            return
        content_types = {
            '.html': 'text/html; charset=utf-8',
            '.js': 'application/javascript; charset=utf-8',
            '.css': 'text/css; charset=utf-8',
        }
        self._send_bytes(
            asset.read_bytes(),
            content_types.get(asset.suffix, 'application/octet-stream'),
        )

    def do_GET(self):  # noqa: N802 - required by BaseHTTPRequestHandler
        path = urlparse(self.path).path
        if path in {'/', '/index.html'}:
            self._asset('index.html')
        elif path == '/app.js':
            self._asset('app.js')
        elif path == '/style.css':
            self._asset('style.css')
        elif path == '/api/state':
            self._send_json(self.dashboard.snapshot())
        elif path == '/api/health':
            snapshot = self.dashboard.snapshot()
            self._send_json({
                'ok': snapshot['status']['level'] < 2,
                'status': snapshot['status'],
            })
        else:
            self._send_json({'error': 'not found'}, 404)

    def do_POST(self):  # noqa: N802 - required by BaseHTTPRequestHandler
        path = urlparse(self.path).path
        length = int(self.headers.get('Content-Length', '0'))
        if length > 32768:
            self._send_json({'error': 'request too large'}, 413)
            return
        try:
            payload = json.loads(self.rfile.read(length) or b'{}')
        except (json.JSONDecodeError, UnicodeDecodeError):
            self._send_json({'error': 'invalid JSON'}, 400)
            return

        if path == '/api/parameters':
            result = self.dashboard.request_parameter_update(payload)
            self._send_json(result, 200 if result.get('ok') else 400)
        elif path == '/api/actions/reset_integrators':
            result = self.dashboard.request_reset_integrators()
            self._send_json(result, 200 if result.get('ok') else 500)
        elif path == '/api/actions/start_demo':
            result = self.dashboard.request_start_demo()
            self._send_json(result, 200 if result.get('ok') else 500)
        else:
            self._send_json({'error': 'not found'}, 404)


class DashboardHttpServer(ThreadingHTTPServer):
    """HTTP server carrying a reference back to its ROS node."""

    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, node):
        self.dashboard_node = node
        super().__init__(address, DashboardHttpHandler)


class StationKeepingDashboard(Node):
    """Collect station-keeping telemetry and expose a research dashboard."""

    def __init__(self):
        super().__init__('research_station_keeping_dashboard')
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('advertise_host', '127.0.0.1')
        self.declare_parameter('port', 8765)
        self.declare_parameter('history_size', 1200)
        self.declare_parameter('open_browser', False)
        self.declare_parameter('status_timeout', 2.0)
        self.declare_parameter('asset_directory', '')

        self.host = str(self.get_parameter('host').value).strip() or '0.0.0.0'
        self.advertise_host = str(
            self.get_parameter('advertise_host').value).strip()
        if not self.advertise_host or self.advertise_host == '0.0.0.0':
            self.advertise_host = '127.0.0.1'
        requested_port = int(self.get_parameter('port').value)
        self.history_size = max(int(self.get_parameter('history_size').value), 50)
        self.status_timeout = max(
            float(self.get_parameter('status_timeout').value), 0.1)
        self._asset_directory = self._resolve_asset_directory()
        self._lock = threading.RLock()
        self._requests: 'queue.Queue[_HttpRequest]' = queue.Queue()
        self._history: Deque[Dict[str, Any]] = deque(maxlen=self.history_size)
        self._latest_status: Dict[str, str] = {}
        self._latest_odom: Optional[Odometry] = None
        self._latest_current = (0.0, 0.0, 0.0)
        self._last_status_wall: Optional[float] = None
        self._last_odom_wall: Optional[float] = None
        self._last_current_wall: Optional[float] = None
        self._last_sim_time: Optional[float] = None
        self._last_clock_wall: Optional[float] = None
        self._sim_time = 0.0
        self._real_time_factor = 0.0
        self._status_level = level_as_int(DiagnosticStatus.ERROR)
        self._status_message = 'waiting for station-keeping diagnostics'

        self._state: Dict[str, Any] = self._empty_state()
        try:
            self._server = DashboardHttpServer(
                (self.host, requested_port), self)
        except OSError as exc:
            if requested_port == 0:
                raise
            self.get_logger().warning(
                f'Dashboard port {requested_port} is unavailable ({exc}); '
                'selecting a free local port.')
            self._server = DashboardHttpServer((self.host, 0), self)
        self.port = int(self._server.server_address[1])
        self._server_thread = threading.Thread(
            target=self._server.serve_forever,
            name='station-keeping-dashboard-http',
            daemon=True,
        )
        self._server_thread.start()

        self.reset_pub = self.create_publisher(Empty, RESET_TOPIC, 10)
        self.start_demo_pub = self.create_publisher(
            Empty, START_DEMO_TOPIC, 10)
        self.controller_client = self.create_client(
            SetParameters,
            CONTROLLER_NODE + '/set_parameters',
        )
        self.current_client = self.create_client(
            SetParameters,
            CURRENT_NODE + '/set_parameters',
        )
        self.create_subscription(Odometry, ODOMETRY_TOPIC, self.odom_callback, 10)
        self.create_subscription(
            DiagnosticArray, STATUS_TOPIC, self.status_callback, 10)
        self.create_subscription(Vector3, CURRENT_TOPIC, self.current_callback, 10)
        self.create_subscription(Clock, CLOCK_TOPIC, self.clock_callback, 10)
        self.create_timer(0.02, self.process_http_requests)
        self.create_timer(0.5, self.refresh_state)

        if bool(self.get_parameter('open_browser').value):
            threading.Timer(1.0, self.open_browser).start()
        self.get_logger().info(
            f'Research dashboard listening on {self.host}:{self.port}; '
            f'open http://{self.advertise_host}:{self.port}')

    def _resolve_asset_directory(self) -> Path:
        """Resolve installed assets, with a source-tree fallback."""
        configured = str(self.get_parameter('asset_directory').value).strip()
        if configured:
            return Path(configured)
        try:
            share = Path(get_package_share_directory('asv_station_keeping'))
            installed = share / 'dashboard'
            if installed.is_dir():
                return installed
        except Exception:  # pragma: no cover - only used outside ROS installs
            pass
        return Path(__file__).resolve().parents[1] / 'dashboard'

    def asset_path(self, name: str) -> Optional[Path]:
        """Return a safe dashboard asset path."""
        allowed = {'index.html', 'app.js', 'style.css'}
        if name not in allowed:
            return None
        return self._asset_directory / name

    def open_browser(self):
        """Open the local dashboard when a desktop session is available."""
        try:
            webbrowser.open(
                f'http://{self.advertise_host}:{self.port}')
        except Exception as exc:  # pragma: no cover - desktop dependent
            self.get_logger().warning(f'Could not open dashboard browser: {exc}')

    @staticmethod
    def _empty_state() -> Dict[str, Any]:
        """Return a JSON-safe initial state for the browser."""
        return {
            'status': {
                'level': 2,
                'label': 'ERROR',
                'message': 'waiting for station-keeping diagnostics',
                'controller_age_s': None,
                'odom_age_s': None,
                'current_age_s': None,
            },
            'simulation': {
                'sim_time_s': 0.0,
                'real_time_factor': 0.0,
                'dashboard_time_s': 0.0,
            },
            'target': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            'pose': {
                'x': 0.0, 'y': 0.0, 'z': 0.0,
                'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            },
            'error': {
                'x_body': 0.0, 'y_body': 0.0, 'yaw': 0.0,
                'position_m': 0.0, 'position_rmse_m': 0.0,
                'max_position_m': 0.0,
            },
            'velocity': {'u': 0.0, 'v': 0.0, 'r': 0.0},
            'current': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'magnitude': 0.0},
            'control': {
                'surge_N': 0.0, 'sway_N': 0.0, 'yaw_Nm': 0.0,
                'integral_x': 0.0, 'integral_y': 0.0,
                'integral_yaw': 0.0,
            },
            'thrusters': {
                'port_front': 0.0, 'starboard_front': 0.0,
                'port_rear': 0.0, 'starboard_rear': 0.0,
            },
            'safety': {
                'tilt_scale': 1.0, 'thrust_saturated': False,
                'tilt_guard_active': False,
            },
            'parameters': {},
            'history': [],
        }

    def _status_value(self, key: str, default: Any = None) -> Any:
        return self._latest_status.get(key, default)

    def odom_callback(self, message: Odometry):
        """Update pose, velocity and attitude telemetry."""
        with self._lock:
            self._latest_odom = message
            self._last_odom_wall = time.monotonic()

    def current_callback(self, message: Vector3):
        """Update the latest environmental current vector."""
        with self._lock:
            self._latest_current = (
                _finite(message.x), _finite(message.y), _finite(message.z))
            self._last_current_wall = time.monotonic()

    def clock_callback(self, message: Clock):
        """Estimate real-time factor from simulation clock progression."""
        sim_time = float(message.clock.sec) + message.clock.nanosec * 1e-9
        now = time.monotonic()
        with self._lock:
            if self._last_sim_time is not None and self._last_clock_wall is not None:
                wall_delta = now - self._last_clock_wall
                sim_delta = sim_time - self._last_sim_time
                if wall_delta > 1e-6 and sim_delta >= 0.0:
                    instantaneous = sim_delta / wall_delta
                    self._real_time_factor = 0.9 * self._real_time_factor + (
                        0.1 * instantaneous)
            self._sim_time = sim_time
            self._last_sim_time = sim_time
            self._last_clock_wall = now

    def status_callback(self, message: DiagnosticArray):
        """Store the controller diagnostic snapshot and append a plot sample."""
        selected = None
        for diagnostic in message.status:
            if diagnostic.name == 'research_catamaran/station_keeping':
                selected = diagnostic
                break
        if selected is None:
            return
        with self._lock:
            self._latest_status = parse_diagnostic_values(selected.values)
            self._status_level = level_as_int(selected.level)
            self._status_message = str(selected.message)
            self._last_status_wall = time.monotonic()
            self._refresh_state_locked()
            self._append_history_locked()

    def refresh_state(self):
        """Refresh data-age fields even when the simulator is paused."""
        with self._lock:
            self._refresh_state_locked()

    def _refresh_state_locked(self):
        now = time.monotonic()
        odom = self._latest_odom
        pose = self._state['pose']
        velocity = self._state['velocity']
        if odom is not None:
            position = odom.pose.pose.position
            orientation = odom.pose.pose.orientation
            roll, pitch, yaw = _rpy_from_quaternion(
                orientation.x, orientation.y, orientation.z, orientation.w)
            pose.update({
                'x': _finite(position.x), 'y': _finite(position.y),
                'z': _finite(position.z), 'roll': roll, 'pitch': pitch,
                'yaw': yaw,
            })
            velocity.update({
                'u': _finite(odom.twist.twist.linear.x),
                'v': _finite(odom.twist.twist.linear.y),
                'r': _finite(odom.twist.twist.angular.z),
            })

        current_x, current_y, current_z = self._latest_current
        current = self._state['current']
        current.update({
            'x': current_x, 'y': current_y, 'z': current_z,
            'magnitude': math.sqrt(
                current_x * current_x + current_y * current_y +
                current_z * current_z),
        })

        self._state['target'].update({
            'x': _finite(self._status_value('target_x_m', 0.0)),
            'y': _finite(self._status_value('target_y_m', 0.0)),
            'yaw': _finite(self._status_value('target_yaw_rad', 0.0)),
        })
        error = self._state['error']
        error.update({
            'x_body': _finite(self._status_value('error_x_body_m', 0.0)),
            'y_body': _finite(self._status_value('error_y_body_m', 0.0)),
            'yaw': _finite(self._status_value('error_yaw_rad', 0.0)),
            'position_m': _finite(self._status_value('position_error_m', 0.0)),
            'position_rmse_m': _finite(
                self._status_value('position_rmse_m', 0.0)),
            'max_position_m': _finite(
                self._status_value('max_position_error_m', 0.0)),
        })
        control = self._state['control']
        control.update({
            'surge_N': _finite(self._status_value('control_surge_N', 0.0)),
            'sway_N': _finite(self._status_value('control_sway_N', 0.0)),
            'yaw_Nm': _finite(self._status_value('control_yaw_Nm', 0.0)),
            'integral_x': _finite(self._status_value('integral_x', 0.0)),
            'integral_y': _finite(self._status_value('integral_y', 0.0)),
            'integral_yaw': _finite(self._status_value('integral_yaw', 0.0)),
        })
        thrusters = self._state['thrusters']
        for name in thrusters:
            thrusters[name] = _finite(
                self._status_value('thruster_' + name + '_N', 0.0))
        safety = self._state['safety']
        safety.update({
            'tilt_scale': _finite(self._status_value('tilt_scale', 1.0), 1.0),
            'thrust_saturated': _bool_value(
                self._status_value('thrust_saturated', False)),
            'tilt_guard_active': _bool_value(
                self._status_value('tilt_guard_active', False)),
        })
        parameters = self._state['parameters']
        for parameter_name, status_key in PARAMETER_STATUS_KEYS.items():
            value = self._status_value(status_key)
            if value is None:
                continue
            if parameter_name in BOOLEAN_PARAMETERS:
                parameters[parameter_name] = _bool_value(value)
            else:
                parameters[parameter_name] = _finite(value)

        controller_age = self._age(self._last_status_wall, now)
        odom_age = self._age(self._last_odom_wall, now)
        current_age = self._age(self._last_current_wall, now)
        visible_level = self._status_level
        visible_message = self._status_message
        if controller_age is None or controller_age > self.status_timeout:
            visible_level = DiagnosticStatus.STALE
            visible_message = 'station-keeping diagnostics stale'
        elif odom_age is None or odom_age > self.status_timeout:
            visible_level = DiagnosticStatus.STALE
            visible_message = 'odometry telemetry stale'
        visible_level = level_as_int(visible_level)
        self._state['status'] = {
            'level': visible_level,
            'label': self._status_label(visible_level),
            'message': visible_message,
            'controller_age_s': controller_age,
            'odom_age_s': odom_age,
            'current_age_s': current_age,
        }
        self._state['simulation'].update({
            'sim_time_s': self._sim_time,
            'real_time_factor': self._real_time_factor,
            'dashboard_time_s': time.time(),
        })

    @staticmethod
    def _age(received: Optional[float], now: float):
        return None if received is None else max(now - received, 0.0)

    @staticmethod
    def _status_label(level: int) -> str:
        return {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}.get(
            level_as_int(level), 'UNKNOWN')

    def _append_history_locked(self):
        state = self._state
        history = {
            't': state['simulation']['sim_time_s'],
            'x': state['pose']['x'], 'y': state['pose']['y'],
            'z': state['pose']['z'], 'roll': state['pose']['roll'],
            'pitch': state['pose']['pitch'], 'yaw': state['pose']['yaw'],
            'target_x': state['target']['x'], 'target_y': state['target']['y'],
            'target_yaw': state['target']['yaw'],
            'position_error': state['error']['position_m'],
            'position_rmse': state['error']['position_rmse_m'],
            'yaw_error': state['error']['yaw'],
            'u': state['velocity']['u'], 'v': state['velocity']['v'],
            'r': state['velocity']['r'],
            'current_x': state['current']['x'],
            'current_y': state['current']['y'],
            'thrust_pf': state['thrusters']['port_front'],
            'thrust_sf': state['thrusters']['starboard_front'],
            'thrust_pr': state['thrusters']['port_rear'],
            'thrust_sr': state['thrusters']['starboard_rear'],
            'rtf': state['simulation']['real_time_factor'],
        }
        self._history.append(history)

    def snapshot(self) -> Dict[str, Any]:
        """Return a deep JSON-safe snapshot for the browser."""
        with self._lock:
            self._refresh_state_locked()
            snapshot = json.loads(
                json.dumps(_json_safe(self._state), allow_nan=False))
            snapshot['history'] = list(self._history)
            return snapshot

    def request_parameter_update(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Validate and forward live-editable parameters to ROS nodes."""
        values = payload.get('parameters', payload)
        if not isinstance(values, dict) or not values:
            return {'ok': False, 'message': 'parameters must be a non-empty object'}
        clean: Dict[str, Any] = {}
        for name, value in values.items():
            if name not in PARAMETER_STATUS_KEYS:
                return {'ok': False, 'message': f'parameter {name} is not editable'}
            if name in BOOLEAN_PARAMETERS:
                if not isinstance(value, bool):
                    return {'ok': False, 'message': f'{name} must be boolean'}
                clean[name] = value
            else:
                numeric = _finite(value, float('nan'))
                if not math.isfinite(numeric):
                    return {'ok': False, 'message': f'{name} must be finite'}
                clean[name] = numeric
        request = _HttpRequest('parameters', {'parameters': clean})
        self._requests.put(request)
        if not request.event.wait(5.0):
            return {'ok': False, 'message': 'ROS parameter update timed out'}
        return request.result

    def request_reset_integrators(self) -> Dict[str, Any]:
        """Queue a controller integrator reset from the browser."""
        request = _HttpRequest('reset_integrators')
        self._requests.put(request)
        if not request.event.wait(3.0):
            return {'ok': False, 'message': 'reset request timed out'}
        return request.result

    def request_start_demo(self) -> Dict[str, Any]:
        """Queue a repeatable two-phase demonstration request."""
        request = _HttpRequest('start_demo')
        self._requests.put(request)
        if not request.event.wait(3.0):
            return {'ok': False, 'message': 'demo request timed out'}
        return request.result

    def process_http_requests(self):
        """Process browser requests from the ROS executor thread."""
        try:
            while True:
                request = self._requests.get_nowait()
                if request.kind == 'reset_integrators':
                    self.reset_pub.publish(Empty())
                    request.result = {'ok': True, 'message': 'integrators reset'}
                    request.event.set()
                elif request.kind == 'start_demo':
                    self.start_demo_pub.publish(Empty())
                    request.result = {
                        'ok': True,
                        'message': 'scripted station-keeping demo started',
                    }
                    request.event.set()
                elif request.kind == 'parameters':
                    self._start_parameter_update(request)
                else:
                    request.result = {'ok': False, 'message': 'unknown request'}
                    request.event.set()
        except queue.Empty:
            return

    def _start_parameter_update(self, request: _HttpRequest):
        values = request.payload['parameters']
        controller_values = {
            name: value for name, value in values.items()
            if name in CONTROLLER_PARAMETER_NAMES
        }
        current_values = {
            name: value for name, value in values.items()
            if name in CURRENT_PARAMETER_NAMES
        }
        self._parameter_service_chain(
            request,
            [(self.controller_client, controller_values),
             (self.current_client, current_values)],
            0,
        )

    def _parameter_service_chain(self, request, stages, index):
        if index >= len(stages):
            request.result = {'ok': True, 'message': 'parameters updated'}
            request.event.set()
            return
        client, values = stages[index]
        if not values:
            self._parameter_service_chain(request, stages, index + 1)
            return
        if not client.service_is_ready():
            request.result = {
                'ok': False,
                'message': 'controller or disturbance parameter service unavailable',
            }
            request.event.set()
            return
        ros_parameters = [self._to_ros_parameter(name, value)
                          for name, value in values.items()]
        service_request = SetParameters.Request()
        service_request.parameters = ros_parameters
        future = client.call_async(service_request)

        def complete(done_future):
            try:
                results = done_future.result().results
                if not results or not all(result.successful for result in results):
                    reason = next(
                        (result.reason for result in results if not result.successful),
                        'parameter update rejected',
                    )
                    request.result = {'ok': False, 'message': reason}
                    request.event.set()
                    return
            except Exception as exc:  # pragma: no cover - ROS transport dependent
                request.result = {'ok': False, 'message': str(exc)}
                request.event.set()
                return
            self._parameter_service_chain(request, stages, index + 1)

        future.add_done_callback(complete)

    @staticmethod
    def _to_ros_parameter(name: str, value: Any) -> Parameter:
        parameter = Parameter()
        parameter.name = name
        parameter.value = ParameterValue()
        if isinstance(value, bool):
            parameter.value.type = ParameterType.PARAMETER_BOOL
            parameter.value.bool_value = value
        else:
            parameter.value.type = ParameterType.PARAMETER_DOUBLE
            parameter.value.double_value = float(value)
        return parameter

    def destroy_node(self):
        """Stop the HTTP server before releasing ROS resources."""
        try:
            self._server.shutdown()
            self._server.server_close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    """Run the research station-keeping dashboard."""
    rclpy.init(args=args)
    node = StationKeepingDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
