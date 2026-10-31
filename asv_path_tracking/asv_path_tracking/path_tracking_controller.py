"""Standalone smooth path-tracking controller for the research catamaran."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, Float64

from asv_path_tracking.path_control_model import (
    mix_thrusters,
    move_toward,
    tilt_scale,
    wrapped_angle,
)


ODOM_TOPIC = '/model/research_catamaran/odometry'
REFERENCE_POSE_TOPIC = '/asv_path_tracking/reference_pose'
REFERENCE_TWIST_TOPIC = '/asv_path_tracking/reference_twist'
REFERENCE_PATH_TOPIC = '/asv_path_tracking/reference_path'
CURRENT_TOPIC = '/ocean_current'
STATUS_TOPIC = '/asv_path_tracking/status'
RESET_TOPIC = '/asv_path_tracking/reset'

THRUSTER_TOPICS = {
    'port_front': (
        '/model/research_catamaran/joint/'
        'port_front_propeller_joint/cmd_thrust'),
    'starboard_front': (
        '/model/research_catamaran/joint/'
        'starboard_front_propeller_joint/cmd_thrust'),
    'port_rear': (
        '/model/research_catamaran/joint/'
        'port_rear_propeller_joint/cmd_thrust'),
    'starboard_rear': (
        '/model/research_catamaran/joint/'
        'starboard_rear_propeller_joint/cmd_thrust'),
}


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return wrapped_angle(angle)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw from a quaternion."""
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def roll_pitch_from_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
) -> Tuple[float, float]:
    """Return roll and pitch from a quaternion."""
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sin_pitch) \
        if abs(sin_pitch) >= 1.0 else math.asin(sin_pitch)
    return roll, pitch


class PathTrackingController(Node):
    """Follow a time-varying planar pose with bounded thruster commands."""

    def __init__(self) -> None:
        super().__init__('asv_path_tracking_controller')
        self.declare_parameter('control_rate', 25.0)
        self.declare_parameter('max_thrust', 3.0)
        self.declare_parameter('kp_x', 2.2)
        self.declare_parameter('ki_x', 0.05)
        self.declare_parameter('kd_x', 1.6)
        self.declare_parameter('kp_y', 2.2)
        self.declare_parameter('ki_y', 0.05)
        self.declare_parameter('kd_y', 1.6)
        self.declare_parameter('kp_yaw', 1.4)
        self.declare_parameter('ki_yaw', 0.03)
        self.declare_parameter('kd_yaw', 0.45)
        self.declare_parameter('integral_limit', 0.40)
        self.declare_parameter('integral_leak', 0.05)
        self.declare_parameter('slew_rate', 1.5)
        self.declare_parameter('velocity_filter_tau', 0.15)
        self.declare_parameter('reference_timeout', 1.0)
        self.declare_parameter('lookahead_distance', 0.25)
        self.declare_parameter('odom_timeout', 1.0)
        self.declare_parameter('enable_current_compensation', True)
        self.declare_parameter('enable_yaw_control', True)
        self.declare_parameter('enable_tilt_guard', True)
        self.declare_parameter('tilt_soft_limit', 0.10)
        self.declare_parameter('tilt_hard_limit', 0.35)
        self.declare_parameter('status_rate', 2.0)
        self.declare_parameter('log_file', '/tmp/asv_path_tracking.csv')

        names = (
            'control_rate', 'max_thrust', 'kp_x', 'ki_x', 'kd_x', 'kp_y',
            'ki_y', 'kd_y', 'kp_yaw', 'ki_yaw', 'kd_yaw', 'integral_limit',
            'integral_leak', 'slew_rate', 'velocity_filter_tau',
            'reference_timeout', 'lookahead_distance', 'odom_timeout',
            'tilt_soft_limit',
            'tilt_hard_limit', 'status_rate',
        )
        for name in names:
            setattr(self, name, float(self.get_parameter(name).value))
        self.max_thrust = max(self.max_thrust, 0.0)
        self.integral_limit = max(self.integral_limit, 0.0)
        self.integral_leak = max(self.integral_leak, 0.0)
        self.slew_rate = max(self.slew_rate, 0.0)
        self.velocity_filter_tau = max(self.velocity_filter_tau, 0.0)
        self.enable_current_compensation = bool(
            self.get_parameter('enable_current_compensation').value)
        self.enable_yaw_control = bool(
            self.get_parameter('enable_yaw_control').value)
        self.enable_tilt_guard = bool(
            self.get_parameter('enable_tilt_guard').value)

        self.latest_odom: Optional[Odometry] = None
        self.reference_pose: Optional[PoseStamped] = None
        self.reference_twist: Optional[TwistStamped] = None
        self.reference_path: Optional[Path] = None
        self.reference_path_signature = None
        self.nearest_path_index: Optional[int] = None
        self.current = (0.0, 0.0, 0.0)
        self.last_odom_wall: Optional[float] = None
        self.last_reference_wall: Optional[float] = None
        self.last_reference_path_wall: Optional[float] = None
        self.last_time: Optional[float] = None
        self.last_status_time: Optional[float] = None
        self.experiment_start_time: Optional[float] = None
        self.filtered_velocity: Optional[Tuple[float, float, float]] = None
        self.integrals = [0.0, 0.0, 0.0]
        self.output_commands = (0.0, 0.0, 0.0, 0.0)
        self.last_wrench = (0.0, 0.0, 0.0)
        self.last_errors = (0.0, 0.0, 0.0)
        self.last_metrics: Dict[str, float] = {}
        self.thrust_saturated = False
        self.tilt_guard_active = False
        self.csv_file = None
        self.csv_writer = None
        self._open_log()

        self.thrust_publishers = {
            name: self.create_publisher(Float64, topic, 10)
            for name, topic in THRUSTER_TOPICS.items()
        }
        self.status_pub = self.create_publisher(
            DiagnosticArray, STATUS_TOPIC, 10)
        self.odom_sub = self.create_subscription(
            Odometry, ODOM_TOPIC, self.odom_callback, 10)
        self.reference_pose_sub = self.create_subscription(
            PoseStamped, REFERENCE_POSE_TOPIC, self.reference_pose_callback, 10)
        self.reference_twist_sub = self.create_subscription(
            TwistStamped,
            REFERENCE_TWIST_TOPIC,
            self.reference_twist_callback,
            10,
        )
        self.reference_path_sub = self.create_subscription(
            Path,
            REFERENCE_PATH_TOPIC,
            self.reference_path_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.current_sub = self.create_subscription(
            Vector3, CURRENT_TOPIC, self.current_callback, 10)
        self.reset_sub = self.create_subscription(
            Empty, RESET_TOPIC, self.reset_callback, 10)
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.timer = self.create_timer(
            1.0 / max(self.control_rate, 1.0), self.control_step)
        self.get_logger().info('Standalone path-tracking controller ready.')

    def _open_log(self) -> None:
        """Create a complete, flush-on-write research CSV if requested."""
        path_text = str(self.get_parameter('log_file').value).strip()
        if not path_text:
            return
        path = Path(path_text).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = path.open('w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time', 'path_x', 'path_y', 'path_yaw', 'x', 'y', 'yaw',
            'cross_track_error', 'along_track_error', 'heading_error',
            'ref_speed', 'u', 'v', 'r', 'current_x', 'current_y',
            'surge_command', 'sway_command', 'yaw_command',
            'thruster_port_front', 'thruster_starboard_front',
            'thruster_port_rear', 'thruster_starboard_rear',
            'roll', 'pitch', 'tilt_scale', 'thrust_saturated',
        ])

    def parameter_callback(self, parameters) -> SetParametersResult:
        """Reject unsafe live parameter values."""
        float_names = {
            'control_rate', 'max_thrust', 'kp_x', 'ki_x', 'kd_x', 'kp_y',
            'ki_y', 'kd_y', 'kp_yaw', 'ki_yaw', 'kd_yaw',
            'integral_limit', 'integral_leak', 'slew_rate',
            'velocity_filter_tau', 'reference_timeout', 'odom_timeout',
            'tilt_soft_limit', 'tilt_hard_limit', 'status_rate',
        }
        minimums = {
            'control_rate': 1.0,
            'max_thrust': 0.0,
            'integral_limit': 0.0,
            'integral_leak': 0.0,
            'slew_rate': 0.0,
            'velocity_filter_tau': 0.0,
            'reference_timeout': 0.1,
            'odom_timeout': 0.1,
            'tilt_soft_limit': 0.0,
            'tilt_hard_limit': 0.01,
            'status_rate': 0.1,
        }
        updates: Dict[str, Any] = {}
        for parameter in parameters:
            if parameter.name in float_names:
                try:
                    value = float(parameter.value)
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False,
                        reason=f'{parameter.name} must be numeric',
                    )
                if not math.isfinite(value) or value < minimums.get(
                        parameter.name, -math.inf):
                    return SetParametersResult(
                        successful=False,
                        reason=f'{parameter.name} is outside its safe range',
                    )
                updates[parameter.name] = value
            elif parameter.name in {
                'enable_current_compensation', 'enable_yaw_control',
                'enable_tilt_guard',
            }:
                updates[parameter.name] = bool(parameter.value)
            elif parameter.name == 'log_file':
                return SetParametersResult(
                    successful=False,
                    reason='log_file is restart-only',
                )
        hard = updates.get('tilt_hard_limit', self.tilt_hard_limit)
        soft = updates.get('tilt_soft_limit', self.tilt_soft_limit)
        if hard <= soft:
            return SetParametersResult(
                successful=False,
                reason='tilt_hard_limit must exceed tilt_soft_limit',
            )
        for name, value in updates.items():
            setattr(self, name, value)
        if self.max_thrust <= 0.0:
            self.output_commands = (0.0, 0.0, 0.0, 0.0)
            self._publish_commands(self.output_commands)
        return SetParametersResult(successful=True, reason='updated')

    def odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message
        self.last_odom_wall = self._wall_time()

    def reference_pose_callback(self, message: PoseStamped) -> None:
        self.reference_pose = message
        self.last_reference_wall = self._wall_time()

    def reference_twist_callback(self, message: TwistStamped) -> None:
        self.reference_twist = message
        self.last_reference_wall = self._wall_time()

    def reference_path_callback(self, message: Path) -> None:
        signature = self._path_signature(message)
        if signature != self.reference_path_signature:
            self.nearest_path_index = None
            self.reference_path_signature = signature
        self.reference_path = message
        self.last_reference_path_wall = self._wall_time()

    def reset_callback(self, _message: Empty) -> None:
        """Reset dynamic controller state for an independent trial."""
        self.integrals = [0.0, 0.0, 0.0]
        self.filtered_velocity = None
        self.output_commands = (0.0, 0.0, 0.0, 0.0)
        self.last_time = None
        self.experiment_start_time = None
        self.nearest_path_index = None
        self._publish_commands(self.output_commands)
        self.get_logger().info('Path-tracking controller state reset.')

    def current_callback(self, message: Vector3) -> None:
        values = (float(message.x), float(message.y), float(message.z))
        if all(math.isfinite(value) for value in values):
            self.current = values

    @staticmethod
    def _wall_time() -> float:
        """Import wall time only for stale-receipt checks."""
        import time
        return time.monotonic()

    def _sim_time(self) -> Optional[float]:
        value = self.get_clock().now().nanoseconds * 1e-9
        return value if value > 0.0 else None

    def control_step(self) -> None:
        """Run one bounded, stale-safe path-tracking update."""
        now = self._sim_time()
        if now is None:
            self._publish_zero('waiting for simulation clock')
            return
        if self.last_time is None:
            dt = 1.0 / max(self.control_rate, 1.0)
        else:
            dt = min(max(now - self.last_time, 1e-3), 0.2)
        self.last_time = now
        if self.experiment_start_time is None:
            self.experiment_start_time = now

        if self.latest_odom is None or self.reference_pose is None:
            self._publish_zero('waiting for odometry and path reference')
            return
        if self._stale(self.last_odom_wall, self.odom_timeout):
            self._publish_zero('odometry is stale')
            return
        if self._stale(self.last_reference_wall, self.reference_timeout):
            self._publish_zero('path reference is stale')
            return

        pose = self.latest_odom.pose.pose
        twist = self.latest_odom.twist.twist
        yaw = yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        roll, pitch = roll_pitch_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        reference = self._selected_reference(pose)
        reference_yaw = yaw_from_quaternion(
            reference.orientation.x,
            reference.orientation.y,
            reference.orientation.z,
            reference.orientation.w,
        )
        dx = reference.position.x - pose.position.x
        dy = reference.position.y - pose.position.y
        ex = math.cos(yaw) * dx + math.sin(yaw) * dy
        ey = -math.sin(yaw) * dx + math.cos(yaw) * dy
        eyaw = wrap_angle(reference_yaw - yaw)

        ref_u, ref_v, ref_r = self._reference_velocity(yaw)
        current_u, current_v = self._world_to_body(yaw, self.current)
        if self.enable_current_compensation:
            ref_u -= current_u
            ref_v -= current_v
        filtered = self._filter_velocity(
            twist.linear.x,
            twist.linear.y,
            twist.angular.z,
            dt,
        )
        self.integrals[0] = self._integrate(self.integrals[0], ex, dt)
        self.integrals[1] = self._integrate(self.integrals[1], ey, dt)
        self.integrals[2] = self._integrate(self.integrals[2], eyaw, dt)

        surge = (
            self.kp_x * ex
            + self.ki_x * self.integrals[0]
            + self.kd_x * (ref_u - filtered[0])
        )
        sway = (
            self.kp_y * ey
            + self.ki_y * self.integrals[1]
            + self.kd_y * (ref_v - filtered[1])
        )
        yaw_command = 0.0
        if self.enable_yaw_control:
            yaw_command = (
                self.kp_yaw * eyaw
                + self.ki_yaw * self.integrals[2]
                + self.kd_yaw * (ref_r - filtered[2])
            )

        raw = mix_thrusters(surge, sway, yaw_command)
        peak = max(abs(value) for value in raw)
        if self.max_thrust <= 0.0:
            raw = (0.0, 0.0, 0.0, 0.0)
            self.thrust_saturated = peak > 0.0
        else:
            self.thrust_saturated = peak > self.max_thrust
        if self.max_thrust > 0.0 and peak > self.max_thrust:
            raw = tuple(value * self.max_thrust / peak for value in raw)
        tilt_scale = self._tilt_scale(roll, pitch)
        self.tilt_guard_active = tilt_scale < 0.999999
        target = tuple(value * tilt_scale for value in raw)
        if self.max_thrust <= 0.0:
            commands = (0.0, 0.0, 0.0, 0.0)
        else:
            maximum_change = max(self.slew_rate, 0.0) * dt
            commands = tuple(
                self._move_toward(previous, desired, maximum_change)
                for previous, desired in zip(self.output_commands, target)
            )
        self.output_commands = commands
        self.last_wrench = (surge, sway, yaw_command)
        self.last_errors = (ex, ey, eyaw)
        position_error = math.hypot(dx, dy)
        self.last_metrics = {
            'cross_track_error_m': ey,
            'along_track_error_m': ex,
            'position_error_m': position_error,
            'heading_error_rad': eyaw,
            'reference_speed_mps': math.hypot(ref_u, ref_v),
            'actual_speed_mps': math.hypot(twist.linear.x, twist.linear.y),
            'roll_rad': roll,
            'pitch_rad': pitch,
            'tilt_scale': tilt_scale,
        }
        self._publish_commands(commands)
        self._publish_status(now, pose, twist, yaw, roll, pitch, position_error)
        self._write_log(
            now,
            reference,
            pose,
            yaw,
            twist,
            roll,
            pitch,
            tilt_scale,
            commands,
        )

    def _reference_velocity(self, yaw: float) -> Tuple[float, float, float]:
        if self.reference_twist is None:
            return 0.0, 0.0, 0.0
        velocity = self.reference_twist.twist.linear
        body_u, body_v = self._world_to_body(
            yaw,
            (velocity.x, velocity.y, velocity.z),
        )
        return body_u, body_v, self.reference_twist.twist.angular.z

    def _selected_reference(self, pose: Any) -> Any:
        """Select a geometric look-ahead pose from the durable path."""
        if self.reference_path is None or len(self.reference_path.poses) < 3:
            return self.reference_pose.pose
        if self._stale(self.last_reference_path_wall, self.reference_timeout):
            return self.reference_pose.pose
        poses = self.reference_path.poses
        cycle_length = len(poses) - 1
        if self.nearest_path_index is None:
            candidates = range(cycle_length)
        else:
            window = max(20, int(0.12 * cycle_length))
            candidates = {
                (self.nearest_path_index + offset) % cycle_length
                for offset in range(-window, window + 1)
            }
        nearest_index = min(
            candidates,
            key=lambda index: (
                (poses[index].pose.position.x - pose.position.x) ** 2
                + (poses[index].pose.position.y - pose.position.y) ** 2
            ),
        )
        self.nearest_path_index = nearest_index
        spacing = 0.0
        if nearest_index + 1 < len(poses):
            first = poses[nearest_index].pose.position
            second = poses[nearest_index + 1].pose.position
            spacing = math.hypot(
                second.x - first.x,
                second.y - first.y,
            )
        steps = max(
            int(round(self.lookahead_distance / max(spacing, 1e-3))),
            1,
        )
        selected = (nearest_index + steps) % cycle_length
        return poses[selected].pose

    @staticmethod
    def _world_to_body(
        yaw: float,
        vector: Tuple[float, float, float],
    ) -> Tuple[float, float]:
        return (
            math.cos(yaw) * vector[0] + math.sin(yaw) * vector[1],
            -math.sin(yaw) * vector[0] + math.cos(yaw) * vector[1],
        )

    def _filter_velocity(
        self,
        surge: float,
        sway: float,
        yaw_rate: float,
        dt: float,
    ) -> Tuple[float, float, float]:
        values = (float(surge), float(sway), float(yaw_rate))
        if self.filtered_velocity is None:
            self.filtered_velocity = values
            return values
        alpha = 1.0 if self.velocity_filter_tau <= 1e-9 else (
            dt / (self.velocity_filter_tau + dt))
        self.filtered_velocity = tuple(
            previous + alpha * (value - previous)
            for previous, value in zip(self.filtered_velocity, values)
        )
        return self.filtered_velocity

    def _integrate(self, value: float, error: float, dt: float) -> float:
        decayed = value * max(0.0, 1.0 - self.integral_leak * dt)
        return min(
            max(decayed + error * dt, -self.integral_limit),
            self.integral_limit,
        )

    def _tilt_scale(self, roll: float, pitch: float) -> float:
        return tilt_scale(
            roll,
            pitch,
            self.tilt_soft_limit,
            self.tilt_hard_limit,
            self.enable_tilt_guard,
        )

    def _publish_zero(self, reason: str) -> None:
        self.output_commands = tuple(
            self._move_toward(value, 0.0, max(self.slew_rate, 0.0) * 0.04)
            for value in self.output_commands
        )
        self._publish_commands(self.output_commands)
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.ERROR
        status.name = 'asv_path_tracking/controller'
        status.message = reason
        self._publish_diagnostic(status)

    def _publish_commands(self, commands: Tuple[float, ...]) -> None:
        for name, value in zip(THRUSTER_TOPICS, commands):
            message = Float64()
            message.data = float(value)
            self.thrust_publishers[name].publish(message)

    def _publish_status(
        self,
        now: float,
        pose: Any,
        twist: Any,
        yaw: float,
        roll: float,
        pitch: float,
        position_error: float,
    ) -> None:
        if (
            self.last_status_time is not None
            and now - self.last_status_time < 1.0 / max(self.status_rate, 0.1)
        ):
            return
        self.last_status_time = now
        status = DiagnosticStatus()
        status.name = 'asv_path_tracking/controller'
        status.level = DiagnosticStatus.OK
        status.message = 'path tracking active'
        if self.tilt_guard_active or self.thrust_saturated:
            status.level = DiagnosticStatus.WARN
            status.message = 'path tracking safety limiting'
        if self.last_metrics['tilt_scale'] <= 0.0:
            status.level = DiagnosticStatus.ERROR
            status.message = 'path tracking stopped by tilt guard'
        values: Dict[str, Any] = {
            'controller_state': status.message,
            'x_m': pose.position.x,
            'y_m': pose.position.y,
            'z_m': pose.position.z,
            'yaw_rad': yaw,
            'roll_rad': roll,
            'pitch_rad': pitch,
            'u_mps': twist.linear.x,
            'v_mps': twist.linear.y,
            'r_radps': twist.angular.z,
            'cross_track_error_m': self.last_errors[1],
            'along_track_error_m': self.last_errors[0],
            'position_error_m': position_error,
            'heading_error_rad': self.last_errors[2],
            'current_x_mps': self.current[0],
            'current_y_mps': self.current[1],
            'surge_command_N': self.last_wrench[0],
            'sway_command_N': self.last_wrench[1],
            'yaw_command_Nm': self.last_wrench[2],
            'thruster_port_front_N': self.output_commands[0],
            'thruster_starboard_front_N': self.output_commands[1],
            'thruster_port_rear_N': self.output_commands[2],
            'thruster_starboard_rear_N': self.output_commands[3],
            'tilt_scale': self.last_metrics['tilt_scale'],
            'thrust_saturated': self.thrust_saturated,
            'tilt_guard_active': self.tilt_guard_active,
            'max_thrust_N': self.max_thrust,
            'slew_rate_Nps': self.slew_rate,
            'kp_x': self.kp_x,
            'ki_x': self.ki_x,
            'kd_x': self.kd_x,
            'kp_y': self.kp_y,
            'ki_y': self.ki_y,
            'kd_y': self.kd_y,
            'kp_yaw': self.kp_yaw,
            'ki_yaw': self.ki_yaw,
            'kd_yaw': self.kd_yaw,
            'integral_limit': self.integral_limit,
            'integral_leak': self.integral_leak,
            'velocity_filter_tau': self.velocity_filter_tau,
            'lookahead_distance_m': self.lookahead_distance,
            'current_compensation': self.enable_current_compensation,
            'yaw_control': self.enable_yaw_control,
            'tilt_guard': self.enable_tilt_guard,
            'tilt_soft_limit_rad': self.tilt_soft_limit,
            'tilt_hard_limit_rad': self.tilt_hard_limit,
        }
        status.values = [
            KeyValue(key=name, value=str(value))
            for name, value in values.items()
        ]
        self._publish_diagnostic(status)

    def _publish_diagnostic(self, status: DiagnosticStatus) -> None:
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.status_pub.publish(message)

    def _write_log(
        self,
        now: float,
        reference: Any,
        pose: Any,
        yaw: float,
        twist: Any,
        roll: float,
        pitch: float,
        tilt_scale: float,
        commands: Tuple[float, ...],
    ) -> None:
        if self.csv_writer is None or self.csv_file is None:
            return
        reference_yaw = yaw_from_quaternion(
            reference.orientation.x,
            reference.orientation.y,
            reference.orientation.z,
            reference.orientation.w,
        )
        self.csv_writer.writerow([
            f'{now:.6f}',
            f'{reference.position.x:.6f}',
            f'{reference.position.y:.6f}',
            f'{reference_yaw:.6f}',
            f'{pose.position.x:.6f}',
            f'{pose.position.y:.6f}',
            f'{yaw:.6f}',
            f'{self.last_errors[1]:.6f}',
            f'{self.last_errors[0]:.6f}',
            f'{self.last_errors[2]:.6f}',
            f'{self.last_metrics.get("reference_speed_mps", 0.0):.6f}',
            f'{twist.linear.x:.6f}',
            f'{twist.linear.y:.6f}',
            f'{twist.angular.z:.6f}',
            f'{self.current[0]:.6f}',
            f'{self.current[1]:.6f}',
            f'{self.last_wrench[0]:.6f}',
            f'{self.last_wrench[1]:.6f}',
            f'{self.last_wrench[2]:.6f}',
            *(f'{value:.6f}' for value in commands),
            f'{roll:.6f}',
            f'{pitch:.6f}',
            f'{tilt_scale:.6f}',
            str(self.thrust_saturated),
        ])
        self.csv_file.flush()

    @staticmethod
    def _move_toward(current: float, target: float, maximum: float) -> float:
        return move_toward(current, target, maximum)

    @staticmethod
    def _stale(received_at: Optional[float], timeout: float) -> bool:
        if received_at is None:
            return True
        return PathTrackingController._wall_time() - received_at > timeout

    @staticmethod
    def _path_signature(message: Path):
        """Identify a rebuilt path without resetting on every durable publish."""
        if not message.poses:
            return (0,)
        geometry = tuple(
            (
                round(pose.pose.position.x, 6),
                round(pose.pose.position.y, 6),
                round(pose.pose.orientation.z, 6),
                round(pose.pose.orientation.w, 6),
            )
            for pose in message.poses
        )
        return len(message.poses), hash(geometry)

    def destroy_node(self) -> bool:
        """Stop thrust and close the research log cleanly."""
        self.output_commands = (0.0, 0.0, 0.0, 0.0)
        self._publish_commands(self.output_commands)
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the standalone path-tracking controller."""
    rclpy.init(args=args)
    node = PathTrackingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
