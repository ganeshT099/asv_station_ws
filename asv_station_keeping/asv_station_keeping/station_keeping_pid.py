"""Small, deterministic 3-DOF station-keeping baseline for the ASV."""

import csv
import math
from pathlib import Path
from typing import Any, cast

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Empty, Float64

from asv_station_keeping.twist_to_thruster import (
    FORCE_PER_UNIT, THRUSTER_AXES, allocate)


def yaw_from_quaternion(x, y, z, w):
    """Return planar yaw from a ROS quaternion."""
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def roll_pitch_from_quaternion(x, y, z, w):
    """Return roll and pitch from a ROS quaternion."""
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sin_pitch) \
        if abs(sin_pitch) >= 1.0 else math.asin(sin_pitch)
    return roll, pitch


class StationKeepingPid(Node):
    """Publish four diagonal-thruster commands from odometry feedback."""

    def __init__(self):
        super().__init__('research_station_keeping_pid')

        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_yaw', 0.0)
        self.declare_parameter('target_slew_rate', 0.25)
        self.declare_parameter('target_yaw_slew_rate', 0.35)
        self.declare_parameter('max_thrust', 4.5)
        self.declare_parameter('kp_x', 8.0)
        self.declare_parameter('kd_x', 5.0)
        self.declare_parameter('ki_x', 0.15)
        self.declare_parameter('kp_y', 8.0)
        self.declare_parameter('kd_y', 5.0)
        self.declare_parameter('ki_y', 0.15)
        self.declare_parameter('kp_yaw', 5.0)
        self.declare_parameter('kd_yaw', 2.5)
        self.declare_parameter('ki_yaw', 0.05)
        self.declare_parameter('integral_limit', 0.5)
        self.declare_parameter('integral_leak', 0.05)
        self.declare_parameter('slew_rate', 3.0)
        self.declare_parameter('velocity_filter_tau', 0.15)
        self.declare_parameter('enable_yaw_control', True)
        self.declare_parameter('enable_tilt_guard', True)
        self.declare_parameter('tilt_soft_limit', 0.10)
        self.declare_parameter('tilt_hard_limit', 0.35)
        self.declare_parameter('status_rate', 2.0)
        self.declare_parameter('position_tolerance', 0.05)
        self.declare_parameter('yaw_tolerance', 0.08)
        self.declare_parameter('current_x', 0.0)
        self.declare_parameter('current_y', 0.0)
        self.declare_parameter('current_z', 0.0)
        self.declare_parameter('log_file', '/tmp/research_catamaran_run.csv')
        # False runs the identical stack with thrusters off: the uncontrolled
        # baseline that shows what the disturbances do to the bare hull.
        self.declare_parameter('hold_enabled', True)
        # Actuator realism (T200-like, scaled to the simulated thrust range):
        # ESC deadband, first-order motor/propeller lag and weaker reverse.
        self.declare_parameter('actuator_model', True)
        self.declare_parameter('thruster_deadband', 0.05)
        self.declare_parameter('thruster_lag_tau', 0.15)
        self.declare_parameter('thruster_reverse_scale', 0.80)
        # Thrust budget reserved for heading so that translation demand
        # cannot saturate the thrusters and leave no yaw authority.
        self.declare_parameter('yaw_reserve', 2.2)
        # Disturbance observer: estimates the lumped external force (wind,
        # current drag, waves) and feeds it forward on top of the PID.
        self.declare_parameter('use_dob', False)
        self.declare_parameter('dob_gain', 1.0)
        self.declare_parameter('dob_tau', 6.0)
        self.declare_parameter('vehicle_mass', 45.0)
        # allocator_health_i: what the controller BELIEVES (fault detection);
        # thruster_gain_i: what the hardware actually delivers.
        for index in range(4):
            self.declare_parameter(f'allocator_health_{index}', 1.0)
            self.declare_parameter(f'thruster_gain_{index}', 1.0)
        # Feedback source.  Point odom_topic at the state estimator output to
        # close the loop on noisy sensors; truth_topic always stays the
        # simulator ground truth and is used only to score performance.
        self.declare_parameter(
            'odom_topic', '/model/research_catamaran/odometry')
        self.declare_parameter(
            'truth_topic', '/model/research_catamaran/odometry')

        self.target_x = self.parameter_float('target_x')
        self.target_y = self.parameter_float('target_y')
        self.target_yaw = self.parameter_float('target_yaw')
        self.target_slew_rate = self.parameter_float('target_slew_rate')
        self.target_yaw_slew_rate = self.parameter_float(
            'target_yaw_slew_rate')
        self.max_thrust = self.parameter_float('max_thrust')
        self.kp_x = self.parameter_float('kp_x')
        self.kd_x = self.parameter_float('kd_x')
        self.ki_x = self.parameter_float('ki_x')
        self.kp_y = self.parameter_float('kp_y')
        self.kd_y = self.parameter_float('kd_y')
        self.ki_y = self.parameter_float('ki_y')
        self.kp_yaw = self.parameter_float('kp_yaw')
        self.kd_yaw = self.parameter_float('kd_yaw')
        self.ki_yaw = self.parameter_float('ki_yaw')
        self.integral_limit = self.parameter_float('integral_limit')
        self.integral_leak = self.parameter_float('integral_leak')
        self.slew_rate = self.parameter_float('slew_rate')
        self.velocity_filter_tau = self.parameter_float('velocity_filter_tau')
        self.enable_yaw_control = bool(
            cast(Any, self.get_parameter('enable_yaw_control').value))
        self.enable_tilt_guard = bool(
            cast(Any, self.get_parameter('enable_tilt_guard').value))
        self.tilt_soft_limit = self.parameter_float('tilt_soft_limit')
        self.tilt_hard_limit = self.parameter_float('tilt_hard_limit')
        self.status_rate = self.parameter_float('status_rate')
        self.position_tolerance = self.parameter_float('position_tolerance')
        self.yaw_tolerance = self.parameter_float('yaw_tolerance')
        self.current_x = self.parameter_float('current_x')
        self.current_y = self.parameter_float('current_y')
        self.current_z = self.parameter_float('current_z')
        self.active_target_x = self.target_x
        self.active_target_y = self.target_y
        self.active_target_yaw = self.target_yaw

        self.hold_enabled = bool(
            cast(Any, self.get_parameter('hold_enabled').value))
        self.actuator_model = bool(
            cast(Any, self.get_parameter('actuator_model').value))
        self.thruster_deadband = self.parameter_float('thruster_deadband')
        self.thruster_lag_tau = self.parameter_float('thruster_lag_tau')
        self.thruster_reverse_scale = self.parameter_float(
            'thruster_reverse_scale')
        self.actuator_state = (0.0, 0.0, 0.0, 0.0)
        self.delivered = (0.0, 0.0, 0.0, 0.0)
        self.yaw_reserve = self.parameter_float('yaw_reserve')
        self.use_dob = bool(cast(Any, self.get_parameter('use_dob').value))
        self.dob_gain = self.parameter_float('dob_gain')
        self.dob_tau = self.parameter_float('dob_tau')
        self.vehicle_mass = self.parameter_float('vehicle_mass')
        self.allocator_health = [
            self.parameter_float(f'allocator_health_{i}') for i in range(4)]
        self.thruster_gain = [
            self.parameter_float(f'thruster_gain_{i}') for i in range(4)]
        self.dob_z = None
        self.dob_estimate = (0.0, 0.0)
        self.power_index = 0.0
        self.energy_index = 0.0
        self.odom = None
        self.truth = None
        self.last_time = None
        self.last_status_time = None
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_yaw = 0.0
        self.filtered_u = None
        self.filtered_v = None
        self.filtered_r = None
        self.output_commands = (0.0, 0.0, 0.0, 0.0)
        self.last_raw_commands = (0.0, 0.0, 0.0, 0.0)
        self.last_control_wrench = (0.0, 0.0, 0.0)
        self.thrust_saturated = False
        self.tilt_guard_active = False
        self.error_squared_sum = 0.0
        self.error_sample_time = 0.0
        self.max_position_error = 0.0
        self.last_status = None
        self.experiment_start_time = None
        self.csv_file = None
        self.csv_writer = None
        log_file = str(self.get_parameter('log_file').value)
        if log_file:
            path = Path(log_file).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            self.csv_file = path.open('w', newline='', encoding='utf-8')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                'time', 'x', 'y', 'yaw', 'target_x', 'target_y',
                'target_yaw', 'current_x', 'current_y', 'current_z',
                'ex_body', 'ey_body', 'eyaw',
                'u_port_front', 'u_starboard_front',
                'u_port_rear', 'u_starboard_rear',
                'roll', 'pitch', 'u', 'v', 'r',
                'integral_x', 'integral_y', 'integral_yaw',
                'position_error', 'position_rmse', 'max_position_error',
                'elapsed_s', 'control_surge_N', 'control_sway_N',
                'control_yaw_Nm', 'tilt_scale', 'thrust_saturated',
                'tilt_guard_active', 'u_filtered', 'v_filtered', 'r_filtered',
                'true_x', 'true_y', 'true_yaw', 'estimate_error',
                'power_index', 'energy_index', 'dob_x', 'dob_y',
            ])

        odom_topic = str(self.get_parameter('odom_topic').value)
        truth_topic = str(self.get_parameter('truth_topic').value)
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)
        self.truth_sub = self.create_subscription(
            Odometry, truth_topic, self.truth_callback, 10)
        self.get_logger().info(
            f'Feedback odometry: {odom_topic}; scoring truth: {truth_topic}')
        self.current_sub = self.create_subscription(
            Vector3,
            '/ocean_current',
            self.current_callback,
            10,
        )
        self.thrust_publishers = {
            'port_front': self.create_publisher(
                Float64,
                '/model/research_catamaran/joint/'
                'port_front_propeller_joint/cmd_thrust',
                10,
            ),
            'starboard_front': self.create_publisher(
                Float64,
                '/model/research_catamaran/joint/'
                'starboard_front_propeller_joint/cmd_thrust',
                10,
            ),
            'port_rear': self.create_publisher(
                Float64,
                '/model/research_catamaran/joint/'
                'port_rear_propeller_joint/cmd_thrust',
                10,
            ),
            'starboard_rear': self.create_publisher(
                Float64,
                '/model/research_catamaran/joint/'
                'starboard_rear_propeller_joint/cmd_thrust',
                10,
            ),
        }
        self.dob_pub = self.create_publisher(
            Vector3, '/research_catamaran/station_keeping/disturbance_estimate',
            10)
        self.status_pub = self.create_publisher(
            DiagnosticArray,
            '/research_catamaran/station_keeping/status',
            10,
        )
        self.reset_sub = self.create_subscription(
            Empty,
            '/research_catamaran/station_keeping/reset_integrators',
            self.reset_integrators_callback,
            10,
        )
        self.add_on_set_parameters_callback(self.on_parameter_change)
        self.timer = self.create_timer(0.05, self.control_step)
        self.get_logger().info(
            'Station keeping target: '
            f'x={self.target_x:.3f}, y={self.target_y:.3f}, '
            f'yaw={self.target_yaw:.3f} rad; '
            f'current=({self.current_x:.3f}, {self.current_y:.3f}) m/s')

    def parameter_float(self, name):
        """Read a numeric ROS parameter without weakening runtime checks."""
        value = cast(Any, self.get_parameter(name).value)
        return float(value)

    def odom_callback(self, message):
        self.odom = message

    def truth_callback(self, message):
        self.truth = message

    def current_callback(self, message):
        """Keep displayed disturbance values synchronized with the bridge."""
        values = (message.x, message.y, message.z)
        try:
            values = tuple(float(value) for value in values)
        except (TypeError, ValueError):
            return
        if all(math.isfinite(value) for value in values):
            self.current_x, self.current_y, self.current_z = values

    def control_step(self):
        if self.odom is None:
            self.publish_commands((0.0, 0.0, 0.0, 0.0))
            self.publish_status(None)
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.experiment_start_time is None:
            self.experiment_start_time = now
        if self.last_time is None:
            dt = 0.05
        else:
            dt = min(max(now - self.last_time, 1e-3), 0.2)
        self.last_time = now
        pose = self.odom.pose.pose
        twist = self.odom.twist.twist
        self.active_target_x = self.move_toward(
            self.active_target_x,
            self.target_x,
            max(self.target_slew_rate, 0.0) * dt,
        )
        self.active_target_y = self.move_toward(
            self.active_target_y,
            self.target_y,
            max(self.target_slew_rate, 0.0) * dt,
        )
        self.active_target_yaw = self.move_angle_toward(
            self.active_target_yaw,
            self.target_yaw,
            max(self.target_yaw_slew_rate, 0.0) * dt,
        )
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

        dx = self.active_target_x - pose.position.x
        dy = self.active_target_y - pose.position.y
        ex_body = math.cos(yaw) * dx + math.sin(yaw) * dy
        ey_body = -math.sin(yaw) * dx + math.cos(yaw) * dy
        eyaw = wrap_angle(self.active_target_yaw - yaw)

        # Conditional integration: no accumulation while the thrusters are
        # saturated, which otherwise winds the integrator up and causes a
        # slow limit cycle.
        wind = 0.0 if self.thrust_saturated else 1.0
        self.integral_x = self.integrate_error(
            self.integral_x, ex_body * wind, dt)
        self.integral_y = self.integrate_error(
            self.integral_y, ey_body * wind, dt)
        self.integral_yaw = self.integrate_error(
            self.integral_yaw, eyaw * wind, dt)
        filtered_velocity = self.filter_velocity(
            twist.linear.x,
            twist.linear.y,
            twist.angular.z,
            dt,
        )
        filtered_u, filtered_v, filtered_r = filtered_velocity
        # Performance is scored on ground truth so a poor estimate cannot hide
        # a real position error; the loop itself runs on the feedback pose.
        scored = self.truth.pose.pose if self.truth is not None else pose
        true_dx = self.active_target_x - scored.position.x
        true_dy = self.active_target_y - scored.position.y
        position_error = math.hypot(true_dx, true_dy)
        true_yaw = yaw_from_quaternion(
            scored.orientation.x, scored.orientation.y,
            scored.orientation.z, scored.orientation.w)
        estimate_error = math.hypot(
            pose.position.x - scored.position.x,
            pose.position.y - scored.position.y)
        self.error_squared_sum += position_error * position_error * dt
        self.error_sample_time += dt
        position_rmse = math.sqrt(
            self.error_squared_sum / max(self.error_sample_time, 1e-6))
        self.max_position_error = max(
            self.max_position_error, position_error)

        ux = (
            self.kp_x * ex_body
            + self.ki_x * self.integral_x
            - self.kd_x * filtered_u
        )
        uy = (
            self.kp_y * ey_body
            + self.ki_y * self.integral_y
            - self.kd_y * filtered_v
        )
        uz = 0.0
        if self.enable_yaw_control:
            uz = (
                self.kp_yaw * eyaw
                + self.ki_yaw * self.integral_yaw
                - self.kd_yaw * filtered_r
            )

        dob_world = self.update_observer(
            yaw, filtered_u, filtered_v, dt)
        if self.use_dob:
            # Feed the estimated disturbance forward, in body axes.
            fx_body = math.cos(yaw) * dob_world[0] + math.sin(yaw) * dob_world[1]
            fy_body = -math.sin(yaw) * dob_world[0] + math.cos(yaw) * dob_world[1]
            ux -= self.dob_gain * fx_body / FORCE_PER_UNIT
            uy -= self.dob_gain * fy_body / FORCE_PER_UNIT

        # Prioritised saturation.  Every thruster carries |ux| + |uy| + |uz|
        # in the worst case, so cap translation and yaw separately to keep a
        # guaranteed share of the thrust budget for heading.
        yaw_limit = min(max(self.yaw_reserve, 0.0), self.max_thrust)
        translation_limit = max(self.max_thrust - yaw_limit, 1e-6)
        translation = abs(ux) + abs(uy)
        clipped = False
        if translation > translation_limit:
            ux *= translation_limit / translation
            uy *= translation_limit / translation
            clipped = True
        if abs(uz) > yaw_limit:
            uz = math.copysign(yaw_limit, uz)
            clipped = True

        # Keep algorithm experiments and manual maneuvers on one canonical
        # four-thruster sign convention.  The allocator drops any thruster
        # the fault detector has marked failed.
        self.last_control_wrench = (ux, uy, uz)
        raw_commands = allocate(ux, uy, uz, self.allocator_health)
        commands = raw_commands
        scale = max(1.0, max(abs(command) for command in commands))
        self.thrust_saturated = clipped or scale > self.max_thrust
        if scale > self.max_thrust:
            commands = tuple(
                command * self.max_thrust / scale for command in commands)

        tilt_scale = self.tilt_scale(roll, pitch)
        self.tilt_guard_active = tilt_scale < 0.999999
        commands = tuple(command * tilt_scale for command in commands)
        max_delta = max(self.slew_rate, 0.0) * dt
        commands = tuple(
            self.move_toward(previous, command, max_delta)
            for previous, command in zip(self.output_commands, commands)
        )
        if not self.hold_enabled:
            commands = (0.0, 0.0, 0.0, 0.0)
        self.output_commands = commands
        self.last_raw_commands = raw_commands

        self.delivered = self.apply_actuator(commands, dt)
        self.publish_commands(self.delivered)
        self.power_index = sum(abs(c) ** 1.5 for c in self.delivered)
        self.energy_index += self.power_index * dt
        self.publish_status({
            'now': now,
            'pose': pose,
            'yaw': yaw,
            'roll': roll,
            'pitch': pitch,
            'twist': twist,
            'ex_body': ex_body,
            'ey_body': ey_body,
            'eyaw': eyaw,
            'position_error': position_error,
            'estimate_error': estimate_error,
            'tilt_scale': tilt_scale,
            'commands': commands,
            'raw_commands': raw_commands,
            'elapsed_s': now - self.experiment_start_time,
            'thrust_saturated': self.thrust_saturated,
            'tilt_guard_active': self.tilt_guard_active,
        })
        if self.csv_writer is not None and self.csv_file is not None:
            self.csv_writer.writerow([
                f'{now:.6f}',
                f'{pose.position.x:.6f}',
                f'{pose.position.y:.6f}',
                f'{yaw:.6f}',
                f'{self.target_x:.6f}',
                f'{self.target_y:.6f}',
                f'{self.target_yaw:.6f}',
                f'{self.current_x:.6f}',
                f'{self.current_y:.6f}',
                f'{self.current_z:.6f}',
                f'{ex_body:.6f}',
                f'{ey_body:.6f}',
                f'{eyaw:.6f}',
                *(f'{command:.6f}' for command in commands),
                f'{roll:.6f}',
                f'{pitch:.6f}',
                f'{twist.linear.x:.6f}',
                f'{twist.linear.y:.6f}',
                f'{twist.angular.z:.6f}',
                f'{self.integral_x:.6f}',
                f'{self.integral_y:.6f}',
                f'{self.integral_yaw:.6f}',
                f'{position_error:.6f}',
                f'{position_rmse:.6f}',
                f'{self.max_position_error:.6f}',
                f'{now - self.experiment_start_time:.6f}',
                f'{ux:.6f}',
                f'{uy:.6f}',
                f'{uz:.6f}',
                f'{tilt_scale:.6f}',
                str(self.thrust_saturated),
                str(self.tilt_guard_active),
                f'{filtered_u:.6f}',
                f'{filtered_v:.6f}',
                f'{filtered_r:.6f}',
                f'{scored.position.x:.6f}',
                f'{scored.position.y:.6f}',
                f'{true_yaw:.6f}',
                f'{estimate_error:.6f}',
                f'{self.power_index:.6f}',
                f'{self.energy_index:.6f}',
                f'{self.dob_estimate[0]:.6f}',
                f'{self.dob_estimate[1]:.6f}',
            ])
            self.csv_file.flush()

    def integrate_error(self, value, error, dt):
        """Integrate one error channel with a symmetric anti-windup limit."""
        limit = max(self.integral_limit, 0.0)
        leak = max(self.integral_leak, 0.0)
        decayed = value * max(0.0, 1.0 - leak * dt)
        return min(max(decayed + error * dt, -limit), limit)

    def filter_velocity(self, surge, sway, yaw_rate, dt):
        """Low-pass noisy velocity feedback before derivative action."""
        values = (float(surge), float(sway), float(yaw_rate))
        if self.filtered_u is None:
            self.filtered_u, self.filtered_v, self.filtered_r = values
            return values
        tau = max(self.velocity_filter_tau, 0.0)
        alpha = 1.0 if tau <= 1e-9 else dt / (tau + dt)
        self.filtered_u += alpha * (values[0] - self.filtered_u)
        self.filtered_v += alpha * (values[1] - self.filtered_v)
        self.filtered_r += alpha * (values[2] - self.filtered_r)
        return self.filtered_u, self.filtered_v, self.filtered_r

    def reset_integrators_callback(self, _message):
        """Reset accumulated errors without restarting the experiment."""
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_yaw = 0.0
        self.get_logger().info('Station-keeping integrators reset.')

    def on_parameter_change(self, parameters):
        """Apply safe controller updates requested by the live dashboard."""
        float_names = {
            'target_x', 'target_y', 'target_yaw', 'target_slew_rate',
            'target_yaw_slew_rate', 'max_thrust',
            'kp_x', 'kd_x', 'ki_x', 'kp_y', 'kd_y', 'ki_y',
            'kp_yaw', 'kd_yaw', 'ki_yaw', 'integral_limit', 'slew_rate',
            'integral_leak', 'velocity_filter_tau',
            'tilt_soft_limit', 'tilt_hard_limit', 'status_rate',
            'position_tolerance', 'yaw_tolerance', 'current_x',
            'current_y', 'current_z',
        }
        float_names |= {
            'thruster_deadband', 'thruster_lag_tau', 'thruster_reverse_scale',
            'dob_gain', 'dob_tau', 'vehicle_mass', 'yaw_reserve'}
        bool_names = {
            'enable_yaw_control', 'enable_tilt_guard', 'hold_enabled',
            'actuator_model', 'use_dob'}
        indexed = {}
        for index in range(4):
            indexed[f'allocator_health_{index}'] = (
                'allocator_health', index)
            indexed[f'thruster_gain_{index}'] = ('thruster_gain', index)
        minimums = {
            'max_thrust': 0.0,
            'target_slew_rate': 0.0,
            'target_yaw_slew_rate': 0.0,
            'integral_limit': 0.0,
            'integral_leak': 0.0,
            'slew_rate': 0.0,
            'velocity_filter_tau': 0.0,
            'tilt_soft_limit': 0.0,
            'tilt_hard_limit': 0.0,
            'status_rate': 0.1,
            'position_tolerance': 0.0,
            'yaw_tolerance': 0.0,
        }

        updates = {}
        for parameter in parameters:
            name = parameter.name
            if name in float_names:
                try:
                    value = float(parameter.value)
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False,
                        reason=f'{name} must be numeric',
                    )
                if not math.isfinite(value):
                    return SetParametersResult(
                        successful=False,
                        reason=f'{name} must be finite',
                    )
                if value < minimums.get(name, -math.inf):
                    return SetParametersResult(
                        successful=False,
                        reason=f'{name} is below its safety minimum',
                    )
                updates[name] = value
            elif name in bool_names:
                updates[name] = bool(parameter.value)
            elif name in indexed:
                try:
                    value = float(parameter.value)
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False, reason=f'{name} must be numeric')
                if not 0.0 <= value <= 1.0:
                    return SetParametersResult(
                        successful=False, reason=f'{name} must be in [0, 1]')
                updates[name] = value
            elif name == 'log_file':
                return SetParametersResult(
                    successful=False,
                    reason='log_file is restart-only',
                )

        if 'tilt_hard_limit' in updates:
            hard_limit = updates['tilt_hard_limit']
            soft_limit = updates.get(
                'tilt_soft_limit', self.tilt_soft_limit)
            if hard_limit <= soft_limit:
                return SetParametersResult(
                    successful=False,
                    reason='tilt_hard_limit must exceed tilt_soft_limit',
                )
        if 'tilt_soft_limit' in updates:
            soft_limit = updates['tilt_soft_limit']
            hard_limit = updates.get(
                'tilt_hard_limit', self.tilt_hard_limit)
            if soft_limit >= hard_limit:
                return SetParametersResult(
                    successful=False,
                    reason='tilt_soft_limit must be below tilt_hard_limit',
                )

        for name, value in updates.items():
            if name in indexed:
                attribute, index = indexed[name]
                getattr(self, attribute)[index] = value
            else:
                setattr(self, name, value)
        return SetParametersResult(successful=True, reason='updated')

    @staticmethod
    def move_toward(current, target, maximum_change):
        """Move a command toward its target by at most the given amount."""
        difference = target - current
        if abs(difference) <= maximum_change:
            return target
        return current + maximum_change * (1.0 if difference > 0 else -1.0)

    @staticmethod
    def move_angle_toward(current, target, maximum_change):
        """Move a heading along the shortest wrapped angular distance."""
        difference = wrap_angle(target - current)
        if abs(difference) <= maximum_change:
            return target
        return wrap_angle(
            current + maximum_change * (1.0 if difference > 0 else -1.0))

    def tilt_scale(self, roll, pitch):
        """Fade commands at high attitude and stop at the hard tilt limit."""
        if not self.enable_tilt_guard:
            return 1.0
        tilt = max(abs(roll), abs(pitch))
        if tilt >= self.tilt_hard_limit:
            return 0.0
        if tilt <= self.tilt_soft_limit:
            return 1.0
        return (self.tilt_hard_limit - tilt) / max(
            self.tilt_hard_limit - self.tilt_soft_limit, 1e-6)

    def publish_status(self, data):
        """Publish and periodically log the complete controller state."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if (
            data is None
            and self.last_status_time is not None
            and now - self.last_status_time < 1.0 / max(self.status_rate, 1.0)
        ):
            return
        if (
            data is not None
            and self.last_status_time is not None
            and now - self.last_status_time < 1.0 / max(self.status_rate, 1.0)
        ):
            return
        self.last_status_time = now
        if data is None:
            level = DiagnosticStatus.ERROR
            message = 'waiting for odometry'
            values = {'controller_state': message}
        else:
            pose = data['pose']
            twist = data['twist']
            position_rmse = math.sqrt(
                self.error_squared_sum / max(self.error_sample_time, 1e-6))
            level = DiagnosticStatus.OK
            message = 'station keeping active'
            if data['position_error'] > 0.25 or data['tilt_scale'] == 0.0:
                level = DiagnosticStatus.ERROR
                message = 'station keeping limit exceeded'
            elif (
                data['position_error'] > self.position_tolerance
                or abs(data['eyaw']) > self.yaw_tolerance
            ):
                level = DiagnosticStatus.WARN
                message = 'station keeping correcting error'
            elif data['thrust_saturated'] or data['tilt_guard_active']:
                level = DiagnosticStatus.WARN
                message = 'station keeping safety limiting'
            values = {
                'controller_state': message,
                'target_x_m': self.active_target_x,
                'target_y_m': self.active_target_y,
                'target_yaw_rad': self.active_target_yaw,
                'commanded_target_x_m': self.target_x,
                'commanded_target_y_m': self.target_y,
                'commanded_target_yaw_rad': self.target_yaw,
                'current_x_mps': self.current_x,
                'current_y_mps': self.current_y,
                'current_z_mps': self.current_z,
                'x_m': pose.position.x,
                'y_m': pose.position.y,
                'z_m': pose.position.z,
                'yaw_rad': data['yaw'],
                'roll_rad': data['roll'],
                'pitch_rad': data['pitch'],
                'u_mps': twist.linear.x,
                'v_mps': twist.linear.y,
                'r_radps': twist.angular.z,
                'error_x_body_m': data['ex_body'],
                'error_y_body_m': data['ey_body'],
                'error_yaw_rad': data['eyaw'],
                'position_error_m': data['position_error'],
                'estimate_error_m': data['estimate_error'],
                'position_rmse_m': position_rmse,
                'max_position_error_m': self.max_position_error,
                'tilt_scale': data['tilt_scale'],
                'integral_x': self.integral_x,
                'integral_y': self.integral_y,
                'integral_yaw': self.integral_yaw,
                'thruster_port_front_N': data['commands'][0],
                'thruster_starboard_front_N': data['commands'][1],
                'thruster_port_rear_N': data['commands'][2],
                'thruster_starboard_rear_N': data['commands'][3],
                'kp_x': self.kp_x,
                'ki_x': self.ki_x,
                'kd_x': self.kd_x,
                'kp_y': self.kp_y,
                'ki_y': self.ki_y,
                'kd_y': self.kd_y,
                'kp_yaw': self.kp_yaw,
                'ki_yaw': self.ki_yaw,
                'kd_yaw': self.kd_yaw,
                'max_thrust_N': self.max_thrust,
                'slew_rate_Nps': self.slew_rate,
                'integral_limit': self.integral_limit,
                'integral_leak': self.integral_leak,
                'velocity_filter_tau': self.velocity_filter_tau,
                'enable_yaw_control': self.enable_yaw_control,
                'enable_tilt_guard': self.enable_tilt_guard,
                'tilt_soft_limit_rad': self.tilt_soft_limit,
                'tilt_hard_limit_rad': self.tilt_hard_limit,
                'position_tolerance_m': self.position_tolerance,
                'yaw_tolerance_rad': self.yaw_tolerance,
                'target_slew_rate_mps': self.target_slew_rate,
                'target_yaw_slew_rate_radps': self.target_yaw_slew_rate,
                'elapsed_s': data.get('elapsed_s', 0.0),
                'control_surge_N': self.last_control_wrench[0],
                'control_sway_N': self.last_control_wrench[1],
                'control_yaw_Nm': self.last_control_wrench[2],
                'thrust_saturated': data.get(
                    'thrust_saturated', self.thrust_saturated),
                'tilt_guard_active': data.get(
                    'tilt_guard_active', self.tilt_guard_active),
                'raw_thruster_port_front_N': data['raw_commands'][0],
                'raw_thruster_starboard_front_N': data['raw_commands'][1],
                'raw_thruster_port_rear_N': data['raw_commands'][2],
                'raw_thruster_starboard_rear_N': data['raw_commands'][3],
            }
        diagnostic = DiagnosticStatus()
        diagnostic.name = 'research_catamaran/station_keeping'
        diagnostic.level = level
        diagnostic.message = message
        diagnostic.values = [
            KeyValue(key=key, value=f'{value}')
            for key, value in values.items()
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [diagnostic]
        self.status_pub.publish(array)
        if data is not None:
            self.get_logger().info(
                f'SK {message} | '
                f'pos=({values["x_m"]:.3f}, {values["y_m"]:.3f}, '
                f'{values["z_m"]:.3f}) '
                f'target=({self.target_x:.3f}, {self.target_y:.3f}, '
                f'{self.target_yaw:.3f}) '
                f'err=({data["ex_body"]:.3f}, {data["ey_body"]:.3f}, '
                f'{data["eyaw"]:.3f}) '
                f'attitude=({data["roll"]:.3f}, {data["pitch"]:.3f}) '
                f'vel=({twist.linear.x:.3f}, {twist.linear.y:.3f}, '
                f'{twist.angular.z:.3f}) '
                f'thrust=({data["commands"][0]:.2f}, '
                f'{data["commands"][1]:.2f}, {data["commands"][2]:.2f}, '
                f'{data["commands"][3]:.2f})')

    def update_observer(self, yaw, u, v, dt):
        """Advance the disturbance observer; return world-frame force (N).

        With ``m dv/dt = F + d - D(v)`` the observer ``dhat = z + L m v``,
        ``dz/dt = L (D(v) - F - dhat)`` converges to ``d`` without
        differentiating the noisy velocity.
        """
        gain = 1.0 / max(self.dob_tau, 0.05)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        vx = cos_yaw * u - sin_yaw * v
        vy = sin_yaw * u + cos_yaw * v
        fx_b = sum(c * a[0] for c, a in zip(self.delivered, THRUSTER_AXES))
        fy_b = sum(c * a[1] for c, a in zip(self.delivered, THRUSTER_AXES))
        fx = cos_yaw * fx_b - sin_yaw * fy_b
        fy = sin_yaw * fx_b + cos_yaw * fy_b
        mass = self.vehicle_mass
        if self.dob_z is None:
            self.dob_z = [-gain * mass * vx, -gain * mass * vy]
        drag_x = 0.7 * vx + 2.5 * abs(vx) * vx
        drag_y = 0.7 * vy + 2.5 * abs(vy) * vy
        estimate_x = self.dob_z[0] + gain * mass * vx
        estimate_y = self.dob_z[1] + gain * mass * vy
        self.dob_z[0] += dt * gain * (drag_x - fx - estimate_x)
        self.dob_z[1] += dt * gain * (drag_y - fy - estimate_y)
        self.dob_estimate = (estimate_x, estimate_y)
        message = Vector3()
        message.x, message.y = float(estimate_x), float(estimate_y)
        self.dob_pub.publish(message)
        return estimate_x, estimate_y

    def apply_actuator(self, commands, dt):
        """Pass commands through lag, ESC deadband and asymmetric thrust."""
        if not self.actuator_model:
            return tuple(
                c * g for c, g in zip(commands, self.thruster_gain))
        tau = max(self.thruster_lag_tau, 0.0)
        alpha = 1.0 if tau <= 1e-9 else dt / (tau + dt)
        lagged = tuple(
            state + alpha * (command - state)
            for state, command in zip(self.actuator_state, commands))
        self.actuator_state = lagged
        delivered = []
        for value in lagged:
            if abs(value) < self.thruster_deadband:
                value = 0.0
            elif value < 0.0:
                value *= self.thruster_reverse_scale
            delivered.append(value)
        return tuple(c * g for c, g in zip(delivered, self.thruster_gain))

    def publish_commands(self, commands):
        for key, command in zip(self.thrust_publishers, commands):
            message = Float64()
            message.data = float(command)
            self.thrust_publishers[key].publish(message)

    def destroy_node(self):
        try:
            self.publish_commands((0.0, 0.0, 0.0, 0.0))
        except Exception:  # context already shut down on SIGINT
            pass
        if self.csv_file is not None:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StationKeepingPid()
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
