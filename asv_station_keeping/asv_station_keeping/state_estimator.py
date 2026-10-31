"""GPS/IMU position estimator so station keeping runs on noisy sensors.

The simulator's ground-truth odometry is perfect, which hides exactly the
problem this project studies: a vehicle that cannot hold station also cannot
localize or map reliably.  This node fuses the noisy ``/research_catamaran/gps``
fix (10 Hz) with the noisy IMU (100 Hz) into a nav_msgs/Odometry message that
the controller can use instead of ground truth.

Each horizontal world axis runs an independent constant-acceleration-input
Kalman filter with state ``[position, velocity]``.  The IMU specific force is
rotated into the ENU world frame with the IMU orientation (gravity only
affects the vertical component, so the horizontal result is the true
acceleration) and drives the prediction; GPS corrects position.  Heading and
yaw rate come straight from the IMU.
"""

import math

import rclpy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix

GRAVITY = 9.80665
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def geodetic_to_local(latitude, longitude, origin_latitude, origin_longitude):
    """Return east/north metres of a fix relative to the world origin."""
    lat0 = math.radians(origin_latitude)
    sin2 = math.sin(lat0) ** 2
    denominator = math.sqrt(1.0 - WGS84_E2 * sin2)
    prime_vertical = WGS84_A / denominator
    meridional = WGS84_A * (1.0 - WGS84_E2) / denominator ** 3
    north = math.radians(latitude - origin_latitude) * meridional
    east = math.radians(longitude - origin_longitude) \
        * prime_vertical * math.cos(lat0)
    return east, north


def rotate_by_quaternion(q, v):
    """Rotate vector ``v`` by quaternion ``q = (x, y, z, w)``."""
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * cross(q.xyz, v); v' = v + w*t + cross(q.xyz, t)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


class AxisFilter:
    """Position/velocity Kalman filter driven by measured acceleration."""

    def __init__(self, accel_std, position_std, initial_position=0.0,
                 accel_limit=6.0, gate=0.5, reset_after=3,
                 speed_limit=1.2):
        self.accel_limit = accel_limit
        self.speed_limit = speed_limit
        self.gate = gate
        self.reset_after = reset_after
        self.rejected = 0
        self.p = initial_position
        self.v = 0.0
        self.accel_var = accel_std ** 2
        self.meas_var = position_std ** 2
        # Covariance entries: [[pp, pv], [pv, vv]]
        self.pp = position_std ** 2
        self.pv = 0.0
        self.vv = 1.0

    def predict(self, accel, dt):
        """Propagate with acceleration input over ``dt`` seconds."""
        # Physically plausible horizontal accelerations only: impacts and
        # slamming produce IMU spikes that would otherwise be integrated.
        accel = max(-self.accel_limit, min(self.accel_limit, accel))
        self.p += self.v * dt + 0.5 * accel * dt * dt
        # A hull in this pool cannot exceed ~1 m/s; an unbounded integrated
        # IMU bias (e.g. gravity leaking in on a tilt) must not run away.
        self.v = max(-self.speed_limit, min(self.speed_limit,
                                            self.v + accel * dt))
        q = self.accel_var
        pp = self.pp + 2.0 * dt * self.pv + dt * dt * self.vv \
            + 0.25 * dt ** 4 * q
        pv = self.pv + dt * self.vv + 0.5 * dt ** 3 * q
        vv = self.vv + dt * dt * q
        self.pp, self.pv, self.vv = pp, pv, vv

    def update(self, measured_position):
        """Correct with a position fix, with outlier gating and recovery."""
        innovation = measured_position - self.p
        if abs(innovation) > self.gate:
            self.rejected += 1
            if self.rejected < self.reset_after:
                return
            # Persistent disagreement: the filter, not the GPS, is wrong.
            self.p, self.v = measured_position, 0.0
            self.pp, self.pv, self.vv = self.meas_var, 0.0, 1.0
            self.rejected = 0
            return
        self.rejected = 0
        s = self.pp + self.meas_var
        k_p = self.pp / s
        k_v = self.pv / s
        self.p += k_p * innovation
        self.v += k_v * innovation
        pp = (1.0 - k_p) * self.pp
        pv = (1.0 - k_p) * self.pv
        vv = self.vv - k_v * self.pv
        self.pp, self.pv, self.vv = pp, pv, vv


class BiasAxisFilter(AxisFilter):
    """Position/velocity/accelerometer-bias Kalman filter for one axis.

    State ``[p, v, b]``: the IMU reports ``a_meas = a_true + b + noise`` and
    the bias ``b`` follows a random walk.  GPS fixes make ``b`` observable,
    so when GPS later drops out the filter dead-reckons with the bias already
    removed, where the two-state filter drifts quadratically.
    """

    def __init__(self, accel_std, position_std, initial_position=0.0,
                 bias_walk_std=0.002, initial_bias_std=0.1, **kwargs):
        super().__init__(accel_std, position_std, initial_position, **kwargs)
        self.b = 0.0
        self.bias_walk_var = bias_walk_std ** 2
        # Covariance, full symmetric 3x3: [[pp, pv, pb], [pv, vv, vb], ...].
        self.pb = 0.0
        self.vb = 0.0
        self.bb = initial_bias_std ** 2

    def predict(self, accel, dt):
        """Propagate with the bias-corrected acceleration."""
        accel = max(-self.accel_limit, min(self.accel_limit, accel))
        a = accel - self.b
        self.p += self.v * dt + 0.5 * a * dt * dt
        self.v = max(-self.speed_limit, min(self.speed_limit,
                                            self.v + a * dt))
        # P' = F P F^T + Q with F = [[1, dt, -dt^2/2], [0, 1, -dt], [0, 0, 1]]
        h = 0.5 * dt * dt
        pp, pv, pb = self.pp, self.pv, self.pb
        vv, vb, bb = self.vv, self.vb, self.bb
        # Rows of F P.
        r0 = (pp + dt * pv - h * pb, pv + dt * vv - h * vb,
              pb + dt * vb - h * bb)
        r1 = (pv - dt * pb, vv - dt * vb, vb - dt * bb)
        r2 = (pb, vb, bb)
        # (F P) F^T.
        self.pp = r0[0] + dt * r0[1] - h * r0[2]
        self.pv = r0[1] - dt * r0[2]
        self.pb = r0[2]
        self.vv = r1[1] - dt * r1[2]
        self.vb = r1[2]
        self.bb = r2[2]
        q = self.accel_var
        self.pp += 0.25 * dt ** 4 * q
        self.pv += 0.5 * dt ** 3 * q
        self.vv += dt * dt * q
        self.bb += self.bias_walk_var * dt

    def update(self, measured_position):
        """Correct with a position fix (same gating as the base filter)."""
        innovation = measured_position - self.p
        if abs(innovation) > self.gate:
            self.rejected += 1
            if self.rejected < self.reset_after:
                return
            self.p, self.v = measured_position, 0.0
            self.pp, self.pv, self.vv = self.meas_var, 0.0, 1.0
            self.pb = self.vb = 0.0
            self.rejected = 0
            return
        self.rejected = 0
        s = self.pp + self.meas_var
        k_p, k_v, k_b = self.pp / s, self.pv / s, self.pb / s
        self.p += k_p * innovation
        self.v += k_v * innovation
        self.b += k_b * innovation
        pp, pv, pb = self.pp, self.pv, self.pb
        vv, vb, bb = self.vv, self.vb, self.bb
        self.pp = pp - k_p * pp
        self.pv = pv - k_p * pv
        self.pb = pb - k_p * pb
        self.vv = vv - k_v * pv
        self.vb = vb - k_v * pb
        self.bb = bb - k_b * pb


class StateEstimator(Node):
    """Publish a fused odometry estimate from noisy GPS and IMU."""

    def __init__(self):
        super().__init__('research_state_estimator')
        self.declare_parameter('origin_latitude', 13.0827)
        self.declare_parameter('origin_longitude', 80.2707)
        self.declare_parameter('gps_topic', '/research_catamaran/gps')
        self.declare_parameter('imu_topic', '/research_catamaran/imu')
        self.declare_parameter(
            'output_topic', '/research_catamaran/estimated_odometry')
        self.declare_parameter(
            'truth_topic', '/model/research_catamaran/odometry')
        self.declare_parameter('gps_position_std', 0.05)
        self.declare_parameter('imu_accel_std', 0.04)
        self.declare_parameter('gyro_filter_tau', 0.05)
        # Estimate the accelerometer bias (random-walk state).  Off keeps the
        # original two-state filter used by the demonstrations.
        self.declare_parameter('estimate_accel_bias', False)
        self.declare_parameter('accel_bias_walk_std', 0.002)
        # Antenna offset from the vehicle origin (body frame, metres).  The
        # mast is on the centreline so this is zero for the default model.
        self.declare_parameter('antenna_offset_x', 0.0)
        self.declare_parameter('antenna_offset_y', 0.0)
        # Fault injection (set live by the scenario runner).
        self.declare_parameter('gps_enabled', True)
        self.declare_parameter('gps_offset_x', 0.0)
        self.declare_parameter('gps_offset_y', 0.0)
        self.declare_parameter('imu_bias_x', 0.0)
        self.declare_parameter('imu_bias_y', 0.0)
        self.faults = {
            'gps_enabled': True, 'gps_offset_x': 0.0, 'gps_offset_y': 0.0,
            'imu_bias_x': 0.0, 'imu_bias_y': 0.0,
        }
        self.add_on_set_parameters_callback(self.on_parameter_change)

        self.origin = (
            float(self.get_parameter('origin_latitude').value),
            float(self.get_parameter('origin_longitude').value),
        )
        self.gps_std = float(self.get_parameter('gps_position_std').value)
        self.accel_std = float(self.get_parameter('imu_accel_std').value)
        self.gyro_tau = float(self.get_parameter('gyro_filter_tau').value)
        self.estimate_bias = bool(
            self.get_parameter('estimate_accel_bias').value)
        self.bias_walk_std = float(
            self.get_parameter('accel_bias_walk_std').value)
        self.antenna = (
            float(self.get_parameter('antenna_offset_x').value),
            float(self.get_parameter('antenna_offset_y').value),
        )
        self.filters = None
        self.last_imu_time = None
        self.orientation = (0.0, 0.0, 0.0, 1.0)
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.truth = None

        self.publisher = self.create_publisher(
            Odometry, str(self.get_parameter('output_topic').value), 10)
        self.error_publisher = self.create_publisher(
            Vector3, '/research_catamaran/estimation_error', 10)
        self.create_subscription(
            NavSatFix, str(self.get_parameter('gps_topic').value),
            self.gps_callback, 10)
        self.create_subscription(
            Imu, str(self.get_parameter('imu_topic').value),
            self.imu_callback, 50)
        self.create_subscription(
            Odometry, str(self.get_parameter('truth_topic').value),
            self.truth_callback, 10)
        self.get_logger().info(
            f'State estimator: GPS sigma={self.gps_std:.3f} m, '
            f'accel sigma={self.accel_std:.3f} m/s^2')

    def on_parameter_change(self, parameters):
        for parameter in parameters:
            if parameter.name in self.faults:
                value = parameter.value
                self.faults[parameter.name] = (
                    bool(value) if parameter.name == 'gps_enabled'
                    else float(value))
        return SetParametersResult(successful=True)

    def truth_callback(self, message):
        self.truth = message

    def gps_callback(self, message):
        """Correct the filters with a GPS fix, referred to the vehicle origin."""
        if message.latitude != message.latitude \
                or message.longitude != message.longitude:
            return
        if not self.faults['gps_enabled']:
            return
        east, north = geodetic_to_local(
            message.latitude, message.longitude, *self.origin)
        east += self.faults['gps_offset_x']
        north += self.faults['gps_offset_y']
        cos_yaw, sin_yaw = math.cos(self.yaw), math.sin(self.yaw)
        east -= cos_yaw * self.antenna[0] - sin_yaw * self.antenna[1]
        north -= sin_yaw * self.antenna[0] + cos_yaw * self.antenna[1]
        if self.filters is None:
            if self.estimate_bias:
                self.filters = (
                    BiasAxisFilter(self.accel_std, self.gps_std, east,
                                   bias_walk_std=self.bias_walk_std),
                    BiasAxisFilter(self.accel_std, self.gps_std, north,
                                   bias_walk_std=self.bias_walk_std),
                )
            else:
                self.filters = (
                    AxisFilter(self.accel_std, self.gps_std, east),
                    AxisFilter(self.accel_std, self.gps_std, north),
                )
            return
        self.filters[0].update(east)
        self.filters[1].update(north)

    def imu_callback(self, message):
        """Predict with the IMU and publish the estimate."""
        now = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        q = message.orientation
        self.orientation = (q.x, q.y, q.z, q.w)
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self.last_imu_time is None:
            self.last_imu_time = now
            return
        dt = now - self.last_imu_time
        self.last_imu_time = now
        if dt <= 0.0 or dt > 0.5:
            return
        alpha = dt / (self.gyro_tau + dt) if self.gyro_tau > 0.0 else 1.0
        self.yaw_rate += alpha * (message.angular_velocity.z - self.yaw_rate)
        if self.filters is None:
            return

        specific_force = (
            message.linear_acceleration.x + self.faults['imu_bias_x'],
            message.linear_acceleration.y + self.faults['imu_bias_y'],
            message.linear_acceleration.z,
        )
        world = rotate_by_quaternion(self.orientation, specific_force)
        self.filters[0].predict(world[0], dt)
        self.filters[1].predict(world[1], dt)
        self.publish_estimate(message.header.stamp)

    def publish_estimate(self, stamp):
        """Publish nav_msgs/Odometry with a body-frame twist."""
        fx, fy = self.filters
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = fx.p
        odom.pose.pose.position.y = fy.p
        odom.pose.pose.orientation.x = self.orientation[0]
        odom.pose.pose.orientation.y = self.orientation[1]
        odom.pose.pose.orientation.z = self.orientation[2]
        odom.pose.pose.orientation.w = self.orientation[3]
        cos_yaw, sin_yaw = math.cos(self.yaw), math.sin(self.yaw)
        odom.twist.twist.linear.x = cos_yaw * fx.v + sin_yaw * fy.v
        odom.twist.twist.linear.y = -sin_yaw * fx.v + cos_yaw * fy.v
        odom.twist.twist.angular.z = self.yaw_rate
        odom.pose.covariance[0] = fx.pp
        odom.pose.covariance[7] = fy.pp
        odom.twist.covariance[0] = fx.vv
        odom.twist.covariance[7] = fy.vv
        self.publisher.publish(odom)
        if self.truth is not None:
            truth = self.truth.pose.pose.position
            error = Vector3()
            error.x = fx.p - truth.x
            error.y = fy.p - truth.y
            error.z = math.hypot(error.x, error.y)
            self.error_publisher.publish(error)


def main(args=None):
    """Run the state estimator."""
    rclpy.init(args=args)
    node = StateEstimator()
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
