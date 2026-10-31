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
                 accel_limit=6.0, gate=0.5, reset_after=3):
        self.accel_limit = accel_limit
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
        self.v += accel * dt
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
