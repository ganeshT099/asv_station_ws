"""Publish the simulated sea state (current, wind, gusts, waves) as one current.

Fortress has no way to apply an arbitrary external wrench to a model, but its
hydrodynamics system drags the hull against the water velocity published on
``/ocean_current``.  This node therefore builds a single *effective* water
velocity from four physical sources:

* a steady current plus Gauss-Markov turbulence,
* wind, whose aerodynamic force on the above-water structure is converted into
  the water velocity that produces the same hull drag at rest,
* wind gusts (Ornstein-Uhlenbeck speed fluctuation with a direction wander),
* regular-wave surface orbital velocity (horizontal, zero mean).

The wind conversion is exact only for a stationary hull, which is the
station-keeping regime.  Wind yaw moments are not modelled.  With every new
parameter at its default of zero the node reduces to the original constant
current.
"""

import math
import random

import rclpy
from geometry_msgs.msg import Point, Vector3, Wrench
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

AIR_DENSITY = 1.225
WATER_DENSITY = 1000.0
GRAVITY = 9.81

NUMERIC_PARAMETERS = (
    'current_x', 'current_y', 'current_z',
    'current_turbulence_std', 'current_turbulence_tau',
    'wind_speed', 'wind_direction_deg', 'wind_gust_std', 'wind_gust_tau',
    'wind_direction_wander_deg', 'wind_windage_area', 'wind_drag_coefficient',
    'wave_height', 'wave_period', 'wave_direction_deg',
    'drag_linear_x', 'drag_quadratic_x', 'drag_linear_y', 'drag_quadratic_y',
    'wind_cp_x', 'wind_cp_y', 'waterplane_length', 'hull_beam',
    'hull_spacing', 'wave_moment_scale', 'wave_heave_scale',
)


def wave_excitation(height, period, direction_deg, x, y, yaw, elapsed,
                    length, beam, spacing):
    """Return (fz, mx_body, my_body) of a regular wave on the twin hulls.

    Quasi-static Froude-Krylov model: the vertical force follows the surface
    elevation at the vehicle and the roll/pitch moments follow the surface
    slope, each scaled by the waterplane stiffness (rho*g*A, rho*g*I).  The
    vehicle then heaves, rolls and pitches with the wave as a real hull does.
    """
    if height <= 0.0 or period <= 0.0:
        return 0.0, 0.0, 0.0
    omega = 2.0 * math.pi / period
    wavenumber = omega * omega / GRAVITY
    heading = math.radians(direction_deg)
    kx, ky = wavenumber * math.cos(heading), wavenumber * math.sin(heading)
    amplitude = 0.5 * height
    phase = kx * x + ky * y - omega * elapsed
    elevation = amplitude * math.cos(phase)
    slope_x = -amplitude * kx * math.sin(phase)
    slope_y = -amplitude * ky * math.sin(phase)
    area = 2.0 * length * beam
    i_pitch = 2.0 * beam * length ** 3 / 12.0
    i_roll = 2.0 * (length * beam ** 3 / 12.0
                    + length * beam * (0.5 * spacing) ** 2)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    slope_forward = cos_yaw * slope_x + sin_yaw * slope_y
    slope_left = -sin_yaw * slope_x + cos_yaw * slope_y
    fz = WATER_DENSITY * GRAVITY * area * elevation
    # Surface rising toward the bow pitches the bow up (negative pitch about
    # +Y); surface rising to port rolls the vehicle to starboard-down (+X).
    my_body = -WATER_DENSITY * GRAVITY * i_pitch * slope_forward
    mx_body = WATER_DENSITY * GRAVITY * i_roll * slope_left
    return fz, mx_body, my_body


def equivalent_water_speed(force, linear, quadratic):
    """Return v with ``linear*v + quadratic*|v|*v == force`` (monotonic)."""
    magnitude = abs(force)
    if magnitude < 1e-12:
        return 0.0
    if quadratic <= 1e-12:
        speed = magnitude / max(linear, 1e-12)
    else:
        speed = (-linear + math.sqrt(
            linear * linear + 4.0 * quadratic * magnitude)) / (2.0 * quadratic)
    return math.copysign(speed, force)


def wave_orbital_speed(height, period, elapsed):
    """Return surface orbital speed amplitude-phase value for a regular wave."""
    if height <= 0.0 or period <= 0.0:
        return 0.0
    omega = 2.0 * math.pi / period
    return 0.5 * height * omega * math.cos(omega * elapsed)


class CurrentDisturbance(Node):
    """Publish an effective water-current vector built from the sea state."""

    def __init__(self):
        super().__init__('research_current_disturbance')
        defaults = {
            'current_x': 0.0, 'current_y': 0.0, 'current_z': 0.0,
            'current_turbulence_std': 0.0, 'current_turbulence_tau': 20.0,
            'wind_speed': 0.0, 'wind_direction_deg': 0.0,
            'wind_gust_std': 0.0, 'wind_gust_tau': 6.0,
            'wind_direction_wander_deg': 0.0,
            'wind_windage_area': 0.15, 'wind_drag_coefficient': 1.0,
            'wave_height': 0.0, 'wave_period': 3.0, 'wave_direction_deg': 0.0,
            # Must match the Hydrodynamics block in research_catamaran.urdf.
            'drag_linear_x': 0.60, 'drag_quadratic_x': 2.0,
            'drag_linear_y': 0.80, 'drag_quadratic_y': 3.0,
            'publish_rate': 20.0,
            # Wind centre of effort relative to the vehicle origin (body
            # frame, m).  Forward of the centre of drag, crosswind turns the
            # bow away from the wind, as on most small boats.
            'wind_cp_x': 0.08, 'wind_cp_y': 0.02,
            # Waterplane geometry of the twin hulls (matches the URDF boxes).
            'waterplane_length': 0.80, 'hull_beam': 0.16,
            'hull_spacing': 0.35,
            # Gazebo's graded buoyancy acts at the box centre, so the
            # simulated hull has about 1/6 of a real waterplane's pitch/roll
            # stiffness (measured).  Scaling the wave moments by the same
            # ratio makes the hull follow the wave slope instead of
            # over-rotating.  Set 1.0 on a platform with true waterplane
            # buoyancy.
            'wave_moment_scale': 0.08,
            # Scale of the vertical wave force.  The simulated hull heaves
            # about twice as far as the wave at full scale, so demos use a
            # smaller value for smooth, ship-like motion.
            'wave_heave_scale': 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.declare_parameter('topic', '/ocean_current')
        self.declare_parameter('seed', 1)
        # force_mode applies wind/waves as true forces and moments through
        # the Gazebo wrench bridge; False falls back to the equivalent-current
        # approximation (no yaw moment, no heave/pitch).
        self.declare_parameter('force_mode', True)
        self.declare_parameter('models', ['research_catamaran'])
        self.declare_parameter('ghost', False)
        self.force_mode = bool(self.get_parameter('force_mode').value)
        self.models = [str(m) for m in self.get_parameter('models').value]
        if bool(self.get_parameter('ghost').value):
            self.models.append('ghost_catamaran')
        self.poses = {m: (0.0, 0.0, 0.0) for m in self.models}
        self.wrench_publishers = {}
        for model in self.models:
            self.wrench_publishers[model] = self.create_publisher(
                Wrench, f'/{model}/environment/wrench', 10)
            self.create_subscription(
                Odometry, f'/model/{model}/odometry',
                lambda msg, m=model: self.pose_callback(m, msg), 10)
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/research_catamaran/environment/gz_markers', 10)
        self.rviz_marker_publisher = self.create_publisher(
            MarkerArray, '/research_catamaran/environment/markers_rviz', 10)
        self.marker_tick = 0
        self.values = {
            name: float(self.get_parameter(name).value)
            for name in NUMERIC_PARAMETERS
        }
        self.rng = random.Random(int(self.get_parameter('seed').value))
        self.turbulence = [0.0, 0.0]
        self.gust_speed = 0.0
        self.gust_heading = 0.0
        self.start_time = None
        self.last_time = None
        self.add_on_set_parameters_callback(self.on_parameter_change)

        topic = str(self.get_parameter('topic').value)
        rate = max(float(self.get_parameter('publish_rate').value), 1.0)
        self.publisher = self.create_publisher(Vector3, topic, 10)
        self.wind_publisher = self.create_publisher(
            Vector3, '/research_catamaran/environment/wind', 10)
        self.wave_publisher = self.create_publisher(
            Vector3, '/research_catamaran/environment/wave_velocity', 10)
        self.water_publisher = self.create_publisher(
            Vector3, '/research_catamaran/environment/water_current', 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_current)
        self.get_logger().info(
            'Sea state: current=(%.2f, %.2f) m/s turbulence=%.3f wind=%.1f '
            'm/s @ %.0f deg gust=%.2f waves H=%.2f m T=%.1f s on %s' % (
                self.values['current_x'], self.values['current_y'],
                self.values['current_turbulence_std'],
                self.values['wind_speed'], self.values['wind_direction_deg'],
                self.values['wind_gust_std'], self.values['wave_height'],
                self.values['wave_period'], topic))

    @property
    def current(self):
        """Steady current as an (x, y, z) tuple."""
        return (
            self.values['current_x'],
            self.values['current_y'],
            self.values['current_z'],
        )

    @staticmethod
    def ou_step(state, std, tau, dt, rng):
        """Advance a zero-mean Ornstein-Uhlenbeck state by exactly dt."""
        if std <= 0.0 or tau <= 0.0:
            return 0.0
        decay = math.exp(-dt / tau)
        return state * decay + std * math.sqrt(1.0 - decay * decay) \
            * rng.gauss(0.0, 1.0)

    def advance(self, now):
        """Update the stochastic states; return the time step used."""
        if self.start_time is None:
            self.start_time = now
            self.last_time = now
        dt = min(max(now - self.last_time, 0.0), 0.5)
        self.last_time = now
        values = self.values
        if dt > 0.0:
            for axis in range(2):
                self.turbulence[axis] = self.ou_step(
                    self.turbulence[axis], values['current_turbulence_std'],
                    values['current_turbulence_tau'], dt, self.rng)
            self.gust_speed = self.ou_step(
                self.gust_speed,
                values['wind_gust_std'] * values['wind_speed'],
                values['wind_gust_tau'], dt, self.rng)
            self.gust_heading = self.ou_step(
                self.gust_heading,
                math.radians(values['wind_direction_wander_deg']),
                values['wind_gust_tau'], dt, self.rng)
        return dt

    def wind_velocity(self):
        """Return the instantaneous wind velocity (m/s, world frame)."""
        values = self.values
        speed = max(values['wind_speed'] + self.gust_speed, 0.0)
        heading = math.radians(values['wind_direction_deg']) \
            + self.gust_heading
        return speed * math.cos(heading), speed * math.sin(heading)

    def wind_equivalent_current(self, wind):
        """Convert wind force into the water velocity giving equal hull drag."""
        values = self.values
        speed = math.hypot(*wind)
        factor = 0.5 * AIR_DENSITY * values['wind_drag_coefficient'] \
            * values['wind_windage_area'] * speed
        return (
            equivalent_water_speed(
                factor * wind[0], values['drag_linear_x'],
                values['drag_quadratic_x']),
            equivalent_water_speed(
                factor * wind[1], values['drag_linear_y'],
                values['drag_quadratic_y']),
        )

    def wave_velocity(self, elapsed):
        """Return horizontal surface orbital velocity of the regular wave."""
        values = self.values
        speed = wave_orbital_speed(
            values['wave_height'], values['wave_period'], elapsed)
        direction = math.radians(values['wave_direction_deg'])
        return speed * math.cos(direction), speed * math.sin(direction)

    def publish_current(self):
        """Publish the effective current and its physical components."""
        now = self.get_clock().now().nanoseconds * 1e-9
        self.advance(now)
        elapsed = now - self.start_time
        steady = self.current
        water = (
            steady[0] + self.turbulence[0],
            steady[1] + self.turbulence[1],
        )
        wind = self.wind_velocity()
        wave = self.wave_velocity(elapsed)
        if self.force_mode:
            wind_eq = (0.0, 0.0)
            for model in self.models:
                self.publish_wrench(model, wind, elapsed)
        else:
            wind_eq = self.wind_equivalent_current(wind)
        self.publish_vector(self.publisher, (
            water[0] + wind_eq[0] + wave[0],
            water[1] + wind_eq[1] + wave[1],
            steady[2],
        ))
        self.marker_tick += 1
        if self.marker_tick % 5 == 0:
            self.publish_markers(wind, water)
        self.publish_vector(self.wind_publisher, (wind[0], wind[1], 0.0))
        self.publish_vector(self.wave_publisher, (wave[0], wave[1], 0.0))
        self.publish_vector(self.water_publisher, (water[0], water[1], steady[2]))

    def pose_callback(self, model, message):
        """Remember a model's planar pose for wave phase and wind moment."""
        q = message.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.poses[model] = (
            message.pose.pose.position.x, message.pose.pose.position.y, yaw)

    def wind_wrench(self, wind, yaw):
        """Return world-frame wind force (x, y) and yaw moment about +Z."""
        values = self.values
        speed = math.hypot(*wind)
        factor = 0.5 * AIR_DENSITY * values['wind_drag_coefficient'] \
            * values['wind_windage_area'] * speed
        fx, fy = factor * wind[0], factor * wind[1]
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        # Lever arm of the centre of effort in the world frame.
        rx = cos_yaw * values['wind_cp_x'] - sin_yaw * values['wind_cp_y']
        ry = sin_yaw * values['wind_cp_x'] + cos_yaw * values['wind_cp_y']
        return fx, fy, rx * fy - ry * fx

    def publish_wrench(self, model, wind, elapsed):
        """Publish the combined wind + wave wrench for one hull."""
        values = self.values
        x, y, yaw = self.poses[model]
        fx, fy, nz = self.wind_wrench(wind, yaw)
        fz, mx_b, my_b = wave_excitation(
            values['wave_height'], values['wave_period'],
            values['wave_direction_deg'], x, y, yaw, elapsed,
            values['waterplane_length'], values['hull_beam'],
            values['hull_spacing'])
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        fz *= values['wave_heave_scale']
        message = Wrench()
        message.force.x, message.force.y, message.force.z = fx, fy, fz
        scale = values['wave_moment_scale']
        mx_b, my_b = mx_b * scale, my_b * scale
        message.torque.x = cos_yaw * mx_b - sin_yaw * my_b
        message.torque.y = sin_yaw * mx_b + cos_yaw * my_b
        message.torque.z = nz
        self.wrench_publishers[model].publish(message)

    def publish_markers(self, wind, water):
        """Draw wind and current arrows and a readout in Gazebo and RViz."""
        anchor = (-4.2, 3.3)

        def arrow(ns, marker_id, origin, vector, scale, colour):
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.ns, marker.id = ns, marker_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x, marker.scale.y, marker.scale.z = 0.04, 0.10, 0.12
            marker.color.r, marker.color.g, marker.color.b = colour
            marker.color.a = 1.0
            start = Point(x=origin[0], y=origin[1], z=0.35)
            length = math.hypot(*vector)
            reach = min(scale * length, 2.5)
            if length > 1e-6:
                end = Point(x=origin[0] + reach * vector[0] / length,
                            y=origin[1] + reach * vector[1] / length, z=0.35)
            else:
                end = Point(x=origin[0] + 0.02, y=origin[1], z=0.35)
            marker.points = [start, end]
            return marker

        def text(ns, marker_id, origin, label, colour):
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.ns, marker.id = ns, marker_id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y = origin
            marker.pose.position.z = 0.8
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.22
            marker.text = label
            marker.color.r, marker.color.g, marker.color.b = colour
            marker.color.a = 1.0
            return marker

        wind_colour, water_colour = (0.95, 0.45, 0.10), (0.10, 0.65, 0.95)
        wind_speed = math.hypot(*wind)
        water_speed = math.hypot(*water)
        array = MarkerArray()
        array.markers = [
            arrow('wind', 1, anchor, wind, 0.35, wind_colour),
            arrow('current', 2, (anchor[0], anchor[1] - 0.9), water, 2.5,
                  water_colour),
            text('wind_label', 3, (anchor[0], anchor[1] + 0.45),
                 f'Wind {wind_speed:.1f} m/s', wind_colour),
            text('current_label', 4, (anchor[0], anchor[1] - 1.45),
                 f'Current {water_speed:.2f} m/s', water_colour),
        ]
        self.marker_publisher.publish(array)
        self.rviz_marker_publisher.publish(array)

    @staticmethod
    def publish_vector(publisher, values):
        """Publish a 3-tuple as geometry_msgs/Vector3."""
        message = Vector3()
        message.x, message.y, message.z = (float(v) for v in values)
        publisher.publish(message)

    def on_parameter_change(self, parameters):
        """Update the sea state without restarting the node."""
        updates = {}
        for parameter in parameters:
            if parameter.name not in self.values:
                continue
            try:
                value = float(parameter.value)
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False,
                    reason=f'{parameter.name} must be numeric',
                )
            if not math.isfinite(value):
                return SetParametersResult(
                    successful=False,
                    reason=f'{parameter.name} must be finite',
                )
            if parameter.name.endswith(('_std', '_height', '_area', '_speed')) \
                    and value < 0.0:
                return SetParametersResult(
                    successful=False,
                    reason=f'{parameter.name} must not be negative',
                )
            updates[parameter.name] = value
        self.values.update(updates)
        return SetParametersResult(successful=True, reason='updated')


def main(args=None):
    """Run the sea-state publisher."""
    rclpy.init(args=args)
    node = CurrentDisturbance()
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
