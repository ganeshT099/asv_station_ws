"""Mission modes layered on the station-keeping controller.

``weathervane``  Turn the bow into the wind to minimise the force the
                 thrusters must resist.  The wind direction is not measured:
                 it is inferred from the controller's disturbance observer.
``waypoints``    Visit a list of (x, y, yaw) stations, holding each one.
"""

import math

import rclpy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node

CONTROLLER = '/research_station_keeping_pid'


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class MissionRunner(Node):
    """Drive the controller's targets according to the selected mode."""

    def __init__(self):
        super().__init__('research_mission_runner')
        self.declare_parameter('mode', 'weathervane')
        self.declare_parameter('min_force', 0.35)
        self.declare_parameter('filter_tau', 10.0)
        self.declare_parameter('update_period', 1.0)
        self.declare_parameter('waypoints', [0.0, 0.0, 0.0])
        self.declare_parameter('dwell', 25.0)
        self.declare_parameter('arrive_radius', 0.10)
        self.mode = str(self.get_parameter('mode').value)
        self.client = self.create_client(
            SetParameters, CONTROLLER + '/set_parameters')
        self.filtered = None
        self.last_update = None
        self.last_yaw_target = None
        self.pose = None
        self.index = -1
        self.arrived_at = None
        self.create_subscription(
            Vector3, '/research_catamaran/station_keeping/disturbance_estimate',
            self.disturbance_callback, 10)
        self.create_subscription(
            Odometry, '/research_catamaran/estimated_odometry',
            lambda m: setattr(self, 'pose', m.pose.pose.position), 10)
        self.create_timer(0.5, self.step)
        self.get_logger().info(f'Mission mode: {self.mode}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def disturbance_callback(self, message):
        now = self.now()
        if self.filtered is None:
            self.filtered = [message.x, message.y]
            self.last_update = now
            return
        dt = max(now - self.last_update, 1e-3)
        self.last_update = now
        tau = float(self.get_parameter('filter_tau').value)
        alpha = dt / (tau + dt)
        self.filtered[0] += alpha * (message.x - self.filtered[0])
        self.filtered[1] += alpha * (message.y - self.filtered[1])

    def send(self, values):
        request = SetParameters.Request()
        for name, value in values.items():
            parameter = Parameter(name=name)
            parameter.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
            request.parameters.append(parameter)
        self.client.call_async(request)

    def step(self):
        if not self.client.service_is_ready():
            return
        if self.mode == 'weathervane':
            self.weathervane()
        elif self.mode == 'waypoints':
            self.waypoints()

    def weathervane(self):
        if self.filtered is None:
            return
        force = math.hypot(*self.filtered)
        if force < float(self.get_parameter('min_force').value):
            return
        # The estimate points where the wind pushes; face the opposite way.
        target = math.atan2(-self.filtered[1], -self.filtered[0])
        if self.last_yaw_target is None \
                or abs(wrap(target - self.last_yaw_target)) > 0.20:
            self.last_yaw_target = target
            self.send({'target_yaw': target})
            self.get_logger().info(
                f'Weathervane: disturbance {force:.2f} N -> '
                f'bow to {math.degrees(target):.0f} deg')

    def waypoints(self):
        flat = list(self.get_parameter('waypoints').value)
        stations = [flat[i:i + 3] for i in range(0, len(flat) - 2, 3)]
        if not stations or self.pose is None:
            return
        now = self.now()
        if self.index < 0:
            self.index = 0
            self.arrived_at = now
            self.send(dict(zip(('target_x', 'target_y', 'target_yaw'),
                               stations[0])))
            return
        x, y, _ = stations[self.index]
        close = math.hypot(self.pose.x - x, self.pose.y - y) \
            < float(self.get_parameter('arrive_radius').value)
        if close and now - self.arrived_at \
                >= float(self.get_parameter('dwell').value):
            self.index = (self.index + 1) % len(stations)
            self.arrived_at = now
            self.send(dict(zip(('target_x', 'target_y', 'target_yaw'),
                               stations[self.index])))
            self.get_logger().info(
                f'Waypoint {self.index + 1}/{len(stations)}: {stations[self.index]}')
        elif not close:
            self.arrived_at = now


def main(args=None):
    rclpy.init(args=args)
    node = MissionRunner()
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
