"""Scripted, narrated demonstration timeline with live fault injection.

Each event sets a parameter on a running node, so the audience watches the
vehicle react.  The default ``narrated`` timeline walks through: a wind step,
a GPS dropout, a thruster failure (with re-allocation after a detection
delay), a GPS jump and finally bow-into-wind weathervaning.  Add your own
with a JSON list via ``events_json`` of ``[time_s, node, param, value, text]``.
"""

import json

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_msgs.msg import String

CONTROLLER = '/research_station_keeping_pid'
ESTIMATOR = '/research_state_estimator'
SEA = '/research_current_disturbance'

NARRATED = [
    [5, SEA, 'wind_speed', 4.0, 'Wind rises to 4 m/s'],
    [20, ESTIMATOR, 'gps_enabled', False,
     'FAULT: GPS signal lost - dead-reckoning on the IMU'],
    [28, ESTIMATOR, 'gps_enabled', True, 'GPS restored'],
    [40, CONTROLLER, 'thruster_gain_1', 0.0,
     'FAULT: starboard-front thruster fails'],
    [42, CONTROLLER, 'allocator_health_1', 0.0,
     'Fault detected - thrust re-allocated to the 3 healthy thrusters'],
    [60, CONTROLLER, 'thruster_gain_1', 1.0, 'Thruster repaired'],
    [60, CONTROLLER, 'allocator_health_1', 1.0, 'Full thrust allocation'],
    [70, ESTIMATOR, 'gps_offset_x', 0.4,
     'FAULT: GPS jumps 0.4 m (multipath)'],
    [78, ESTIMATOR, 'gps_offset_x', 0.0, 'GPS jump cleared'],
    [90, CONTROLLER, 'use_dob', True,
     'Disturbance observer ON - wind is now fed forward'],
    [105, SEA, 'wind_direction_deg', 300.0, 'Wind direction swings 165 deg'],
]


class ScenarioRunner(Node):
    """Fire timed parameter events on the live nodes."""

    def __init__(self):
        super().__init__('research_scenario_runner')
        self.declare_parameter('scenario', 'narrated')
        self.declare_parameter('events_json', '')
        self.declare_parameter('start_delay', 12.0)
        text = str(self.get_parameter('events_json').value)
        name = str(self.get_parameter('scenario').value)
        self.events = json.loads(text) if text else (
            NARRATED if name == 'narrated' else [])
        self.events = sorted(self.events, key=lambda e: e[0])
        self.param_clients = {}
        self.status = self.create_publisher(
            String, '/research_catamaran/scenario/status', 10)
        self.t0 = None
        self.index = 0
        self.create_timer(0.25, self.step)
        self.get_logger().info(
            f'Scenario "{name}" with {len(self.events)} events.')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def step(self):
        if self.t0 is None:
            if self.now() >= float(self.get_parameter('start_delay').value):
                self.t0 = self.now()
            return
        elapsed = self.now() - self.t0
        while self.index < len(self.events) \
                and self.events[self.index][0] <= elapsed:
            self.fire(self.events[self.index])
            self.index += 1

    def fire(self, event):
        _time, node, param, value, text = event
        client = self.param_clients.setdefault(
            node, self.create_client(SetParameters, node + '/set_parameters'))
        if not client.service_is_ready():
            self.get_logger().warning(f'{node} not ready for "{text}"')
        parameter = Parameter(name=param)
        parameter.value = ParameterValue()
        if isinstance(value, bool):
            parameter.value.type = ParameterType.PARAMETER_BOOL
            parameter.value.bool_value = value
        else:
            parameter.value.type = ParameterType.PARAMETER_DOUBLE
            parameter.value.double_value = float(value)
        request = SetParameters.Request()
        request.parameters = [parameter]
        client.call_async(request)
        self.get_logger().info(f'[SCENARIO t={_time:g}s] {text}')
        self.status.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioRunner()
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
