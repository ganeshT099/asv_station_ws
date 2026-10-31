"""Repeatable two-phase station-keeping demonstration scenario."""

import time
from typing import Any, Optional

import rclpy
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_msgs.msg import Empty


CONTROLLER_NODE = '/research_station_keeping_pid'
CURRENT_NODE = '/research_current_disturbance'
START_TOPIC = '/research_catamaran/station_keeping/start_demo'
RESET_TOPIC = '/research_catamaran/station_keeping/reset_integrators'


class StationKeepingDemo(Node):
    """Move between two stations while changing the water current."""

    def __init__(self):
        super().__init__('research_station_keeping_demo')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('start_delay', 2.0)
        self.declare_parameter('phase_duration', 12.0)
        self.declare_parameter('initial_target_x', 0.40)
        self.declare_parameter('initial_target_y', 0.20)
        self.declare_parameter('initial_target_yaw', 0.20)
        self.declare_parameter('initial_current_x', 0.10)
        self.declare_parameter('initial_current_y', 0.04)
        self.declare_parameter('initial_current_z', 0.00)
        self.declare_parameter('challenge_target_x', -0.35)
        self.declare_parameter('challenge_target_y', -0.22)
        self.declare_parameter('challenge_target_yaw', -0.20)
        self.declare_parameter('challenge_current_x', 0.04)
        self.declare_parameter('challenge_current_y', -0.08)
        self.declare_parameter('challenge_current_z', 0.00)
        # Wind shift during the challenge phase; negative speed leaves the
        # launch-time wind untouched.
        self.declare_parameter('challenge_wind_speed', 5.0)
        self.declare_parameter('challenge_wind_direction_deg', -45.0)

        self.controller_client = self.create_client(
            SetParameters, CONTROLLER_NODE + '/set_parameters')
        self.current_client = self.create_client(
            SetParameters, CURRENT_NODE + '/set_parameters')
        self.reset_pub = self.create_publisher(Empty, RESET_TOPIC, 10)
        self.start_sub = self.create_subscription(
            Empty, START_TOPIC, self.start_callback, 10)
        self.timer = self.create_timer(0.25, self.step)

        self.phase = -1
        self.phase_started_at: Optional[float] = None
        self.start_requested_at = self._now()
        self.pending_phase: Optional[int] = None
        self.pending_requests = 0
        self.pending_failed = False
        self.get_logger().info(
            'Station-keeping demo ready; phases change station and current.')

    def start_callback(self, _message: Empty):
        """Start or restart the scripted demonstration."""
        if self.pending_phase is not None:
            return
        self.phase = -1
        self.phase_started_at = None
        self.start_requested_at = self._now()
        self.get_logger().info('Station-keeping demo requested.')

    def step(self):
        """Advance the scenario state machine without blocking ROS callbacks."""
        now = self._now()
        if self.phase < 0:
            auto_start = bool(self.get_parameter('auto_start').value)
            delay = max(float(self.get_parameter('start_delay').value), 0.0)
            if auto_start and now - self.start_requested_at >= delay:
                self._request_phase(0)
            return
        if self.pending_phase is not None or self.phase_started_at is None:
            return
        duration = max(float(self.get_parameter('phase_duration').value), 1.0)
        if now - self.phase_started_at < duration:
            return
        if self.phase == 0:
            self._request_phase(1)
        else:
            self.phase = 2
            self.get_logger().info(
                'Station-keeping demo complete; controller is holding the '
                'challenge station.')

    def _request_phase(self, phase: int):
        """Send one phase's target and current values to the live nodes."""
        if not self.controller_client.service_is_ready():
            self.controller_client.wait_for_service(timeout_sec=0.0)
        if not self.current_client.service_is_ready():
            self.current_client.wait_for_service(timeout_sec=0.0)
        if (
            not self.controller_client.service_is_ready()
            or not self.current_client.service_is_ready()
        ):
            return

        prefix = 'initial_' if phase == 0 else 'challenge_'
        values = {
            'target_x': self._float_parameter(prefix + 'target_x'),
            'target_y': self._float_parameter(prefix + 'target_y'),
            'target_yaw': self._float_parameter(prefix + 'target_yaw'),
            'current_x': self._float_parameter(prefix + 'current_x'),
            'current_y': self._float_parameter(prefix + 'current_y'),
            'current_z': self._float_parameter(prefix + 'current_z'),
        }
        controller_values = {
            key: value for key, value in values.items()
            if key.startswith('target_')
        }
        current_values = {
            key: value for key, value in values.items()
            if key.startswith('current_')
        }
        if phase == 1:
            wind_speed = self._float_parameter('challenge_wind_speed')
            if wind_speed >= 0.0:
                current_values['wind_speed'] = wind_speed
                current_values['wind_direction_deg'] = self._float_parameter(
                    'challenge_wind_direction_deg')
        self.pending_phase = phase
        self.pending_requests = 2
        self.pending_failed = False
        self._call_parameters(
            self.controller_client,
            controller_values,
        )
        self._call_parameters(self.current_client, current_values)

    def _call_parameters(self, client, values):
        request = SetParameters.Request()
        request.parameters = [
            self._ros_parameter(name, value)
            for name, value in values.items()
        ]
        future = client.call_async(request)
        future.add_done_callback(self._parameter_done)

    def _parameter_done(self, future):
        try:
            results = future.result().results
            if not results or not all(result.successful for result in results):
                self.pending_failed = True
        except Exception:
            self.pending_failed = True
        self.pending_requests -= 1
        if self.pending_requests > 0:
            return
        phase = self.pending_phase
        self.pending_phase = None
        if self.pending_failed or phase is None:
            self.phase = -1
            self.start_requested_at = self._now()
            self.get_logger().warning(
                'Demo phase parameter update failed; will retry.')
            return
        self.reset_pub.publish(Empty())
        self.phase = phase
        self.phase_started_at = self._now()
        self.get_logger().info(
            f'Demo phase {phase + 1}/2 active: '
            f'hold target and reject configured current.')

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _now(self) -> float:
        """Use simulation time when available, with a startup fallback."""
        simulation_time = self.get_clock().now().nanoseconds * 1e-9
        return simulation_time if simulation_time > 0.0 else time.monotonic()

    @staticmethod
    def _ros_parameter(name: str, value: Any) -> Parameter:
        parameter = Parameter()
        parameter.name = name
        parameter.value = ParameterValue()
        parameter.value.type = ParameterType.PARAMETER_DOUBLE
        parameter.value.double_value = float(value)
        return parameter


def main(args=None):
    """Run the repeatable station-keeping demonstration scenario."""
    rclpy.init(args=args)
    node = StationKeepingDemo()
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
