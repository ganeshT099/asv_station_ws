"""Mix planar motion commands into the four diagonal thruster commands."""

import math
from functools import lru_cache
from typing import Any, Dict, cast

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


# Thruster geometry measured in Fortress: thrust acts along these body-frame
# unit vectors at these positions (port-front, starboard-front, port-rear,
# starboard-rear).
THRUSTER_POSITIONS = ((0.25, 0.175), (0.25, -0.175),
                      (-0.25, 0.175), (-0.25, -0.175))
_R = 2.0 ** -0.5
THRUSTER_AXES = ((_R, _R), (_R, -_R), (-_R, _R), (-_R, -_R))
# Force and moment produced by one unit of the dimensionless mixer inputs.
FORCE_PER_UNIT = 4.0 * _R
MOMENT_PER_UNIT = 4.0 * _R * (0.25 - 0.175)


def allocation_matrix():
    """Return the 3x4 map from thruster forces to (Fx, Fy, Mz)."""
    columns = []
    for (x, y), (ax, ay) in zip(THRUSTER_POSITIONS, THRUSTER_AXES):
        columns.append((ax, ay, x * ay - y * ax))
    return [[columns[j][i] for j in range(4)] for i in range(3)]


def allocate(surge, sway, yaw, health=(1.0, 1.0, 1.0, 1.0)):
    """Least-squares thrust allocation that excludes failed thrusters.

    With all four thrusters healthy this equals :func:`mix_thrusters`.  When a
    thruster is marked unhealthy (``health`` below 0.5) the remaining three are
    re-solved so the commanded force and moment are still produced exactly.
    """
    import numpy as np
    wrench = np.array([
        FORCE_PER_UNIT * surge, FORCE_PER_UNIT * sway, MOMENT_PER_UNIT * yaw])
    matrix = np.array(allocation_matrix())
    active = [i for i, h in enumerate(health) if h >= 0.5]
    commands = np.zeros(4)
    if active:
        solution, *_ = np.linalg.lstsq(matrix[:, active], wrench, rcond=None)
        commands[active] = solution
    return tuple(float(c) for c in commands)


@lru_cache(maxsize=32)
def _allocation_factors(active):
    """Cache the pseudo-inverse and null space for a set of healthy thrusters."""
    import numpy as np
    sub = np.array(allocation_matrix())[:, list(active)]
    _u, s_vals, vt = np.linalg.svd(sub)
    rank = int(np.sum(s_vals > 1e-9))
    return sub, np.linalg.pinv(sub), vt[rank:].T


def allocate_optimal(surge, sway, yaw, health=(1.0, 1.0, 1.0, 1.0),
                     limit=None):
    """Minimum-energy allocation that respects the per-thruster limit.

    Solves ``min ||T||^2  s.t.  A T = w,  |T_i| <= limit`` exactly.  Four
    thrusters give three degrees of freedom, so the exact solutions form a
    line ``T = T0 + s n`` with ``T0`` the minimum-norm solution and ``n`` the
    null-space direction (orthogonal to ``T0``).  The norm is therefore
    ``|T0|^2 + s^2`` and the optimum is the feasible ``s`` closest to zero:

    * nothing saturates  -> identical to :func:`allocate` (``s = 0``);
    * a thruster would saturate -> the null space redistributes thrust to the
      other thrusters so the *commanded wrench is still delivered*, where the
      plain least-squares solution would have to scale the whole demand down.

    The map is continuous in the wrench (no thruster-pattern jumps).  If the
    wrench is infeasible even so, translation is scaled back first (yaw keeps
    priority), then everything, preserving the force direction.

    Returns ``(commands, scale)``; ``scale`` < 1 means the demand had to be
    reduced to stay within the thrust limit.
    """
    import numpy as np
    active = [i for i, h in enumerate(health) if h >= 0.5]
    commands = np.zeros(4)
    if not active:
        return tuple(float(c) for c in commands), 0.0
    sub, pinv, null = _allocation_factors(tuple(active))
    wrench = np.array([
        FORCE_PER_UNIT * surge, FORCE_PER_UNIT * sway, MOMENT_PER_UNIT * yaw])
    if limit is None:
        commands[active] = pinv @ wrench
        return tuple(float(c) for c in commands), 1.0

    def solve(scaled):
        """Return (feasible, thrust vector) for a wrench."""
        base = pinv @ scaled
        if null.shape[1] == 0:
            ok = bool(np.all(np.abs(base) <= limit + 1e-9)) and np.allclose(
                sub @ base, scaled, atol=1e-6)
            return ok, base
        direction = null[:, 0]
        low, high = -np.inf, np.inf
        for t0, d in zip(base, direction):
            if abs(d) < 1e-12:
                if abs(t0) > limit + 1e-9:
                    return False, base
                continue
            a, b = (-limit - t0) / d, (limit - t0) / d
            low, high = max(low, min(a, b)), min(high, max(a, b))
        if low > high + 1e-12:
            return False, base
        shift = min(max(0.0, low), high)
        return True, base + shift * direction

    def with_scale(translation_scale, yaw_scale):
        return wrench * np.array(
            [translation_scale, translation_scale, yaw_scale])

    scale = 1.0
    ok, solution = solve(wrench)
    if not ok:
        # Yaw has priority: shrink translation, then everything.
        lo, hi = 0.0, 1.0
        yaw_only = solve(with_scale(0.0, 1.0))[0]
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            trial = with_scale(mid, 1.0) if yaw_only else with_scale(mid, mid)
            if solve(trial)[0]:
                lo = mid
            else:
                hi = mid
        scale = lo
        trial = with_scale(lo, 1.0) if yaw_only else with_scale(lo, lo)
        ok, solution = solve(trial)
        if not ok:
            solution = np.clip(solution, -limit, limit)
    commands[active] = solution
    return tuple(float(c) for c in commands), scale


def mix_thrusters(surge, sway, yaw):
    """
    Return port-front, starboard-front, port-rear, starboard-rear commands.

    The signs follow the four diagonal axes in the research URDF:

    - forward: ``(+,+,-,-)``
    - positive sway / left: ``(+,-,+,-)``
    - positive yaw / left turn: ``(+,-,-,+)``

    Values are intentionally dimensionless mixer inputs. The node applies the
    configured command scale and limit before publishing Float64 messages.
    """
    return (
        surge + sway + yaw,
        surge - sway - yaw,
        -surge + sway - yaw,
        -surge - sway + yaw,
    )


def roll_pitch_from_quaternion(quaternion):
    """Return roll and pitch from a ROS quaternion."""
    sin_roll = 2.0 * (quaternion.w * quaternion.x
                      + quaternion.y * quaternion.z)
    cos_roll = 1.0 - 2.0 * (quaternion.x * quaternion.x
                            + quaternion.y * quaternion.y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = 2.0 * (quaternion.w * quaternion.y
                       - quaternion.z * quaternion.x)
    pitch = math.copysign(math.pi / 2.0, sin_pitch) \
        if abs(sin_pitch) >= 1.0 else math.asin(sin_pitch)
    return roll, pitch


class TwistToThruster(Node):
    """Publish held four-thruster commands from a planar Twist command."""

    def __init__(self):
        super().__init__('research_twist_to_thruster')
        self.declare_parameter('input_topic', '/research_catamaran/cmd_vel')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('command_scale', 1.5)
        self.declare_parameter('max_thrust', 6.0)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('slew_rate', 0.35)
        self.declare_parameter('enable_tilt_guard', True)
        self.declare_parameter('tilt_soft_limit', 0.10)
        self.declare_parameter('tilt_hard_limit', 0.35)
        self.declare_parameter('imu_timeout', 0.5)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.publish_rate = self.parameter_float('publish_rate')
        self.command_scale = self.parameter_float('command_scale')
        self.max_thrust = self.parameter_float('max_thrust')
        self.command_timeout = self.parameter_float('command_timeout')
        self.slew_rate = self.parameter_float('slew_rate')
        self.enable_tilt_guard = bool(
            cast(Any, self.get_parameter('enable_tilt_guard').value))
        self.tilt_soft_limit = self.parameter_float('tilt_soft_limit')
        self.tilt_hard_limit = self.parameter_float('tilt_hard_limit')
        self.imu_timeout = self.parameter_float('imu_timeout')
        self.last_command_time = None
        self.last_publish_time = None
        self.command = (0.0, 0.0, 0.0)
        self.output_command = (0.0, 0.0, 0.0)
        self.last_imu_time = None
        self.roll = 0.0
        self.pitch = 0.0

        self.thrust_publishers: Dict[str, Any] = {
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
        self.subscription = self.create_subscription(
            Twist,
            self.input_topic,
            self.command_callback,
            10,
        )
        self.imu_subscription = self.create_subscription(
            Imu,
            '/research_catamaran/imu',
            self.imu_callback,
            10,
        )
        self.timer = self.create_timer(
            1.0 / max(self.publish_rate, 1.0),
            self.publish_step,
        )
        self.get_logger().info(
            'Listening on %s. Use normalized linear.x / linear.y / '
            'angular.z commands; slew rate %.2f; tilt guard %s.' % (
                self.input_topic, self.slew_rate,
                'on' if self.enable_tilt_guard else 'off'))

    def parameter_float(self, name):
        """Read a numeric parameter as a Python float."""
        value = cast(Any, self.get_parameter(name).value)
        return float(value)

    def command_callback(self, message):
        """Store the latest normalized surge, sway and yaw command."""
        self.command = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        )
        self.last_command_time = self.get_clock().now().nanoseconds * 1e-9

    def imu_callback(self, message):
        """Track roll and pitch for the conservative motion safety guard."""
        self.roll, self.pitch = roll_pitch_from_quaternion(
            message.orientation)
        self.last_imu_time = self.get_clock().now().nanoseconds * 1e-9

    def tilt_scale(self, now):
        """Return a thrust scale that fades to zero at the hard tilt limit."""
        if not self.enable_tilt_guard:
            return 1.0
        if self.last_imu_time is None:
            return 0.0
        if now - self.last_imu_time > self.imu_timeout:
            return 0.0
        tilt = max(abs(self.roll), abs(self.pitch))
        if tilt >= self.tilt_hard_limit:
            return 0.0
        if tilt <= self.tilt_soft_limit:
            return 1.0
        limit_range = self.tilt_hard_limit - self.tilt_soft_limit
        return (self.tilt_hard_limit - tilt) / max(limit_range, 1e-6)

    def publish_step(self):
        """Publish a slew-limited command or zero after the safety timeout."""
        now = self.get_clock().now().nanoseconds * 1e-9
        timed_out = (
            self.last_command_time is None
            or now - self.last_command_time > self.command_timeout
        )
        target = (0.0, 0.0, 0.0) if timed_out else self.command
        safety_scale = self.tilt_scale(now)
        target = tuple(value * safety_scale for value in target)
        if self.last_publish_time is None:
            step = 0.0
        else:
            step = max(0.0, now - self.last_publish_time)
        self.last_publish_time = now

        allowed_change = max(0.0, self.slew_rate) * step
        self.output_command = tuple(
            self.move_toward(current, desired, allowed_change)
            for current, desired in zip(self.output_command, target)
        )
        mixed = mix_thrusters(*self.output_command)
        scaled = tuple(value * self.command_scale for value in mixed)
        peak = max(abs(value) for value in scaled)
        if peak > self.max_thrust:
            scaled = tuple(
                value * self.max_thrust / peak for value in scaled)

        for publisher, value in zip(self.thrust_publishers.values(), scaled):
            message = Float64()
            message.data = value
            publisher.publish(message)

    @staticmethod
    def move_toward(current, target, maximum_change):
        """Move one scalar toward a target by at most the given amount."""
        difference = target - current
        if abs(difference) <= maximum_change:
            return target
        return current + maximum_change * (1.0 if difference > 0 else -1.0)

    def destroy_node(self):
        self.command = (0.0, 0.0, 0.0)
        self.output_command = (0.0, 0.0, 0.0)
        self.publish_step()
        super().destroy_node()


def main(args=None):
    """Run the reusable Twist-to-thruster mixer."""
    rclpy.init(args=args)
    node = TwistToThruster()
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
