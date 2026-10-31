"""Run and score a repeatable four-direction motion stability experiment."""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def euler_from_quaternion(quaternion):
    """Return roll, pitch and yaw from a ROS quaternion."""
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sin_pitch) \
        if abs(sin_pitch) >= 1.0 else math.asin(sin_pitch)
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return roll, pitch, yaw


def wrap_angle(angle):
    """Wrap an angle to the interval [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class MotionValidation(Node):
    """Publish maneuvers and evaluate direction and attitude stability."""

    def __init__(self):
        super().__init__('research_motion_validation')
        self.declare_parameter('command', 0.15)
        self.declare_parameter('settle_time', 3.0)
        self.declare_parameter('neutral_time', 1.5)
        self.declare_parameter('maneuver_time', 3.0)
        self.declare_parameter('min_displacement', 0.05)
        self.declare_parameter('max_attitude', 0.35)
        self.declare_parameter('max_z_excursion', 0.30)

        self.command = float(self.get_parameter('command').value)
        self.settle_time = float(self.get_parameter('settle_time').value)
        self.neutral_time = float(self.get_parameter('neutral_time').value)
        self.maneuver_time = float(
            self.get_parameter('maneuver_time').value)
        self.min_displacement = float(
            self.get_parameter('min_displacement').value)
        self.max_attitude = float(self.get_parameter('max_attitude').value)
        self.max_z_excursion = float(
            self.get_parameter('max_z_excursion').value)
        self.publisher = self.create_publisher(
            Twist, '/research_catamaran/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/model/research_catamaran/odometry',
            self.odom_callback,
            10,
        )
        self.latest_odom = None
        self.phase = 'startup'
        self.samples = {}

    def odom_callback(self, message):
        """Store the latest odometry sample under the active phase."""
        self.latest_odom = message
        if self.phase == 'startup':
            return
        roll, pitch, yaw = euler_from_quaternion(
            message.pose.pose.orientation)
        self.samples.setdefault(self.phase, []).append({
            'x': message.pose.pose.position.x,
            'y': message.pose.pose.position.y,
            'z': message.pose.pose.position.z,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
        })

    def publish_command(self, surge=0.0, sway=0.0, yaw=0.0):
        """Publish one normalized planar command."""
        message = Twist()
        message.linear.x = surge
        message.linear.y = sway
        message.angular.z = yaw
        self.publisher.publish(message)

    def spin_for(self, duration, command):
        """Publish a command and process callbacks for wall-clock duration."""
        start = time.monotonic()
        next_publish = start
        while rclpy.ok() and time.monotonic() - start < duration:
            now = time.monotonic()
            if now >= next_publish:
                self.publish_command(*command)
                next_publish = now + 0.05
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for_odometry(self):
        """Wait for the model state before starting the measurement phases."""
        start = time.monotonic()
        while rclpy.ok() and self.latest_odom is None:
            if time.monotonic() - start > 15.0:
                return False
            self.publish_command()
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.latest_odom is not None

    def run(self):
        """Run the validation sequence and return whether it passed."""
        if not self.wait_for_odometry():
            self.get_logger().error('No odometry received within 15 seconds.')
            return False

        self.phase = 'settle'
        self.spin_for(self.settle_time, (0.0, 0.0, 0.0))
        sequence = (
            ('forward', (self.command, 0.0, 0.0)),
            ('backward', (-self.command, 0.0, 0.0)),
            ('left', (0.0, 0.0, self.command)),
            ('right', (0.0, 0.0, -self.command)),
        )
        for name, command in sequence:
            self.phase = name
            self.spin_for(self.maneuver_time, command)
            self.phase = f'{name}_neutral'
            self.spin_for(self.neutral_time, (0.0, 0.0, 0.0))
        self.phase = 'complete'
        self.publish_command()
        return self.report(sequence)

    def report(self, sequence):
        """Print measured results and return the acceptance result."""
        passed = True
        motion_results = {}
        for name, _ in sequence:
            samples = self.samples.get(name, [])
            if len(samples) < 2:
                self.get_logger().error('%s: insufficient odometry samples', name)
                passed = False
                continue
            first = samples[0]
            last = samples[-1]
            dx = last['x'] - first['x']
            dy = last['y'] - first['y']
            dyaw = wrap_angle(last['yaw'] - first['yaw'])
            max_roll = max(abs(sample['roll']) for sample in samples)
            max_pitch = max(abs(sample['pitch']) for sample in samples)
            motion_results[name] = (dx, dy, dyaw, max_roll, max_pitch)
            if name == 'forward' and dx <= self.min_displacement:
                passed = False
            if name == 'backward' and dx >= -self.min_displacement:
                passed = False
            if name == 'left' and dyaw <= self.min_displacement:
                passed = False
            if name == 'right' and dyaw >= -self.min_displacement:
                passed = False
            if max(max_roll, max_pitch) > self.max_attitude:
                passed = False

        surge_samples = self.samples.get('forward', []) + self.samples.get(
            'backward', [])
        if surge_samples:
            z_values = [sample['z'] for sample in surge_samples]
            z_excursion = max(z_values) - min(z_values)
            if z_excursion > self.max_z_excursion:
                passed = False
        else:
            z_excursion = float('inf')
            passed = False

        for name in ('forward', 'backward', 'left', 'right'):
            if name not in motion_results:
                continue
            dx, dy, dyaw, max_roll, max_pitch = motion_results[name]
            print(
                f'{name:8s} dx={dx:+.3f} dy={dy:+.3f} '
                f'dyaw={dyaw:+.3f} max_roll={max_roll:.3f} '
                f'max_pitch={max_pitch:.3f}')
        print(f'surge_z_excursion={z_excursion:.3f}')
        print('MOTION_VALIDATION_PASS' if passed else 'MOTION_VALIDATION_FAIL')
        return passed


def main(args=None):
    """Run the motion validation executable."""
    rclpy.init(args=args)
    node = MotionValidation()
    try:
        result = node.run()
    except KeyboardInterrupt:
        result = False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if result else 1


if __name__ == '__main__':
    sys.exit(main())
