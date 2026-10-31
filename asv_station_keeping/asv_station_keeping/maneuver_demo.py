"""Publish one repeatable forward/backward/turn maneuver then stop."""

from typing import Any, cast

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


MANEUVERS = {
    'forward': (1.0, 0.0, 0.0),
    'backward': (-1.0, 0.0, 0.0),
    'left': (0.0, 0.0, 1.0),
    'right': (0.0, 0.0, -1.0),
    'strafe_left': (0.0, 1.0, 0.0),
    'strafe_right': (0.0, -1.0, 0.0),
    'stop': (0.0, 0.0, 0.0),
}


class ManeuverDemo(Node):
    """Send a normalized command for a fixed duration."""

    def __init__(self):
        super().__init__('research_maneuver_demo')
        self.declare_parameter('maneuver', 'forward')
        self.declare_parameter('duration', 3.0)
        self.declare_parameter('command', 0.5)
        self.declare_parameter('topic', '/research_catamaran/cmd_vel')
        self.declare_parameter('publish_rate', 20.0)

        maneuver = str(self.get_parameter('maneuver').value)
        if maneuver not in MANEUVERS:
            choices = ', '.join(sorted(MANEUVERS))
            raise ValueError(
                f'Unknown maneuver {maneuver!r}; choose one of {choices}.')
        self.vector = MANEUVERS[maneuver]
        self.done = False
        self.command = float(cast(Any, self.get_parameter('command').value))
        self.duration = float(
            cast(Any, self.get_parameter('duration').value))
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.publisher = self.create_publisher(
            Twist,
            str(self.get_parameter('topic').value),
            10,
        )
        self.timer = self.create_timer(
            1.0 / max(
                float(cast(Any, self.get_parameter('publish_rate').value)),
                1.0,
            ),
            self.publish_step,
        )
        self.get_logger().info(
            f'Maneuver {maneuver!r} for {self.duration:.2f} s.')

    def publish_step(self):
        """Publish the active command and stop when the duration ends."""
        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - self.start_time
        message = Twist()
        if elapsed <= self.duration:
            message.linear.x = self.command * self.vector[0]
            message.linear.y = self.command * self.vector[1]
            message.angular.z = self.command * self.vector[2]
        else:
            self.publisher.publish(message)
            self.get_logger().info('Maneuver complete; published stop.')
            self.timer.cancel()
            self.done = True
            return
        self.publisher.publish(message)


def main(args=None):
    """Run the maneuver demo."""
    rclpy.init(args=args)
    node = ManeuverDemo()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ValueError) as error:
        if isinstance(error, ValueError):
            node.get_logger().error(str(error))
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
