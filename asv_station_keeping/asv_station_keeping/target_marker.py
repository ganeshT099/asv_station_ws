"""Move the green station marker in Gazebo to the controller's live target."""

import math
import subprocess
import threading

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node


class TargetMarker(Node):
    """Follow the active target so the guide can see what is being held."""

    def __init__(self):
        super().__init__('research_target_marker')
        self.declare_parameter('world', 'asv_research_pool')
        self.declare_parameter('model', 'target_marker')
        self.declare_parameter('height', 0.02)
        self.world = str(self.get_parameter('world').value)
        self.model = str(self.get_parameter('model').value)
        self.height = float(self.get_parameter('height').value)
        self.last_sent = None
        self.pending = None
        self.busy = False
        self.create_subscription(
            DiagnosticArray, '/research_catamaran/station_keeping/status',
            self.status_callback, 10)
        self.create_timer(0.5, self.flush)

    def status_callback(self, message):
        for status in message.status:
            values = {item.key: item.value for item in status.values}
            if 'target_x_m' not in values:
                continue
            try:
                self.pending = tuple(round(float(values[key]), 3) for key in (
                    'target_x_m', 'target_y_m', 'target_yaw_rad'))
            except ValueError:
                return

    def flush(self):
        """Send a set_pose request only when the target actually moved."""
        if self.busy or self.pending is None or self.pending == self.last_sent:
            return
        target = self.pending
        self.busy = True
        threading.Thread(target=self.send, args=(target,), daemon=True).start()

    def send(self, target):
        x, y, yaw = target
        request = (
            f'name: "{self.model}" '
            f'position {{x: {x} y: {y} z: {self.height}}} '
            f'orientation {{x: 0 y: 0 z: {math.sin(yaw / 2.0):.6f} '
            f'w: {math.cos(yaw / 2.0):.6f}}}')
        try:
            result = subprocess.run(
                ['ign', 'service', '-s', f'/world/{self.world}/set_pose',
                 '--reqtype', 'ignition.msgs.Pose',
                 '--reptype', 'ignition.msgs.Boolean',
                 '--timeout', '1000', '--req', request],
                capture_output=True, text=True, timeout=5.0, check=False)
            if 'true' in result.stdout:
                self.last_sent = target
        except (OSError, subprocess.TimeoutExpired):
            pass
        finally:
            self.busy = False


def main(args=None):
    """Run the target marker follower."""
    rclpy.init(args=args)
    node = TargetMarker()
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
