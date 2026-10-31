"""Measure how much localization error corrupts a lidar map of the pool.

Every scan is projected into the world twice: with the *estimated* pose (what
a real robot would use to build its map) and with the simulator's true pose
(the lidar-noise floor).  The distance of each projected point to the known
pool walls is the map error.  If the vehicle drifts or the estimate degrades
(GPS loss, bias, bad holding), the estimated-pose map smears while the
true-pose map stays sharp, which is the coupling this project studies.
"""

import collections
import csv
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

# Inner faces of the pool walls in the world file.
WALL_X, WALL_Y = 5.0, 4.0


def wall_distance(x, y):
    """Distance from a world point to the nearest inner wall of the pool."""
    return min(abs(WALL_X - abs(x)), abs(WALL_Y - abs(y)))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MapQuality(Node):
    """Accumulate map error statistics from lidar and two pose sources."""

    def __init__(self):
        super().__init__('research_map_quality')
        self.declare_parameter('scan_topic', '/research_catamaran/scan')
        self.declare_parameter(
            'estimate_topic', '/research_catamaran/estimated_odometry')
        self.declare_parameter('truth_topic', '/model/research_catamaran/odometry')
        self.declare_parameter('window_scans', 100)
        self.declare_parameter('log_file', '')
        self.estimate = None
        self.truth = None
        window = int(self.get_parameter('window_scans').value)
        self.est_errors = collections.deque(maxlen=window)
        self.true_errors = collections.deque(maxlen=window)
        self.map_points = collections.deque(maxlen=1500)
        self.create_subscription(
            Odometry, str(self.get_parameter('estimate_topic').value),
            lambda m: setattr(self, 'estimate', m), 10)
        self.create_subscription(
            Odometry, str(self.get_parameter('truth_topic').value),
            lambda m: setattr(self, 'truth', m), 10)
        self.create_subscription(
            LaserScan, str(self.get_parameter('scan_topic').value),
            self.scan_callback, 5)
        self.error_pub = self.create_publisher(
            Vector3, '/research_catamaran/mapping/error', 10)
        self.marker_pub = self.create_publisher(
            Marker, '/research_catamaran/mapping/map_points', 10)
        self.csv_file = self.csv_writer = None
        path = str(self.get_parameter('log_file').value)
        if path:
            target = Path(path).expanduser()
            target.parent.mkdir(parents=True, exist_ok=True)
            self.csv_file = target.open('w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(
                ['time', 'map_rms_est', 'map_rms_true', 'map_p95_est'])
        self.scan_count = 0

    @staticmethod
    def project(scan, pose):
        x0, y0 = pose.position.x, pose.position.y
        yaw = yaw_of(pose.orientation)
        points = []
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
                continue
            a = yaw + scan.angle_min + i * scan.angle_increment
            points.append((x0 + r * math.cos(a), y0 + r * math.sin(a)))
        return points

    def scan_callback(self, scan):
        if self.estimate is None or self.truth is None:
            return
        est_pts = self.project(scan, self.estimate.pose.pose)
        true_pts = self.project(scan, self.truth.pose.pose)
        self.est_errors.extend(wall_distance(*p) for p in est_pts)
        self.true_errors.extend(wall_distance(*p) for p in true_pts)
        self.map_points.extend(est_pts[::6])
        self.scan_count += 1
        if not self.est_errors:
            return
        rms_est = math.sqrt(sum(e * e for e in self.est_errors)
                            / len(self.est_errors))
        rms_true = math.sqrt(sum(e * e for e in self.true_errors)
                             / max(len(self.true_errors), 1))
        ordered = sorted(self.est_errors)
        p95 = ordered[int(0.95 * (len(ordered) - 1))]
        message = Vector3(x=rms_est, y=rms_true, z=p95)
        self.error_pub.publish(message)
        if self.csv_writer is not None:
            now = self.get_clock().now().nanoseconds * 1e-9
            self.csv_writer.writerow(
                [f'{now:.3f}', f'{rms_est:.5f}', f'{rms_true:.5f}',
                 f'{p95:.5f}'])
            self.csv_file.flush()
        if self.scan_count % 5 == 0:
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.ns, marker.id = 'map', 0
            marker.type, marker.action = Marker.POINTS, Marker.ADD
            marker.scale.x = marker.scale.y = 0.04
            marker.color.r, marker.color.g, marker.color.b = 0.1, 0.8, 0.4
            marker.color.a = 1.0
            marker.points = [Point(x=x, y=y, z=0.05)
                             for x, y in self.map_points]
            self.marker_pub.publish(marker)

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MapQuality()
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
