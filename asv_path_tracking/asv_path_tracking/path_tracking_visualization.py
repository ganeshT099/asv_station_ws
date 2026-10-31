"""RViz visualization for the standalone geometric path-tracking experiment."""

from __future__ import annotations

import math
import time
from collections import deque
from typing import Any, Deque, Dict, Optional, Tuple

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point, PoseStamped, TransformStamped, Vector3
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


ODOM_TOPIC = '/model/research_catamaran/odometry'
REFERENCE_PATH_TOPIC = '/asv_path_tracking/reference_path'
REFERENCE_POSE_TOPIC = '/asv_path_tracking/reference_pose'
REFERENCE_MARKERS_TOPIC = '/asv_path_tracking/reference_markers'
CONTROLLER_STATUS_TOPIC = '/asv_path_tracking/status'
PATH_STATUS_TOPIC = '/asv_path_tracking/path_status'
CURRENT_TOPIC = '/ocean_current'
ACTUAL_PATH_TOPIC = '/asv_path_tracking/actual_path'
MARKERS_TOPIC = '/asv_path_tracking/markers'
RESET_TOPIC = '/asv_path_tracking/reset'


def path_qos() -> QoSProfile:
    """Return durable QoS for a path that RViz may subscribe to late."""
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def parse_values(message: DiagnosticArray) -> Dict[str, str]:
    """Flatten the first diagnostic status into a string dictionary."""
    if not message.status:
        return {}
    return {item.key: item.value for item in message.status[0].values}


def value(values: Dict[str, str], key: str, default: float = 0.0) -> float:
    """Read one finite diagnostic float."""
    try:
        parsed = float(values.get(key, default))
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw from a quaternion."""
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


class PathTrackingVisualization(Node):
    """Publish an actual path, TF, vectors, labels, and safety markers."""

    def __init__(self) -> None:
        super().__init__('asv_path_tracking_visualization')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('path_min_distance', 0.01)
        self.declare_parameter('max_path_length', 3000)
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('path_z', 0.10)
        self.declare_parameter('current_z', 0.45)
        self.declare_parameter('velocity_z', 0.30)
        self.declare_parameter('vector_scale', 1.2)
        self.declare_parameter('line_width', 0.035)
        self.declare_parameter('broadcast_tf', True)

        self.publish_rate = max(
            float(self.get_parameter('publish_rate').value), 1.0)
        self.path_min_distance = max(
            float(self.get_parameter('path_min_distance').value), 0.0)
        self.max_path_length = max(
            int(self.get_parameter('max_path_length').value), 2)
        self.frame_id = str(self.get_parameter('frame_id').value).strip() or 'world'
        self.path_z = float(self.get_parameter('path_z').value)
        self.current_z = float(self.get_parameter('current_z').value)
        self.velocity_z = float(self.get_parameter('velocity_z').value)
        self.vector_scale = max(
            float(self.get_parameter('vector_scale').value), 0.0)
        self.line_width = max(
            float(self.get_parameter('line_width').value), 0.005)
        self.broadcast_tf = bool(self.get_parameter('broadcast_tf').value)

        self.latest_odom: Optional[Odometry] = None
        self.reference_pose: Optional[PoseStamped] = None
        self.reference_path: Optional[Path] = None
        self.controller_values: Dict[str, str] = {}
        self.controller_message = 'waiting for controller'
        self.path_values: Dict[str, str] = {}
        self.path_message = 'waiting for path reference'
        self.current = (0.0, 0.0, 0.0)
        self.received_odom: Optional[float] = None
        self.received_reference: Optional[float] = None
        self.actual_path: Deque[PoseStamped] = deque(
            maxlen=self.max_path_length)
        self.tf_broadcaster = TransformBroadcaster(self) \
            if self.broadcast_tf else None

        self.actual_path_pub = self.create_publisher(
            Path, ACTUAL_PATH_TOPIC, path_qos())
        self.marker_pub = self.create_publisher(
            MarkerArray, MARKERS_TOPIC, 10)
        self.odom_sub = self.create_subscription(
            Odometry, ODOM_TOPIC, self.odom_callback, 10)
        self.reference_pose_sub = self.create_subscription(
            PoseStamped, REFERENCE_POSE_TOPIC, self.reference_pose_callback, 10)
        self.reference_path_sub = self.create_subscription(
            Path,
            REFERENCE_PATH_TOPIC,
            self.reference_path_callback,
            path_qos(),
        )
        self.controller_sub = self.create_subscription(
            DiagnosticArray,
            CONTROLLER_STATUS_TOPIC,
            self.controller_status_callback,
            10,
        )
        self.path_status_sub = self.create_subscription(
            DiagnosticArray,
            PATH_STATUS_TOPIC,
            self.path_status_callback,
            10,
        )
        self.current_sub = self.create_subscription(
            Vector3, CURRENT_TOPIC, self.current_callback, 10)
        self.reset_sub = self.create_subscription(
            Empty, RESET_TOPIC, self.reset_callback, 10)
        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.publish)
        self.get_logger().info('Path-tracking RViz visualization ready.')

    def odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message
        self.received_odom = time.monotonic()
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = message.header.stamp
        pose.pose = message.pose.pose
        if not self.actual_path or self._distance(
                pose, self.actual_path[-1]) >= self.path_min_distance:
            self.actual_path.append(pose)
        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = message.header.stamp
            transform.header.frame_id = self.frame_id
            transform.child_frame_id = 'base_footprint'
            transform.transform.translation.x = pose.pose.position.x
            transform.transform.translation.y = pose.pose.position.y
            transform.transform.translation.z = pose.pose.position.z
            transform.transform.rotation = pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def reference_pose_callback(self, message: PoseStamped) -> None:
        self.reference_pose = message
        self.received_reference = time.monotonic()

    def reference_path_callback(self, message: Path) -> None:
        self.reference_path = message

    def controller_status_callback(self, message: DiagnosticArray) -> None:
        self.controller_values = parse_values(message)
        self.controller_message = (
            message.status[0].message if message.status else 'unknown')

    def path_status_callback(self, message: DiagnosticArray) -> None:
        self.path_values = parse_values(message)
        self.path_message = message.status[0].message if message.status else 'unknown'

    def current_callback(self, message: Vector3) -> None:
        self.current = (float(message.x), float(message.y), float(message.z))

    def reset_callback(self, _message: Empty) -> None:
        """Clear the actual trace when a new experiment is requested."""
        self.actual_path.clear()
        self.get_logger().info('Path-tracking actual trace reset.')

    def publish(self) -> None:
        """Publish the actual path and one stable MarkerArray snapshot."""
        stamp = self.get_clock().now().to_msg()
        actual = Path()
        actual.header.frame_id = self.frame_id
        actual.header.stamp = stamp
        actual.poses = list(self.actual_path)
        self.actual_path_pub.publish(actual)
        self.marker_pub.publish(self._markers(stamp))

    def _markers(self, stamp: Any) -> MarkerArray:
        markers = MarkerArray()
        if len(self.actual_path) >= 2:
            markers.markers.append(self._line_marker(
                'asv_path_tracking/actual',
                1,
                [
                    Point(
                        x=pose.pose.position.x,
                        y=pose.pose.position.y,
                        z=pose.pose.position.z + self.path_z,
                    )
                    for pose in self.actual_path
                ],
                (0.05, 0.95, 0.95, 0.98),
                stamp,
                self.line_width,
            ))
        else:
            markers.markers.append(self._delete('asv_path_tracking/actual', 1, stamp))

        if self.reference_pose is not None:
            ref = self.reference_pose.pose
            markers.markers.append(self._sphere(
                'asv_path_tracking/reference',
                10,
                ref.position.x,
                ref.position.y,
                self.path_z + 0.04,
                (1.0, 0.15, 0.85, 0.98),
                0.13,
                stamp,
            ))
            ref_yaw = yaw_from_quaternion(
                ref.orientation.x,
                ref.orientation.y,
                ref.orientation.z,
                ref.orientation.w,
            )
            markers.markers.append(self._arrow(
                'asv_path_tracking/reference',
                11,
                (ref.position.x, ref.position.y, self.path_z + 0.05),
                (
                    ref.position.x + 0.35 * math.cos(ref_yaw),
                    ref.position.y + 0.35 * math.sin(ref_yaw),
                    self.path_z + 0.05,
                ),
                (1.0, 0.15, 0.85, 0.95),
                stamp,
            ))
        else:
            markers.markers.extend([
                self._delete('asv_path_tracking/reference', 10, stamp),
                self._delete('asv_path_tracking/reference', 11, stamp),
            ])

        if self.latest_odom is not None:
            pose = self.latest_odom.pose.pose
            yaw = yaw_from_quaternion(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
            position = (pose.position.x, pose.position.y, pose.position.z)
            if self.reference_pose is not None:
                ref = self.reference_pose.pose.position
                markers.markers.append(self._arrow(
                    'asv_path_tracking/error',
                    20,
                    position,
                    (ref.x, ref.y, position[2]),
                    (1.0, 0.2, 0.1, 0.9),
                    stamp,
                ))
            linear = self.latest_odom.twist.twist.linear
            world_velocity = self._rotate_z(
                (linear.x, linear.y, linear.z), yaw)
            markers.markers.append(self._arrow(
                'asv_path_tracking/velocity',
                30,
                (position[0], position[1], position[2] + self.velocity_z),
                (
                    position[0] + world_velocity[0] * self.vector_scale,
                    position[1] + world_velocity[1] * self.vector_scale,
                    position[2] + self.velocity_z
                    + world_velocity[2] * self.vector_scale,
                ),
                (0.2, 0.9, 0.3, 0.9),
                stamp,
            ))
            markers.markers.append(self._arrow(
                'asv_path_tracking/current',
                31,
                (position[0], position[1], position[2] + self.current_z),
                (
                    position[0] + self.current[0] * self.vector_scale,
                    position[1] + self.current[1] * self.vector_scale,
                    position[2] + self.current_z
                    + self.current[2] * self.vector_scale,
                ),
                (0.2, 0.55, 1.0, 0.9),
                stamp,
            ))
            markers.markers.extend(self._body_axes(position, yaw, stamp))
            markers.markers.append(self._text(
                'asv_path_tracking/status',
                40,
                (position[0], position[1], position[2] + 0.75),
                self._status_text(),
                self._status_color(),
                stamp,
            ))
        else:
            markers.markers.append(self._delete('asv_path_tracking/status', 40, stamp))
        return markers

    def _status_text(self) -> str:
        """Format path, tracking, and safety telemetry for RViz."""
        return (
            f'path={self.path_values.get("path_type", "--")} '
            f'progress={value(self.path_values, "progress") * 100.0:.1f}%\n'
            f'cross={value(self.controller_values, "cross_track_error_m"):+.3f} m '
            f'along={value(self.controller_values, "along_track_error_m"):+.3f} m\n'
            f'heading={value(self.controller_values, "heading_error_rad"):+.3f} rad\n'
            f'thrusters={value(self.controller_values, "thruster_port_front_N"):+.2f}, '
            f'{value(self.controller_values, "thruster_starboard_front_N"):+.2f}, '
            f'{value(self.controller_values, "thruster_port_rear_N"):+.2f}, '
            f'{value(self.controller_values, "thruster_starboard_rear_N"):+.2f}\n'
            f'{self.path_message}; {self.controller_message}'
        )

    def _status_color(self) -> Tuple[float, float, float, float]:
        if 'stale' in self.controller_message.lower() or 'waiting' in (
                self.controller_message.lower()):
            return 1.0, 0.2, 0.1, 0.95
        if 'safety' in self.controller_message.lower():
            return 1.0, 0.75, 0.1, 0.95
        return 0.25, 1.0, 0.35, 0.95

    def _body_axes(
        self,
        position: Tuple[float, float, float],
        yaw: float,
        stamp: Any,
    ) -> list[Marker]:
        x, y, z = position
        return [
            self._arrow(
                'asv_path_tracking/body',
                50,
                position,
                (x + 0.42 * math.cos(yaw), y + 0.42 * math.sin(yaw), z),
                (1.0, 0.25, 0.1, 0.9),
                stamp,
            ),
            self._arrow(
                'asv_path_tracking/body',
                51,
                position,
                (x - 0.30 * math.sin(yaw), y + 0.30 * math.cos(yaw), z),
                (0.2, 1.0, 0.25, 0.9),
                stamp,
            ),
        ]

    def _line_marker(
        self,
        namespace: str,
        marker_id: int,
        points: list[Point],
        color: Tuple[float, float, float, float],
        stamp: Any,
        width: float,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = width
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points = points
        return marker

    def _arrow(
        self,
        namespace: str,
        marker_id: int,
        start: Tuple[float, float, float],
        end: Tuple[float, float, float],
        color: Tuple[float, float, float, float],
        stamp: Any,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.points = [
            Point(x=start[0], y=start[1], z=start[2]),
            Point(x=end[0], y=end[1], z=end[2]),
        ]
        marker.scale.x = 0.035
        marker.scale.y = 0.07
        marker.scale.z = 0.08
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def _sphere(
        self,
        namespace: str,
        marker_id: int,
        x: float,
        y: float,
        z: float,
        color: Tuple[float, float, float, float],
        size: float,
        stamp: Any,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = size
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def _text(
        self,
        namespace: str,
        marker_id: int,
        position: Tuple[float, float, float],
        text: str,
        color: Tuple[float, float, float, float],
        stamp: Any,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.065
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.text = text
        return marker

    def _delete(self, namespace: str, marker_id: int, stamp: Any) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.DELETE
        return marker

    @staticmethod
    def _distance(first: PoseStamped, second: PoseStamped) -> float:
        return math.hypot(
            first.pose.position.x - second.pose.position.x,
            first.pose.position.y - second.pose.position.y,
        )

    @staticmethod
    def _rotate_z(
        vector: Tuple[float, float, float],
        yaw: float,
    ) -> Tuple[float, float, float]:
        return (
            math.cos(yaw) * vector[0] - math.sin(yaw) * vector[1],
            math.sin(yaw) * vector[0] + math.cos(yaw) * vector[1],
            vector[2],
        )


def main(args=None) -> None:
    """Run the standalone path visualization node."""
    rclpy.init(args=args)
    node = PathTrackingVisualization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
