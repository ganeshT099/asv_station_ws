"""Publish a smooth, time-parameterized geometric reference path."""

from __future__ import annotations

import math
from typing import Any, Dict, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from nav_msgs.msg import Path
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty
from visualization_msgs.msg import Marker, MarkerArray

from asv_path_tracking.path_geometry import (
    SUPPORTED_PATHS,
    PathPlan,
    PathPoint,
    generate_path,
    path_points,
)


REFERENCE_PATH_TOPIC = '/asv_path_tracking/reference_path'
REFERENCE_POSE_TOPIC = '/asv_path_tracking/reference_pose'
REFERENCE_TWIST_TOPIC = '/asv_path_tracking/reference_twist'
REFERENCE_MARKERS_TOPIC = '/asv_path_tracking/reference_markers'
STATUS_TOPIC = '/asv_path_tracking/path_status'
RUN_TOPIC = '/asv_path_tracking/run'
RESET_TOPIC = '/asv_path_tracking/reset'


def path_qos() -> QoSProfile:
    """Return a durable QoS profile so late RViz subscribers get the path."""
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class PathGenerator(Node):
    """Generate a closed reference path and advance a smooth path target."""

    def __init__(self) -> None:
        super().__init__('asv_path_generator')
        self.declare_parameter('path_type', 'circle')
        self.declare_parameter('path_width', 1.6)
        self.declare_parameter('path_height', 1.2)
        self.declare_parameter('path_center_x', 0.0)
        self.declare_parameter('path_center_y', 0.0)
        self.declare_parameter('corner_roundness', 4.0)
        self.declare_parameter('sample_count', 500)
        self.declare_parameter('speed', 0.10)
        self.declare_parameter('publish_rate', 25.0)
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('repeat_path', True)
        self.declare_parameter('lap_count', 0)
        self.declare_parameter('start_phase', 0.0)
        self.declare_parameter('path_z', 0.08)

        self.path_type = str(self.get_parameter('path_type').value).strip()
        self.reference_frame = str(
            self.get_parameter('reference_frame').value).strip() or 'world'
        self.plan: PathPlan = self._make_plan()
        self.speed = max(float(self.get_parameter('speed').value), 0.0)
        self.publish_rate = max(
            float(self.get_parameter('publish_rate').value), 1.0)
        self.path_z = float(self.get_parameter('path_z').value)
        self.repeat_path = bool(self.get_parameter('repeat_path').value)
        self.lap_count = max(int(self.get_parameter('lap_count').value), 0)
        self.auto_start = bool(self.get_parameter('auto_start').value)
        self.start_phase = float(self.get_parameter('start_phase').value) % 1.0

        self.running = False
        self._auto_started = False
        self.lap = 0
        self.distance = self.plan.total_length * self.start_phase
        self.last_time: Optional[float] = None
        self._rebuild_requested = False
        self._path_published = False

        durable = path_qos()
        self.path_pub = self.create_publisher(Path, REFERENCE_PATH_TOPIC, durable)
        self.pose_pub = self.create_publisher(
            PoseStamped, REFERENCE_POSE_TOPIC, 10)
        self.twist_pub = self.create_publisher(
            TwistStamped, REFERENCE_TWIST_TOPIC, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, REFERENCE_MARKERS_TOPIC, durable)
        self.status_pub = self.create_publisher(
            DiagnosticArray, STATUS_TOPIC, 10)
        self.run_sub = self.create_subscription(
            Bool, RUN_TOPIC, self.run_callback, 10)
        self.reset_sub = self.create_subscription(
            Empty, RESET_TOPIC, self.reset_callback, 10)
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.step)

        self.get_logger().info(
            'Path generator ready: %s, length=%.3f m, speed=%.3f m/s',
            self.path_type,
            self.plan.total_length,
            self.speed,
        )

    def _make_plan(self) -> PathPlan:
        """Build a plan from the current ROS parameters."""
        path_type = str(self.get_parameter('path_type').value).strip().lower()
        if path_type not in SUPPORTED_PATHS:
            raise ValueError(
                f'path_type must be one of {SUPPORTED_PATHS}, got {path_type!r}')
        return generate_path(
            shape=path_type,
            width=float(self.get_parameter('path_width').value),
            height=float(self.get_parameter('path_height').value),
            center_x=float(self.get_parameter('path_center_x').value),
            center_y=float(self.get_parameter('path_center_y').value),
            sample_count=int(self.get_parameter('sample_count').value),
            corner_roundness=float(
                self.get_parameter('corner_roundness').value),
        )

    def parameter_callback(self, parameters) -> SetParametersResult:
        """Validate safe path parameter updates and rebuild on the timer."""
        numeric_nonnegative = {
            'path_width', 'path_height', 'sample_count', 'speed',
            'publish_rate', 'corner_roundness', 'lap_count', 'path_z',
        }
        for parameter in parameters:
            if parameter.name == 'path_type':
                if str(parameter.value).strip().lower() not in SUPPORTED_PATHS:
                    return SetParametersResult(
                        successful=False,
                        reason=f'path_type must be one of {SUPPORTED_PATHS}',
                    )
            elif parameter.name == 'start_phase':
                try:
                    value = float(parameter.value)
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False,
                        reason='start_phase must be numeric',
                    )
                if not math.isfinite(value) or not 0.0 <= value <= 1.0:
                    return SetParametersResult(
                        successful=False,
                        reason='start_phase must be between 0 and 1',
                    )
            elif parameter.name in numeric_nonnegative:
                try:
                    value = float(parameter.value)
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False,
                        reason=f'{parameter.name} must be numeric',
                    )
                if not math.isfinite(value) or value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f'{parameter.name} must be finite and non-negative',
                    )
        self._rebuild_requested = True
        return SetParametersResult(successful=True, reason='accepted')

    def run_callback(self, message: Bool) -> None:
        """Start, pause, or resume the trajectory clock."""
        self.running = bool(message.data)
        if self.running:
            self._auto_started = True
        self.last_time = None
        if self.running:
            self.get_logger().info('Path tracking reference started.')
        else:
            self.get_logger().info('Path tracking reference paused.')

    def reset_callback(self, _message: Empty) -> None:
        """Reset to the first point without changing the selected shape."""
        self.distance = self.plan.total_length * self.start_phase
        self.lap = 0
        self.last_time = None
        self.running = False
        self._auto_started = True
        self._path_published = False
        self.get_logger().info('Path reference reset to its starting point.')

    def _sim_time(self) -> Optional[float]:
        """Return simulation time, or None until Gazebo publishes /clock."""
        value = self.get_clock().now().nanoseconds * 1e-9
        return value if value > 0.0 else None

    def step(self) -> None:
        """Advance the reference and publish every user-visible output."""
        if self._rebuild_requested:
            self.path_type = str(
                self.get_parameter('path_type').value).strip().lower()
            self.reference_frame = str(
                self.get_parameter('reference_frame').value).strip() or 'world'
            self.plan = self._make_plan()
            self.speed = max(float(self.get_parameter('speed').value), 0.0)
            self.publish_rate = max(
                float(self.get_parameter('publish_rate').value), 1.0)
            self.path_z = float(self.get_parameter('path_z').value)
            self.repeat_path = bool(self.get_parameter('repeat_path').value)
            self.lap_count = max(
                int(self.get_parameter('lap_count').value), 0)
            self.distance = 0.0
            self.start_phase = float(
                self.get_parameter('start_phase').value) % 1.0
            self.distance = self.plan.total_length * self.start_phase
            self.lap = 0
            self.last_time = None
            self._path_published = False
            self._rebuild_requested = False

        now = self._sim_time()
        if now is not None and self.auto_start and not self._auto_started:
            self.running = True
            self._auto_started = True
            self.get_logger().info('Automatic path tracking reference started.')
        if now is not None and self.running:
            if self.last_time is not None:
                dt = min(max(now - self.last_time, 0.0), 0.25)
                self.distance += self.speed * dt
            self.last_time = now
            while self.distance >= self.plan.total_length:
                self.distance -= self.plan.total_length
                self.lap += 1
                if not self.repeat_path or (
                    self.lap_count > 0 and self.lap >= self.lap_count
                ):
                    self.running = False
                    self.distance = self.plan.total_length
                    break

        stamp = self.get_clock().now().to_msg()
        self._publish_path(stamp)
        self._publish_reference(stamp)
        self._publish_status(stamp, now)

    def _publish_path(self, stamp: Any) -> None:
        """Publish the full planned path and its RViz line markers."""
        path = Path()
        path.header.frame_id = self.reference_frame
        path.header.stamp = stamp
        path.poses = [self._pose_from_point(point, stamp)
                      for point in path_points(self.plan)]
        self.path_pub.publish(path)

        markers = MarkerArray()
        line = Marker()
        line.header.frame_id = self.reference_frame
        line.header.stamp = stamp
        line.ns = 'asv_path_tracking/reference'
        line.id = 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.035
        line.color.r = 1.0
        line.color.g = 0.75
        line.color.b = 0.05
        line.color.a = 0.95
        line.points = [
            Point(x=point.x, y=point.y, z=self.path_z)
            for point in path_points(self.plan)
        ]
        markers.markers.append(line)

        stride = max(len(self.plan.points) // 20, 1)
        for index, point in enumerate(self.plan.points[::stride]):
            arrow = Marker()
            arrow.header.frame_id = self.reference_frame
            arrow.header.stamp = stamp
            arrow.ns = 'asv_path_tracking/direction'
            arrow.id = 100 + index
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = point.x
            arrow.pose.position.y = point.y
            arrow.pose.position.z = self.path_z + 0.015
            arrow.pose.orientation.z = math.sin(point.yaw * 0.5)
            arrow.pose.orientation.w = math.cos(point.yaw * 0.5)
            arrow.scale.x = 0.12
            arrow.scale.y = 0.025
            arrow.scale.z = 0.025
            arrow.color.r = 1.0
            arrow.color.g = 0.75
            arrow.color.b = 0.05
            arrow.color.a = 0.85
            markers.markers.append(arrow)
        self.marker_pub.publish(markers)
        self._path_published = True

    def _publish_reference(self, stamp: Any) -> None:
        """Publish the live target pose and tangent velocity."""
        distance = min(self.distance, self.plan.total_length)
        point = self.plan.sample(distance)
        pose = self._pose_from_point(point, stamp)
        self.pose_pub.publish(pose)

        twist = TwistStamped()
        twist.header = pose.header
        twist.twist.linear.x = self.speed * math.cos(point.yaw)
        twist.twist.linear.y = self.speed * math.sin(point.yaw)
        twist.twist.angular.z = self.speed * point.curvature
        self.twist_pub.publish(twist)

    def _publish_status(self, stamp: Any, now: Optional[float]) -> None:
        """Publish a complete machine-readable reference status."""
        status = DiagnosticStatus()
        status.name = 'asv_path_tracking/reference'
        if now is None:
            status.level = DiagnosticStatus.ERROR
            status.message = 'waiting for Gazebo simulation clock'
        elif self.running:
            status.level = DiagnosticStatus.OK
            status.message = 'path reference active'
        else:
            status.level = DiagnosticStatus.WARN
            status.message = 'path reference paused'
        values: Dict[str, Any] = {
            'path_type': self.path_type,
            'path_width_m': self.get_parameter('path_width').value,
            'path_height_m': self.get_parameter('path_height').value,
            'path_center_x_m': self.get_parameter('path_center_x').value,
            'path_center_y_m': self.get_parameter('path_center_y').value,
            'corner_roundness': self.get_parameter('corner_roundness').value,
            'sample_count': self.get_parameter('sample_count').value,
            'total_length_m': self.plan.total_length,
            'speed_mps': self.speed,
            'reference_rate_hz': self.publish_rate,
            'distance_m': min(self.distance, self.plan.total_length),
            'progress': min(
                self.distance / max(self.plan.total_length, 1e-9), 1.0),
            'lap': self.lap,
            'running': self.running,
            'repeat_path': self.repeat_path,
            'lap_count': self.lap_count,
            'frame': self.reference_frame,
        }
        status.values = [
            KeyValue(key=name, value=str(value))
            for name, value in values.items()
        ]
        message = DiagnosticArray()
        message.header.stamp = stamp
        message.status = [status]
        self.status_pub.publish(message)

    def _pose_from_point(self, point: PathPoint, stamp: Any) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.reference_frame
        pose.header.stamp = stamp
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.pose.position.z = self.path_z
        pose.pose.orientation.z = math.sin(point.yaw * 0.5)
        pose.pose.orientation.w = math.cos(point.yaw * 0.5)
        return pose


def main(args=None) -> None:
    """Run the standalone path generator."""
    rclpy.init(args=args)
    node = PathGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
