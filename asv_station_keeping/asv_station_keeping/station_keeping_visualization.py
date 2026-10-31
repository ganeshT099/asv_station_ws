"""ROS 2 visualization node for the research catamaran station keeper."""

import math
import time
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Tuple

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Point, PoseStamped, TransformStamped, Vector3
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from asv_station_keeping.visualization_model import (
    level_as_int,
    HEALTH_LABELS,
    MARKER_IDS,
    MARKER_NAMESPACES,
    THRUSTER_NAMES,
    THRUSTER_DIRECTIONS,
    THRUSTER_OFFSETS,
    Health,
    color_for_health,
    diagnostic_float,
    health_from_status,
    parse_diagnostic_values,
    quaternion_from_yaw,
    quaternion_to_yaw,
    rotate_vector_by_quaternion,
    source_color,
)


ODOMETRY_TOPIC = '/model/research_catamaran/odometry'
STATUS_TOPIC = '/research_catamaran/station_keeping/status'
CURRENT_TOPIC = '/ocean_current'
MARKER_TOPIC = '/research_catamaran/station_keeping/visualization/markers'
ACTUAL_PATH_TOPIC = (
    '/research_catamaran/station_keeping/visualization/actual_path')
TARGET_PATH_TOPIC = (
    '/research_catamaran/station_keeping/visualization/target_path')


class StationKeepingVisualization(Node):
    """Publish stable markers and paths for station-keeping experiments."""

    def __init__(self) -> None:
        super().__init__('research_station_keeping_visualization')

        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('stale_timeout', 1.0)
        self.declare_parameter('max_path_length', 1000)
        self.declare_parameter('path_min_distance', 0.01)
        self.declare_parameter('visualization_frame', '')
        self.declare_parameter('fallback_frame', 'world')
        self.declare_parameter('target_z', 0.0)
        self.declare_parameter('default_position_tolerance', 0.05)
        self.declare_parameter('default_yaw_tolerance', 0.08)
        self.declare_parameter('axis_length', 0.45)
        self.declare_parameter('heading_length', 0.65)
        self.declare_parameter('current_scale', 1.2)
        self.declare_parameter('velocity_scale', 1.0)
        self.declare_parameter('current_z_offset', 0.45)
        self.declare_parameter('velocity_z_offset', 0.30)
        self.declare_parameter('thruster_scale', 0.12)
        self.declare_parameter('thruster_min_length', 0.04)
        self.declare_parameter('text_height', 0.07)
        self.declare_parameter('path_visual_offset', 0.12)
        self.declare_parameter('path_line_width', 0.035)
        self.declare_parameter('odom_topic', ODOMETRY_TOPIC)
        self.declare_parameter('status_topic', STATUS_TOPIC)
        self.declare_parameter('current_topic', CURRENT_TOPIC)
        self.declare_parameter('marker_topic', MARKER_TOPIC)
        self.declare_parameter('actual_path_topic', ACTUAL_PATH_TOPIC)
        self.declare_parameter('target_path_topic', TARGET_PATH_TOPIC)
        self.declare_parameter('broadcast_tf', True)
        self.declare_parameter('tf_parent_frame', 'world')
        self.declare_parameter('tf_child_frame', 'base_footprint')

        self.publish_rate = max(self._parameter_float('publish_rate'), 0.1)
        self.stale_timeout = max(self._parameter_float('stale_timeout'), 0.0)
        self.max_path_length = max(
            self._parameter_int('max_path_length'),
            2,
        )
        self.path_min_distance = max(
            self._parameter_float('path_min_distance'),
            0.0,
        )
        self.visualization_frame = str(
            self.get_parameter('visualization_frame').value).strip()
        self.fallback_frame = str(
            self.get_parameter('fallback_frame').value).strip() or 'world'
        self.target_z = self._parameter_float('target_z')
        self.default_position_tolerance = max(
            self._parameter_float('default_position_tolerance'),
            0.01,
        )
        self.default_yaw_tolerance = max(
            self._parameter_float('default_yaw_tolerance'),
            0.01,
        )
        self.axis_length = max(self._parameter_float('axis_length'), 0.05)
        self.heading_length = max(
            self._parameter_float('heading_length'),
            0.05,
        )
        self.current_scale = max(self._parameter_float('current_scale'), 0.0)
        self.velocity_scale = max(
            self._parameter_float('velocity_scale'),
            0.0,
        )
        self.current_z_offset = self._parameter_float('current_z_offset')
        self.velocity_z_offset = self._parameter_float('velocity_z_offset')
        self.thruster_scale = max(
            self._parameter_float('thruster_scale'),
            0.0,
        )
        self.thruster_min_length = max(
            self._parameter_float('thruster_min_length'),
            0.01,
        )
        self.text_height = max(self._parameter_float('text_height'), 0.01)
        self.path_visual_offset = self._parameter_float('path_visual_offset')
        self.path_line_width = max(
            self._parameter_float('path_line_width'), 0.005)
        self.broadcast_tf = bool(self.get_parameter('broadcast_tf').value)
        self.tf_parent_frame = str(
            self.get_parameter('tf_parent_frame').value).strip() or 'world'
        self.tf_child_frame = str(
            self.get_parameter('tf_child_frame').value).strip() or 'base_footprint'

        self.latest_odom: Optional[Odometry] = None
        self.latest_current: Optional[Tuple[float, float, float]] = None
        self.status_values: Dict[str, str] = {}
        self.status_level = DiagnosticStatus.ERROR
        self.status_message = 'waiting for diagnostics'
        self.odom_received_at: Optional[float] = None
        self.status_received_at: Optional[float] = None
        self.current_received_at: Optional[float] = None
        self.frame_id = self.visualization_frame or self.fallback_frame
        self.actual_path: Deque[PoseStamped] = deque(
            maxlen=self.max_path_length)
        self.tf_broadcaster = (
            TransformBroadcaster(self) if self.broadcast_tf else None)

        self.marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('marker_topic').value),
            10,
        )
        self.actual_path_pub = self.create_publisher(
            Path,
            str(self.get_parameter('actual_path_topic').value),
            10,
        )
        self.target_path_pub = self.create_publisher(
            Path,
            str(self.get_parameter('target_path_topic').value),
            10,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self.odom_callback,
            10,
        )
        self.status_sub = self.create_subscription(
            DiagnosticArray,
            str(self.get_parameter('status_topic').value),
            self.status_callback,
            10,
        )
        self.current_sub = self.create_subscription(
            Vector3,
            str(self.get_parameter('current_topic').value),
            self.current_callback,
            10,
        )
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_visualization,
        )
        self.get_logger().info(
            'Station-keeping visualization ready: '
            f'frame={self.frame_id}, markers={MARKER_TOPIC}, '
            f'actual_path={ACTUAL_PATH_TOPIC}')

    def _parameter_float(self, name: str) -> float:
        """Read a declared parameter as a float."""
        return float(self.get_parameter(name).value)

    def _parameter_int(self, name: str) -> int:
        """Read a declared parameter as an integer."""
        return int(self.get_parameter(name).value)

    def _resolve_frame(self, message: Odometry) -> str:
        """Choose a common frame for markers and paths."""
        if self.visualization_frame:
            return self.visualization_frame
        message_frame = str(message.header.frame_id).strip()
        return message_frame or self.fallback_frame

    def _set_frame(self, frame_id: str) -> None:
        """Switch path frames without mixing samples from two frames."""
        if frame_id != self.frame_id:
            self.actual_path.clear()
            self.frame_id = frame_id

    def odom_callback(self, message: Odometry) -> None:
        """Store odometry and append a distance-thinned actual path sample."""
        self.latest_odom = message
        self.odom_received_at = time.monotonic()
        self._set_frame(self._resolve_frame(message))

        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = message.header.stamp
        pose.pose = message.pose.pose
        if not self.actual_path or self._path_distance(pose) >= (
                self.path_min_distance):
            self.actual_path.append(pose)
        self._broadcast_odometry_transform(message)

    def _broadcast_odometry_transform(self, message: Odometry) -> None:
        """Publish a world-to-vehicle transform so RViz can move the model."""
        if self.tf_broadcaster is None:
            return
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = self.tf_parent_frame
        transform.child_frame_id = self.tf_child_frame
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def status_callback(self, message: DiagnosticArray) -> None:
        """Store the station-keeping diagnostic key/value snapshot."""
        self.status_received_at = time.monotonic()
        selected = None
        for diagnostic in message.status:
            if diagnostic.name == 'research_catamaran/station_keeping':
                selected = diagnostic
                break
            if 'station_keeping' in diagnostic.name:
                selected = diagnostic
        if selected is None:
            self.status_values = {}
            self.status_level = DiagnosticStatus.ERROR
            self.status_message = 'station-keeping diagnostic missing'
            return
        self.status_values = parse_diagnostic_values(selected.values)
        self.status_level = level_as_int(selected.level)
        self.status_message = str(selected.message) or 'diagnostic received'

    def current_callback(self, message: Vector3) -> None:
        """Store the latest world-frame current vector."""
        self.latest_current = (
            self._finite_or_zero(message.x),
            self._finite_or_zero(message.y),
            self._finite_or_zero(message.z),
        )
        self.current_received_at = time.monotonic()

    @staticmethod
    def _finite_or_zero(value: Any) -> float:
        """Convert a message scalar to a finite float."""
        try:
            value = float(value)
        except (TypeError, ValueError):
            return 0.0
        return value if math.isfinite(value) else 0.0

    def _path_distance(self, pose: PoseStamped) -> float:
        """Return planar distance from the last path pose."""
        if not self.actual_path:
            return math.inf
        previous = self.actual_path[-1].pose.position
        current = pose.pose.position
        return math.hypot(
            current.x - previous.x,
            current.y - previous.y,
        )

    def _age(
        self,
        received_at: Optional[float],
        now: float,
    ) -> Optional[float]:
        """Return a non-negative wall-clock age for a received sample."""
        if received_at is None:
            return None
        return max(now - received_at, 0.0)

    def _source_health(
        self,
        received_at: Optional[float],
        now: float,
    ) -> Health:
        """Return OK while a data source is fresh and STALE otherwise."""
        age = self._age(received_at, now)
        return health_from_status(
            0,
            age,
            self.stale_timeout,
            received_at is not None,
        )

    def _controller_health(self, now: float) -> Health:
        """Return diagnostic health, including a missing/stale stream."""
        return health_from_status(
            self.status_level,
            self._age(self.status_received_at, now),
            self.stale_timeout,
            self.status_received_at is not None,
        )

    def _diagnostic_float(
        self,
        key: str,
        default: Optional[float] = None,
    ) -> Optional[float]:
        """Read one finite number from the latest diagnostic snapshot."""
        return diagnostic_float(self.status_values, key, default)

    def _target(self) -> Optional[Tuple[float, float, float]]:
        """Return target x/y/yaw when diagnostics provide a target."""
        x = self._diagnostic_float('target_x_m')
        y = self._diagnostic_float('target_y_m')
        if x is None or y is None:
            return None
        yaw = self._diagnostic_float('target_yaw_rad', 0.0)
        return x, y, yaw if yaw is not None else 0.0

    def _commands(self) -> Dict[str, Optional[float]]:
        """Return all four named thruster diagnostics in stable order."""
        return {
            name: self._diagnostic_float(
                'thruster_' + name + '_N')
            for name in THRUSTER_NAMES
        }

    def _position(self) -> Optional[Tuple[float, float, float]]:
        """Return the latest odometry position."""
        if self.latest_odom is None:
            return None
        position = self.latest_odom.pose.pose.position
        return position.x, position.y, position.z

    def _quaternion(self) -> Optional[Tuple[float, float, float, float]]:
        """Return the latest odometry orientation."""
        if self.latest_odom is None:
            return None
        orientation = self.latest_odom.pose.pose.orientation
        return (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

    def _target_tolerances(self) -> Tuple[float, float]:
        """Return position and yaw tolerances from diagnostics or defaults."""
        position = self._diagnostic_float(
            'position_tolerance_m',
            self.default_position_tolerance,
        )
        yaw = self._diagnostic_float(
            'yaw_tolerance_rad',
            self.default_yaw_tolerance,
        )
        return (
            max(abs(position if position is not None else 0.05), 0.01),
            max(abs(yaw if yaw is not None else 0.08), 0.01),
        )

    def publish_visualization(self) -> None:
        """Publish the current marker set and both path topics."""
        now_wall = time.monotonic()
        stamp = self.get_clock().now().to_msg()
        controller_health = self._controller_health(now_wall)
        odom_health = self._source_health(self.odom_received_at, now_wall)
        current_health = self._source_health(
            self.current_received_at,
            now_wall,
        )
        overall_health = self._worst_health(
            controller_health,
            odom_health,
            current_health,
        )

        marker_array = MarkerArray()
        marker_array.markers = self._build_markers(
            stamp,
            controller_health,
            odom_health,
            current_health,
            overall_health,
            now_wall,
        )
        self.marker_pub.publish(marker_array)
        self._publish_paths(stamp)

    def _build_markers(
        self,
        stamp: Any,
        controller_health: Health,
        odom_health: Health,
        current_health: Health,
        overall_health: Health,
        now_wall: float,
    ) -> List[Marker]:
        """Build one stable add/delete marker for each visualization item."""
        markers: List[Marker] = []
        target = self._target()
        position = self._position()
        quaternion = self._quaternion()

        if target is None:
            markers.extend(self._delete_target_markers(stamp))
        else:
            markers.extend(self._target_markers(
                stamp,
                target,
                controller_health,
            ))

        if target is None or position is None:
            markers.extend([
                self._delete_marker(
                    MARKER_NAMESPACES['error'],
                    MARKER_IDS['error_vector'],
                    stamp,
                ),
                self._delete_marker(
                    MARKER_NAMESPACES['error'],
                    MARKER_IDS['error_text'],
                    stamp,
                ),
            ])
        else:
            markers.extend(self._error_markers(
                stamp,
                target,
                position,
                quaternion_to_yaw(*quaternion) if quaternion else 0.0,
                self._worst_health(controller_health, odom_health),
            ))

        if position is None:
            markers.append(self._delete_marker(
                MARKER_NAMESPACES['path'],
                MARKER_IDS['actual_path'],
                stamp,
            ))
            markers.extend(self._delete_motion_markers(stamp))
            markers.extend(self._delete_thruster_markers(stamp))
            status_anchor = (
                (target[0], target[1], self.target_z + 0.65)
                if target is not None else (0.0, 0.0, 0.65)
            )
        else:
            if quaternion is None:
                quaternion = (0.0, 0.0, 0.0, 1.0)
            markers.append(self._actual_path_marker(stamp))
            markers.extend(self._motion_markers(
                stamp,
                position,
                quaternion,
                odom_health,
                current_health,
            ))
            markers.extend(self._thruster_markers(
                stamp,
                position,
                quaternion,
                controller_health,
            ))
            status_anchor = (position[0], position[1], position[2] + 0.75)

        markers.append(self._status_marker(
            stamp,
            status_anchor,
            overall_health,
            now_wall,
        ))
        return markers

    def _actual_path_marker(self, stamp: Any) -> Marker:
        """Draw the trajectory as a marker as well as a nav_msgs/Path.

        RViz's Path display is the primary interface.  The duplicate marker is
        intentional: it keeps the trajectory visible when a saved RViz config
        has a stale Path display or when a presentation uses only MarkerArray.
        """
        if len(self.actual_path) < 2:
            return self._delete_marker(
                MARKER_NAMESPACES['path'],
                MARKER_IDS['actual_path'],
                stamp,
            )
        points = [
            self._point(
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z + self.path_visual_offset,
            )
            for pose in self.actual_path
        ]
        return self._line_marker(
            MARKER_NAMESPACES['path'],
            MARKER_IDS['actual_path'],
            points,
            self.path_line_width,
            stamp,
            (0.10, 0.75, 1.00, 0.95),
        )

    @staticmethod
    def _worst_health(*health_states: Health) -> Health:
        """Return the most severe state across visualization data sources."""
        return max(health_states, default=Health.STALE)

    def _delete_target_markers(self, stamp: Any) -> List[Marker]:
        """Delete target markers when the diagnostic target is unavailable."""
        return [
            self._delete_marker(
                MARKER_NAMESPACES['target'],
                MARKER_IDS[key],
                stamp,
            )
            for key in (
                'target_point',
                'target_tolerance',
                'target_heading',
                'target_heading_tolerance',
            )
        ]

    def _target_markers(
        self,
        stamp: Any,
        target: Tuple[float, float, float],
        health: Health,
    ) -> List[Marker]:
        """Create target point, position tolerance, and heading visuals."""
        target_x, target_y, target_yaw = target
        tolerance, yaw_tolerance = self._target_tolerances()
        color = color_for_health(health, 0.95)
        ring_color = color_for_health(health, 0.50)
        heading_color = color_for_health(health, 0.85)
        target_z = self.target_z
        markers = []

        point = self._marker(
            MARKER_NAMESPACES['target'],
            MARKER_IDS['target_point'],
            Marker.SPHERE,
            stamp,
        )
        point.pose.position.x = target_x
        point.pose.position.y = target_y
        point.pose.position.z = target_z
        point.scale.x = 0.14
        point.scale.y = 0.14
        point.scale.z = 0.14
        self._set_color(point, color)
        markers.append(point)

        ring_points = []
        for index in range(37):
            angle = 2.0 * math.pi * index / 36.0
            ring_points.append(self._point(
                target_x + tolerance * math.cos(angle),
                target_y + tolerance * math.sin(angle),
                target_z + 0.015,
            ))
        ring = self._line_marker(
            MARKER_NAMESPACES['target'],
            MARKER_IDS['target_tolerance'],
            ring_points,
            0.018,
            stamp,
            ring_color,
        )
        markers.append(ring)

        heading_start = (target_x, target_y, target_z + 0.03)
        heading_end = (
            target_x + self.heading_length * math.cos(target_yaw),
            target_y + self.heading_length * math.sin(target_yaw),
            target_z + 0.03,
        )
        markers.append(self._arrow_marker(
            MARKER_NAMESPACES['target'],
            MARKER_IDS['target_heading'],
            heading_start,
            heading_end,
            stamp,
            heading_color,
            shaft=0.025,
            head=0.07,
            head_length=0.10,
        ))

        left = target_yaw + yaw_tolerance
        right = target_yaw - yaw_tolerance
        tolerance_points = [
            self._point(*heading_start),
            self._point(
                target_x + 0.85 * self.heading_length * math.cos(left),
                target_y + 0.85 * self.heading_length * math.sin(left),
                target_z + 0.03,
            ),
            self._point(*heading_start),
            self._point(
                target_x + 0.85 * self.heading_length * math.cos(right),
                target_y + 0.85 * self.heading_length * math.sin(right),
                target_z + 0.03,
            ),
        ]
        markers.append(self._line_marker(
            MARKER_NAMESPACES['target'],
            MARKER_IDS['target_heading_tolerance'],
            tolerance_points,
            0.014,
            stamp,
            ring_color,
            marker_type=Marker.LINE_LIST,
        ))
        return markers

    def _error_markers(
        self,
        stamp: Any,
        target: Tuple[float, float, float],
        position: Tuple[float, float, float],
        yaw: float,
        health: Health,
    ) -> List[Marker]:
        """Create world-frame error and body-frame error text."""
        target_x, target_y, target_yaw = target
        start = position
        planar_end = (target_x, target_y, position[2])
        end = self._minimum_length_end(start, planar_end, (1.0, 0.0, 0.0))
        error_color = color_for_health(health, 0.95)
        error = self._arrow_marker(
            MARKER_NAMESPACES['error'],
            MARKER_IDS['error_vector'],
            start,
            end,
            stamp,
            error_color,
            shaft=0.03,
            head=0.09,
            head_length=0.12,
        )

        dx = target_x - position[0]
        dy = target_y - position[1]
        ex_body = self._diagnostic_float('error_x_body_m')
        ey_body = self._diagnostic_float('error_y_body_m')
        if ex_body is None:
            ex_body = math.cos(yaw) * dx + math.sin(yaw) * dy
        if ey_body is None:
            ey_body = -math.sin(yaw) * dx + math.cos(yaw) * dy
        error_yaw = self._diagnostic_float('error_yaw_rad')
        if error_yaw is None:
            error_yaw = self._wrap_angle(target_yaw - yaw)
        label_position = (
            0.5 * (position[0] + target_x),
            0.5 * (position[1] + target_y),
            position[2] + 0.12,
        )
        label = self._text_marker(
            MARKER_NAMESPACES['error'],
            MARKER_IDS['error_text'],
            label_position,
            'e_body=(%+.2f, %+.2f) m\ne_yaw=%+.2f rad' % (
                ex_body,
                ey_body,
                error_yaw,
            ),
            stamp,
            error_color,
        )
        return [error, label]

    def _motion_markers(
        self,
        stamp: Any,
        position: Tuple[float, float, float],
        quaternion: Tuple[float, float, float, float],
        odom_health: Health,
        current_health: Health,
    ) -> List[Marker]:
        """Create current, velocity, heading, and body-axis markers."""
        markers: List[Marker] = []
        odom = self.latest_odom
        if odom is None:
            return self._delete_motion_markers(stamp)

        base_color = source_color((0.10, 0.75, 1.00), current_health, 0.95)
        current = self.latest_current or (0.0, 0.0, 0.0)
        current_start = (
            position[0],
            position[1],
            position[2] + self.current_z_offset,
        )
        current_end = self._scaled_vector_end(
            current_start,
            current,
            self.current_scale,
            (1.0, 0.0, 0.0),
        )
        markers.append(self._arrow_marker(
            MARKER_NAMESPACES['current'],
            MARKER_IDS['current_vector'],
            current_start,
            current_end,
            stamp,
            base_color,
            shaft=0.025,
            head=0.075,
            head_length=0.10,
        ))

        linear = odom.twist.twist.linear
        body_velocity = (linear.x, linear.y, linear.z)
        world_velocity = rotate_vector_by_quaternion(body_velocity, quaternion)
        velocity_start = (
            position[0],
            position[1],
            position[2] + self.velocity_z_offset,
        )
        velocity_end = self._scaled_vector_end(
            velocity_start,
            world_velocity,
            self.velocity_scale,
            (1.0, 0.0, 0.0),
        )
        velocity_color = source_color((0.75, 0.25, 1.00), odom_health, 0.95)
        markers.append(self._arrow_marker(
            MARKER_NAMESPACES['velocity'],
            MARKER_IDS['velocity_vector'],
            velocity_start,
            velocity_end,
            stamp,
            velocity_color,
            shaft=0.022,
            head=0.07,
            head_length=0.09,
        ))

        yaw = quaternion_to_yaw(*quaternion)
        heading_start = (position[0], position[1], position[2] + 0.06)
        heading_end = (
            position[0] + self.heading_length * math.cos(yaw),
            position[1] + self.heading_length * math.sin(yaw),
            position[2] + 0.06,
        )
        heading_color = source_color((1.00, 0.25, 0.85), odom_health, 0.95)
        markers.append(self._arrow_marker(
            MARKER_NAMESPACES['heading'],
            MARKER_IDS['heading'],
            heading_start,
            heading_end,
            stamp,
            heading_color,
            shaft=0.028,
            head=0.08,
            head_length=0.11,
        ))

        axis_vectors = (
            ((1.0, 0.0, 0.0), (1.00, 0.10, 0.10), 'body_x'),
            ((0.0, 1.0, 0.0), (0.10, 1.00, 0.15), 'body_y'),
            ((0.0, 0.0, 1.0), (0.15, 0.35, 1.00), 'body_z'),
        )
        for local_axis, axis_color, marker_key in axis_vectors:
            world_axis = rotate_vector_by_quaternion(local_axis, quaternion)
            axis_end = (
                position[0] + self.axis_length * world_axis[0],
                position[1] + self.axis_length * world_axis[1],
                position[2] + self.axis_length * world_axis[2],
            )
            markers.append(self._arrow_marker(
                MARKER_NAMESPACES['body_axes'],
                MARKER_IDS[marker_key],
                position,
                axis_end,
                stamp,
                source_color(axis_color, odom_health, 0.90),
                shaft=0.018,
                head=0.055,
                head_length=0.075,
            ))
        return markers

    def _delete_motion_markers(self, stamp: Any) -> List[Marker]:
        """Delete pose-derived motion markers when odometry is unavailable."""
        return [
            self._delete_marker(
                namespace,
                marker_id,
                stamp,
            )
            for namespace, marker_id in (
                (MARKER_NAMESPACES['current'], MARKER_IDS['current_vector']),
                (MARKER_NAMESPACES['velocity'], MARKER_IDS['velocity_vector']),
                (MARKER_NAMESPACES['heading'], MARKER_IDS['heading']),
                (MARKER_NAMESPACES['body_axes'], MARKER_IDS['body_x']),
                (MARKER_NAMESPACES['body_axes'], MARKER_IDS['body_y']),
                (MARKER_NAMESPACES['body_axes'], MARKER_IDS['body_z']),
            )
        ]

    def _thruster_markers(
        self,
        stamp: Any,
        position: Tuple[float, float, float],
        quaternion: Tuple[float, float, float, float],
        health: Health,
    ) -> List[Marker]:
        """Create four signed command arrows and stable command labels."""
        commands = self._commands()
        if not any(value is not None for value in commands.values()):
            return self._delete_thruster_markers(stamp)

        markers: List[Marker] = []
        for name in THRUSTER_NAMES:
            command = commands[name]
            value = command if command is not None else 0.0
            offset = rotate_vector_by_quaternion(
                THRUSTER_OFFSETS[name],
                quaternion,
            )
            origin = (
                position[0] + offset[0],
                position[1] + offset[1],
                position[2] + offset[2],
            )
            direction_body = THRUSTER_DIRECTIONS[name]
            direction = rotate_vector_by_quaternion(
                direction_body,
                quaternion,
            )
            if value < 0.0:
                direction = tuple(-component for component in direction)
            length = max(
                abs(value) * self.thruster_scale,
                self.thruster_min_length,
            )
            end = (
                origin[0] + length * direction[0],
                origin[1] + length * direction[1],
                origin[2] + length * direction[2],
            )
            command_color = self._command_color(value, health)
            markers.append(self._arrow_marker(
                MARKER_NAMESPACES['thrusters'],
                MARKER_IDS['thruster_' + name],
                origin,
                end,
                stamp,
                command_color,
                shaft=0.022,
                head=0.065,
                head_length=0.085,
            ))
            label_position = (
                end[0],
                end[1],
                end[2] + 0.055,
            )
            short_name = {
                'port_front': 'PF',
                'starboard_front': 'SF',
                'port_rear': 'PR',
                'starboard_rear': 'SR',
            }[name]
            label_text = (
                short_name + ' ' + (
                    '--' if command is None else '%+.2f N' % value)
            )
            markers.append(self._text_marker(
                MARKER_NAMESPACES['thrusters'],
                MARKER_IDS['thruster_' + name + '_text'],
                label_position,
                label_text,
                stamp,
                command_color,
            ))
        return markers

    def _delete_thruster_markers(self, stamp: Any) -> List[Marker]:
        """Delete all thruster arrows and labels."""
        return [
            self._delete_marker(
                MARKER_NAMESPACES['thrusters'],
                marker_id,
                stamp,
            )
            for marker_id in (
                MARKER_IDS['thruster_port_front'],
                MARKER_IDS['thruster_starboard_front'],
                MARKER_IDS['thruster_port_rear'],
                MARKER_IDS['thruster_starboard_rear'],
                MARKER_IDS['thruster_port_front_text'],
                MARKER_IDS['thruster_starboard_front_text'],
                MARKER_IDS['thruster_port_rear_text'],
                MARKER_IDS['thruster_starboard_rear_text'],
            )
        ]

    @staticmethod
    def _command_color(value: float, health: Health) -> Tuple[float, ...]:
        """Color signed thrust while retaining explicit stale/error colors."""
        if health in (Health.ERROR, Health.WARN, Health.STALE):
            return color_for_health(health, 0.95)
        if value > 1e-6:
            return 0.10, 0.90, 0.35, 0.95
        if value < -1e-6:
            return 1.00, 0.30, 0.10, 0.95
        return 0.65, 0.65, 0.65, 0.90

    def _status_marker(
        self,
        stamp: Any,
        anchor: Tuple[float, float, float],
        health: Health,
        now_wall: float,
    ) -> Marker:
        """Create a compact status/age/velocity/current text readout."""
        status_age = self._age(self.status_received_at, now_wall)
        odom_age = self._age(self.odom_received_at, now_wall)
        current_age = self._age(self.current_received_at, now_wall)
        position = self._position()
        target = self._target()

        position_error = self._diagnostic_float('position_error_m')
        yaw_error = self._diagnostic_float('error_yaw_rad')
        if (
            position_error is None
            and position is not None
            and target is not None
        ):
            position_error = math.hypot(
                target[0] - position[0],
                target[1] - position[1],
            )
        if yaw_error is None and target is not None and self._quaternion():
            yaw_error = self._wrap_angle(
                target[2] - quaternion_to_yaw(*self._quaternion()))

        if self.latest_odom is not None:
            twist = self.latest_odom.twist.twist
            velocity_text = '(%+.2f, %+.2f, %+.2f)' % (
                twist.linear.x,
                twist.linear.y,
                twist.angular.z,
            )
        else:
            velocity_text = '(--, --, --)'
        if self.latest_current is None:
            current_text = '(--, --, --)'
        else:
            current_text = '(%+.2f, %+.2f, %+.2f)' % self.latest_current

        commands = self._commands()
        command_text = ' '.join(
            label + '=' + (
                '--' if commands[name] is None else '%+.2f' % commands[name])
            for name, label in (
                ('port_front', 'PF'),
                ('starboard_front', 'SF'),
                ('port_rear', 'PR'),
                ('starboard_rear', 'SR'),
            )
        )
        text = (
            'STATION KEEPING [%s] %s\n'
            'error=(%s m, %s rad)  velocity body=(%s)\n'
            'current=(%s) m/s  thrust N: %s\n'
            'age status=%s  odom=%s  current=%s' % (
                HEALTH_LABELS[health],
                self.status_message,
                '--' if position_error is None else '%.3f' % position_error,
                '--' if yaw_error is None else '%+.3f' % yaw_error,
                velocity_text,
                current_text,
                command_text,
                self._format_age(status_age),
                self._format_age(odom_age),
                self._format_age(current_age),
            )
        )
        return self._text_marker(
            MARKER_NAMESPACES['status'],
            MARKER_IDS['status_text'],
            anchor,
            text,
            stamp,
            color_for_health(health, 0.95),
        )

    @staticmethod
    def _format_age(age: Optional[float]) -> str:
        """Format an optional data age for the RViz text marker."""
        return '--' if age is None else '%.1fs' % age

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """Wrap an angle without importing controller implementation."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def _publish_paths(self, stamp: Any) -> None:
        """Publish actual history and a one-pose target path."""
        actual_path = Path()
        actual_path.header.frame_id = self.frame_id
        actual_path.header.stamp = stamp
        actual_path.poses = list(self.actual_path)
        self.actual_path_pub.publish(actual_path)

        target_path = Path()
        target_path.header.frame_id = self.frame_id
        target_path.header.stamp = stamp
        target = self._target()
        if target is not None:
            target_pose = PoseStamped()
            target_pose.header.frame_id = self.frame_id
            target_pose.header.stamp = stamp
            target_pose.pose.position.x = target[0]
            target_pose.pose.position.y = target[1]
            target_pose.pose.position.z = self.target_z
            qx, qy, qz, qw = quaternion_from_yaw(target[2])
            target_pose.pose.orientation.x = qx
            target_pose.pose.orientation.y = qy
            target_pose.pose.orientation.z = qz
            target_pose.pose.orientation.w = qw
            target_path.poses = [target_pose]
        self.target_path_pub.publish(target_path)

    def _marker(
        self,
        namespace: str,
        marker_id: int,
        marker_type: int,
        stamp: Any,
    ) -> Marker:
        """Create a marker with the common frame, identity, and action."""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _delete_marker(
        self,
        namespace: str,
        marker_id: int,
        stamp: Any,
    ) -> Marker:
        """Create a stable delete marker for an unavailable visual."""
        marker = self._marker(namespace, marker_id, Marker.SPHERE, stamp)
        marker.action = Marker.DELETE
        return marker

    @staticmethod
    def _set_color(marker: Marker, color: Tuple[float, ...]) -> None:
        """Assign an RGBA tuple to a ROS marker."""
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

    def _arrow_marker(
        self,
        namespace: str,
        marker_id: int,
        start: Tuple[float, float, float],
        end: Tuple[float, float, float],
        stamp: Any,
        color: Tuple[float, ...],
        shaft: float,
        head: float,
        head_length: float,
    ) -> Marker:
        """Create an RViz arrow using two endpoint points."""
        marker = self._marker(namespace, marker_id, Marker.ARROW, stamp)
        marker.points = [self._point(*start), self._point(*end)]
        marker.scale.x = max(shaft, 0.001)
        marker.scale.y = max(head, marker.scale.x * 1.5)
        marker.scale.z = max(head_length, 0.001)
        self._set_color(marker, color)
        return marker

    def _line_marker(
        self,
        namespace: str,
        marker_id: int,
        points: List[Point],
        width: float,
        stamp: Any,
        color: Tuple[float, ...],
        marker_type: int = Marker.LINE_STRIP,
    ) -> Marker:
        """Create a line strip or line list marker."""
        marker = self._marker(namespace, marker_id, marker_type, stamp)
        marker.points = points
        marker.scale.x = max(width, 0.001)
        self._set_color(marker, color)
        return marker

    def _text_marker(
        self,
        namespace: str,
        marker_id: int,
        position: Tuple[float, float, float],
        text: str,
        stamp: Any,
        color: Tuple[float, ...],
    ) -> Marker:
        """Create a camera-facing text marker."""
        marker = self._marker(
            namespace,
            marker_id,
            Marker.TEXT_VIEW_FACING,
            stamp,
        )
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.scale.z = self.text_height
        marker.text = text
        self._set_color(marker, color)
        return marker

    @staticmethod
    def _point(x: float, y: float, z: float) -> Point:
        """Create a geometry point without relying on keyword constructors."""
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        return point

    @staticmethod
    def _minimum_length_end(
        start: Tuple[float, float, float],
        end: Tuple[float, float, float],
        fallback: Tuple[float, float, float],
    ) -> Tuple[float, float, float]:
        """Keep a zero error vector visible as a small directional arrow."""
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        dz = end[2] - start[2]
        if math.sqrt(dx * dx + dy * dy + dz * dz) >= 1e-4:
            return end
        return (
            start[0] + 0.04 * fallback[0],
            start[1] + 0.04 * fallback[1],
            start[2] + 0.04 * fallback[2],
        )

    @staticmethod
    def _scaled_vector_end(
        start: Tuple[float, float, float],
        vector: Tuple[float, float, float],
        scale: float,
        fallback: Tuple[float, float, float],
    ) -> Tuple[float, float, float]:
        """Scale a vector for display and keep zero vectors visible."""
        length = math.sqrt(sum(component * component for component in vector))
        if length <= 1e-8:
            return StationKeepingVisualization._minimum_length_end(
                start,
                start,
                fallback,
            )
        return (
            start[0] + scale * vector[0],
            start[1] + scale * vector[1],
            start[2] + scale * vector[2],
        )


def main(args: Optional[List[str]] = None) -> None:
    """Run the station-keeping visualization node."""
    rclpy.init(args=args)
    node = StationKeepingVisualization()
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
