"""Pure helpers shared by the station-keeping visualization node and tests."""

import math
from enum import IntEnum
from typing import Any, Dict, Iterable, Mapping, Optional, Tuple


class Health(IntEnum):
    """Health states used to make stale and diagnostic failures visible."""

    OK = 0
    WARN = 1
    ERROR = 2
    STALE = 3


HEALTH_LABELS = {
    Health.OK: 'OK',
    Health.WARN: 'WARN',
    Health.ERROR: 'ERROR',
    Health.STALE: 'STALE',
}


# Keep these IDs stable: RViz overwrites a marker by namespace and ID.
MARKER_IDS = {
    'target_point': 0,
    'target_tolerance': 1,
    'target_heading': 2,
    'target_heading_tolerance': 3,
    'error_vector': 4,
    'current_vector': 5,
    'velocity_vector': 6,
    'heading': 7,
    'body_x': 10,
    'body_y': 11,
    'body_z': 12,
    'status_text': 30,
    'error_text': 31,
    'actual_path': 32,
    'thruster_port_front': 40,
    'thruster_starboard_front': 41,
    'thruster_port_rear': 42,
    'thruster_starboard_rear': 43,
    'thruster_port_front_text': 50,
    'thruster_starboard_front_text': 51,
    'thruster_port_rear_text': 52,
    'thruster_starboard_rear_text': 53,
}


MARKER_NAMESPACES = {
    'target': 'station_keeping/target',
    'error': 'station_keeping/error',
    'current': 'station_keeping/current',
    'velocity': 'station_keeping/velocity',
    'heading': 'station_keeping/heading',
    'body_axes': 'station_keeping/body_axes',
    'thrusters': 'station_keeping/thrusters',
    'status': 'station_keeping/status',
    'path': 'station_keeping/path',
}


THRUSTER_NAMES = (
    'port_front',
    'starboard_front',
    'port_rear',
    'starboard_rear',
)


# Body-frame positions from the research_catamaran model documentation.
THRUSTER_OFFSETS = {
    'port_front': (0.25, 0.175, -0.05),
    'starboard_front': (0.25, -0.175, -0.05),
    'port_rear': (-0.25, 0.175, -0.05),
    'starboard_rear': (-0.25, -0.175, -0.05),
}


# Unit body-frame force directions implied by the research mixer.  These are
# visualization directions for the diagonal thrusters; the Gazebo plugin
# remains the source of the simulated force application.
THRUSTER_DIRECTIONS = {
    'port_front': (2.0 ** -0.5, 2.0 ** -0.5, 0.0),
    'starboard_front': (2.0 ** -0.5, -2.0 ** -0.5, 0.0),
    'port_rear': (-2.0 ** -0.5, 2.0 ** -0.5, 0.0),
    'starboard_rear': (-2.0 ** -0.5, -2.0 ** -0.5, 0.0),
}


def level_as_int(level) -> int:
    r"""Return a DiagnosticStatus level as an int.

    ROS 2 Humble's generated Python message exposes the ``byte`` field as a
    one-byte ``bytes`` object (``b'\\x02'``), which ``int()`` rejects.
    """
    if isinstance(level, (bytes, bytearray)):
        return level[0] if level else 0
    return int(level)


def clamp(value: float, lower: float, upper: float) -> float:
    """Return *value* bounded inclusively by *lower* and *upper*."""
    return min(max(value, lower), upper)


def finite_float(
    value: Any,
    default: Optional[float] = None,
) -> Optional[float]:
    """Convert a value to a finite float, or return the supplied default."""
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(result):
        return default
    return result


def parse_diagnostic_values(items: Iterable[Any]) -> Dict[str, str]:
    """Return diagnostic key/value pairs from ROS messages or test tuples."""
    values: Dict[str, str] = {}
    for item in items:
        if hasattr(item, 'key') and hasattr(item, 'value'):
            key = str(item.key)
            value = str(item.value)
        elif isinstance(item, (tuple, list)) and len(item) >= 2:
            key = str(item[0])
            value = str(item[1])
        else:
            continue
        if key:
            values[key] = value
    return values


def diagnostic_float(
    values: Mapping[str, Any],
    key: str,
    default: Optional[float] = None,
) -> Optional[float]:
    """Read a finite numeric diagnostic value by key."""
    return finite_float(values.get(key), default)


def health_from_status(
    level: int,
    age: Optional[float],
    stale_timeout: float,
    received: bool,
) -> Health:
    """Map a diagnostic level and receipt age to a visualization health."""
    if not received or age is None or age > max(stale_timeout, 0.0):
        return Health.STALE
    if level == 3:
        return Health.STALE
    if level >= 2:
        return Health.ERROR
    if level == 1:
        return Health.WARN
    return Health.OK


def color_for_health(health: Health, alpha: float = 1.0) -> Tuple[float, ...]:
    """Return an RGBA color that clearly distinguishes health states."""
    colors = {
        Health.OK: (0.20, 0.90, 0.35),
        Health.WARN: (1.00, 0.75, 0.10),
        Health.ERROR: (1.00, 0.15, 0.10),
        Health.STALE: (0.45, 0.45, 0.45),
    }
    red, green, blue, _ = (*colors[health], 1.0)
    return red, green, blue, clamp(alpha, 0.0, 1.0)


def source_color(
    base: Tuple[float, float, float],
    health: Health,
    alpha: float = 1.0,
) -> Tuple[float, ...]:
    """Use a data color while fresh and gray when its source is stale."""
    if health == Health.STALE:
        return color_for_health(health, alpha)
    return base[0], base[1], base[2], clamp(alpha, 0.0, 1.0)


def wrap_angle(angle: float) -> float:
    """Wrap an angle to the interval [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw from a quaternion."""
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """Return an x/y/z/w quaternion for a planar yaw."""
    half_yaw = 0.5 * yaw
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def rotate_vector_by_quaternion(
    vector: Tuple[float, float, float],
    quaternion: Tuple[float, float, float, float],
) -> Tuple[float, float, float]:
    """Rotate a vector by an x/y/z/w quaternion."""
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        x, y, z, w = 0.0, 0.0, 0.0, 1.0
    else:
        x, y, z, w = x / norm, y / norm, z / norm, w / norm

    vx, vy, vz = vector
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return (
        (1.0 - 2.0 * (yy + zz)) * vx
        + 2.0 * (xy - wz) * vy
        + 2.0 * (xz + wy) * vz,
        2.0 * (xy + wz) * vx
        + (1.0 - 2.0 * (xx + zz)) * vy
        + 2.0 * (yz - wx) * vz,
        2.0 * (xz - wy) * vx
        + 2.0 * (yz + wx) * vy
        + (1.0 - 2.0 * (xx + yy)) * vz,
    )
