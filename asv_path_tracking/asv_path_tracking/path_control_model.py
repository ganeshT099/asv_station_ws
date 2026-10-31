"""ROS-independent control math used by the path-tracking node and tests."""

import math
from typing import Tuple


def mix_thrusters(surge: float, sway: float, yaw: float) -> Tuple[float, ...]:
    """Apply the diagonal four-thruster allocation matrix."""
    return (
        surge - sway + yaw,
        surge + sway - yaw,
        -surge - sway - yaw,
        -surge + sway + yaw,
    )


def move_toward(current: float, target: float, maximum: float) -> float:
    """Move a scalar toward a target by a bounded amount."""
    difference = target - current
    if abs(difference) <= maximum:
        return target
    return current + maximum * (1.0 if difference > 0.0 else -1.0)


def tilt_scale(
    roll: float,
    pitch: float,
    soft_limit: float,
    hard_limit: float,
    enabled: bool = True,
) -> float:
    """Return a smooth thrust scale from tilt safety limits."""
    if not enabled:
        return 1.0
    tilt = max(abs(roll), abs(pitch))
    if tilt >= hard_limit:
        return 0.0
    if tilt <= soft_limit:
        return 1.0
    return (hard_limit - tilt) / max(hard_limit - soft_limit, 1e-9)


def wrapped_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))
