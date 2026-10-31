"""Deterministic, ROS-independent geometric path generation helpers."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Iterable, List, Sequence, Tuple


SUPPORTED_PATHS = (
    'circle',
    'ellipse',
    'figure_eight',
    'rounded_rectangle',
    'triangle',
)


def wrap_angle(angle: float) -> float:
    """Wrap an angle to the interval [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class PathPoint:
    """One path sample with tangent and cumulative arc-length information."""

    x: float
    y: float
    yaw: float
    curvature: float
    s: float


@dataclass(frozen=True)
class PathPlan:
    """A closed, uniformly sampled path with interpolation helpers."""

    name: str
    points: Tuple[PathPoint, ...]
    total_length: float
    center_x: float
    center_y: float
    closed: bool = True

    def sample(self, distance: float) -> PathPoint:
        """Interpolate a path point at a wrapped arc length."""
        if len(self.points) < 2 or self.total_length <= 1e-9:
            return self.points[0]
        if self.closed:
            distance %= self.total_length
        else:
            distance = min(max(distance, 0.0), self.total_length)

        for first, second in zip(self.points, self.points[1:]):
            if distance <= second.s:
                span = max(second.s - first.s, 1e-12)
                ratio = (distance - first.s) / span
                yaw_delta = wrap_angle(second.yaw - first.yaw)
                return PathPoint(
                    x=first.x + ratio * (second.x - first.x),
                    y=first.y + ratio * (second.y - first.y),
                    yaw=wrap_angle(first.yaw + ratio * yaw_delta),
                    curvature=first.curvature + ratio * (
                        second.curvature - first.curvature),
                    s=distance,
                )
        return self.points[-1]

    def nearest(self, x: float, y: float) -> Tuple[PathPoint, float]:
        """Return the closest sampled/interpolated path point and distance."""
        if not self.points:
            raise ValueError('path plan has no points')
        best = min(
            self.points,
            key=lambda point: (point.x - x) ** 2 + (point.y - y) ** 2,
        )
        distance = math.hypot(best.x - x, best.y - y)
        return best, distance


def _shape_xy(
    shape: str,
    phase: float,
    width: float,
    height: float,
    center_x: float,
    center_y: float,
    corner_roundness: float,
) -> Tuple[float, float]:
    """Evaluate one closed shape at a phase in [0, 2*pi]."""
    shape = shape.lower().strip()
    a = max(abs(width) * 0.5, 0.05)
    b = max(abs(height) * 0.5, 0.05)
    c = math.cos(phase)
    s = math.sin(phase)

    if shape == 'circle':
        a = b = max(a, b)
        return center_x + a * c, center_y + a * s
    if shape == 'ellipse':
        return center_x + a * c, center_y + b * s
    if shape == 'figure_eight':
        return center_x + a * s, center_y + b * s * c
    if shape == 'rounded_rectangle':
        exponent = max(2.0, min(float(corner_roundness), 12.0))
        x = math.copysign(abs(c) ** (2.0 / exponent), c)
        y = math.copysign(abs(s) ** (2.0 / exponent), s)
        return center_x + a * x, center_y + b * y
    if shape == 'triangle':
        # A three-lobed polar curve is a continuously differentiable,
        # corner-smoothed triangle suitable for a velocity reference.
        radius = 0.78 + 0.22 * math.cos(3.0 * phase)
        return (
            center_x + a * radius * c,
            center_y + b * radius * s,
        )
    raise ValueError(
        f'unsupported path {shape!r}; choose one of {SUPPORTED_PATHS}')


def _finite_difference(
    evaluator: Callable[[float], Tuple[float, float]],
    phase: float,
    step: float,
) -> Tuple[float, float, float, float]:
    """Return first and second derivatives of a periodic planar curve."""
    x0, y0 = evaluator(phase)
    xp, yp = evaluator(phase + step)
    xm, ym = evaluator(phase - step)
    xpp, ypp = evaluator(phase + 2.0 * step)
    xmm, ymm = evaluator(phase - 2.0 * step)
    dx = (xp - xm) / (2.0 * step)
    dy = (yp - ym) / (2.0 * step)
    ddx = (xpp - 2.0 * x0 + xmm) / (4.0 * step * step)
    ddy = (ypp - 2.0 * y0 + ymm) / (4.0 * step * step)
    return dx, dy, ddx, ddy


def generate_path(
    shape: str = 'circle',
    width: float = 1.6,
    height: float = 1.2,
    center_x: float = 0.0,
    center_y: float = 0.0,
    sample_count: int = 500,
    corner_roundness: float = 4.0,
) -> PathPlan:
    """Generate a smooth, closed path with tangent yaw and curvature."""
    shape = shape.lower().strip()
    if shape not in SUPPORTED_PATHS:
        raise ValueError(
            f'unsupported path {shape!r}; choose one of {SUPPORTED_PATHS}')
    count = max(int(sample_count), 32)
    step = 2.0 * math.pi / count
    evaluator = lambda phase: _shape_xy(
        shape,
        phase,
        width,
        height,
        center_x,
        center_y,
        corner_roundness,
    )

    raw: List[Tuple[float, float, float, float]] = []
    for index in range(count + 1):
        phase = index * step
        x, y = evaluator(phase)
        dx, dy, ddx, ddy = _finite_difference(evaluator, phase, step)
        speed = max(math.hypot(dx, dy), 1e-9)
        yaw = math.atan2(dy, dx)
        curvature = (dx * ddy - dy * ddx) / (speed ** 3)
        raw.append((x, y, yaw, curvature))

    cumulative = [0.0]
    for first, second in zip(raw, raw[1:]):
        cumulative.append(cumulative[-1] + math.hypot(
            second[0] - first[0], second[1] - first[1]))
    total = max(cumulative[-1], 1e-9)
    points = tuple(
        PathPoint(
            x=item[0],
            y=item[1],
            yaw=item[2],
            curvature=item[3],
            s=cumulative[index],
        )
        for index, item in enumerate(raw)
    )
    return PathPlan(
        name=shape,
        points=points,
        total_length=total,
        center_x=center_x,
        center_y=center_y,
    )


def path_points(plan: PathPlan) -> Iterable[PathPoint]:
    """Return the plan samples without exposing its internal tuple."""
    return tuple(plan.points)
