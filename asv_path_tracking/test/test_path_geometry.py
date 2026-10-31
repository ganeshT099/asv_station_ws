"""Pure geometry tests for every supported research demonstration shape."""

import math
import unittest

from asv_path_tracking.path_geometry import (
    SUPPORTED_PATHS,
    generate_path,
    wrap_angle,
)


class PathGeometryTest(unittest.TestCase):
    """Protect the finite, closed and smooth path contract."""

    def test_all_shapes_are_closed_and_finite(self):
        for shape in SUPPORTED_PATHS:
            with self.subTest(shape=shape):
                plan = generate_path(
                    shape=shape,
                    width=1.6,
                    height=1.0,
                    sample_count=360,
                )
                self.assertGreaterEqual(len(plan.points), 33)
                self.assertGreater(plan.total_length, 0.1)
                self.assertAlmostEqual(
                    plan.points[0].x,
                    plan.points[-1].x,
                    places=5,
                )
                self.assertAlmostEqual(
                    plan.points[0].y,
                    plan.points[-1].y,
                    places=5,
                )
                for point in plan.points:
                    self.assertTrue(math.isfinite(point.x))
                    self.assertTrue(math.isfinite(point.y))
                    self.assertTrue(math.isfinite(point.yaw))
                    self.assertTrue(math.isfinite(point.curvature))
                    self.assertGreaterEqual(point.s, 0.0)

    def test_sample_wraps_and_nearest_is_consistent(self):
        plan = generate_path('figure_eight', width=1.8, height=0.9)
        first = plan.sample(0.0)
        wrapped = plan.sample(plan.total_length * 3.0)
        self.assertAlmostEqual(first.x, wrapped.x, places=6)
        self.assertAlmostEqual(first.y, wrapped.y, places=6)
        nearest, distance = plan.nearest(first.x + 0.01, first.y)
        self.assertLess(distance, 0.03)
        self.assertGreaterEqual(nearest.s, 0.0)

    def test_yaw_wrap_is_bounded(self):
        plan = generate_path('circle', width=1.0, height=1.0)
        for first, second in zip(plan.points, plan.points[1:]):
            self.assertLessEqual(abs(wrap_angle(second.yaw - first.yaw)), math.pi)

    def test_invalid_shape_is_rejected(self):
        with self.assertRaises(ValueError):
            generate_path('invalid-shape')


if __name__ == '__main__':
    unittest.main()
