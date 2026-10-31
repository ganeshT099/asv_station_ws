"""Pure safety and mixer checks for the standalone path controller."""

import unittest

from asv_path_tracking.path_control_model import (
    mix_thrusters,
    move_toward,
    tilt_scale,
    wrapped_angle,
)


class PathControlModelTest(unittest.TestCase):
    """Verify allocation signs and bounded command helpers."""

    def test_mixer_direction_patterns(self):
        self.assertEqual(mix_thrusters(1.0, 0.0, 0.0), (1.0, 1.0, -1.0, -1.0))
        self.assertEqual(mix_thrusters(0.0, 1.0, 0.0), (-1.0, 1.0, -1.0, 1.0))
        self.assertEqual(mix_thrusters(0.0, 0.0, 1.0), (1.0, -1.0, -1.0, 1.0))

    def test_slew_limit_is_bounded(self):
        self.assertEqual(move_toward(0.0, 1.0, 0.2), 0.2)
        self.assertEqual(move_toward(1.0, 0.0, 0.2), 0.8)
        self.assertEqual(move_toward(0.0, 0.1, 0.2), 0.1)

    def test_tilt_guard_is_continuous_and_safe(self):
        self.assertEqual(tilt_scale(0.0, 0.0, 0.1, 0.35), 1.0)
        self.assertEqual(tilt_scale(0.4, 0.0, 0.1, 0.35), 0.0)
        middle = tilt_scale(0.225, 0.0, 0.1, 0.35)
        self.assertGreater(middle, 0.0)
        self.assertLess(middle, 1.0)

    def test_angle_wrap_is_bounded(self):
        self.assertGreaterEqual(wrapped_angle(100.0), -3.141592653589793)
        self.assertLessEqual(wrapped_angle(-100.0), 3.141592653589793)


if __name__ == '__main__':
    unittest.main()
