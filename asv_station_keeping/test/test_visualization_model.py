"""Offline tests for the station-keeping visualization data model."""

import math
import unittest

from asv_station_keeping.visualization_model import (
    MARKER_IDS,
    THRUSTER_DIRECTIONS,
    THRUSTER_NAMES,
    Health,
    color_for_health,
    diagnostic_float,
    health_from_status,
    parse_diagnostic_values,
    quaternion_from_yaw,
    quaternion_to_yaw,
    rotate_vector_by_quaternion,
)


class DiagnosticValue:
    """Small ROS-message-shaped object for offline parsing tests."""

    def __init__(self, key, value):
        self.key = key
        self.value = value


class VisualizationModelTest(unittest.TestCase):
    """Check the pure parsing, health, geometry, and ID contracts."""

    def test_diagnostic_values_accept_ros_objects_and_tuples(self):
        values = parse_diagnostic_values([
            DiagnosticValue('target_x_m', '1.25'),
            ('target_y_m', '-0.5'),
            ('ignored', 'not used'),
        ])
        self.assertEqual(values['target_x_m'], '1.25')
        self.assertEqual(diagnostic_float(values, 'target_y_m'), -0.5)
        self.assertIsNone(diagnostic_float(values, 'missing'))
        self.assertEqual(
            diagnostic_float(values, 'invalid', 0.25),
            0.25,
        )

    def test_health_prioritizes_staleness_and_diagnostic_level(self):
        self.assertEqual(
            health_from_status(0, 0.1, 1.0, True),
            Health.OK,
        )
        self.assertEqual(
            health_from_status(1, 0.1, 1.0, True),
            Health.WARN,
        )
        self.assertEqual(
            health_from_status(2, 0.1, 1.0, True),
            Health.ERROR,
        )
        self.assertEqual(
            health_from_status(3, 0.1, 1.0, True),
            Health.STALE,
        )
        self.assertEqual(
            health_from_status(0, 1.1, 1.0, True),
            Health.STALE,
        )
        self.assertEqual(
            health_from_status(2, None, 1.0, False),
            Health.STALE,
        )

    def test_health_colors_keep_error_and_stale_distinguishable(self):
        error = color_for_health(Health.ERROR)
        stale = color_for_health(Health.STALE)
        self.assertGreater(error[0], error[1])
        self.assertEqual(stale[0], stale[1])
        self.assertEqual(stale[1], stale[2])

    def test_planar_quaternion_round_trip_and_vector_rotation(self):
        yaw = math.pi / 2.0
        quaternion = quaternion_from_yaw(yaw)
        self.assertAlmostEqual(quaternion_to_yaw(*quaternion), yaw)
        rotated = rotate_vector_by_quaternion((1.0, 0.0, 0.0), quaternion)
        self.assertAlmostEqual(rotated[0], 0.0, places=7)
        self.assertAlmostEqual(rotated[1], 1.0, places=7)
        self.assertAlmostEqual(rotated[2], 0.0, places=7)

    def test_marker_ids_and_thruster_order_are_stable(self):
        self.assertEqual(
            THRUSTER_NAMES,
            ('port_front', 'starboard_front', 'port_rear', 'starboard_rear'),
        )
        self.assertEqual(len(MARKER_IDS), len(set(MARKER_IDS.values())))
        self.assertEqual(MARKER_IDS['target_point'], 0)
        self.assertEqual(MARKER_IDS['thruster_port_front'], 40)
        self.assertEqual(MARKER_IDS['thruster_starboard_rear_text'], 53)

    def test_thruster_visual_directions_match_diagonal_mixer(self):
        for name in THRUSTER_NAMES:
            direction = THRUSTER_DIRECTIONS[name]
            self.assertAlmostEqual(
                math.sqrt(sum(component * component for component in direction)),
                1.0,
            )
        self.assertGreater(THRUSTER_DIRECTIONS['port_front'][1], 0.0)
        self.assertLess(THRUSTER_DIRECTIONS['starboard_front'][1], 0.0)


if __name__ == '__main__':
    unittest.main()
