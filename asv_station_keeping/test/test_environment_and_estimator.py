"""Sea-state conversion and estimator maths (no ROS graph needed)."""

import math

from asv_station_keeping.current_disturbance import (
    equivalent_water_speed, wave_orbital_speed)
from asv_station_keeping.state_estimator import (
    AxisFilter, geodetic_to_local, rotate_by_quaternion)


def test_equivalent_speed_reproduces_the_drag_force():
    for force in (-3.0, -0.2, 0.0, 0.5, 4.0):
        speed = equivalent_water_speed(force, 0.6, 2.0)
        assert math.isclose(
            0.6 * speed + 2.0 * abs(speed) * speed, force, abs_tol=1e-9)


def test_wave_orbital_speed_has_zero_mean_and_expected_peak():
    period, height = 2.5, 0.1
    samples = [
        wave_orbital_speed(height, period, k * period / 100)
        for k in range(100)]
    assert abs(sum(samples)) < 1e-9
    assert math.isclose(
        max(samples), 0.5 * height * 2 * math.pi / period, rel_tol=1e-3)
    assert wave_orbital_speed(0.0, period, 1.0) == 0.0


def test_geodetic_to_local_matches_one_degree_scales():
    east, north = geodetic_to_local(13.0827 + 1e-5, 80.2707, 13.0827, 80.2707)
    assert math.isclose(north, 1.1058, rel_tol=2e-3)
    assert abs(east) < 1e-9
    east, _ = geodetic_to_local(13.0827, 80.2707 + 1e-5, 13.0827, 80.2707)
    assert math.isclose(east, 1.0850, rel_tol=2e-3)


def test_quaternion_rotation_by_ninety_degrees_about_z():
    s = math.sqrt(0.5)
    x, y, z = rotate_by_quaternion((0.0, 0.0, s, s), (1.0, 0.0, 0.0))
    assert math.isclose(x, 0.0, abs_tol=1e-12)
    assert math.isclose(y, 1.0, abs_tol=1e-12)
    assert math.isclose(z, 0.0, abs_tol=1e-12)


def test_filter_converges_to_constant_velocity_from_position_fixes():
    axis = AxisFilter(0.04, 0.05, 0.0)
    true_velocity = 0.3
    for step in range(1, 400):
        for _ in range(10):
            axis.predict(0.0, 0.01)
        axis.update(true_velocity * step * 0.1)
    assert math.isclose(axis.v, true_velocity, abs_tol=0.02)
    assert math.isclose(axis.p, true_velocity * 399 * 0.1, abs_tol=0.05)


def test_filter_recovers_from_an_imu_shock():
    """A violent IMU spike must not leave the estimate running away."""
    axis = AxisFilter(0.04, 0.05, 0.0)
    for _ in range(300):
        axis.predict(0.0, 0.01)
    axis.predict(350.0, 0.01)  # impact spike
    for step in range(200):
        for _ in range(10):
            axis.predict(0.0, 0.01)
        axis.update(0.0)  # vehicle actually stayed put
    assert abs(axis.p) < 0.1
    assert abs(axis.v) < 0.2
