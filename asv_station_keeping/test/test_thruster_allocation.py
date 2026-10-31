"""The mixer must produce the intended body wrench from the real axes.

The axes below were measured in Fortress by firing each thruster alone: thrust
acts along +X of the propeller link, which the URDF points outward along the
diagonals.  A sign slip here makes the sway loop unstable, which is exactly
what happened before this test existed.
"""

import math

from asv_station_keeping.twist_to_thruster import mix_thrusters
from asv_station_keeping.visualization_model import THRUSTER_DIRECTIONS

POSITIONS = {
    'port_front': (0.25, 0.175),
    'starboard_front': (0.25, -0.175),
    'port_rear': (-0.25, 0.175),
    'starboard_rear': (-0.25, -0.175),
}
NAMES = ('port_front', 'starboard_front', 'port_rear', 'starboard_rear')


def wrench(surge, sway, yaw):
    commands = mix_thrusters(surge, sway, yaw)
    fx = fy = torque = 0.0
    for name, thrust in zip(NAMES, commands):
        dx, dy, _ = THRUSTER_DIRECTIONS[name]
        x, y = POSITIONS[name]
        fx += thrust * dx
        fy += thrust * dy
        torque += x * thrust * dy - y * thrust * dx
    return fx, fy, torque


def test_surge_produces_only_forward_force():
    fx, fy, torque = wrench(1.0, 0.0, 0.0)
    assert fx > 0.0
    assert math.isclose(fy, 0.0, abs_tol=1e-9)
    assert math.isclose(torque, 0.0, abs_tol=1e-9)


def test_sway_produces_only_leftward_force():
    fx, fy, torque = wrench(0.0, 1.0, 0.0)
    assert fy > 0.0
    assert math.isclose(fx, 0.0, abs_tol=1e-9)
    assert math.isclose(torque, 0.0, abs_tol=1e-9)


def test_yaw_produces_only_left_turn_torque():
    fx, fy, torque = wrench(0.0, 0.0, 1.0)
    assert torque > 0.0
    assert math.isclose(fx, 0.0, abs_tol=1e-9)
    assert math.isclose(fy, 0.0, abs_tol=1e-9)


def test_diagnostic_level_accepts_humble_byte_values():
    from asv_station_keeping.visualization_model import level_as_int
    assert level_as_int(b'\x02') == 2
    assert level_as_int(b'\x00') == 0
    assert level_as_int(1) == 1
