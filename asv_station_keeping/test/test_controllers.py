# Copyright 2026 ASV Station Keeping contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Closed-loop checks for the pluggable station-keeping control laws."""

import numpy as np
import pytest

from asv_station_keeping.controllers import (
    CONTROLLER_NAMES, LqrController, VehicleSpec, axis_model, make_controller)
from asv_station_keeping.offline_sim import RunConfig, SEAS, simulate


@pytest.mark.parametrize('name', CONTROLLER_NAMES)
def test_zero_error_gives_zero_wrench(name):
    """A vehicle already on station with no velocity needs no force."""
    controller = make_controller(name)
    wrench = controller.compute((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.05)
    assert wrench == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)


@pytest.mark.parametrize('name', CONTROLLER_NAMES)
def test_positive_error_pushes_toward_target(name):
    """Target ahead/left/counter-clockwise must command positive force."""
    controller = make_controller(name)
    wrench = controller.compute((0.3, 0.3, 0.2), (0.0, 0.0, 0.0), 0.05)
    assert all(component > 0.0 for component in wrench)


@pytest.mark.parametrize('name', CONTROLLER_NAMES)
def test_velocity_is_damped(name):
    """Moving toward the target at speed must reduce the commanded force."""
    controller = make_controller(name)
    still = controller.compute((0.3, 0.0, 0.0), (0.0, 0.0, 0.0), 0.05)
    controller.reset()
    moving = controller.compute((0.3, 0.0, 0.0), (0.4, 0.0, 0.0), 0.05)
    assert moving[0] < still[0]


def test_unknown_controller_is_rejected():
    """Typos must fail loudly instead of silently running PID."""
    with pytest.raises(ValueError):
        make_controller('bang-bang')


def test_lqr_closed_loop_is_stable():
    """Every LQR axis loop has all discrete eigenvalues inside the circle."""
    controller = LqrController()
    vehicle = VehicleSpec()
    for index, gain in enumerate(controller.gains):
        inertia, linear, _quad = vehicle.axis(index)
        a_d, b_d = axis_model(inertia, linear, 0.05)
        closed = a_d - b_d @ gain.reshape(1, -1)
        assert np.max(np.abs(np.linalg.eigvals(closed))) < 1.0


def test_mpc_respects_force_limit():
    """The receding-horizon plan never asks for more than the bound."""
    controller = make_controller('mpc')
    limits = (2.0, 2.0, 0.1)
    for _ in range(20):
        wrench = controller.compute(
            (3.0, -3.0, 1.0), (0.0, 0.0, 0.0), 0.05, False, limits)
        for force, bound in zip(wrench, limits):
            assert abs(force) <= bound + 1e-9


@pytest.mark.parametrize('name', CONTROLLER_NAMES)
@pytest.mark.parametrize('sea', ['calm', 'moderate'])
def test_controller_holds_station_in_simulation(name, sea):
    """Every law keeps the surrogate vessel within 15 cm RMS."""
    config = RunConfig(controller=name, sea=SEAS[sea], duration=80.0)
    metrics, _trace = simulate(config, seed=3)
    assert metrics['valid'] and not metrics['diverged']
    assert metrics['rms'] < 0.15


def test_simulation_is_reproducible_and_seed_dependent():
    """Same seed -> identical run; different seed -> different noise."""
    config = RunConfig(controller='lqr', sea=SEAS['moderate'], duration=40.0)
    first, _ = simulate(config, seed=5)
    again, _ = simulate(config, seed=5)
    other, _ = simulate(config, seed=6)
    assert first['rms'] == again['rms']
    assert first['rms'] != other['rms']


def test_thruster_failure_is_survivable_with_reallocation():
    """Re-allocating after a thruster failure keeps the vessel on station."""
    events = ((20.0, 'thruster_gain', (1, 0.0)),
              (22.0, 'allocator_health', (1, 0.0)))
    config = RunConfig(controller='lqr', sea=SEAS['moderate'],
                       duration=80.0, events=events)
    metrics, _trace = simulate(config, seed=2)
    assert metrics['rms'] < 0.25
