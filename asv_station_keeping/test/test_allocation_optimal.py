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

"""Checks for the saturation-aware minimum-energy thrust allocator."""

import random

import numpy as np

from asv_station_keeping.actuation import prioritised_wrench
from asv_station_keeping.twist_to_thruster import (
    FORCE_PER_UNIT, MOMENT_PER_UNIT, allocate, allocate_optimal,
    allocation_matrix)

LIMIT = 5.5


def wrench_of(surge, sway, yaw):
    """Return the demanded (Fx, Fy, Mz)."""
    return np.array([FORCE_PER_UNIT * surge, FORCE_PER_UNIT * sway,
                     MOMENT_PER_UNIT * yaw])


def random_demands(count, seed=0):
    """Yield demands already through the production saturation."""
    rng = random.Random(seed)
    for _ in range(count):
        yield prioritised_wrench(
            rng.uniform(-6, 6), rng.uniform(-6, 6), rng.uniform(-3, 3),
            LIMIT, 2.2)[:3]


def test_matches_least_squares_when_nothing_saturates():
    """Away from the limits the new allocator is the existing one."""
    for demand in random_demands(2000):
        reference = np.array(allocate(*demand))
        if np.max(np.abs(reference)) <= LIMIT:
            commands, scale = allocate_optimal(*demand, limit=LIMIT)
            assert scale == 1.0
            assert np.allclose(commands, reference, atol=1e-6)


def test_never_exceeds_the_limit_and_delivers_the_wrench():
    """Thrust stays in bounds and the commanded wrench is produced."""
    matrix = np.array(allocation_matrix())
    for demand in random_demands(2000, seed=1):
        commands, scale = allocate_optimal(*demand, limit=LIMIT)
        assert np.max(np.abs(commands)) <= LIMIT + 1e-6
        if scale >= 0.999999:
            assert np.allclose(
                matrix @ np.array(commands), wrench_of(*demand), atol=1e-5)


def test_redistributes_where_least_squares_saturates():
    """A demand that clips plain least squares is still fully delivered."""
    demand = (2.0, 2.0, 1.5)
    assert np.max(np.abs(allocate(*demand))) > 4.0
    demand = (2.4, 1.6, 2.2)
    reference = np.array(allocate(*demand))
    assert np.max(np.abs(reference)) > LIMIT
    commands, scale = allocate_optimal(*demand, limit=LIMIT)
    assert scale == 1.0
    assert np.max(np.abs(commands)) <= LIMIT + 1e-6


def test_failed_thruster_gets_no_thrust():
    """A thruster marked unhealthy is excluded and the rest compensate."""
    health = (1.0, 0.0, 1.0, 1.0)
    commands, scale = allocate_optimal(1.0, 0.5, 0.5, health, limit=LIMIT)
    assert commands[1] == 0.0
    assert scale == 1.0
    matrix = np.array(allocation_matrix())
    assert np.allclose(matrix @ np.array(commands),
                       wrench_of(1.0, 0.5, 0.5), atol=1e-6)


def test_infeasible_demand_is_scaled_not_clipped():
    """Over-demand shrinks along the demanded direction."""
    commands, scale = allocate_optimal(8.0, 0.0, 0.0, limit=LIMIT)
    assert 0.0 < scale < 1.0
    assert np.max(np.abs(commands)) <= LIMIT + 1e-6
    assert commands[0] == commands[1] == -commands[2] == -commands[3]
