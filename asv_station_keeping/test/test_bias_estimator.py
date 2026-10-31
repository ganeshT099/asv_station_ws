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

"""The accelerometer-bias filter must beat the plain filter on a biased IMU."""

import random

from asv_station_keeping.state_estimator import AxisFilter, BiasAxisFilter


def run_axis(filter_cls, bias, outage_from, steps=6000, seed=4):
    """Stationary vehicle, biased IMU, GPS lost after outage_from s."""
    rng = random.Random(seed)
    estimator = filter_cls(0.04, 0.05, 0.0)
    dt = 0.01
    worst = 0.0
    for k in range(steps):
        t = k * dt
        estimator.predict(bias + rng.gauss(0.0, 0.04), dt)
        if k % 10 == 0 and t < outage_from:
            estimator.update(rng.gauss(0.0, 0.05))
        if t > (outage_from + 1.0 if outage_from < 1e8 else 5.0):
            worst = max(worst, abs(estimator.p))
    return worst


def test_bias_state_limits_drift_during_gps_outage():
    """A 0.05 m/s^2 bias, learned while GPS is up, is removed in the outage."""
    plain = run_axis(AxisFilter, 0.05, outage_from=30.0)
    biased = run_axis(BiasAxisFilter, 0.05, outage_from=30.0)
    assert biased < 0.5 * plain


def test_bias_state_converges_to_the_true_bias():
    """After 30 s of GPS the bias estimate is close to the injected bias."""
    rng = random.Random(1)
    estimator = BiasAxisFilter(0.04, 0.05, 0.0)
    for k in range(3000):
        estimator.predict(0.05 + rng.gauss(0.0, 0.04), 0.01)
        if k % 10 == 0:
            estimator.update(rng.gauss(0.0, 0.05))
    assert abs(estimator.b - 0.05) < 0.02


def test_bias_state_is_harmless_without_bias():
    """With an unbiased IMU the extra state costs no accuracy."""
    plain = run_axis(AxisFilter, 0.0, outage_from=1e9)
    biased = run_axis(BiasAxisFilter, 0.0, outage_from=1e9)
    assert biased < 1.5 * plain + 0.02
