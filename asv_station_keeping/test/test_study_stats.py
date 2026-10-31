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

"""Checks for the paired statistics used by the controller study."""

import numpy as np

from asv_station_keeping.study_stats import (
    bootstrap_ci, compare_to_baseline, holm_correction,
    paired_effect_size, paired_permutation_test)


def test_bootstrap_interval_contains_the_estimate():
    """The percentile interval brackets the sample mean."""
    values = np.random.default_rng(1).normal(5.0, 1.0, 40)
    mean, low, high = bootstrap_ci(values)
    assert low < mean < high
    assert abs(mean - 5.0) < 0.5


def test_permutation_test_detects_a_clear_difference():
    """A consistent paired shift is significant."""
    rng = np.random.default_rng(2)
    base = rng.normal(10.0, 1.0, 30)
    assert paired_permutation_test(base - 1.0, base) < 0.001


def test_permutation_test_accepts_no_difference():
    """Pure noise around zero is not significant."""
    rng = np.random.default_rng(3)
    base = rng.normal(10.0, 1.0, 30)
    other = base + rng.normal(0.0, 0.2, 30)
    assert paired_permutation_test(other, base) > 0.05


def test_identical_samples_give_p_one():
    """No difference at all -> p = 1."""
    data = [1.0, 2.0, 3.0]
    assert paired_permutation_test(data, data) == 1.0


def test_exact_permutation_for_small_samples():
    """With 4 pairs all the same sign the exact two-sided p is 2/16."""
    assert abs(paired_permutation_test([2, 2, 2, 2], [1, 1, 1, 1])
               - 2 / 16) < 1e-12


def test_holm_is_monotone_and_not_smaller_than_raw():
    """Holm adjustment never lowers a p-value and keeps the ordering."""
    raw = [0.01, 0.04, 0.03, 0.2]
    adjusted = holm_correction(raw)
    assert all(a >= r for a, r in zip(adjusted, raw))
    order = np.argsort(raw)
    assert all(adjusted[order[i]] <= adjusted[order[i + 1]]
               for i in range(len(raw) - 1))


def test_effect_size_sign():
    """Lower values than baseline give a negative d_z."""
    a = np.array([1.0, 1.1, 0.9, 1.0, 1.2])
    b = a + 1.0
    assert paired_effect_size(a, b) < 0


def test_compare_to_baseline_reports_percent_change():
    """A method 20 % below baseline on every seed shows -20 %."""
    results = {
        'base': {s: {'rms': 1.0 + 0.1 * s} for s in range(12)},
        'better': {s: {'rms': 0.8 * (1.0 + 0.1 * s)} for s in range(12)},
    }
    row = compare_to_baseline(results, 'base', 'rms')[0]
    assert abs(row['percent'] + 20.0) < 1e-6
    assert row['p_holm'] < 0.01
