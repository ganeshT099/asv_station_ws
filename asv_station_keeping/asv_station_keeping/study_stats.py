"""Small, dependency-light statistics for the controller comparison.

All routines use numpy only and take an explicit seed, so every number in a
report can be regenerated exactly.  Experiments are *paired*: the same random
disturbance realisation (seed) is replayed for every controller, so
differences are tested on per-seed differences.
"""

import math

import numpy as np


def bootstrap_ci(values, level=0.95, resamples=5000, seed=0, statistic=np.mean):
    """Return ``(estimate, low, high)``: percentile bootstrap interval."""
    data = np.asarray(values, dtype=float)
    data = data[np.isfinite(data)]
    if data.size == 0:
        return (math.nan, math.nan, math.nan)
    estimate = float(statistic(data))
    if data.size == 1:
        return (estimate, estimate, estimate)
    rng = np.random.default_rng(seed)
    index = rng.integers(0, data.size, size=(resamples, data.size))
    stats = statistic(data[index], axis=1)
    alpha = 0.5 * (1.0 - level)
    low, high = np.quantile(stats, [alpha, 1.0 - alpha])
    return (estimate, float(low), float(high))


def paired_permutation_test(a, b, resamples=20000, seed=0):
    """Two-sided paired sign-flip permutation test on ``mean(a - b)``.

    Exact when there are at most 16 pairs, Monte Carlo otherwise.  Returns
    the p-value (never zero: the observed statistic counts as one sample).
    """
    diff = np.asarray(a, dtype=float) - np.asarray(b, dtype=float)
    diff = diff[np.isfinite(diff)]
    n = diff.size
    if n == 0 or np.allclose(diff, 0.0):
        return 1.0
    observed = abs(diff.mean())
    if n <= 16:
        signs = np.array(
            [[1 if (mask >> k) & 1 else -1 for k in range(n)]
             for mask in range(2 ** n)], dtype=float)
        stats = np.abs((signs * diff).mean(axis=1))
        return float(np.mean(stats >= observed - 1e-15))
    rng = np.random.default_rng(seed)
    signs = rng.choice([-1.0, 1.0], size=(resamples, n))
    stats = np.abs((signs * diff).mean(axis=1))
    return float((np.sum(stats >= observed - 1e-15) + 1) / (resamples + 1))


def paired_effect_size(a, b):
    """Cohen's d_z for paired samples (mean difference / std of differences)."""
    diff = np.asarray(a, dtype=float) - np.asarray(b, dtype=float)
    diff = diff[np.isfinite(diff)]
    if diff.size < 2:
        return math.nan
    spread = diff.std(ddof=1)
    if spread < 1e-15:
        if abs(diff.mean()) < 1e-15:
            return 0.0
        return math.copysign(math.inf, diff.mean())
    return float(diff.mean() / spread)


def holm_correction(p_values):
    """Holm-Bonferroni adjusted p-values (controls family-wise error)."""
    p = np.asarray(p_values, dtype=float)
    order = np.argsort(p)
    adjusted = np.empty_like(p)
    running = 0.0
    m = p.size
    for rank, idx in enumerate(order):
        running = max(running, (m - rank) * p[idx])
        adjusted[idx] = min(1.0, running)
    return adjusted.tolist()


def compare_to_baseline(results, baseline, metric, seed=0):
    """Compare every method with ``baseline`` on one metric.

    ``results`` maps method -> {seed: metrics-dict}.  Returns a list of dict
    rows (one per non-baseline method) with the mean paired difference, its
    95 % bootstrap interval, the percent change, effect size and the
    Holm-adjusted permutation p-value.
    """
    base = results[baseline]
    rows, raw_p = [], []
    for method, runs in results.items():
        if method == baseline:
            continue
        seeds = sorted(set(base) & set(runs))
        a = np.array([runs[s][metric] for s in seeds], dtype=float)
        b = np.array([base[s][metric] for s in seeds], dtype=float)
        mask = np.isfinite(a) & np.isfinite(b)
        a, b = a[mask], b[mask]
        mean_diff, low, high = bootstrap_ci(a - b, seed=seed)
        base_mean = float(b.mean()) if b.size else math.nan
        rows.append({
            'method': method, 'n': int(a.size),
            'mean': float(a.mean()) if a.size else math.nan,
            'diff': mean_diff, 'diff_low': low, 'diff_high': high,
            'percent': 100.0 * mean_diff / base_mean if base_mean else math.nan,
            'effect_size': paired_effect_size(a, b),
        })
        raw_p.append(paired_permutation_test(a, b, seed=seed))
    adjusted = holm_correction(raw_p) if raw_p else []
    for row, raw, adj in zip(rows, raw_p, adjusted):
        row['p_raw'], row['p_holm'] = raw, adj
    return rows


def significance_label(p_value):
    """Compact significance marker for tables."""
    if not np.isfinite(p_value):
        return ''
    if p_value < 0.001:
        return '***'
    if p_value < 0.01:
        return '**'
    if p_value < 0.05:
        return '*'
    return 'ns'
