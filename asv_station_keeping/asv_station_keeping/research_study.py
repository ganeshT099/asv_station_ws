"""Controller-comparison study: seeded experiments, statistics and report.

Research question
    Which station-keeping control law holds position best on a twin-hull,
    four-thruster ASV under combined wind, current and waves, at what thruster
    energy cost, and how does each degrade under actuator/sensor faults and
    model error?

Method
    Every experiment is *paired*: seed ``k`` replays the identical
    disturbance and sensor-noise realisation for every method, so methods are
    compared on per-seed differences (bootstrap 95 % intervals, paired
    sign-flip permutation tests, Holm correction, effect sizes).  Wind
    direction and speed are randomised per seed so no law is tuned to one
    geometry.

Sub-commands::

    ros2 run asv_station_keeping research_study offline --out ~/study --seeds 30
    ros2 run asv_station_keeping research_study gazebo  --out ~/study --seeds 3 --jobs 3
    ros2 run asv_station_keeping research_study report  --out ~/study

``offline`` uses the fast surrogate (see ``offline_sim``); ``gazebo`` runs a
smaller confirmation set in the real simulator and the report cross-checks
the two.
"""

import argparse
import base64
import csv
import dataclasses
import html
import io
import json
import math
import os
from pathlib import Path
import random
import signal
import subprocess
import sys
import time

import numpy as np

from asv_station_keeping.offline_sim import (
    RunConfig, SEAS, SeaState, simulate)
from asv_station_keeping.study_stats import (
    bootstrap_ci, compare_to_baseline, significance_label)

CONTROLLERS = ('pid', 'lqr', 'smc', 'mpc')
LABEL = {'pid': 'PID', 'lqr': 'LQR', 'smc': 'Sliding mode', 'mpc': 'MPC'}
COLOUR = {'pid': '#2a78d6', 'lqr': '#1baf7a', 'smc': '#eb6834',
          'mpc': '#7a4fd1', 'off': '#2a78d6', 'dob': '#1baf7a',
          'lstsq': '#2a78d6', 'optimal': '#1baf7a'}
METRICS = ('rms', 'p95', 'max', 'yaw_rms_deg', 'power', 'sat', 'within_5cm')
EXPERIMENT_ORDER = ('controllers', 'observer', 'allocation', 'faults',
                    'robustness')
TITLES = {
    'controllers': 'E1 - Control law comparison by sea state',
    'observer': 'E2 - Effect of the disturbance-observer feed-forward',
    'allocation': 'E3 - Thrust allocation: least-squares vs saturation-aware',
    'faults': 'E4 - Fault and disturbance-step response',
    'robustness': 'E5 - Robustness to plant model error',
}
FAULTS = {
    'thruster_failure_blind': (
        'Thruster fails, allocator unaware',
        [(25.0, 'thruster_gain', (1, 0.0))]),
    'thruster_failure_realloc': (
        'Thruster fails, re-allocated after 2 s',
        [(25.0, 'thruster_gain', (1, 0.0)),
         (27.0, 'allocator_health', (1, 0.0))]),
    'gps_dropout': (
        'GPS lost for 10 s (dead reckoning)',
        [(25.0, 'gps', False), (35.0, 'gps', True)]),
    'gps_jump': (
        'GPS jumps 0.4 m (multipath) for 10 s',
        [(25.0, 'gps_jump', (0.4, 0.0)), (35.0, 'gps_jump', (0.0, 0.0))]),
    'wind_step': (
        'Wind steps from 3 to 6 m/s',
        [(25.0, 'wind_speed', 6.0)]),
}
MODEL_ERRORS = {
    'mass_x0.7': dict(mass=0.7),
    'mass_x1.3': dict(mass=1.3),
    'drag_x0.5': dict(drag=0.5),
    'drag_x2.0': dict(drag=2.0),
}
HEAVY = SeaState(5.0, 135.0, 0.35, 12.0, 0.15, 0.05, 0.03, 0.15,
                 2.5, 135.0)
STUDY_SEAS = dict(SEAS)
STUDY_SEAS['heavy'] = HEAVY


# ---------------------------------------------------------------------------
# Experiment definition
# ---------------------------------------------------------------------------
def randomised_sea(base, seed):
    """Per-seed sea: random wind/wave direction and +/-15 % wind speed."""
    rng = random.Random(seed * 7919 + 13)
    direction = rng.uniform(0.0, 360.0)
    return dataclasses.replace(
        base,
        wind_speed=base.wind_speed * rng.uniform(0.85, 1.15),
        wind_direction_deg=direction,
        wave_direction_deg=(direction + rng.uniform(-30.0, 30.0)) % 360.0,
        current_x=base.current_x * rng.uniform(-1.0, 1.0)
        if base.current_x else 0.0,
        current_y=base.current_y + base.current_x * rng.uniform(-1.0, 1.0),
    )


def trial(experiment, condition, method, seed, **fields):
    """Build one picklable trial specification."""
    spec = dict(experiment=experiment, condition=condition, method=method,
                seed=seed, controller='pid', sea='moderate', use_dob=False,
                allocation='lstsq', events=(), model_error={},
                duration=120.0, randomise=True)
    spec.update(fields)
    return spec


def build_trials(seeds, experiments=EXPERIMENT_ORDER, include_limit=False):
    """Return the full list of trial specifications."""
    seas_main = ['calm', 'moderate', 'strong', 'heavy']
    if include_limit:
        STUDY_SEAS['limit'] = SEAS['limit']
        seas_main.append('limit')
    trials = []
    for seed in range(1, seeds + 1):
        if 'controllers' in experiments:
            for sea in seas_main:
                for ctrl in CONTROLLERS:
                    trials.append(trial('controllers', sea, ctrl, seed,
                                        controller=ctrl, sea=sea))
        if 'observer' in experiments:
            for sea in ('moderate', 'strong', 'heavy'):
                for ctrl in ('pid', 'lqr', 'mpc'):
                    for dob in (False, True):
                        trials.append(trial(
                            'observer', f'{sea} / {LABEL[ctrl]}',
                            'dob' if dob else 'off', seed,
                            controller=ctrl, sea=sea, use_dob=dob))
        if 'allocation' in experiments:
            for sea in ('strong', 'heavy'):
                for ctrl in ('pid', 'lqr', 'mpc'):
                    for mode in ('lstsq', 'optimal'):
                        trials.append(trial(
                            'allocation', f'{sea} / {LABEL[ctrl]}',
                            mode, seed, controller=ctrl, sea=sea,
                            allocation=mode))
        if 'faults' in experiments:
            for key, (_text, events) in FAULTS.items():
                for ctrl in CONTROLLERS:
                    trials.append(trial(
                        'faults', key, ctrl, seed, controller=ctrl,
                        sea='moderate', events=tuple(events),
                        duration=90.0))
        if 'robustness' in experiments:
            for key, error in MODEL_ERRORS.items():
                for ctrl in CONTROLLERS:
                    trials.append(trial(
                        'robustness', key, ctrl, seed, controller=ctrl,
                        sea='moderate', model_error=dict(error)))
    return trials


def run_trial(spec):
    """Execute one trial in the surrogate; return a flat result row."""
    base = STUDY_SEAS.get(spec['sea']) or SEAS[spec['sea']]
    sea = randomised_sea(base, spec['seed']) if spec['randomise'] else base
    config = RunConfig(
        controller=spec['controller'], sea=sea, duration=spec['duration'],
        use_dob=spec['use_dob'], allocation=spec['allocation'],
        events=tuple(spec['events']), model_error=dict(spec['model_error']))
    metrics, _trace = simulate(config, seed=spec['seed'])
    row = {k: spec[k] for k in ('experiment', 'condition', 'method', 'seed')}
    row['source'] = 'surrogate'
    row.update({k: metrics.get(k, math.nan) for k in METRICS})
    row['event_peak'] = metrics.get('event_peak', math.nan)
    row['recovery_s'] = metrics.get('recovery_s', math.nan)
    row['diverged'] = int(bool(metrics.get('diverged', False)))
    row['valid'] = int(bool(metrics.get('valid', False)))
    return row


COLUMNS = (['source', 'experiment', 'condition', 'method', 'seed']
           + list(METRICS)
           + ['event_peak', 'recovery_s', 'diverged', 'valid'])


def write_rows(path, rows):
    """Write result rows to CSV."""
    with Path(path).open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, '') for k in COLUMNS})


def read_rows(path):
    """Read result rows back, converting numeric columns."""
    rows = []
    with Path(path).open(newline='', encoding='utf-8') as handle:
        for raw in csv.DictReader(handle):
            row = dict(raw)
            for key in list(METRICS) + ['event_peak', 'recovery_s']:
                row[key] = float(raw[key]) if raw[key] not in ('', None) \
                    else math.nan
            row['seed'] = int(raw['seed'])
            rows.append(row)
    return rows


# ---------------------------------------------------------------------------
# Statistics over result rows
# ---------------------------------------------------------------------------
def group(rows, experiment, source='surrogate'):
    """Return {condition: {method: {seed: row}}} for one experiment."""
    out = {}
    for row in rows:
        if row['experiment'] != experiment or row.get('source') != source:
            continue
        if str(row.get('valid', '1')) in ('0', 'False'):
            continue
        out.setdefault(row['condition'], {}).setdefault(
            row['method'], {})[row['seed']] = row
    return out


def baseline_of(experiment, methods):
    """Pick the reference method for an experiment."""
    preferred = {'controllers': 'pid', 'faults': 'pid', 'robustness': 'pid',
                 'observer': 'off', 'allocation': 'lstsq'}[experiment]
    return preferred if preferred in methods else sorted(methods)[0]


def analyse(rows, experiment, metrics=('rms', 'p95', 'power', 'sat'),
            source='surrogate'):
    """Per-condition statistics: {condition: {metric: [comparison rows]}}."""
    out = {}
    for condition, methods in group(rows, experiment, source).items():
        base = baseline_of(experiment, methods)
        entry = {'baseline': base, 'methods': {}, 'compare': {}}
        for method, runs in methods.items():
            stats = {}
            for metric in list(metrics) + ['event_peak', 'recovery_s',
                                           'yaw_rms_deg', 'within_5cm']:
                values = [r[metric] for r in runs.values()]
                stats[metric] = bootstrap_ci(values)
            stats['n'] = len(runs)
            stats['diverged'] = sum(
                int(str(r.get('diverged', 0)) in ('1', 'True'))
                for r in runs.values())
            entry['methods'][method] = stats
        for metric in list(metrics) + ['event_peak', 'recovery_s']:
            numeric = {m: {s: r for s, r in runs.items()}
                       for m, runs in methods.items()}
            entry['compare'][metric] = compare_to_baseline(
                numeric, base, metric)
        out[condition] = entry
    return out


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------
def figure_to_img(fig, alt):
    """Embed a matplotlib figure as a base64 PNG."""
    buffer = io.BytesIO()
    fig.savefig(buffer, format='png', dpi=130, bbox_inches='tight',
                facecolor='white')
    encoded = base64.b64encode(buffer.getvalue()).decode()
    return f'<img alt="{html.escape(alt)}" src="data:image/png;base64,{encoded}">'


def fmt_ci(triple, scale=1.0, digits=1):
    """Format ``mean [low, high]``."""
    mean, low, high = triple
    if not math.isfinite(mean):
        return 'n/a'
    return (f'{mean * scale:.{digits}f} '
            f'<span class="ci">[{low * scale:.{digits}f}, '
            f'{high * scale:.{digits}f}]</span>')


def comparison_table(stats, columns, label):
    """HTML table: methods x metrics with deltas against the baseline."""
    base = stats['baseline']
    head = ''.join(f'<th>{html.escape(t)}</th>' for t, *_ in columns)
    lines = [f'<tr><th>{html.escape(label)}</th><th>n</th>{head}'
             f'<th>vs {html.escape(LABEL.get(base, base))} (primary)</th></tr>']
    primary = columns[0][1]
    compare = {c['method']: c for c in stats['compare'].get(primary, [])}
    for method, values in stats['methods'].items():
        cells = ''.join(
            f'<td>{fmt_ci(values[key], scale, digits)}</td>'
            for _t, key, scale, digits in columns)
        if method == base:
            delta = '<td class="base">baseline</td>'
        else:
            c = compare.get(method)
            if c is None or not math.isfinite(c['percent']):
                delta = '<td>n/a</td>'
            else:
                cls = 'good' if c['percent'] < 0 and c['p_holm'] < 0.05 \
                    else ('bad' if c['percent'] > 0 and c['p_holm'] < 0.05
                          else '')
                delta = (f'<td class="{cls}">{c["percent"]:+.0f}% '
                         f'<span class="ci">p={c["p_holm"]:.3f} '
                         f'{significance_label(c["p_holm"])} '
                         f'd={c["effect_size"]:.2f}</span></td>')
        div = f' <span class="warn">({values["diverged"]} diverged)</span>' \
            if values['diverged'] else ''
        lines.append(
            f'<tr><td>{html.escape(LABEL.get(method, method))}{div}</td>'
            f'<td>{values["n"]}</td>{cells}{delta}</tr>')
    return '<table>' + ''.join(lines) + '</table>'


def style_axes(ax):
    """Light, minimal axes."""
    ax.set_facecolor('white')
    ax.grid(True, color='#e4e3df', linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    ax.tick_params(colors='#52514e', labelsize=8)


def bar_chart(analysis, conditions, methods, metric, scale, ylabel, title):
    """Draw grouped bars with 95 % CI whiskers."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(1.6 * len(conditions) + 2.5, 3.4))
    style_axes(ax)
    width = 0.8 / max(len(methods), 1)
    for j, method in enumerate(methods):
        xs, ys, lows, highs = [], [], [], []
        for i, condition in enumerate(conditions):
            stats = analysis.get(condition, {}).get('methods', {}).get(method)
            if not stats:
                continue
            mean, low, high = stats[metric]
            xs.append(i + (j - (len(methods) - 1) / 2) * width)
            ys.append(mean * scale)
            lows.append((mean - low) * scale)
            highs.append((high - mean) * scale)
        ax.bar(xs, ys, width * 0.9, color=COLOUR.get(method, '#888'),
               label=LABEL.get(method, method),
               yerr=[lows, highs], capsize=2, error_kw={'lw': 0.9})
    ax.set_xticks(range(len(conditions)))
    ax.set_xticklabels(conditions, fontsize=8, rotation=15, ha='right')
    ax.set_ylabel(ylabel, color='#52514e')
    ax.set_title(title, loc='left', fontsize=10)
    ax.legend(frameon=False, fontsize=8)
    fig.tight_layout()
    return fig


def trace_figure(fault_keys):
    """Error-vs-time traces (seed 1) of every controller for each fault."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, len(fault_keys),
                             figsize=(3.6 * len(fault_keys), 3.1),
                             sharey=True)
    axes = [axes] if len(fault_keys) == 1 else list(axes)
    for ax, key in zip(axes, fault_keys):
        style_axes(ax)
        text, events = FAULTS[key]
        for ctrl in CONTROLLERS:
            config = RunConfig(
                controller=ctrl, sea=randomised_sea(SEAS['moderate'], 1),
                duration=90.0, events=tuple(events))
            _m, trace = simulate(config, seed=1, record=True)
            ax.plot(trace['t'], [e * 100 for e in trace['err']],
                    color=COLOUR[ctrl], lw=1.0, label=LABEL[ctrl])
        for event in events:
            ax.axvline(event[0], color='#999', ls='--', lw=0.8)
        ax.set_title(text, fontsize=8, loc='left')
        ax.set_xlabel('time (s)', color='#52514e')
    axes[0].set_ylabel('position error (cm)', color='#52514e')
    axes[0].legend(frameon=False, fontsize=7)
    fig.tight_layout()
    return fig


def build_report(out_dir):
    """Write ``report.html`` and ``summary.json`` from the saved rows."""
    out_dir = Path(out_dir)
    rows = []
    for name in ('trials.csv', 'gazebo_trials.csv'):
        if (out_dir / name).exists():
            rows += read_rows(out_dir / name)
    if not rows:
        raise SystemExit('No results found; run "offline" first.')
    seeds = sorted({r['seed'] for r in rows if r['source'] == 'surrogate'})
    sections, summary = [], {}
    have_plots = True
    try:
        import matplotlib  # noqa: F401
    except ImportError:
        have_plots = False

    primary = [('RMS (cm)', 'rms', 100.0, 1), ('P95 (cm)', 'p95', 100.0, 1),
               ('Power', 'power', 1.0, 2), ('Sat. (%)', 'sat', 100.0, 1),
               ('Yaw RMS (&deg;)', 'yaw_rms_deg', 1.0, 1)]
    event = [('Peak error after event (cm)', 'event_peak', 100.0, 1),
             ('Recovery to &lt;10 cm (s)', 'recovery_s', 1.0, 1),
             ('RMS (cm)', 'rms', 100.0, 1), ('Power', 'power', 1.0, 2)]
    for experiment in EXPERIMENT_ORDER:
        analysis = analyse(rows, experiment)
        if not analysis:
            continue
        summary[experiment] = analysis
        body = []
        conditions = list(analysis)
        methods = list(next(iter(analysis.values()))['methods'])
        columns = event if experiment == 'faults' else primary
        if have_plots and experiment in ('controllers', 'robustness'):
            metric = 'rms'
            fig = bar_chart(analysis, conditions, methods, metric, 100.0,
                            'RMS position error (cm)', TITLES[experiment])
            body.append(figure_to_img(fig, TITLES[experiment]))
        elif have_plots and experiment == 'allocation':
            fig = bar_chart(analysis, conditions, methods, 'sat', 100.0,
                            'Time at thrust limit (%)', TITLES[experiment])
            body.append(figure_to_img(fig, TITLES[experiment]))
        elif have_plots and experiment == 'faults':
            body.append(figure_to_img(trace_figure(list(analysis)), 'Fault'))
        for condition in conditions:
            text = FAULTS.get(condition, (condition,))[0] \
                if experiment == 'faults' else condition
            body.append(f'<h3>{html.escape(text)}</h3>')
            body.append('<div class="table-wrap">'
                        + comparison_table(analysis[condition], columns,
                                           'Method') + '</div>')
        sections.append(
            f'<section><h2>{html.escape(TITLES[experiment])}</h2>'
            + ''.join(body) + '</section>')

    gazebo = gazebo_section(rows)
    validity = validity_section(rows)
    css = '''
body{font:15px/1.5 system-ui,sans-serif;color:#0b0b0b;background:#fff;margin:0}
main{max-width:1000px;margin:0 auto;padding:1.5rem 1rem 4rem}
h1{font-size:1.6rem}h2{margin-top:2.4rem;border-top:1px solid #e4e3df;
padding-top:1.2rem}h3{font-size:1rem;margin:1.2rem 0 .3rem;color:#52514e}
.table-wrap{overflow-x:auto}table{border-collapse:collapse;font-size:.86rem;
width:100%}th,td{padding:.35rem .6rem;border-bottom:1px solid #e4e3df;
text-align:right;white-space:nowrap}th:first-child,td:first-child{text-align:left}
th{color:#52514e;font-weight:600}.ci{color:#52514e;font-size:.78em}
.good{color:#0d7a52;font-weight:600}.bad{color:#b3340c;font-weight:600}
.base{color:#52514e}.warn{color:#b3340c}.note{color:#52514e;font-size:.86rem}
img{max-width:100%}code{background:#f3f2ee;padding:0 .25rem;border-radius:3px}
'''
    page = f'''<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Station-keeping controller study</title><style>{css}</style></head>
<body><main>
<h1>Station-keeping controller study</h1>
<p class="note">Twin-hull, four-thruster ASV. {len(seeds)} paired seeds per
condition (seed k replays the identical disturbance and sensor noise for every
method; wind direction and speed are randomised per seed). Values are
<b>mean [95% bootstrap CI]</b>. Differences from the baseline use a paired
sign-flip permutation test with Holm correction (<code>*</code> p&lt;.05,
<code>**</code> p&lt;.01, <code>***</code> p&lt;.001, <code>ns</code> not
significant) and the paired effect size d<sub>z</sub>. Green = significantly
better than baseline, red = significantly worse. Scores use ground truth,
after a 20 s settling period.</p>
{validity}{''.join(sections)}{gazebo}
<section><h2>Method and limitations</h2><ul>
<li>Experiments E1-E5 run on a 3-DOF surrogate that shares the production
controller, saturation, allocation and actuator code with the ROS node. Its
plant constants are nominal; E5 shows sensitivity to them.</li>
<li>The surrogate was checked against Gazebo (table above): close in calm and
moderate seas, <b>not</b> in the strongest seas, where the simulated vessel
saturates harder. Conclusions for strong/heavy seas should be read from the
Gazebo confirmation runs.</li>
<li>Hydrodynamic constants are simulation estimates, not yet identified from
hardware (<code>identify_hydrodynamics.py</code> is provided for that).</li>
</ul></section></main></body></html>'''
    (out_dir / 'report.html').write_text(page, encoding='utf-8')

    def clean(obj):
        if isinstance(obj, dict):
            return {str(k): clean(v) for k, v in obj.items()}
        if isinstance(obj, (list, tuple)):
            return [clean(v) for v in obj]
        if isinstance(obj, float) and not math.isfinite(obj):
            return None
        return obj
    (out_dir / 'summary.json').write_text(
        json.dumps(clean(summary), indent=1), encoding='utf-8')
    print(f'[report] {out_dir / "report.html"}')


def gazebo_section(rows):
    """Table of the Gazebo confirmation runs, if present."""
    gz = [r for r in rows if r['source'] == 'gazebo']
    if not gz:
        return ''
    by = {}
    for r in gz:
        by.setdefault(r['condition'], {}).setdefault(
            r['method'], {})[r['seed']] = r
    parts = []
    for condition, methods in by.items():
        base = 'pid' if 'pid' in methods else sorted(methods)[0]
        stats = {'baseline': base, 'methods': {}, 'compare': {}}
        for method, runs in methods.items():
            stats['methods'][method] = {
                m: bootstrap_ci([r[m] for r in runs.values()])
                for m in ('rms', 'p95', 'power', 'sat', 'yaw_rms_deg')}
            stats['methods'][method]['n'] = len(runs)
            stats['methods'][method]['diverged'] = 0
        for metric in ('rms',):
            stats['compare'][metric] = compare_to_baseline(
                methods, base, metric)
        columns = [('RMS (cm)', 'rms', 100.0, 1),
                   ('P95 (cm)', 'p95', 100.0, 1),
                   ('Power', 'power', 1.0, 2),
                   ('Sat. (%)', 'sat', 100.0, 1),
                   ('Yaw RMS (&deg;)', 'yaw_rms_deg', 1.0, 1)]
        parts.append(f'<h3>{html.escape(condition)}</h3><div class="table-wrap">'
                     + comparison_table(stats, columns, 'Method') + '</div>')
    return ('<section><h2>G - Gazebo confirmation (full simulator)</h2>'
            '<p class="note">Real physics, Gazebo wrench-based wind and waves, '
            'noisy GPS/IMU estimator in the loop. Few seeds, so intervals are '
            'wide; treat as confirmation of the surrogate ranking.</p>'
            + ''.join(parts) + '</section>')


def validity_section(rows):
    """Surrogate-vs-Gazebo agreement table for the baseline controller."""
    gz = {}
    for r in rows:
        if r['source'] == 'gazebo' and r['method'] == 'pid':
            gz.setdefault(r['condition'], []).append(r['rms'])
    sg = {}
    for r in rows:
        if (r['source'] == 'surrogate' and r['experiment'] == 'controllers'
                and r['method'] == 'pid'):
            sg.setdefault(r['condition'], []).append(r['rms'])
    common = [c for c in sg if c in gz]
    if not common:
        return ''
    lines = ''.join(
        f'<tr><td>{c}</td><td>{np.mean(sg[c]) * 100:.1f}</td>'
        f'<td>{np.mean(gz[c]) * 100:.1f}</td>'
        f'<td>{100 * (np.mean(sg[c]) / np.mean(gz[c]) - 1):+.0f}%</td></tr>'
        for c in common)
    return ('<section><h2>Surrogate validity (PID, RMS cm)</h2>'
            '<table><tr><th>Sea</th><th>Surrogate</th><th>Gazebo</th>'
            f'<th>Difference</th></tr>{lines}</table></section>')


# ---------------------------------------------------------------------------
# Gazebo confirmation runs
# ---------------------------------------------------------------------------
SETTLE_S = 20.0


def gazebo_metrics(csv_path):
    """Score one Gazebo CSV (same definitions as the surrogate)."""
    with Path(csv_path).open(newline='') as handle:
        rows = list(csv.DictReader(handle))
    keep = [r for r in rows if float(r['elapsed_s']) >= SETTLE_S]
    if len(keep) < 20:
        return None
    err = sorted(float(r['position_error']) for r in keep)
    yaw = []
    for r in keep:
        d = float(r['target_yaw']) - float(r['true_yaw'])
        yaw.append(abs(math.atan2(math.sin(d), math.cos(d))))
    span = float(keep[-1]['elapsed_s']) - float(keep[0]['elapsed_s'])
    power = 0.0
    if span > 0:
        power = (float(keep[-1]['energy_index'])
                 - float(keep[0]['energy_index'])) / span
    return {
        'rms': math.sqrt(sum(e * e for e in err) / len(err)),
        'p95': err[int(0.95 * (len(err) - 1))], 'max': err[-1],
        'yaw_rms_deg': math.degrees(
            math.sqrt(sum(y * y for y in yaw) / len(yaw))),
        'power': power,
        'sat': sum(r['thrust_saturated'] == 'True' for r in keep) / len(keep),
        'within_5cm': sum(e < 0.05 for e in err) / len(err),
    }


def run_gazebo_job(job):
    """Launch one headless Gazebo run and score it."""
    out_dir, name, spec, slot, wall = job
    csv_path = Path(out_dir) / 'gazebo' / f'{name}.csv'
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    if csv_path.exists():
        csv_path.unlink()
    sea = randomised_sea(STUDY_SEAS[spec['sea']], spec['seed'])
    args = {
        'headless': 'true', 'log_file': str(csv_path),
        'map_log': str(csv_path.with_suffix('.map.csv')),
        'seed': spec['seed'], 'controller_type': spec['controller'],
        'allocation_mode': spec['allocation'],
        'use_dob': str(spec['use_dob']).lower(),
        'target_x': 0.0, 'target_y': 0.0, 'target_yaw': 0.0,
        'wave_heave_scale': 1.0, 'wave_moment_scale': 0.08,
    }
    args.update(dataclasses.asdict(sea))
    env = dict(os.environ)
    env['IGN_PARTITION'] = f'asv_study_{slot}'
    env['ROS_DOMAIN_ID'] = str(60 + slot)
    command = ['ros2', 'launch', 'asv_station_keeping',
               'dynamic_station_keeping.launch.py',
               *[f'{k}:={v}' for k, v in args.items()]]
    log = Path(out_dir) / 'gazebo' / f'{name}.log'
    print(f'[gazebo] {name}', flush=True)
    with log.open('w') as handle:
        process = subprocess.Popen(
            command, stdout=handle, stderr=subprocess.STDOUT, env=env,
            start_new_session=True)
        try:
            process.wait(timeout=wall)
        except subprocess.TimeoutExpired:
            pass
        for sig, wait in ((signal.SIGINT, 15), (signal.SIGKILL, 5)):
            try:
                os.killpg(process.pid, sig)
                process.wait(timeout=wait)
                break
            except ProcessLookupError:
                break
            except subprocess.TimeoutExpired:
                continue
    time.sleep(1.0)
    metrics = gazebo_metrics(csv_path) if csv_path.exists() else None
    row = {k: spec[k] for k in ('experiment', 'condition', 'method', 'seed')}
    row['source'] = 'gazebo'
    if metrics is None:
        row.update({k: math.nan for k in METRICS})
        row['valid'] = 0
    else:
        row.update(metrics)
        row['valid'] = 1
    row.update(event_peak=math.nan, recovery_s=math.nan, diverged=0)
    return row


def cleanup_gazebo():
    """Kill stray simulator processes between batches."""
    for pattern in ('ign gazebo', 'gz sim', 'parameter_bridge'):
        subprocess.run(['pkill', '-f', pattern], check=False)
    time.sleep(2.0)


def command_gazebo(args):
    """Run the Gazebo confirmation set (real-time; slow)."""
    from concurrent.futures import ThreadPoolExecutor
    import queue
    out_dir = Path(args.out).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    specs = []
    for seed in range(1, args.seeds + 1):
        for sea in args.seas:
            for ctrl in args.controllers:
                specs.append(trial(
                    'gazebo', sea, ctrl, seed, controller=ctrl, sea=sea))
    cleanup_gazebo()
    slots = queue.Queue()
    for slot in range(max(args.jobs, 1)):
        slots.put(slot)

    def job(spec):
        slot = slots.get()
        try:
            name = f"{spec['condition']}_{spec['method']}_s{spec['seed']}"
            return run_gazebo_job((out_dir, name, spec, slot, args.duration))
        finally:
            slots.put(slot)

    with ThreadPoolExecutor(max_workers=max(args.jobs, 1)) as pool:
        results = [f.result() for f in [pool.submit(job, s) for s in specs]]
    cleanup_gazebo()
    existing = []
    path = out_dir / 'gazebo_trials.csv'
    if path.exists() and not args.fresh:
        done = {(r['condition'], r['method'], r['seed']) for r in results}
        existing = [r for r in read_rows(path)
                    if (r['condition'], r['method'], r['seed']) not in done]
    write_rows(path, existing + results)
    build_report(out_dir)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def command_offline(args):
    """Run the surrogate experiments in parallel and write the report."""
    from multiprocessing import Pool
    out_dir = Path(args.out).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    trials = build_trials(args.seeds, tuple(args.experiments),
                          args.include_limit)
    print(f'[offline] {len(trials)} trials on {args.jobs} processes',
          flush=True)
    started = time.time()
    # Slowest first (MPC) keeps the pool balanced.
    trials.sort(key=lambda t: t['controller'] != 'mpc')
    with Pool(args.jobs) as pool:
        rows = pool.map(run_trial, trials, chunksize=4)
    print(f'[offline] done in {time.time() - started:.0f} s', flush=True)
    rows.sort(key=lambda r: (r['experiment'], r['condition'], r['method'],
                             r['seed']))
    write_rows(out_dir / 'trials.csv', rows)
    build_report(out_dir)


def command_report(args):
    """Rebuild the report from saved results."""
    build_report(Path(args.out).expanduser())


def main(argv=None):
    """Entry point."""
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    off = sub.add_parser('offline', help='surrogate Monte Carlo study')
    off.add_argument('--out', required=True)
    off.add_argument('--seeds', type=int, default=30)
    off.add_argument('--jobs', type=int, default=max(1, (os.cpu_count() or 2) - 2))
    off.add_argument('--experiments', nargs='+', default=list(EXPERIMENT_ORDER),
                     choices=EXPERIMENT_ORDER)
    off.add_argument('--include-limit', action='store_true',
                     help='add the beyond-authority stress sea')
    off.set_defaults(func=command_offline)
    gz = sub.add_parser('gazebo', help='real-simulator confirmation runs')
    gz.add_argument('--out', required=True)
    gz.add_argument('--seeds', type=int, default=3)
    gz.add_argument('--jobs', type=int, default=3)
    gz.add_argument('--duration', type=float, default=90.0,
                    help='wall-clock seconds per run')
    gz.add_argument('--seas', nargs='+', default=['moderate', 'strong'])
    gz.add_argument('--controllers', nargs='+', default=list(CONTROLLERS))
    gz.add_argument('--fresh', action='store_true')
    gz.set_defaults(func=command_gazebo)
    rep = sub.add_parser('report', help='rebuild report from saved results')
    rep.add_argument('--out', required=True)
    rep.set_defaults(func=command_report)
    args = parser.parse_args(argv)
    args.func(args)
    return 0


if __name__ == '__main__':
    sys.exit(main())
