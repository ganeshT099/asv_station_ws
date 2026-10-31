#!/usr/bin/env python3
"""Run the station-keeping comparison suite headless and write an HTML report.

Usage (from a shell where ROS 2 and the workspace are sourced)::

    python3 run_experiments.py --out ~/asv_results --jobs 3
    python3 run_experiments.py --out ~/asv_results --montecarlo 12 --jobs 3
    python3 run_experiments.py --out ~/asv_results --report-only

Every condition runs the same vehicle, world and sensors; only the sea state,
seed and the switches under test change.  "Uncontrolled" is the identical
stack with the thrusters off.  All scores use simulator ground truth.
Parallel jobs are isolated with separate Gazebo partitions and ROS domains.
"""

import argparse
import base64
import csv
import html
import io
import json
import math
import os
import queue
import random
import signal
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

SETTLE_S = 20.0  # discard the initial transient from the statistics
POOL_HALF = (4.95, 3.95)
CTRL = '/research_station_keeping_pid'
EST = '/research_state_estimator'
SEA = '/research_current_disturbance'

SEAS = {
    'calm': dict(
        wind_speed=0.0, wave_height=0.0, current_turbulence_std=0.0,
        current_x=0.10, current_y=0.0, wind_gust_std=0.0),
    'moderate': dict(
        wind_speed=3.0, wave_height=0.10, current_turbulence_std=0.02,
        current_x=0.10, current_y=0.0, wind_gust_std=0.30),
    'strong': dict(
        wind_speed=4.5, wave_height=0.15, current_turbulence_std=0.03,
        current_x=0.15, current_y=0.05, wind_gust_std=0.30),
    # Beyond what the current thruster layout can resist (yaw authority).
    'limit': dict(
        wind_speed=5.5, wave_height=0.15, current_turbulence_std=0.03,
        current_x=0.15, current_y=0.05, wind_gust_std=0.40),
}
CTRL_ARGS = {
    'off': {'hold_enabled': 'false'},
    'pid': {},
    'dob': {'use_dob': 'true'},
}
CTRL_TEXT = {'off': 'uncontrolled', 'pid': 'PID', 'dob': 'PID + observer'}
CTRL_LABEL = {'off': 'Uncontrolled', 'pid': 'PID', 'dob': 'PID + observer'}


def cond(sea, ctrl, seed=1, extra=None, text=None, events=None, group='sea',
         sea_override=None):
    return dict(sea=sea, ctrl=ctrl, seed=seed, extra=extra or {},
                text=text or f'{sea.capitalize()}, {CTRL_TEXT[ctrl]}',
                events=events, group=group, sea_override=sea_override)


def build_conditions():
    conditions = {}
    for sea in ('calm', 'moderate', 'strong'):
        for ctrl in ('off', 'pid', 'dob'):
            conditions[f'{sea}_{ctrl}'] = cond(sea, ctrl)
    for seed in (2, 3):
        conditions[f'strong_pid_s{seed}'] = cond(
            'strong', 'pid', seed, group='seeds',
            text=f'Strong, PID, seed {seed}')
    conditions['mission_weathervane'] = cond(
        'moderate', 'pid', extra={'mission': 'weathervane'}, group='mission',
        text='Moderate, PID, bow into wind (weathervane)')
    conditions['limit_pid'] = cond(
        'limit', 'pid', group='limit',
        text='Limit case (5.5 m/s, 40% gusts), PID')
    for seed in (2, 3):
        for ctrl in ('pid', 'dob'):
            conditions[f'moderate_{ctrl}_s{seed}'] = cond(
                'moderate', ctrl, seed,
                text=f'Moderate, {CTRL_TEXT[ctrl]}, seed {seed}',
                group='seeds')
    conditions['abl_truth_feedback'] = cond(
        'moderate', 'pid', extra={'use_estimator': 'false'},
        text='Moderate, PID, feedback on ground truth', group='ablation')
    conditions['abl_ideal_thrusters'] = cond(
        'moderate', 'pid', extra={'actuator_model': 'false'},
        text='Moderate, PID, ideal thrusters', group='ablation')
    thruster_fail = [[25, CTRL, 'thruster_gain_1', 0.0,
                      'Starboard-front thruster fails']]
    realloc = thruster_fail + [[27, CTRL, 'allocator_health_1', 0.0,
                                'Fault detected, thrust re-allocated']]
    conditions['fault_thruster_blind'] = cond(
        'moderate', 'pid', events=thruster_fail, group='fault',
        text='Thruster failure, allocator unaware')
    conditions['fault_thruster_realloc'] = cond(
        'moderate', 'pid', events=realloc, group='fault',
        text='Thruster failure, thrust re-allocated')
    conditions['fault_gps_dropout'] = cond(
        'moderate', 'pid', group='fault', text='GPS lost for 10 s',
        events=[[25, EST, 'gps_enabled', False, 'GPS lost'],
                [35, EST, 'gps_enabled', True, 'GPS restored']])
    step = [[25, SEA, 'wind_speed', 6.0, 'Wind jumps from 3 to 6 m/s']]
    for ctrl in ('pid', 'dob'):
        conditions[f'step_{ctrl}'] = cond(
            'moderate', ctrl, events=step, group='step',
            text=f'Wind step 3 to 6 m/s, {CTRL_TEXT[ctrl]}')
    return conditions


CONDITIONS = build_conditions()


def monte_carlo_conditions(count):
    rng = random.Random(2026)
    out = {}
    for k in range(count):
        sea = dict(
            wind_speed=round(rng.uniform(1.0, 6.0), 2),
            wind_direction_deg=round(rng.uniform(0, 360), 0),
            wave_height=round(rng.uniform(0.0, 0.15), 3),
            wave_direction_deg=round(rng.uniform(0, 360), 0),
            current_turbulence_std=0.02,
            current_x=round(rng.uniform(-0.15, 0.15), 3),
            current_y=round(rng.uniform(-0.15, 0.15), 3),
            wind_gust_std=0.3)
        for ctrl in ('pid', 'dob'):
            out[f'mc{k:02d}_{ctrl}'] = cond(
                'moderate', ctrl, seed=100 + k, group='mc',
                text=f'MC {k}: {sea["wind_speed"]} m/s wind, '
                     f'{CTRL_TEXT[ctrl]}',
                sea_override=sea)
    return out


def launch_args(spec, csv_path):
    sea_values = dict(SEAS[spec['sea']])
    args = {
        'headless': 'true', 'log_file': str(csv_path),
        'map_log': str(csv_path.with_suffix('.map.csv')),
        'seed': str(spec['seed']),
        'target_x': '0.0', 'target_y': '0.0', 'target_yaw': '0.0',
        'wind_direction_deg': '135.0', 'wave_direction_deg': '135.0',
        # Keep the published results reproducible: full wave coupling.
        'wave_heave_scale': '1.0', 'wave_moment_scale': '0.08',
    }
    sea_values.update(spec['sea_override'] or {})
    args.update({k: str(v) for k, v in sea_values.items()})
    args.update(CTRL_ARGS[spec['ctrl']])
    args.update({k: str(v) for k, v in spec['extra'].items()})
    if spec['events']:
        args['scenario'] = 'custom'
        args['scenario_events'] = json.dumps(spec['events'])
    return [f'{k}:={v}' for k, v in args.items()]


def global_cleanup():
    for pattern in ('ign gazebo', 'gz sim', 'parameter_bridge',
                    'sea_state_gz_bridge'):
        subprocess.run(['pkill', '-f', pattern], check=False)
    time.sleep(2.0)


def run_condition(name, spec, out_dir, duration, slot):
    csv_path = out_dir / f'{name}.csv'
    log_path = out_dir / f'{name}.log'
    if csv_path.exists():
        csv_path.unlink()
    env = dict(os.environ)
    env['IGN_PARTITION'] = f'asv_job_{slot}'
    env['ROS_DOMAIN_ID'] = str(40 + slot)
    command = ['ros2', 'launch', 'asv_station_keeping',
               'dynamic_station_keeping.launch.py',
               *launch_args(spec, csv_path)]
    print(f'[run] {name}: {spec["text"]}', flush=True)
    with log_path.open('w') as log:
        process = subprocess.Popen(
            command, stdout=log, stderr=subprocess.STDOUT, env=env,
            start_new_session=True)
        try:
            process.wait(timeout=duration)
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


def load(path):
    with Path(path).open(newline='') as handle:
        rows = list(csv.DictReader(handle))
    cols = {}
    for key in rows[0]:
        try:
            cols[key] = [float(row[key]) for row in rows]
        except ValueError:
            cols[key] = [1.0 if row[key] == 'True' else 0.0 for row in rows]
    return cols


def percentile(values, q):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(round(q * (len(ordered) - 1))))]


def metrics(cols):
    keep = [i for i, t in enumerate(cols['elapsed_s']) if t >= SETTLE_S]
    if len(keep) < 20:
        return None
    err = [cols['position_error'][i] for i in keep]
    est = [cols['estimate_error'][i] for i in keep]
    yaw_err = []
    for i in keep:
        d = cols['target_yaw'][i] - cols['true_yaw'][i]
        yaw_err.append(abs(math.atan2(math.sin(d), math.cos(d))))
    span = cols['elapsed_s'][keep[-1]] - cols['elapsed_s'][keep[0]]
    energy = 0.0
    if 'energy_index' in cols and span > 0:
        energy = (cols['energy_index'][keep[-1]]
                  - cols['energy_index'][keep[0]]) / span
    return {
        'n': len(keep),
        'rms': math.sqrt(sum(e * e for e in err) / len(err)),
        'p95': percentile(err, 0.95),
        'max': max(err),
        'yaw_rms_deg': math.degrees(
            math.sqrt(sum(e * e for e in yaw_err) / len(yaw_err))),
        'est_rms': math.sqrt(sum(e * e for e in est) / len(est)),
        'power': energy,
        'sat': sum(cols['thrust_saturated'][i] for i in keep) / len(keep),
        'max_pitch_deg': math.degrees(
            max(abs(cols['pitch'][i]) for i in keep)),
    }


# --------------------------------------------------------------------------
# Report
# --------------------------------------------------------------------------
BLUE, ORANGE, AQUA = '#2a78d6', '#eb6834', '#1baf7a'
INK, MUTED, GRID = '#0b0b0b', '#52514e', '#e4e3df'
COLOUR = {'off': ORANGE, 'pid': BLUE, 'dob': AQUA}


def figure_to_img(fig, alt):
    buffer = io.BytesIO()
    fig.savefig(buffer, format='png', dpi=140, bbox_inches='tight',
                facecolor='white')
    encoded = base64.b64encode(buffer.getvalue()).decode()
    return (f'<img alt="{html.escape(alt)}" '
            f'src="data:image/png;base64,{encoded}">')


def style_axes(ax):
    ax.set_facecolor('white')
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color('#b9b8b2')
    ax.tick_params(colors=MUTED, labelsize=8)


def build_report(out_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    specs = dict(CONDITIONS)
    for path in out_dir.glob('mc*_*.csv'):
        if '.map' in path.name:
            continue
        name = path.stem
        k, ctrl = name.split('_')
        specs[name] = cond('moderate', ctrl, group='mc',
                           text=f'Monte Carlo {k[2:]}, {CTRL_TEXT[ctrl]}')
    data, results = {}, {}
    for name in specs:
        path = out_dir / f'{name}.csv'
        if path.exists():
            try:
                data[name] = load(path)
                results[name] = metrics(data[name])
            except (ValueError, KeyError, IndexError):
                results[name] = None
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 9})
    sections = []

    # ---- A: paths by sea state
    fig, axes = plt.subplots(2, 3, figsize=(11, 7.2))
    for col, sea in enumerate(('calm', 'moderate', 'strong')):
        top, bottom = axes[0][col], axes[1][col]
        for ax in (top, bottom):
            style_axes(ax)
            ax.set_aspect('equal')
        top.add_patch(Rectangle((-POOL_HALF[0], -POOL_HALF[1]),
                                2 * POOL_HALF[0], 2 * POOL_HALF[1],
                                fill=False, edgecolor='#b9b8b2'))
        top.set_xlim(-5.4, 5.4)
        top.set_ylim(-4.4, 4.4)
        top.set_title(sea.capitalize() + ' sea', color=INK, fontsize=10,
                      loc='left')
        for ctrl in ('off', 'pid', 'dob'):
            cols = data.get(f'{sea}_{ctrl}')
            if not cols:
                continue
            top.plot(cols['true_x'], cols['true_y'], color=COLOUR[ctrl],
                     linewidth=1.5, label=CTRL_LABEL[ctrl] if col == 0 else None)
            if ctrl != 'off':
                bottom.plot(cols['true_x'], cols['true_y'],
                            color=COLOUR[ctrl], linewidth=1.1)
        top.plot([0], [0], marker='+', color=INK, markersize=9)
        bottom.add_patch(Circle((0, 0), 0.05, fill=False, edgecolor=INK,
                                linestyle='--', linewidth=1))
        bottom.plot([0], [0], marker='+', color=INK, markersize=8)
        bottom.set_xlim(-0.35, 0.35)
        bottom.set_ylim(-0.35, 0.35)
        bottom.set_xlabel('x (m)', color=MUTED)
        if col == 0:
            top.set_ylabel('y (m) — pool view', color=MUTED)
            bottom.set_ylabel('y (m) — zoom ±0.35 m', color=MUTED)
            top.legend(frameon=False, fontsize=8, loc='lower left')
    fig.tight_layout()
    sections.append((
        'Where the vehicle goes',
        'Top: full pool. The uncontrolled hull is carried away by wind and '
        'current. Bottom: the controlled tracks, zoomed; the dashed circle '
        'is the 5 cm tolerance.', figure_to_img(fig, 'Vehicle paths')))
    plt.close(fig)

    # ---- B: RMS bars by sea and controller (log)
    fig, ax = plt.subplots(figsize=(7, 3.4))
    style_axes(ax)
    ax.set_yscale('log')
    width = 0.26
    for j, ctrl in enumerate(('off', 'pid', 'dob')):
        for i, sea in enumerate(('calm', 'moderate', 'strong')):
            res = results.get(f'{sea}_{ctrl}')
            if not res:
                continue
            x = i + (j - 1) * width
            ax.bar(x, res['rms'], width * 0.92, color=COLOUR[ctrl],
                   label=CTRL_LABEL[ctrl] if i == 0 else None)
            ax.text(x, res['rms'] * 1.12, f"{res['rms'] * 100:.0f}",
                    ha='center', fontsize=7.5, color=INK)
    ax.set_xticks(range(3))
    ax.set_xticklabels(['Calm', 'Moderate', 'Strong'])
    ax.set_ylabel('RMS position error (m, log)', color=MUTED)
    ax.legend(frameon=False, fontsize=8, loc='upper left')
    fig.tight_layout()
    sections.append((
        'RMS error by sea state and controller',
        'Labels are cm. Log scale because the uncontrolled and controlled '
        'cases differ by one to two orders of magnitude.',
        figure_to_img(fig, 'RMS error by sea state')))
    plt.close(fig)

    # ---- C: faults
    fault_names = [n for n in ('fault_thruster_blind', 'fault_thruster_realloc',
                               'fault_gps_dropout') if n in data]
    if fault_names:
        fig, axes = plt.subplots(1, len(fault_names), figsize=(11, 3.3),
                                 sharey=True)
        axes = [axes] if len(fault_names) == 1 else list(axes)
        for ax, name in zip(axes, fault_names):
            style_axes(ax)
            cols = data[name]
            ax.plot(cols['elapsed_s'], cols['position_error'], color=BLUE,
                    linewidth=1.3)
            for event in specs[name]['events'] or []:
                ax.axvline(event[0] + 4, color=ORANGE, linestyle='--',
                           linewidth=1)
                ax.text(event[0] + 4.5, ax.get_ylim()[1] * 0.92,
                        event[4], fontsize=6.5, color=MUTED, rotation=90,
                        va='top')
            ax.set_title(specs[name]['text'], fontsize=9, loc='left')
            ax.set_xlabel('time since control start (s)', color=MUTED)
        axes[0].set_ylabel('position error (m)', color=MUTED)
        fig.tight_layout()
        sections.append((
            'Fault injection',
            'Dashed lines mark injected faults (timing approximate). '
            'Re-allocating thrust over the three healthy thrusters recovers '
            'station keeping; ignoring the failure does not.',
            figure_to_img(fig, 'Fault injection responses')))
        plt.close(fig)

    # ---- C1: lidar map error vs localization (GPS dropout)
    def map_series(name):
        path = out_dir / f'{name}.map.csv'
        if not path.exists():
            return None
        with path.open(newline='') as handle:
            rows = list(csv.DictReader(handle))
        if len(rows) < 10:
            return None
        t0 = float(rows[0]['time'])
        return ([float(r['time']) - t0 for r in rows],
                [float(r['map_rms_est']) * 100 for r in rows],
                [float(r['map_rms_true']) * 100 for r in rows])

    series = {n: map_series(n) for n in ('fault_gps_dropout', 'moderate_pid')}
    if series['fault_gps_dropout'] and series['moderate_pid']:
        fig, ax = plt.subplots(figsize=(7.5, 3.3))
        style_axes(ax)
        t, est, true = series['fault_gps_dropout']
        ax.plot(t, est, color=ORANGE, linewidth=1.3,
                label='Map built with estimated pose (GPS lost 10 s)')
        t2, est2, _ = series['moderate_pid']
        ax.plot(t2, est2, color=BLUE, linewidth=1.3,
                label='Map built with estimated pose (GPS healthy)')
        ax.plot(t, true, color=MUTED, linewidth=1.0, linestyle=':',
                label='Map built with true pose (lidar noise floor)')
        ax.set_xlabel('time since mapping start (s)', color=MUTED)
        ax.set_ylabel('map error vs pool walls (cm)', color=MUTED)
        ax.legend(frameon=False, fontsize=7.5)
        fig.tight_layout()
        sections.append((
            'Localization quality sets map quality',
            'The lidar map is only as good as the pose used to place each '
            'scan. When GPS is lost the estimate drifts and the map smears; '
            'with healthy localization it stays near the lidar noise floor.',
            figure_to_img(fig, 'Map error versus localization')))
        plt.close(fig)

    # ---- C2: wind step
    if 'step_pid' in data or 'step_dob' in data:
        fig, ax = plt.subplots(figsize=(7.5, 3.3))
        style_axes(ax)
        notes = []
        for ctrl in ('pid', 'dob'):
            cols = data.get(f'step_{ctrl}')
            if not cols:
                continue
            ax.plot(cols['elapsed_s'], cols['position_error'],
                    color=COLOUR[ctrl], linewidth=1.3,
                    label=CTRL_TEXT[ctrl])
            after = [e for t, e in zip(cols['elapsed_s'],
                                       cols['position_error']) if t > 29]
            if after:
                notes.append(f'{CTRL_TEXT[ctrl]} peak {max(after) * 100:.0f} cm')
        ax.axvline(29, color=INK, linestyle='--', linewidth=1)
        ax.text(29.5, ax.get_ylim()[1] * 0.92, 'wind 3 to 6 m/s',
                fontsize=7, color=MUTED, va='top')
        ax.set_xlabel('time since control start (s)', color=MUTED)
        ax.set_ylabel('position error (m)', color=MUTED)
        ax.legend(frameon=False, fontsize=8)
        fig.tight_layout()
        sections.append((
            'Sudden wind step',
            'Wind doubles at the dashed line. ' + '; '.join(notes) + '.',
            figure_to_img(fig, 'Wind step response')))
        plt.close(fig)

    # ---- D: Monte Carlo
    mc = {}
    for name, res in results.items():
        if name.startswith('mc') and res:
            mc.setdefault(name.split('_')[1], []).append(res)
    mc_text = ''
    if mc:
        fig, ax = plt.subplots(figsize=(7, 3.4))
        style_axes(ax)
        for j, ctrl in enumerate(('pid', 'dob')):
            values = [r['rms'] * 100 for r in mc.get(ctrl, [])]
            if not values:
                continue
            jitter = [j + (random.Random(i).random() - 0.5) * 0.25
                      for i in range(len(values))]
            ax.scatter(jitter, values, color=COLOUR[ctrl], s=28,
                       label=CTRL_TEXT[ctrl])
            ax.hlines(sum(values) / len(values), j - 0.3, j + 0.3,
                      color=INK, linewidth=1.2)
        ax.axhline(5, color=INK, linestyle='--', linewidth=1)
        ax.set_xticks([0, 1])
        ax.set_xticklabels(['PID', 'PID + observer'])
        ax.set_ylabel('RMS error per run (cm)', color=MUTED)
        ax.set_xlim(-0.6, 1.6)
        fig.tight_layout()
        parts = []
        for ctrl in ('pid', 'dob'):
            rs = mc.get(ctrl, [])
            if rs:
                ok = sum(1 for r in rs if r['max'] < 0.15)
                parts.append(
                    f'{CTRL_TEXT[ctrl]}: {ok}/{len(rs)} runs stayed within '
                    f'15 cm (mean RMS '
                    f'{sum(r["rms"] for r in rs) / len(rs) * 100:.1f} cm)')
        mc_text = '; '.join(parts)
        sections.append((
            'Monte Carlo over random sea states',
            'Each dot is one run with random wind (1-6 m/s, any direction), '
            'waves, current and noise seed. ' + mc_text + '. Dashed line: '
            '5 cm.', figure_to_img(fig, 'Monte Carlo results')))
        plt.close(fig)

    # ---- table
    rows = []
    order = [n for n in specs if specs[n]['group'] != 'mc']
    for name in order:
        spec, res = specs[name], results.get(name)
        if not res:
            rows.append(f'<tr><td>{html.escape(spec["text"])}</td>'
                        '<td colspan="8">not run</td></tr>')
            continue
        rows.append(
            f'<tr><td>{html.escape(spec["text"])}</td><td>{spec["seed"]}</td>'
            f"<td>{res['rms'] * 100:.1f}</td><td>{res['p95'] * 100:.1f}</td>"
            f"<td>{res['max'] * 100:.1f}</td><td>{res['yaw_rms_deg']:.1f}</td>"
            f"<td>{res['est_rms'] * 100:.1f}</td><td>{res['power']:.2f}</td>"
            f"<td>{res['sat'] * 100:.0f}</td></tr>")

    seed_text = ''
    for ctrl in ('pid', 'dob'):
        names = [f'moderate_{ctrl}'] + [f'moderate_{ctrl}_s{s}' for s in (2, 3)]
        vals = [results[n]['rms'] * 100 for n in names if results.get(n)]
        if len(vals) > 1:
            seed_text += (f'{CTRL_TEXT[ctrl]}: {min(vals):.1f}–{max(vals):.1f}'
                          f' cm over {len(vals)} seeds. ')
    body = ''.join(
        f'<section><h2>{html.escape(t)}</h2><p class="note">{html.escape(n)}'
        f'</p>{img}</section>' for t, n, img in sections)
    page = f'''<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ASV Station Keeping Results</title>
<style>
:root {{ color-scheme: light; --bg:#fcfcfb; --ink:#0b0b0b; --muted:#52514e;
  --line:#e4e3df; }}
body {{ margin:0; background:var(--bg); color:var(--ink);
  font:15px/1.55 system-ui,sans-serif; padding:2rem 1.25rem 4rem; }}
main {{ max-width:60rem; margin:0 auto; }}
h1 {{ font-size:1.6rem; margin:0 0 .25rem; }}
h2 {{ font-size:1.1rem; margin:2.2rem 0 .3rem; }}
.note, .sub {{ color:var(--muted); margin:.1rem 0 .8rem; }}
img {{ max-width:100%; height:auto; border:1px solid var(--line);
  border-radius:6px; background:#fff; }}
.table-wrap {{ overflow-x:auto; }}
table {{ border-collapse:collapse; width:100%; font-size:.85rem; }}
th,td {{ padding:.4rem .6rem; border-bottom:1px solid var(--line);
  text-align:right; white-space:nowrap; }}
th:first-child, td:first-child {{ text-align:left; white-space:normal; }}
th {{ color:var(--muted); font-weight:600; }}
.key span {{ display:inline-block; width:.8rem; height:.8rem;
  border-radius:2px; margin:0 .3rem 0 1rem; vertical-align:-1px; }}
ul {{ padding-left:1.2rem; }}
</style></head><body><main>
<h1>Dynamic station keeping — simulation results</h1>
<p class="sub">Two-hull, four-thruster ASV · ROS 2 Humble / Gazebo Fortress ·
target held at the origin, heading 0°.</p>
<p class="key"><span style="background:{ORANGE}"></span>Uncontrolled
<span style="background:{BLUE}"></span>PID station keeping
<span style="background:{AQUA}"></span>PID + disturbance observer</p>
<p>{html.escape(seed_text)}</p>
{body}
<section><h2>All conditions</h2>
<p class="note">Statistics exclude the first {SETTLE_S:.0f} s of each run and
are scored against simulator ground truth. Power is a relative index
(sum of |thrust|^1.5, simulation units). Saturation is the share of samples
at the thrust limit.</p>
<div class="table-wrap"><table>
<tr><th>Condition</th><th>Seed</th><th>RMS (cm)</th><th>P95 (cm)</th>
<th>Max (cm)</th><th>Heading RMS (°)</th><th>Estimator RMS (cm)</th>
<th>Power idx</th><th>Sat. (%)</th></tr>
{''.join(rows)}
</table></div></section>
<section><h2>How realistic is this, and what is approximated</h2><ul>
<li>The controller closes its loop on <b>noisy GPS (5 cm) and IMU data fused
by a Kalman filter</b>, not on ground truth, unless noted.</li>
<li>Wind, gusts and waves are applied as <b>real forces and moments</b> on the
hull (including a wind yaw moment and wave heave/pitch/roll). These
experiments use the full wave coupling, which makes the hull bob visibly;
the live demo uses a gentler setting (<code>wave_heave_scale</code>,
<code>wave_moment_scale</code>) so the motion looks smooth. Station-keeping
error in the plane is barely affected by that choice.</li>
<li>Thrusters include an ESC deadband, first-order lag and weaker reverse
thrust, scaled to the simulated thrust range.</li>
<li>Hull drag, mass distribution, windage area and thrust coefficients are
simulation estimates, <b>not yet identified from the real vehicle</b>. Absolute
numbers will change once they are; the comparison between conditions is the
point of this study. A parameter-identification tool is included for that.</li>
<li>Uncontrolled runs end at the pool wall (±5 m × ±4 m).</li>
</ul></section>
</main></body></html>'''
    report_path = out_dir / 'report.html'
    report_path.write_text(page, encoding='utf-8')
    (out_dir / 'results.json').write_text(
        json.dumps(results, indent=2), encoding='utf-8')
    print(f'[report] {report_path}')
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    parser.add_argument('--duration', type=float, default=90.0,
                        help='wall-clock seconds per run (real-time sim)')
    parser.add_argument('--jobs', type=int, default=1,
                        help='parallel simulations')
    parser.add_argument('--only', nargs='*', help='run just these conditions')
    parser.add_argument('--montecarlo', type=int, default=0,
                        help='add N random sea states x {PID, observer}')
    parser.add_argument('--skip-standard', action='store_true')
    parser.add_argument('--report-only', action='store_true')
    args = parser.parse_args()
    out_dir = Path(args.out).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    if not args.report_only:
        todo = {}
        if not args.skip_standard:
            todo.update(CONDITIONS)
        if args.montecarlo:
            todo.update(monte_carlo_conditions(args.montecarlo))
        if args.only:
            todo = {k: v for k, v in todo.items() if k in args.only}
        global_cleanup()
        slots = queue.Queue()
        for slot in range(max(args.jobs, 1)):
            slots.put(slot)

        def job(name, spec):
            slot = slots.get()
            try:
                run_condition(name, spec, out_dir, args.duration, slot)
            finally:
                slots.put(slot)

        with ThreadPoolExecutor(max_workers=max(args.jobs, 1)) as pool:
            for future in [pool.submit(job, n, sp)
                           for n, sp in todo.items()]:
                future.result()
        global_cleanup()
    results = build_report(out_dir)
    for name, res in results.items():
        if res:
            print(f"{name:26s} rms={res['rms'] * 100:6.1f} cm "
                  f"p95={res['p95'] * 100:6.1f} cm "
                  f"max={res['max'] * 100:6.1f} cm")
    return 0


if __name__ == '__main__':
    sys.exit(main())
