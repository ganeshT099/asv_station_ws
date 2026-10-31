"""Evaluate a station-keeping CSV produced by station_keeping_pid."""

import argparse
import csv
from datetime import datetime, timezone
import html
import json
import math
from pathlib import Path


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def load_rows(path):
    """Load a run CSV while keeping the evaluator independent of ROS."""
    with Path(path).open(newline='', encoding='utf-8') as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError('The run file contains no samples.')
    return rows


def evaluate(path, rows=None):
    """Return basic position, heading and settling metrics."""
    rows = load_rows(path) if rows is None else rows

    position_errors = []
    heading_errors = []
    roll_values = []
    pitch_values = []
    z_values = []
    thrust_values = []
    speed_values = []
    yaw_rate_values = []
    control_effort = 0.0
    saturated_samples = 0
    integral_values = []
    times = []
    for row in rows:
        times.append(float(row['time']))
        # Score against simulator ground truth when the run recorded it, so a
        # noisy estimate cannot flatter the result.
        px = float(row.get('true_x', row['x']))
        py = float(row.get('true_y', row['y']))
        pyaw = float(row.get('true_yaw', row['yaw']))
        dx = float(row['target_x']) - px
        dy = float(row['target_y']) - py
        position_errors.append(math.hypot(dx, dy))
        heading_errors.append(
            abs(wrap_angle(float(row['target_yaw']) - pyaw))
        )
        if 'roll' in row:
            roll_values.append(abs(float(row['roll'])))
        if 'pitch' in row:
            pitch_values.append(abs(float(row['pitch'])))
        if 'z' in row:
            z_values.append(float(row['z']))
        if all(key in row for key in ('u', 'v')):
            speed_values.append(math.hypot(float(row['u']), float(row['v'])))
        if 'r' in row:
            yaw_rate_values.append(abs(float(row['r'])))
        for key in ('integral_x', 'integral_y', 'integral_yaw'):
            if key in row:
                integral_values.append(abs(float(row[key])))
        thrust_keys = (
            'u_port_front', 'u_starboard_front',
            'u_port_rear', 'u_starboard_rear',
        )
        if all(key in row for key in thrust_keys):
            commands = [abs(float(row[key])) for key in thrust_keys]
            thrust_values.extend(commands)
            if len(times) > 1:
                dt = max(times[-1] - times[-2], 0.0)
                control_effort += sum(commands) * dt
        if row.get('thrust_saturated', '').strip().lower() == 'true':
            saturated_samples += 1

    final_window = position_errors[max(0, len(position_errors) - 20):]
    final_error = sum(final_window) / len(final_window)
    settling_time = None
    threshold = max(0.05, final_error * 1.5)
    for index, error in enumerate(position_errors):
        if all(value <= threshold for value in position_errors[index:]):
            settling_time = float(rows[index]['time']) - float(rows[0]['time'])
            break

    metrics = {
        'samples': len(rows),
        'duration_s': float(rows[-1]['time']) - float(rows[0]['time']),
        'mean_position_error_m': sum(position_errors) / len(position_errors),
        'position_rmse_m': math.sqrt(
            sum(error * error for error in position_errors)
            / len(position_errors)),
        'position_max_error_m': max(position_errors),
        'heading_rmse_rad': math.sqrt(
            sum(error * error for error in heading_errors)
            / len(heading_errors)),
        'heading_max_error_rad': max(heading_errors),
        'final_heading_error_rad': heading_errors[-1],
        'final_position_error_m': final_error,
        'settling_time_s': settling_time,
        'control_effort_Ns': control_effort,
        'saturation_fraction': saturated_samples / len(rows),
    }
    if roll_values:
        metrics['max_abs_roll_rad'] = max(roll_values)
    if pitch_values:
        metrics['max_abs_pitch_rad'] = max(pitch_values)
    if z_values:
        metrics['z_excursion_m'] = max(z_values) - min(z_values)
    if thrust_values:
        metrics['max_abs_thruster_command_N'] = max(thrust_values)
    if speed_values:
        metrics['max_horizontal_speed_mps'] = max(speed_values)
        metrics['rms_horizontal_speed_mps'] = math.sqrt(
            sum(value * value for value in speed_values)
            / len(speed_values))
    if yaw_rate_values:
        metrics['max_abs_yaw_rate_radps'] = max(yaw_rate_values)
    if integral_values:
        metrics['max_abs_integral_state'] = max(integral_values)
    return metrics


def _svg_chart(rows, series, title, width=860, height=240):
    """Return a dependency-free SVG line chart for the HTML report."""
    left, right, top, bottom = 54, 18, 28, 34
    plot_width = width - left - right
    plot_height = height - top - bottom
    values = []
    for key, _color, _label in series:
        values.extend(float(row.get(key, 0.0) or 0.0) for row in rows)
    if not values:
        values = [0.0]
    minimum = min(values)
    maximum = max(values)
    if maximum - minimum < 1e-9:
        minimum -= 1.0
        maximum += 1.0
    first_time = float(rows[0].get('time', 0.0))
    last_time = float(rows[-1].get('time', first_time + 1.0))
    time_span = max(last_time - first_time, 1e-9)

    def x_at(row):
        return left + (
            float(row.get('time', first_time)) - first_time
        ) / time_span * plot_width

    def y_at(value):
        return top + (maximum - value) / (maximum - minimum) * plot_height

    parts = [
        f'<svg viewBox="0 0 {width} {height}" role="img" '
        f'aria-label="{html.escape(title)}">',
        f'<rect width="{width}" height="{height}" fill="#0a1929"/>',
        f'<text x="{left}" y="18" fill="#b9d0e4" font-size="12">'
        f'{html.escape(title)}</text>',
    ]
    for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
        y = top + fraction * plot_height
        value = maximum - fraction * (maximum - minimum)
        parts.append(
            f'<line x1="{left}" y1="{y:.2f}" '
            f'x2="{left + plot_width}" y2="{y:.2f}" '
            'stroke="#24405a" stroke-width="1"/>')
        parts.append(
            f'<text x="4" y="{y + 4:.2f}" fill="#7691aa" '
            f'font-size="10">{value:.3g}</text>')
    for key, color, label in series:
        points = ' '.join(
            f'{x_at(row):.2f},{y_at(float(row.get(key, 0.0) or 0.0)):.2f}'
            for row in rows)
        parts.append(
            f'<polyline points="{points}" fill="none" '
            f'stroke="{color}" stroke-width="2"/>')
        parts.append(
            f'<text x="{left + 8 + 90 * series.index((key, color, label))}" '
            f'y="{height - 10}" fill="{color}" font-size="10">'
            f'{html.escape(label)}</text>')
    parts.append('</svg>')
    return ''.join(parts)


def write_json_summary(output_path, input_path, metrics):
    """Write a machine-readable report with provenance metadata."""
    payload = {
        'input_file': str(Path(input_path).resolve()),
        'generated_utc': datetime.now(timezone.utc).isoformat(),
        'metrics': metrics,
    }
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(payload, indent=2, sort_keys=True),
        encoding='utf-8',
    )


def write_html_report(output_path, input_path, rows, metrics):
    """Write a self-contained HTML report suitable for a paper appendix."""
    metric_rows = ''.join(
        '<tr><th>{}</th><td>{}</td></tr>'.format(
            html.escape(key.replace('_', ' ')),
            html.escape(str(value)),
        )
        for key, value in metrics.items()
    )
    position_chart = _svg_chart(
        rows,
        [('position_error', '#43d9ff', 'position error'),
         ('position_rmse', '#ffca55', 'running RMSE')],
        'Position error response',
    )
    heading_chart = _svg_chart(
        rows,
        [('yaw', '#43d9ff', 'actual yaw'),
         ('target_yaw', '#ffca55', 'target yaw')],
        'Heading response',
    )
    thrust_chart = _svg_chart(
        rows,
        [('u_port_front', '#43d9ff', 'PF'),
         ('u_starboard_front', '#5596ff', 'SF'),
         ('u_port_rear', '#ffca55', 'PR'),
         ('u_starboard_rear', '#ff6d74', 'SR')],
        'Thruster commands',
    )
    document = f'''<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<title>Dynamic station-keeping report</title>
<style>
body{{background:#07111f;color:#e8f1fb;font:14px system-ui;margin:0;padding:32px}}
main{{max-width:1100px;margin:auto}} h1{{font-size:30px}}
 .meta,.card{{background:#0d1b2d;border:1px solid #203b58;border-radius:9px;
 padding:18px;margin:14px 0}}
.grid{{display:grid;grid-template-columns:1fr 1fr;gap:14px}}
svg{{display:block;width:100%;height:auto}} table{{border-collapse:collapse;width:100%}}
th,td{{padding:7px;border-bottom:1px solid #203b58;text-align:left}}
th{{color:#8da4bb;text-transform:capitalize}} td{{font-variant-numeric:tabular-nums}}
small{{color:#8da4bb}} @media(max-width:800px){{.grid{{grid-template-columns:1fr}}}}
</style></head><body><main>
<h1>Dynamic station-keeping experiment report</h1>
<div class="meta"><strong>Input:</strong> {html.escape(str(Path(input_path).resolve()))}<br>
<strong>Samples:</strong> {len(rows)}<br><small>Generated UTC:
{datetime.now(timezone.utc).isoformat()}</small></div>
<div class="grid"><div class="card">{position_chart}</div>
<div class="card">{heading_chart}</div></div>
<div class="card">{thrust_chart}</div>
<div class="card"><h2>Metrics</h2><table>{metric_rows}</table></div>
</main></body></html>'''
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(document, encoding='utf-8')


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('run_file')
    parser.add_argument('--json-output')
    parser.add_argument('--html-output')
    options = parser.parse_args(args)
    rows = load_rows(options.run_file)
    metrics = evaluate(options.run_file, rows)
    for key, value in metrics.items():
        print(f'{key}: {value}')
    if options.json_output:
        write_json_summary(options.json_output, options.run_file, metrics)
        print(f'json_report: {options.json_output}')
    if options.html_output:
        write_html_report(options.html_output, options.run_file, rows, metrics)
        print(f'html_report: {options.html_output}')


if __name__ == '__main__':
    main()
