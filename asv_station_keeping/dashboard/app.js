/* global document, window, fetch */

const parameterDefinitions = [
  ['target_x', 'Target x', 'number', 0.01],
  ['target_y', 'Target y', 'number', 0.01],
  ['target_yaw', 'Target yaw', 'number', 0.01],
  ['target_slew_rate', 'Target slew', 'number', 0.01],
  ['target_yaw_slew_rate', 'Yaw target slew', 'number', 0.01],
  ['current_x', 'Current x', 'number', 0.01],
  ['current_y', 'Current y', 'number', 0.01],
  ['current_z', 'Current z', 'number', 0.01],
  ['kp_x', 'Kp surge', 'number', 0.01],
  ['ki_x', 'Ki surge', 'number', 0.01],
  ['kd_x', 'Kd surge', 'number', 0.01],
  ['kp_y', 'Kp sway', 'number', 0.01],
  ['ki_y', 'Ki sway', 'number', 0.01],
  ['kd_y', 'Kd sway', 'number', 0.01],
  ['kp_yaw', 'Kp yaw', 'number', 0.01],
  ['ki_yaw', 'Ki yaw', 'number', 0.01],
  ['kd_yaw', 'Kd yaw', 'number', 0.01],
  ['max_thrust', 'Max thrust', 'number', 0.01],
  ['slew_rate', 'Slew rate', 'number', 0.01],
  ['integral_limit', 'Integral limit', 'number', 0.01],
  ['integral_leak', 'Integral leak', 'number', 0.01],
  ['velocity_filter_tau', 'Velocity filter', 'number', 0.01],
  ['position_tolerance', 'Position tolerance', 'number', 0.001],
  ['yaw_tolerance', 'Yaw tolerance', 'number', 0.001],
  ['enable_yaw_control', 'Yaw control', 'boolean', null],
  ['enable_tilt_guard', 'Tilt guard', 'boolean', null],
];

const $ = (id) => document.getElementById(id);
const fmt = (value, digits = 3) => {
  const number = Number(value);
  return Number.isFinite(number) ? number.toFixed(digits) : '--';
};
const fmtAge = (value) => value == null ? '--' : `${fmt(value, 1)} s`;
const deg = (value) => `${fmt(Number(value) * 180 / Math.PI, 1)}°`;

function setText(id, value) { const element = $(id); if (element) element.textContent = value; }

function applyStatus(status) {
  const card = $('statusCard');
  const level = String(status.label || 'ERROR').toLowerCase();
  card.className = `card status-card status-${level}`;
  $('statusLevel').className = `status-pill status-${level}`;
  setText('statusLevel', status.label || 'ERROR');
  setText('statusMessage', status.message || 'No controller message');
  setText('controllerAge', fmtAge(status.controller_age_s));
  setText('odomAge', fmtAge(status.odom_age_s));
  setText('currentAge', fmtAge(status.current_age_s));
  const badge = $('connectionBadge');
  badge.className = `badge badge-${level}`;
  badge.textContent = level === 'ok' ? 'LIVE' : level.toUpperCase();
}

function updateBars(thrusters, parameters) {
  const max = Math.max(Number(parameters.max_thrust || 6), 0.001);
  const entries = [
    ['port_front', 'PF', 'barPF', 'thrustPF'],
    ['starboard_front', 'SF', 'barSF', 'thrustSF'],
    ['port_rear', 'PR', 'barPR', 'thrustPR'],
    ['starboard_rear', 'SR', 'barSR', 'thrustSR'],
  ];
  entries.forEach(([name, _short, bar, label]) => {
    const value = Number(thrusters[name] || 0);
    setText(label, `${value >= 0 ? '+' : ''}${fmt(value, 2)} N`);
    const percent = Math.min(Math.abs(value) / max, 1);
    const element = $(bar);
    element.style.width = `${percent * 50}%`;
    element.style.marginLeft = value >= 0 ? '50%' : `${50 - percent * 50}%`;
    element.style.background = value >= 0
      ? 'linear-gradient(90deg, #2f83ff, #43d9ff)'
      : 'linear-gradient(90deg, #ff6d74, #ffca55)';
  });
}

function updateValues(state) {
  const target = state.target;
  const pose = state.pose;
  const error = state.error;
  const velocity = state.velocity;
  const current = state.current;
  const safety = state.safety;
  const parameters = state.parameters || {};
  setText('positionError', `${fmt(error.position_m, 3)} m`);
  setText('positionRmse', `${fmt(error.position_rmse_m, 3)} m`);
  setText('positionMax', `${fmt(error.max_position_m, 3)} m`);
  setText('headingError', `${fmt(error.yaw, 3)} rad`);
  setText('targetYawMetric', `${fmt(target.yaw, 3)} rad`);
  setText('actualYawMetric', `${fmt(pose.yaw, 3)} rad`);
  setText('rtf', fmt(state.simulation.real_time_factor, 2));
  setText('simTime', `${fmt(state.simulation.sim_time_s, 2)} s`);
  setText('targetX', fmt(target.x)); setText('targetY', fmt(target.y));
  setText('targetYaw', fmt(target.yaw));
  setText('poseX', fmt(pose.x)); setText('poseY', fmt(pose.y)); setText('poseZ', fmt(pose.z));
  setText('poseRoll', fmt(pose.roll)); setText('posePitch', fmt(pose.pitch)); setText('poseYaw', fmt(pose.yaw));
  setText('errorXBody', fmt(error.x_body)); setText('errorYBody', fmt(error.y_body)); setText('errorYaw', fmt(error.yaw));
  setText('velocityU', fmt(velocity.u)); setText('velocityV', fmt(velocity.v)); setText('velocityR', fmt(velocity.r));
  setText('controlSurge', fmt(state.control.surge_N, 2));
  setText('controlSway', fmt(state.control.sway_N, 2));
  setText('controlYaw', fmt(state.control.yaw_Nm, 2));
  setText('integralState', `${fmt(state.control.integral_x, 3)} / ${fmt(state.control.integral_y, 3)} / ${fmt(state.control.integral_yaw, 3)}`);
  setText('currentX', fmt(current.x)); setText('currentY', fmt(current.y));
  setText('currentZ', fmt(current.z)); setText('currentMagnitude', fmt(current.magnitude));
  setText('tiltScale', fmt(safety.tilt_scale, 2));
  setText('saturationState', safety.thrust_saturated ? 'ACTIVE' : 'clear');
  setText('tiltGuardState', safety.tilt_guard_active ? 'ACTIVE' : 'clear');
  setText('attitudeState', `${deg(pose.roll)} / ${deg(pose.pitch)}`);
  const safetyTag = $('thrustSafety');
  safetyTag.textContent = safety.thrust_saturated ? 'SATURATED' : 'NOMINAL';
  safetyTag.style.color = safety.thrust_saturated ? 'var(--red)' : 'var(--green)';
  updateBars(state.thrusters, parameters);
  updateParameterInputs(parameters);
}

function resizeCanvas(canvas) {
  const ratio = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.max(1, Math.floor(rect.width * ratio));
  canvas.height = Math.max(1, Math.floor(rect.height * ratio));
  const context = canvas.getContext('2d');
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  return { context, width: rect.width, height: rect.height };
}

function drawChart(id, history, series, options = {}) {
  const canvas = $(id);
  if (!canvas) return;
  const { context, width, height } = resizeCanvas(canvas);
  context.clearRect(0, 0, width, height);
  const left = 34; const right = 8; const top = 8; const bottom = 20;
  const plotWidth = Math.max(10, width - left - right);
  const plotHeight = Math.max(10, height - top - bottom);
  context.strokeStyle = 'rgba(136, 174, 205, .15)'; context.lineWidth = 1;
  context.fillStyle = '#7d99b0'; context.font = '10px system-ui';
  for (let i = 0; i <= 4; i += 1) {
    const y = top + plotHeight * i / 4;
    context.beginPath(); context.moveTo(left, y); context.lineTo(left + plotWidth, y); context.stroke();
    if (options.fixedRange == null) context.fillText('', 2, y);
  }
  if (!history.length) return;
  const values = [];
  series.forEach((item) => history.forEach((row) => values.push(Number(row[item.key]) || 0)));
  let min = options.min != null ? options.min : Math.min(...values);
  let max = options.max != null ? options.max : Math.max(...values);
  if (options.symmetric) { const extent = Math.max(Math.abs(min), Math.abs(max), 0.01); min = -extent; max = extent; }
  if (Math.abs(max - min) < 1e-9) { min -= 1; max += 1; }
  const firstTime = Number(history[0].t || 0); const lastTime = Number(history[history.length - 1].t || firstTime + 1);
  const timeSpan = Math.max(lastTime - firstTime, 1e-6);
  const xAt = (time) => left + (Number(time) - firstTime) / timeSpan * plotWidth;
  const yAt = (value) => top + (max - Number(value)) / (max - min) * plotHeight;
  context.fillStyle = '#7d99b0';
  context.fillText(fmt(max, options.digits || 2), 3, top + 4);
  context.fillText(fmt(min, options.digits || 2), 3, top + plotHeight);
  series.forEach((item) => {
    context.strokeStyle = item.color; context.lineWidth = 1.8; context.beginPath();
    history.forEach((row, index) => {
      const x = xAt(row.t); const y = yAt(row[item.key]);
      if (index === 0) context.moveTo(x, y); else context.lineTo(x, y);
    });
    context.stroke();
  });
  if (min < 0 && max > 0) {
    context.strokeStyle = 'rgba(230, 240, 250, .28)'; context.setLineDash([3, 3]);
    const zero = yAt(0); context.beginPath(); context.moveTo(left, zero); context.lineTo(left + plotWidth, zero); context.stroke(); context.setLineDash([]);
  }
  context.fillStyle = '#7d99b0'; context.fillText(`${fmt(firstTime, 1)}s`, left, height - 4);
  context.fillText(`${fmt(lastTime, 1)}s`, Math.max(left, width - 34), height - 4);
}

function drawPlots(state) {
  const history = state.history || [];
  drawChart('errorChart', history, [
    { key: 'position_error', color: '#43d9ff' },
    { key: 'position_rmse', color: '#ffca55' },
  ], { min: 0, digits: 2 });
  drawChart('headingChart', history, [
    { key: 'yaw', color: '#43d9ff' },
    { key: 'target_yaw', color: '#ffca55' },
  ], { symmetric: true, digits: 2 });
  drawChart('velocityChart', history, [
    { key: 'u', color: '#b38cff' }, { key: 'v', color: '#43d9ff' }, { key: 'r', color: '#ffca55' },
  ], { symmetric: true, digits: 2 });
  drawChart('currentChart', history, [
    { key: 'current_x', color: '#5596ff' }, { key: 'current_y', color: '#43d9ff' },
  ], { symmetric: true, digits: 2 });
  drawChart('thrustChart', history, [
    { key: 'thrust_pf', color: '#43d9ff' }, { key: 'thrust_sf', color: '#5596ff' },
    { key: 'thrust_pr', color: '#ffca55' }, { key: 'thrust_sr', color: '#ff6d74' },
  ], { symmetric: true, digits: 1 });
}

function createParameterForm() {
  const form = $('parameterForm');
  parameterDefinitions.forEach(([key, label, type, step]) => {
    const wrapper = document.createElement('div'); wrapper.className = 'parameter';
    const labelElement = document.createElement('label'); labelElement.textContent = label; labelElement.htmlFor = `param-${key}`;
    const input = document.createElement('input'); input.id = `param-${key}`; input.dataset.parameter = key; input.type = type === 'boolean' ? 'checkbox' : 'number';
    if (type === 'number') { input.step = step; input.inputMode = 'decimal'; }
    const button = document.createElement('button'); button.textContent = type === 'boolean' ? 'Apply' : 'Set';
    button.addEventListener('click', () => submitParameter(key, input, button));
    wrapper.append(labelElement, input, button); form.appendChild(wrapper);
  });
}

function updateParameterInputs(parameters) {
  parameterDefinitions.forEach(([key, _label, type]) => {
    const input = $(`param-${key}`); if (!input || document.activeElement === input) return;
    if (type === 'boolean') input.checked = Boolean(parameters[key]);
    else if (parameters[key] != null) input.value = Number(parameters[key]);
  });
}

async function submitParameter(key, input, button) {
  const original = button.textContent; button.textContent = '...'; button.disabled = true;
  const value = input.type === 'checkbox' ? input.checked : Number(input.value);
  const notice = $('parameterNotice');
  try {
    const response = await fetch('/api/parameters', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ parameters: { [key]: value } }) });
    const result = await response.json(); notice.textContent = result.message || (result.ok ? 'Updated' : 'Update rejected'); notice.className = `form-notice ${result.ok ? 'success' : 'error'}`;
  } catch (error) { notice.textContent = error.message; notice.className = 'form-notice error'; }
  button.textContent = original; button.disabled = false;
}

async function poll() {
  try {
    const response = await fetch('/api/state', { cache: 'no-store' });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const state = await response.json(); applyStatus(state.status); updateValues(state); drawPlots(state);
  } catch (error) {
    applyStatus({ label: 'STALE', message: `Dashboard data unavailable: ${error.message}` });
  }
}

async function resetIntegrators() {
  const notice = $('parameterNotice');
  try {
    const response = await fetch('/api/actions/reset_integrators', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' });
    const result = await response.json(); notice.textContent = result.message; notice.className = `form-notice ${result.ok ? 'success' : 'error'}`;
  } catch (error) { notice.textContent = error.message; notice.className = 'form-notice error'; }
}

async function startDemo() {
  const notice = $('parameterNotice');
  try {
    const response = await fetch('/api/actions/start_demo', {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}',
    });
    const result = await response.json();
    notice.textContent = result.message;
    notice.className = `form-notice ${result.ok ? 'success' : 'error'}`;
  } catch (error) {
    notice.textContent = error.message;
    notice.className = 'form-notice error';
  }
}

function exportSnapshot() {
  fetch('/api/state', { cache: 'no-store' }).then((response) => response.json()).then((state) => {
    const blob = new Blob([JSON.stringify(state, null, 2)], { type: 'application/json' });
    const link = document.createElement('a'); link.href = URL.createObjectURL(blob); link.download = `station_keeping_snapshot_${Date.now()}.json`; link.click(); URL.revokeObjectURL(link.href);
  });
}

createParameterForm();
$('demoButton').addEventListener('click', startDemo);
$('resetButton').addEventListener('click', resetIntegrators);
$('snapshotButton').addEventListener('click', exportSnapshot);
window.addEventListener('resize', () => { poll(); });
poll();
setInterval(poll, 700);
