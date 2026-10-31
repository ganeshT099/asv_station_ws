const $ = (id) => document.getElementById(id);
const state = { latest: null };

function number(value, digits = 3) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed.toFixed(digits) : '--';
}

async function api(path, options = {}) {
  const response = await fetch(path, { cache: 'no-store', ...options });
  if (!response.ok) throw new Error(`${response.status} ${response.statusText}`);
  return response.json();
}

function updateHealth(data) {
  const health = data.health || {};
  const element = $('health');
  element.textContent = health.level || 'STALE';
  element.className = `health ${(health.level || 'STALE').toLowerCase()}`;
  $('health-detail').textContent = JSON.stringify({
    health,
    path_status: data.path,
    controller_status: data.controller,
  }, null, 2);
}

function updateMetrics(data) {
  const path = data.path || {};
  const errors = data.errors || {};
  $('metric-path').textContent = path.path_type || '--';
  $('metric-progress').textContent = `${number(Number(path.progress || 0) * 100, 1)}%`;
  $('metric-cross').textContent = `${number(errors.cross_track_m)} m`;
  $('metric-along').textContent = `${number(errors.along_track_m)} m`;
  $('metric-heading').textContent = `${number(errors.heading_rad)} rad`;
  const current = data.current || {};
  $('metric-current').textContent = `(${number(current.x)}, ${number(current.y)}) m/s`;
  $('path-message').textContent = `${path.path_type || '--'} · ${path.running === 'True' ? 'running' : 'paused'}`;
}

function updateTelemetry(data) {
  const pose = data.pose || {};
  const reference = data.reference || {};
  $('telemetry').textContent = JSON.stringify({
    reference,
    pose,
    errors: data.errors,
    controller: data.controller,
  }, null, 2);
}

function drawPlot(data) {
  const canvas = $('plot');
  const context = canvas.getContext('2d');
  const width = canvas.width;
  const height = canvas.height;
  context.clearRect(0, 0, width, height);
  context.fillStyle = '#0d1724';
  context.fillRect(0, 0, width, height);
  const history = data.history || [];
  const points = [];
  history.forEach((item) => points.push([Number(item.x), Number(item.y)]));
  const referencePath = (data.reference_path || []).map((item) => [
    Number(item.x), Number(item.y),
  ]);
  referencePath.forEach((point) => points.push(point));
  const reference = data.reference || {};
  points.push([Number(reference.x || 0), Number(reference.y || 0)]);
  if (points.length === 0) return;
  const xs = points.map((point) => point[0]);
  const ys = points.map((point) => point[1]);
  const minX = Math.min(...xs, -1.0);
  const maxX = Math.max(...xs, 1.0);
  const minY = Math.min(...ys, -1.0);
  const maxY = Math.max(...ys, 1.0);
  const margin = 42;
  const scale = Math.min(
    (width - 2 * margin) / Math.max(maxX - minX, 0.1),
    (height - 2 * margin) / Math.max(maxY - minY, 0.1),
  );
  const map = (x, y) => [
    margin + (x - minX) * scale,
    height - margin - (y - minY) * scale,
  ];
  context.strokeStyle = '#30465c';
  context.lineWidth = 1;
  context.beginPath();
  const originA = map(0, minY);
  const originB = map(0, maxY);
  context.moveTo(originA[0], originA[1]);
  context.lineTo(originB[0], originB[1]);
  const originC = map(minX, 0);
  const originD = map(maxX, 0);
  context.moveTo(originC[0], originC[1]);
  context.lineTo(originD[0], originD[1]);
  context.stroke();
  if (history.length > 1) {
    context.strokeStyle = '#28e7e7';
    context.lineWidth = 3;
    context.beginPath();
    history.forEach((item, index) => {
      const point = map(Number(item.x), Number(item.y));
      if (index === 0) context.moveTo(point[0], point[1]);
      else context.lineTo(point[0], point[1]);
    });
    context.stroke();
  }
  if (referencePath.length > 1) {
    context.strokeStyle = '#ffbe14';
    context.lineWidth = 2;
    context.beginPath();
    referencePath.forEach((item, index) => {
      const point = map(item[0], item[1]);
      if (index === 0) context.moveTo(point[0], point[1]);
      else context.lineTo(point[0], point[1]);
    });
    context.stroke();
  }
  const ref = map(Number(reference.x || 0), Number(reference.y || 0));
  context.fillStyle = '#ffbe14';
  context.beginPath();
  context.arc(ref[0], ref[1], 7, 0, Math.PI * 2);
  context.fill();
  if (history.length) {
    const last = history[history.length - 1];
    const actual = map(Number(last.x), Number(last.y));
    context.fillStyle = '#28e7e7';
    context.beginPath();
    context.arc(actual[0], actual[1], 6, 0, Math.PI * 2);
    context.fill();
  }
}

async function refresh() {
  try {
    const data = await api('/api/state');
    state.latest = data;
    updateHealth(data);
    updateMetrics(data);
    updateTelemetry(data);
    drawPlot(data);
  } catch (error) {
    $('health').textContent = 'OFFLINE';
    $('health').className = 'health stale';
    $('health-detail').textContent = error.toString();
  }
}

async function post(path, body = {}) {
  await api(path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  await refresh();
}

$('start').addEventListener('click', () => post('/api/run', { running: true }));
$('pause').addEventListener('click', () => post('/api/run', { running: false }));
$('reset').addEventListener('click', () => post('/api/reset'));
$('apply').addEventListener('click', () => post('/api/parameters', {
  path_type: $('path_type').value,
  speed: Number($('speed').value),
  path_width: Number($('path_width').value),
  path_height: Number($('path_height').value),
}));

refresh();
setInterval(refresh, 500);
