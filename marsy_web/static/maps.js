const els = {
  status: document.getElementById('mapStatus'),
  refresh: document.getElementById('refreshMapsBtn'),
  search: document.getElementById('mapSearch'),
  finalOnly: document.getElementById('finalOnly'),
  followLatest: document.getElementById('followLatest'),
  list: document.getElementById('mapList'),
  title: document.getElementById('mapTitle'),
  subtitle: document.getElementById('mapSubtitle'),
  frame: document.getElementById('mapFrame'),
  canvas: document.getElementById('mapCanvas'),
  empty: document.getElementById('mapEmpty'),
  stats: document.getElementById('mapStats'),
  note: document.getElementById('mapNote'),
  openJson: document.getElementById('openJsonLink'),
  openSvg: document.getElementById('openSvgLink'),
  fit: document.getElementById('fitMapBtn'),
  zoomIn: document.getElementById('zoomInBtn'),
  zoomOut: document.getElementById('zoomOutBtn'),
  showGrid: document.getElementById('showGrid'),
  showFree: document.getElementById('showFree'),
  showOccupied: document.getElementById('showOccupied'),
  showVisits: document.getElementById('showVisits'),
  showPath: document.getElementById('showPath'),
};

const state = {
  maps: [],
  selectedName: null,
  selectedSummary: null,
  mapData: null,
  mapModifiedAt: null,
  view: { scale: 12, offsetX: 0, offsetY: 0 },
  dragging: false,
  dragPointerId: null,
  dragX: 0,
  dragY: 0,
  refreshBusy: false,
};

const ctx = els.canvas.getContext('2d');

function setStatus(text, kind = 'normal') {
  els.status.textContent = text;
  els.status.dataset.kind = kind;
}

async function getJSON(url) {
  const response = await fetch(url, { cache: 'no-store' });
  const data = await response.json().catch(() => ({}));
  if (!response.ok) {
    throw new Error(data.error || `${url} failed: ${response.status}`);
  }
  return data;
}

function formatTime(epochSeconds) {
  const value = Number(epochSeconds);
  if (!Number.isFinite(value)) return 'unknown time';
  return new Date(value * 1000).toLocaleString();
}

function formatNumber(value, digits = 1) {
  const number = Number(value);
  return Number.isFinite(number) ? number.toFixed(digits) : '—';
}

function humanBytes(value) {
  const bytes = Number(value);
  if (!Number.isFinite(bytes)) return '—';
  if (bytes < 1024) return `${bytes} B`;
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
  return `${(bytes / (1024 * 1024)).toFixed(1)} MB`;
}

function filteredMaps() {
  const query = els.search.value.trim().toLowerCase();
  return state.maps.filter((item) => {
    if (els.finalOnly.checked && !item.is_final) return false;
    if (!query) return true;
    return `${item.name} ${item.run_id}`.toLowerCase().includes(query);
  });
}

function mapStepLabel(item) {
  if (item.is_final) return 'complete';
  const step = item.metadata?.step;
  return step === null || step === undefined ? 'map' : `step ${step}`;
}

function renderMapList() {
  const items = filteredMaps();
  els.list.replaceChildren();

  if (!items.length) {
    const empty = document.createElement('div');
    empty.className = 'map-list-empty';
    empty.textContent = 'No current session map. Start Explore area from the dashboard.';
    els.list.appendChild(empty);
    return;
  }

  items.forEach((item) => {
    const button = document.createElement('button');
    button.type = 'button';
    button.className = 'map-card';
    if (item.name === state.selectedName) button.classList.add('selected');
    if (item.error) button.classList.add('error');

    const title = document.createElement('div');
    title.className = 'map-card-title';

    const name = document.createElement('span');
    name.textContent = item.run_id || item.name;
    title.appendChild(name);

    const badge = document.createElement('span');
    badge.className = `map-badge${item.is_final ? ' final' : ''}`;
    badge.textContent = mapStepLabel(item);
    title.appendChild(badge);

    const meta = document.createElement('div');
    meta.className = 'map-card-meta';
    meta.textContent = formatTime(item.modified_at);

    const counts = document.createElement('div');
    counts.className = 'map-card-counts';
    counts.textContent = item.error
      ? item.error
      : `${item.cells} cells · ${item.occupied_cells} occupied · ${item.path_points} path points`;

    button.append(title, meta, counts);
    button.addEventListener('click', () => {
      els.followLatest.checked = false;
      loadMap(item.name, true).catch(showError);
    });
    els.list.appendChild(button);
  });
}

function summaryByName(name) {
  return state.maps.find((item) => item.name === name) || null;
}

async function refreshMapList({ initial = false } = {}) {
  if (state.refreshBusy) return;
  state.refreshBusy = true;
  try {
    const data = await getJSON('/api/maps');
    state.maps = Array.isArray(data.maps) ? data.maps : [];
    setStatus(state.maps.length ? 'current map ready' : 'waiting for map');

    const requested = initial ? new URLSearchParams(window.location.search).get('map') : null;
    const newest = state.maps.find((item) => !item.error) || null;
    let nextName = state.selectedName;

    if (requested && summaryByName(requested)) {
      nextName = requested;
    } else if (!nextName && newest) {
      nextName = newest.name;
    } else if (els.followLatest.checked && newest) {
      nextName = newest.name;
    } else if (nextName && !summaryByName(nextName)) {
      nextName = newest?.name || null;
    }

    renderMapList();

    if (nextName) {
      const summary = summaryByName(nextName);
      const changed = nextName !== state.selectedName || summary?.modified_at !== state.mapModifiedAt;
      if (changed) await loadMap(nextName, false);
    } else {
      clearMap();
    }
  } finally {
    state.refreshBusy = false;
  }
}

async function loadMap(name, updateUrl) {
  const summary = summaryByName(name);
  if (!summary) throw new Error(`Map not found: ${name}`);
  if (summary.error) throw new Error(summary.error);

  setStatus('loading map');
  const data = await getJSON(`/api/maps/${encodeURIComponent(name)}`);
  state.selectedName = name;
  state.selectedSummary = summary;
  state.mapData = data;
  state.mapModifiedAt = summary.modified_at;

  if (updateUrl) {
    const url = new URL(window.location.href);
    url.searchParams.set('map', name);
    window.history.replaceState({}, '', url);
  }

  renderMapList();
  renderMapInfo();
  updateFileLinks();
  fitMap();
  els.empty.classList.add('hidden');
  setStatus(summary.is_final ? 'final map' : mapStepLabel(summary));
}

function clearMap() {
  state.selectedName = null;
  state.selectedSummary = null;
  state.mapData = null;
  state.mapModifiedAt = null;
  els.title.textContent = 'No map selected';
  els.subtitle.textContent = 'Run an exploration mission or choose a saved map.';
  els.empty.classList.remove('hidden');
  els.stats.innerHTML = '<div><dt>Run</dt><dd>—</dd></div>';
  updateFileLinks();
  drawMap();
}

function setLink(link, href) {
  if (href) {
    link.href = href;
    link.classList.remove('disabled');
  } else {
    link.href = '#';
    link.classList.add('disabled');
  }
}

function updateFileLinks() {
  setLink(els.openJson, state.selectedSummary?.json_url || null);
  setLink(els.openSvg, state.selectedSummary?.svg_url || null);
}

function pathLengthCm(points) {
  if (!Array.isArray(points) || points.length < 2) return 0;
  let total = 0;
  for (let index = 1; index < points.length; index += 1) {
    const a = points[index - 1];
    const b = points[index];
    total += Math.hypot(Number(b.x_cm) - Number(a.x_cm), Number(b.y_cm) - Number(a.y_cm));
  }
  return total;
}

function statRow(label, value) {
  const row = document.createElement('div');
  const dt = document.createElement('dt');
  const dd = document.createElement('dd');
  dt.textContent = label;
  dd.textContent = value;
  row.append(dt, dd);
  return row;
}

function renderMapInfo() {
  const data = state.mapData || {};
  const summary = state.selectedSummary || {};
  const cells = Array.isArray(data.cells) ? data.cells : [];
  const path = Array.isArray(data.path) ? data.path : [];
  const resolution = Number(data.resolution_cm) || 0;
  const knownAreaM2 = cells.length * resolution * resolution / 10000;
  const pose = data.current_pose;

  els.title.textContent = summary.run_id || summary.name || 'Map';
  els.subtitle.textContent = `${mapStepLabel(summary)} · ${formatTime(summary.modified_at)}`;
  els.stats.replaceChildren(
    statRow('Run', summary.run_id || '—'),
    statRow('Stage', mapStepLabel(summary)),
    statRow('Resolution', resolution ? `${formatNumber(resolution)} cm` : '—'),
    statRow('Known area', `${formatNumber(knownAreaM2, 2)} m²`),
    statRow('Free cells', String(summary.free_cells ?? 0)),
    statRow('Occupied', String(summary.occupied_cells ?? 0)),
    statRow('Visited', String(summary.visited_cells ?? 0)),
    statRow('Path length', `${formatNumber(pathLengthCm(path) / 100, 2)} m`),
    statRow(
      'Pose',
      pose
        ? `${formatNumber(pose.x_cm)} / ${formatNumber(pose.y_cm)} cm · ${formatNumber(pose.heading_deg, 0)}°`
        : '—',
    ),
    statRow('File', humanBytes(summary.size_bytes)),
  );

  const note = data.metadata?.note || data.mapping_mode || 'Occupancy map';
  els.note.textContent = note;
}

function mapCoordinates() {
  const data = state.mapData || {};
  const resolution = Number(data.resolution_cm) || 1;
  const points = [];

  (Array.isArray(data.cells) ? data.cells : []).forEach((cell) => {
    const x = Number(cell.x);
    const y = Number(cell.y);
    if (Number.isFinite(x) && Number.isFinite(y)) points.push({ x, y });
  });

  (Array.isArray(data.path) ? data.path : []).forEach((pose) => {
    const x = Number(pose.x_cm) / resolution;
    const y = Number(pose.y_cm) / resolution;
    if (Number.isFinite(x) && Number.isFinite(y)) points.push({ x, y });
  });

  if (data.current_pose) {
    const x = Number(data.current_pose.x_cm) / resolution;
    const y = Number(data.current_pose.y_cm) / resolution;
    if (Number.isFinite(x) && Number.isFinite(y)) points.push({ x, y });
  }

  return points;
}

function canvasSize() {
  const rect = els.canvas.getBoundingClientRect();
  return { width: Math.max(1, rect.width), height: Math.max(1, rect.height) };
}

function fitMap() {
  const points = mapCoordinates();
  const { width, height } = canvasSize();
  if (!points.length) {
    state.view.scale = 12;
    state.view.offsetX = width / 2;
    state.view.offsetY = height / 2;
    drawMap();
    return;
  }

  const xs = points.map((point) => point.x);
  const ys = points.map((point) => point.y);
  const minX = Math.min(...xs) - 3;
  const maxX = Math.max(...xs) + 3;
  const minY = Math.min(...ys) - 3;
  const maxY = Math.max(...ys) + 3;
  const rangeX = Math.max(1, maxX - minX);
  const rangeY = Math.max(1, maxY - minY);
  state.view.scale = Math.max(2, Math.min(50, Math.min(width / rangeX, height / rangeY)));
  const centerX = (minX + maxX) / 2;
  const centerY = (minY + maxY) / 2;
  state.view.offsetX = width / 2 - centerX * state.view.scale;
  state.view.offsetY = height / 2 + centerY * state.view.scale;
  drawMap();
}

function screenPoint(x, y) {
  return {
    x: state.view.offsetX + x * state.view.scale,
    y: state.view.offsetY - y * state.view.scale,
  };
}

function resizeCanvas() {
  const rect = els.canvas.getBoundingClientRect();
  const dpr = Math.max(1, window.devicePixelRatio || 1);
  const targetWidth = Math.max(1, Math.round(rect.width * dpr));
  const targetHeight = Math.max(1, Math.round(rect.height * dpr));
  if (els.canvas.width !== targetWidth || els.canvas.height !== targetHeight) {
    els.canvas.width = targetWidth;
    els.canvas.height = targetHeight;
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

function drawGrid(width, height) {
  if (!els.showGrid.checked || state.view.scale < 5) return;
  const scale = state.view.scale;
  const minX = Math.floor((0 - state.view.offsetX) / scale) - 1;
  const maxX = Math.ceil((width - state.view.offsetX) / scale) + 1;
  const minY = Math.floor((state.view.offsetY - height) / scale) - 1;
  const maxY = Math.ceil(state.view.offsetY / scale) + 1;
  const step = scale < 9 ? 5 : 1;

  ctx.lineWidth = 1;
  for (let x = Math.ceil(minX / step) * step; x <= maxX; x += step) {
    const point = screenPoint(x, 0);
    ctx.strokeStyle = x === 0 ? 'rgba(245,165,85,0.30)' : 'rgba(255,255,255,0.045)';
    ctx.beginPath();
    ctx.moveTo(Math.round(point.x) + 0.5, 0);
    ctx.lineTo(Math.round(point.x) + 0.5, height);
    ctx.stroke();
  }
  for (let y = Math.ceil(minY / step) * step; y <= maxY; y += step) {
    const point = screenPoint(0, y);
    ctx.strokeStyle = y === 0 ? 'rgba(245,165,85,0.30)' : 'rgba(255,255,255,0.045)';
    ctx.beginPath();
    ctx.moveTo(0, Math.round(point.y) + 0.5);
    ctx.lineTo(width, Math.round(point.y) + 0.5);
    ctx.stroke();
  }
}

function drawCells(data) {
  const cells = Array.isArray(data.cells) ? data.cells : [];
  const size = Math.max(1, state.view.scale - 0.5);

  cells.forEach((cell) => {
    const x = Number(cell.x);
    const y = Number(cell.y);
    if (!Number.isFinite(x) || !Number.isFinite(y)) return;
    const point = screenPoint(x, y);
    const left = point.x - state.view.scale / 2;
    const top = point.y - state.view.scale / 2;

    if (cell.state === 'free' && els.showFree.checked) {
      ctx.fillStyle = '#dcd6ca';
      ctx.fillRect(left, top, size, size);
    } else if (cell.state === 'occupied' && els.showOccupied.checked) {
      ctx.fillStyle = '#e06b3c';
      ctx.fillRect(left, top, size, size);
    }

    const visits = Number(cell.visits) || 0;
    if (visits > 0 && els.showVisits.checked) {
      const alpha = Math.min(0.75, 0.22 + Math.log2(visits + 1) * 0.12);
      ctx.fillStyle = `rgba(108,166,193,${alpha})`;
      const inset = Math.max(1, state.view.scale * 0.18);
      ctx.fillRect(left + inset, top + inset, Math.max(1, size - inset * 2), Math.max(1, size - inset * 2));
    }
  });
}

function drawPath(data) {
  if (!els.showPath.checked) return;
  const path = Array.isArray(data.path) ? data.path : [];
  const resolution = Number(data.resolution_cm) || 1;
  if (!path.length) return;

  ctx.strokeStyle = '#6ca6c1';
  ctx.lineWidth = Math.max(2, Math.min(4, state.view.scale * 0.16));
  ctx.lineJoin = 'round';
  ctx.lineCap = 'round';
  ctx.beginPath();
  path.forEach((pose, index) => {
    const point = screenPoint(Number(pose.x_cm) / resolution, Number(pose.y_cm) / resolution);
    if (index === 0) ctx.moveTo(point.x, point.y);
    else ctx.lineTo(point.x, point.y);
  });
  ctx.stroke();

  const home = path[0];
  const homePoint = screenPoint(Number(home.x_cm) / resolution, Number(home.y_cm) / resolution);
  ctx.fillStyle = '#f5a555';
  ctx.beginPath();
  ctx.arc(homePoint.x, homePoint.y, Math.max(3, state.view.scale * 0.26), 0, Math.PI * 2);
  ctx.fill();
}

function drawRover(data) {
  const pose = data.current_pose;
  if (!pose) return;
  const resolution = Number(data.resolution_cm) || 1;
  const point = screenPoint(Number(pose.x_cm) / resolution, Number(pose.y_cm) / resolution);
  const heading = Number(pose.heading_deg) * Math.PI / 180;
  const radius = Math.max(5, Math.min(11, state.view.scale * 0.42));
  const dx = Math.sin(heading);
  const dy = -Math.cos(heading);

  ctx.fillStyle = '#6fd08c';
  ctx.strokeStyle = '#163b24';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(point.x, point.y, radius, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();

  ctx.strokeStyle = '#eaffef';
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.moveTo(point.x, point.y);
  ctx.lineTo(point.x + dx * radius * 1.7, point.y + dy * radius * 1.7);
  ctx.stroke();
}

function drawMap() {
  resizeCanvas();
  const { width, height } = canvasSize();
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = '#0c0f10';
  ctx.fillRect(0, 0, width, height);
  drawGrid(width, height);

  if (!state.mapData) return;
  drawCells(state.mapData);
  drawPath(state.mapData);
  drawRover(state.mapData);
}

function zoomAt(factor, clientX, clientY) {
  const rect = els.canvas.getBoundingClientRect();
  const x = clientX - rect.left;
  const y = clientY - rect.top;
  const worldX = (x - state.view.offsetX) / state.view.scale;
  const worldY = (state.view.offsetY - y) / state.view.scale;
  const nextScale = Math.max(1, Math.min(100, state.view.scale * factor));
  state.view.scale = nextScale;
  state.view.offsetX = x - worldX * nextScale;
  state.view.offsetY = y + worldY * nextScale;
  drawMap();
}

function showError(error) {
  console.error(error);
  setStatus(error.message || String(error), 'error');
}

els.refresh.addEventListener('click', () => refreshMapList().catch(showError));
els.search.addEventListener('input', renderMapList);
els.finalOnly.addEventListener('change', renderMapList);
els.fit.addEventListener('click', fitMap);
els.zoomIn.addEventListener('click', () => {
  const rect = els.canvas.getBoundingClientRect();
  zoomAt(1.25, rect.left + rect.width / 2, rect.top + rect.height / 2);
});
els.zoomOut.addEventListener('click', () => {
  const rect = els.canvas.getBoundingClientRect();
  zoomAt(0.8, rect.left + rect.width / 2, rect.top + rect.height / 2);
});

[els.showGrid, els.showFree, els.showOccupied, els.showVisits, els.showPath].forEach((input) => {
  input.addEventListener('change', drawMap);
});

els.frame.addEventListener('wheel', (event) => {
  event.preventDefault();
  zoomAt(event.deltaY < 0 ? 1.12 : 0.89, event.clientX, event.clientY);
}, { passive: false });

els.frame.addEventListener('pointerdown', (event) => {
  state.dragging = true;
  state.dragPointerId = event.pointerId;
  state.dragX = event.clientX;
  state.dragY = event.clientY;
  els.frame.classList.add('dragging');
  els.frame.setPointerCapture(event.pointerId);
});

els.frame.addEventListener('pointermove', (event) => {
  if (!state.dragging || event.pointerId !== state.dragPointerId) return;
  state.view.offsetX += event.clientX - state.dragX;
  state.view.offsetY += event.clientY - state.dragY;
  state.dragX = event.clientX;
  state.dragY = event.clientY;
  drawMap();
});

function endDrag(event) {
  if (!state.dragging || event.pointerId !== state.dragPointerId) return;
  state.dragging = false;
  state.dragPointerId = null;
  els.frame.classList.remove('dragging');
  try {
    els.frame.releasePointerCapture(event.pointerId);
  } catch (_error) {
    // Pointer capture may already be released by the browser.
  }
}

els.frame.addEventListener('pointerup', endDrag);
els.frame.addEventListener('pointercancel', endDrag);
els.frame.addEventListener('dblclick', fitMap);

const resizeObserver = new ResizeObserver(() => drawMap());
resizeObserver.observe(els.frame);

refreshMapList({ initial: true }).catch(showError);
setInterval(() => refreshMapList().catch(showError), 3500);
