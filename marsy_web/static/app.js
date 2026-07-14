const stateEls = {
  lastError: document.getElementById('lastError'),
  cameraErrorHud: document.getElementById('cameraErrorHud'),
  cameraErrorMessage: document.getElementById('cameraErrorMessage'),
  cameraStatus: document.getElementById('cameraStatus'),
  logs: document.getElementById('logs'),
  rangeHud: document.getElementById('rangeHud'),
};

function setText(el, value) {
  if (el) el.textContent = value;
}

const controls = {
  speed: document.getElementById('speed'),
  steerAngle: document.getElementById('steerAngle'),
  mastAngle: document.getElementById('mastAngle'),
  speedValue: document.getElementById('speedValue'),
  steerValue: document.getElementById('steerValue'),
  mastValue: document.getElementById('mastValue'),
  safeDistance: document.getElementById('safeDistance'),
  dangerDistance: document.getElementById('dangerDistance'),
  safeDistanceValue: document.getElementById('safeDistanceValue'),
  dangerDistanceValue: document.getElementById('dangerDistanceValue'),
  runSeconds: document.getElementById('runSeconds'),
};

const RANGE_SLOTS = [
  { key: 'far_left', angle: -75 },
  { key: 'left', angle: -50 },
  { key: 'half_left', angle: -25 },
  { key: 'center', angle: 0 },
  { key: 'half_right', angle: 25 },
  { key: 'right', angle: 50 },
  { key: 'far_right', angle: 75 },
];

const rangeValues = Object.fromEntries(
  RANGE_SLOTS.map(({ key }) => [key, { distance: null, quality: 'unmeasured' }]),
);
const rangeMarkers = Object.fromEntries(
  Array.from(document.querySelectorAll('[data-range-slot]')).map((marker) => [marker.dataset.rangeSlot, marker]),
);

let activeHoldButton = null;
let lastKeyboardCommand = null;
let shuttingDown = false;
let rangeHudBusy = false;
let missionRangeHudBusy = false;
let activeSweepId = null;

function numericDistance(value) {
  if (value === null || value === undefined || value === '') return null;
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : null;
}

function formatDistance(entry) {
  const quality = entry?.quality || 'unmeasured';
  if (quality === 'unmeasured') return '—';
  if (quality !== 'measured') return '?';
  const n = numericDistance(entry?.distance);
  return n === null ? '?' : `${Math.round(n)} cm`;
}

function nearestRangeSlot(angleDeg) {
  const angle = Number(angleDeg);
  if (!Number.isFinite(angle)) return RANGE_SLOTS[3];
  return RANGE_SLOTS.reduce((best, slot) => (
    Math.abs(slot.angle - angle) < Math.abs(best.angle - angle) ? slot : best
  ));
}

function clearRangeValues() {
  for (const slot of RANGE_SLOTS) {
    rangeValues[slot.key] = { distance: null, quality: 'unmeasured' };
  }
}

function setRangeValue(slotKey, value, quality = null) {
  if (!(slotKey in rangeValues)) return;
  const distance = numericDistance(value);
  rangeValues[slotKey] = {
    distance,
    quality: quality || (distance === null ? 'unknown' : 'measured'),
  };
}

function distanceSeverity(entry) {
  if (!entry || entry.quality !== 'measured') return 'unknown';
  const distance = numericDistance(entry.distance);
  if (distance === null || distance <= 0) return 'unknown';
  const danger = Number(controls.dangerDistance?.value || 35);
  const safe = Number(controls.safeDistance?.value || 55);
  if (distance < danger) return 'danger';
  if (distance < safe) return 'warning';
  return 'clear';
}

function renderRangeHud() {
  for (const slot of RANGE_SLOTS) {
    const marker = rangeMarkers[slot.key];
    if (!marker) continue;
    const entry = rangeValues[slot.key];
    const valueEl = marker.querySelector('.range-value');
    setText(valueEl, formatDistance(entry));
    marker.classList.remove('range-unknown', 'range-clear', 'range-warning', 'range-danger');
    marker.classList.add(`range-${distanceSeverity(entry)}`);
  }
  stateEls.rangeHud?.classList.toggle('is-scanning', rangeHudBusy || missionRangeHudBusy);
}

function applyLidarScan(scan) {
  if (!scan) return;
  missionRangeHudBusy = Boolean(scan.scanning);

  if (scan.kind === 'sweep' && Array.isArray(scan.samples)) {
    const sweepId = scan.scan_id || null;
    if (sweepId && sweepId !== activeSweepId) {
      activeSweepId = sweepId;
      clearRangeValues();
    }
    for (const sample of scan.samples) {
      const slot = nearestRangeSlot(sample.angle_deg);
      const quality = sample.quality || (sample.no_echo ? 'no_echo' : 'measured');
      setRangeValue(slot.key, sample.distance_cm, quality);
    }
    renderRangeHud();
    return;
  }

  if (scan.kind === 'triad' && scan.distances) {
    // Keep the most recent outer sweep points and refresh the three
    // directions measured by obstacle avoidance.
    setRangeValue('left', scan.distances.left, scan.distances.left == null ? 'unknown' : 'measured');
    setRangeValue('center', scan.distances.center, scan.distances.center == null ? 'unknown' : 'measured');
    setRangeValue('right', scan.distances.right, scan.distances.right == null ? 'unknown' : 'measured');
    renderRangeHud();
    return;
  }

  if (scan.kind === 'point') {
    const slot = nearestRangeSlot(scan.angle_deg);
    setRangeValue(slot.key, scan.distance_cm, scan.distance_cm == null ? 'unknown' : 'measured');
    renderRangeHud();
  }
}

function renderError(errorValue) {
  const hasError = Boolean(errorValue && String(errorValue).trim() && String(errorValue).trim() !== '—');
  const text = hasError ? String(errorValue) : '';
  setText(stateEls.lastError, text);
  setText(stateEls.cameraErrorMessage, text);
  stateEls.cameraErrorHud?.classList.toggle('has-error', hasError);
  stateEls.cameraErrorHud?.setAttribute('aria-hidden', hasError ? 'false' : 'true');
}

function commandPayload(command) {
  return {
    command,
    speed: Number(controls.speed?.value || 30),
    steer_angle: Number(controls.steerAngle?.value || 24),
    mast_angle: Number(controls.mastAngle?.value || 45),
    safe_distance: Number(controls.safeDistance?.value || 55),
    danger_distance: Number(controls.dangerDistance?.value || 35),
  };
}

async function postJSON(url, payload = {}) {
  const response = await fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(payload),
  });
  if (!response.ok) throw new Error(`${url} failed: ${response.status}`);
  return response.json();
}

async function getJSON(url) {
  const response = await fetch(url, { cache: 'no-store' });
  if (!response.ok) throw new Error(`${url} failed: ${response.status}`);
  return response.json();
}

function updateState(data) {
  renderError(data.last_error);
  setText(stateEls.cameraStatus, data.camera?.available ? 'camera online' : (data.camera?.error || 'no camera'));

  if (data.lidar_scan) applyLidarScan(data.lidar_scan);
  // The continuously polled front measurement belongs to the centre marker.
  const exploreRunning = Boolean(data.mission?.running && data.mission?.name === 'explore_area');
  const frontDistance = numericDistance(data.distance_cm);
  if (!exploreRunning && frontDistance !== null && frontDistance > 0 && frontDistance <= 400) {
    setRangeValue('center', frontDistance, 'measured');
    renderRangeHud();
  }

  const logs = data.logs || [];
  if (stateEls.logs) {
    stateEls.logs.textContent = logs.length ? logs.slice(-80).join('\n') : 'Waiting for telemetry...';
    stateEls.logs.scrollTop = stateEls.logs.scrollHeight;
  }
}

async function sendCommand(command) {
  const data = await postJSON('/api/command', commandPayload(command));
  updateState(data);
}

async function sendLidar(action, extra = {}) {
  rangeHudBusy = true;
  renderRangeHud();
  try {
    const data = await postJSON('/api/lidar', {
      action,
      mast_angle: Number(controls.mastAngle?.value || 45),
      safe_distance: Number(controls.safeDistance?.value || 55),
      danger_distance: Number(controls.dangerDistance?.value || 35),
      ...extra,
    });
    updateState(data);
  } finally {
    rangeHudBusy = false;
    renderRangeHud();
  }
}

async function emergencyStop() {
  const data = await postJSON('/api/stop');
  updateState(data);
}

async function startAvoidObstacle() {
  const speed = Number(controls.speed?.value || 30);
  const payload = {
    mission: 'avoid_obstacle',
    forward_speed: speed,
    turn_speed: speed,
    reverse_speed: Math.max(10, Math.round(speed * 0.75)),
    safe_distance: Number(controls.safeDistance?.value || 55),
    danger_distance: Number(controls.dangerDistance?.value || 35),
  };
  if (controls.runSeconds?.value) payload.run_seconds = Number(controls.runSeconds.value);
  const data = await postJSON('/api/mission/start', payload);
  updateState(data);
}

async function stopMission() {
  const data = await postJSON('/api/mission/stop');
  updateState(data);
}

function updateSliderLabels(event) {
  if (controls.safeDistance && controls.dangerDistance) {
    const safe = Number(controls.safeDistance.value || 55);
    const danger = Number(controls.dangerDistance.value || 35);
    if (event?.target === controls.safeDistance && danger > safe) {
      controls.dangerDistance.value = String(safe);
    } else if (event?.target === controls.dangerDistance && danger > safe) {
      controls.safeDistance.value = String(danger);
    }
  }
  if (controls.speedValue && controls.speed) controls.speedValue.textContent = controls.speed.value;
  if (controls.steerValue && controls.steerAngle) controls.steerValue.textContent = `${controls.steerAngle.value}°`;
  if (controls.mastValue && controls.mastAngle) controls.mastValue.textContent = `${controls.mastAngle.value}°`;
  if (controls.safeDistanceValue && controls.safeDistance) controls.safeDistanceValue.textContent = `${controls.safeDistance.value} cm`;
  if (controls.dangerDistanceValue && controls.dangerDistance) controls.dangerDistanceValue.textContent = `${controls.dangerDistance.value} cm`;
  renderRangeHud();
}

function bindDriveButtons() {
  document.querySelectorAll('[data-command]').forEach((button) => {
    const command = button.dataset.command;
    const isHold = button.classList.contains('hold');

    if (isHold) {
      const start = async (event) => {
        event.preventDefault();
        activeHoldButton = button;
        button.classList.add('active');
        try { await sendCommand(command); } catch (err) { renderError(err.message); }
      };
      const stop = async (event) => {
        if (activeHoldButton !== button) return;
        if (event) event.preventDefault();
        activeHoldButton = null;
        button.classList.remove('active');
        if (command === 'steer_left' || command === 'steer_right') return;
        try { await sendCommand('stop'); } catch (err) { renderError(err.message); }
      };
      button.addEventListener('pointerdown', start);
      button.addEventListener('pointerup', stop);
      button.addEventListener('pointerleave', stop);
      button.addEventListener('pointercancel', stop);
      return;
    }

    button.addEventListener('click', async () => {
      try { await sendCommand(command); } catch (err) { renderError(err.message); }
    });
  });
}

function bindKeyboard() {
  function commandFromKey(event) {
    if (event.code === 'KeyW' || event.key === 'ArrowUp') return 'forward';
    if (event.code === 'KeyS' || event.key === 'ArrowDown') return 'reverse';
    if (event.code === 'KeyA' || event.key === 'ArrowLeft') return 'steer_left';
    if (event.code === 'KeyD' || event.key === 'ArrowRight') return 'steer_right';
    return null;
  }

  window.addEventListener('keydown', async (event) => {
    if (event.target instanceof HTMLInputElement) return;
    if (event.code === 'Space' || event.key === ' ') {
      event.preventDefault();
      lastKeyboardCommand = null;
      try { await emergencyStop(); } catch (err) { renderError(err.message); }
      return;
    }
    const command = commandFromKey(event);
    if (!command || lastKeyboardCommand === command) return;
    event.preventDefault();
    lastKeyboardCommand = command;
    try { await sendCommand(command); } catch (err) { renderError(err.message); }
  });

  window.addEventListener('keyup', async (event) => {
    const command = commandFromKey(event);
    if (!command || lastKeyboardCommand !== command) return;
    event.preventDefault();
    lastKeyboardCommand = null;
    if (command === 'steer_left' || command === 'steer_right') return;
    try { await sendCommand('stop'); } catch (err) { renderError(err.message); }
  });
}

function bindLidarButtons() {
  document.querySelectorAll('[data-lidar]').forEach((button) => {
    button.addEventListener('click', async () => {
      try { await sendLidar(button.dataset.lidar); } catch (err) { renderError(err.message); }
    });
  });
  document.querySelectorAll('[data-lidar-to]').forEach((button) => {
    button.addEventListener('click', async () => {
      try { await sendLidar('to', { angle_deg: Number(button.dataset.lidarTo) }); } catch (err) { renderError(err.message); }
    });
  });
  document.getElementById('lidarScanBtn')?.addEventListener('click', async () => {
    try { await sendLidar('scan'); } catch (err) { renderError(err.message); }
  });
  document.getElementById('lidarSweepBtn')?.addEventListener('click', async () => {
    try {
      await sendLidar('sweep', { angles: RANGE_SLOTS.map(({ angle }) => angle) });
    } catch (err) {
      renderError(err.message);
    }
  });
}

function bindTopActions() {
  const bindClick = (id, handler) => document.getElementById(id)?.addEventListener('click', handler);

  bindClick('stopBtn', async () => {
    try { await emergencyStop(); } catch (err) { renderError(err.message); }
  });
  bindClick('startAvoidBtn', async () => {
    try { await startAvoidObstacle(); } catch (err) { renderError(err.message); }
  });
  bindClick('stopMissionBtn', async () => {
    try { await stopMission(); } catch (err) { renderError(err.message); }
  });
  bindClick('manualModeBtn', async () => {
    try {
      await stopMission();
      await sendCommand('stop');
    } catch (err) { renderError(err.message); }
  });
  bindClick('shutdownBtn', async () => {
    const ok = window.confirm('Shutdown Marsy dashboard server? Motors will brake first.');
    if (!ok) return;
    shuttingDown = true;
    try {
      const data = await postJSON('/api/shutdown');
      updateState(data);
      if (stateEls.logs) stateEls.logs.textContent += '\nDashboard shutdown requested. Map files will be removed before exit.';
    } catch (err) {
      renderError(err.message);
    }
  });
}

function startPolling() {
  setInterval(async () => {
    if (shuttingDown) return;
    try {
      const data = await getJSON('/api/state');
      updateState(data);
    } catch (err) {
      renderError(err.message);
    }
  }, 700);

  setInterval(async () => {
    if (shuttingDown) return;
    try {
      const data = await postJSON('/api/heartbeat');
      updateState(data);
    } catch (err) {
      renderError(err.message);
    }
  }, 500);
}

window.addEventListener('beforeunload', () => {
  if (navigator.sendBeacon) {
    navigator.sendBeacon('/api/command', new Blob(
      [JSON.stringify(commandPayload('stop'))],
      { type: 'application/json' },
    ));
  }
});

Object.values(controls).forEach((control) => {
  if (control?.addEventListener && control.type !== 'hidden') {
    control.addEventListener('input', updateSliderLabels);
  }
});

updateSliderLabels();
renderError(null);
renderRangeHud();
bindDriveButtons();
bindKeyboard();
bindLidarButtons();
bindTopActions();
startPolling();

getJSON('/api/state')
  .then(updateState)
  .catch((err) => { renderError(err.message); });
