const stateEls = {
  distance: document.getElementById('distance'),
  battery: document.getElementById('battery'),
  heartbeat: document.getElementById('heartbeat'),
  lastCommand: document.getElementById('lastCommand'),
  lastError: document.getElementById('lastError'),
  modePill: document.getElementById('modePill'),
  cameraStatus: document.getElementById('cameraStatus'),
  logs: document.getElementById('logs'),
  lidarReadout: document.getElementById('lidarReadout'),
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

let activeHoldButton = null;
let lastKeyboardCommand = null;
let shuttingDown = false;
let lastScanText = '';

function refreshLidarReadout(currentDistanceText = null) {
  const distanceLine = currentDistanceText ?? `Distance: ${formatDistance(null)}`;
  const content = lastScanText ? `${distanceLine}\n\n${lastScanText}` : distanceLine;
  setText(stateEls.lidarReadout, content);
}


function commandPayload(command) {
  return {
    command,
    speed: Number(controls.speed.value),
    steer_angle: Number(controls.steerAngle.value),
    mast_angle: Number(controls.mastAngle.value),
    safe_distance: Number(controls.safeDistance.value),
    danger_distance: Number(controls.dangerDistance.value),
  };
}

async function postJSON(url, payload = {}) {
  const response = await fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(payload),
  });
  if (!response.ok) {
    throw new Error(`${url} failed: ${response.status}`);
  }
  return response.json();
}

async function getJSON(url) {
  const response = await fetch(url, { cache: 'no-store' });
  if (!response.ok) {
    throw new Error(`${url} failed: ${response.status}`);
  }
  return response.json();
}

function formatDistance(value) {
  if (value === null || value === undefined) return '—';
  const n = Number(value);
  if (!Number.isFinite(n)) return '—';
  if (n === 0) return 'no echo';
  return `${n.toFixed(1)} cm`;
}

function formatBattery(value) {
  if (value === null || value === undefined) return '—';
  const n = Number(value);
  if (!Number.isFinite(n)) return '—';
  return `${n.toFixed(2)} V`;
}

function renderScan(scan) {
  if (!scan) return;
  if (scan.kind === 'sweep' && Array.isArray(scan.samples)) {
    const lines = scan.samples.map((s) => `${String(s.angle_deg).padStart(4)}°  ${formatDistance(s.distance_cm)}`);
    lastScanText = lines.join('\n');
    return;
  }
  if (scan.kind === 'triad' && scan.distances) {
    const d = scan.distances;
    lastScanText = [
      `left   ${formatDistance(d.left)}`,
      `center ${formatDistance(d.center)}`,
      `right  ${formatDistance(d.right)}`,
    ].join('\n');
    return;
  }
  if (scan.kind === 'point') {
    lastScanText = `${scan.angle_deg}°  ${formatDistance(scan.distance_cm)}`;
  }
}

function updateState(data) {
  setText(stateEls.distance, formatDistance(data.distance_cm));
  setText(stateEls.battery, formatBattery(data.battery_v));
  setText(stateEls.heartbeat, data.last_heartbeat_age_s === null || data.last_heartbeat_age_s === undefined
    ? '—'
    : `${Number(data.last_heartbeat_age_s).toFixed(2)} s`);
  setText(stateEls.lastCommand, data.last_command ?? '—');
  setText(stateEls.lastError, data.last_error ?? '—');
  setText(stateEls.modePill, data.mode ?? 'manual');
  setText(stateEls.cameraStatus, data.camera?.available ? 'camera online' : (data.camera?.error || 'no camera'));

  if (data.lidar_scan) {
    renderScan(data.lidar_scan);
  }
  refreshLidarReadout(`Distance: ${formatDistance(data.distance_cm)}`);

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
  setText(stateEls.lidarReadout, 'Scanning...');
  const data = await postJSON('/api/lidar', {
    action,
    mast_angle: Number(controls.mastAngle.value),
    ...extra,
  });
  updateState(data);
}

async function emergencyStop() {
  const data = await postJSON('/api/stop');
  updateState(data);
}

async function startAvoidObstacle() {
  const payload = {
    mission: 'avoid_obstacle',
    forward_speed: Number(controls.speed.value),
    turn_speed: Number(controls.speed.value),
    reverse_speed: Math.max(10, Math.round(Number(controls.speed.value) * 0.75)),
    safe_distance: Number(controls.safeDistance.value),
    danger_distance: Number(controls.dangerDistance.value),
  };
  if (controls.runSeconds.value) {
    payload.run_seconds = Number(controls.runSeconds.value);
  }
  const data = await postJSON('/api/mission/start', payload);
  updateState(data);
}

async function stopMission() {
  const data = await postJSON('/api/mission/stop');
  updateState(data);
}

function updateSliderLabels() {
  controls.speedValue.textContent = controls.speed.value;
  controls.steerValue.textContent = `${controls.steerAngle.value}°`;
  controls.mastValue.textContent = `${controls.mastAngle.value}°`;
  controls.safeDistanceValue.textContent = `${controls.safeDistance.value} cm`;
  controls.dangerDistanceValue.textContent = `${controls.dangerDistance.value} cm`;
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
        try { await sendCommand(command); } catch (err) { console.error(err); }
      };
      const stop = async (event) => {
        if (activeHoldButton !== button) return;
        if (event) event.preventDefault();
        activeHoldButton = null;
        button.classList.remove('active');
        if (command === 'steer_left' || command === 'steer_right') {
          // Sticky steering: left/right buttons set wheel angle and keep it
          // until Center wheels / Reset pose / STOP is pressed.
          return;
        }
        try { await sendCommand('stop'); } catch (err) { console.error(err); }
      };
      button.addEventListener('pointerdown', start);
      button.addEventListener('pointerup', stop);
      button.addEventListener('pointerleave', stop);
      button.addEventListener('pointercancel', stop);
      return;
    }

    button.addEventListener('click', async () => {
      try { await sendCommand(command); } catch (err) { console.error(err); }
    });
  });
}

function bindKeyboard() {
  // Use event.code for W/A/S/D so physical keys work even when the
  // current keyboard layout is not English. event.key remains useful
  // for arrows and Space.
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
      try { await emergencyStop(); } catch (err) { console.error(err); }
      return;
    }
    const command = commandFromKey(event);
    if (!command || lastKeyboardCommand === command) return;
    event.preventDefault();
    lastKeyboardCommand = command;
    try { await sendCommand(command); } catch (err) { console.error(err); }
  });

  window.addEventListener('keyup', async (event) => {
    const command = commandFromKey(event);
    if (!command || lastKeyboardCommand !== command) return;
    event.preventDefault();
    lastKeyboardCommand = null;

    if (command === 'steer_left' || command === 'steer_right') {
      // Sticky steering: releasing A/D or ←/→ does not center the wheels.
      // Use Center wheels, Reset pose, STOP, or Brake to return to neutral.
      return;
    }

    try { await sendCommand('stop'); } catch (err) { console.error(err); }
  });
}

function bindLidarButtons() {
  document.querySelectorAll('[data-lidar]').forEach((button) => {
    button.addEventListener('click', async () => {
      try { await sendLidar(button.dataset.lidar); } catch (err) { console.error(err); }
    });
  });
  document.querySelectorAll('[data-lidar-to]').forEach((button) => {
    button.addEventListener('click', async () => {
      try { await sendLidar('to', { angle_deg: Number(button.dataset.lidarTo) }); } catch (err) { console.error(err); }
    });
  });
  document.getElementById('lidarScanBtn').addEventListener('click', async () => {
    try { await sendLidar('scan'); } catch (err) { console.error(err); }
  });
  document.getElementById('lidarSweepBtn').addEventListener('click', async () => {
    try { await sendLidar('sweep'); } catch (err) { console.error(err); }
  });
}

function bindTopActions() {
  document.getElementById('stopBtn').addEventListener('click', async () => {
    try { await emergencyStop(); } catch (err) { console.error(err); }
  });
  document.getElementById('startAvoidBtn').addEventListener('click', async () => {
    try { await startAvoidObstacle(); } catch (err) { console.error(err); }
  });
  document.getElementById('stopMissionBtn').addEventListener('click', async () => {
    try { await stopMission(); } catch (err) { console.error(err); }
  });
  document.getElementById('manualModeBtn').addEventListener('click', async () => {
    try {
      await stopMission();
      await sendCommand('stop');
    } catch (err) { console.error(err); }
  });
  document.getElementById('shutdownBtn').addEventListener('click', async () => {
    const ok = window.confirm('Shutdown Marsy dashboard server? Motors will brake first.');
    if (!ok) return;
    shuttingDown = true;
    try {
      const data = await postJSON('/api/shutdown');
      updateState(data);
      stateEls.logs.textContent += '\nDashboard shutdown requested. Terminal process should exit.';
    } catch (err) {
      console.error(err);
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
      setText(stateEls.lastError, err.message);
    }
  }, 700);

  setInterval(async () => {
    if (shuttingDown) return;
    try {
      const data = await postJSON('/api/heartbeat');
      updateState(data);
    } catch (err) {
      setText(stateEls.lastError, err.message);
    }
  }, 500);
}

window.addEventListener('beforeunload', () => {
  if (navigator.sendBeacon) {
    navigator.sendBeacon('/api/command', new Blob([JSON.stringify(commandPayload('stop'))], { type: 'application/json' }));
  }
});

Object.values(controls).forEach((control) => {
  if (control && control.addEventListener) {
    control.addEventListener('input', updateSliderLabels);
  }
});

updateSliderLabels();
bindDriveButtons();
bindKeyboard();
bindLidarButtons();
bindTopActions();
startPolling();

getJSON('/api/state')
  .then(updateState)
  .catch((err) => { setText(stateEls.lastError, err.message); });

refreshLidarReadout();
