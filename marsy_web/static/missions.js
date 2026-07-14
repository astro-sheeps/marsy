(() => {
  function activateRightTab(name) {
    document.querySelectorAll('[data-right-tab]').forEach((button) => {
      const active = button.dataset.rightTab === name;
      button.classList.toggle('active', active);
      button.setAttribute('aria-selected', active ? 'true' : 'false');
    });
    document.querySelectorAll('[data-right-panel]').forEach((panel) => {
      const active = panel.dataset.rightPanel === name;
      panel.classList.toggle('active', active);
      panel.hidden = !active;
    });
  }

  document.querySelectorAll('[data-right-tab]').forEach((button) => {
    button.addEventListener('click', () => activateRightTab(button.dataset.rightTab));
  });
  // Always open on telemetry so rover state is visible immediately.
  activateRightTab('telemetry');

  const listEl = document.getElementById('missionList');
  const selectorEl = document.getElementById('missionSelector');
  const summaryEl = document.getElementById('activeMissionSummary');
  const statePillEl = document.getElementById('missionStatePill');
  if (!listEl || !selectorEl || !summaryEl || !statePillEl) return;

  let catalog = [];
  let activeMission = null;
  let selectedMissionId = null;
  let requestInFlight = false;
  let stopButton = null;

  async function getJSON(url) {
    const response = await fetch(url, { cache: 'no-store' });
    const data = await response.json().catch(() => ({}));
    if (!response.ok) throw new Error(data.error || `${url} failed: ${response.status}`);
    return data;
  }

  async function postJSON(url, payload = {}) {
    const response = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    const data = await response.json().catch(() => ({}));
    if (!response.ok) throw new Error(data.error || `${url} failed: ${response.status}`);
    return data;
  }

  function inputId(missionId, parameterName) {
    return `mission-${missionId}-${parameterName}`;
  }

  function createParameterField(mission, parameter) {
    const label = document.createElement('label');
    label.className = 'mission-field';
    if (parameter.type === 'checkbox') label.classList.add('checkbox-field');

    const input = document.createElement('input');
    input.id = inputId(mission.id, parameter.name);
    input.dataset.parameter = parameter.name;
    input.dataset.parameterType = parameter.type;

    if (parameter.type === 'checkbox') {
      input.type = 'checkbox';
      input.checked = Boolean(parameter.default);
      label.append(input, document.createTextNode(parameter.label));
      return label;
    }

    const caption = document.createElement('span');
    caption.textContent = parameter.optional ? `${parameter.label} (optional)` : parameter.label;
    input.type = parameter.type || 'number';
    if (parameter.min !== undefined) input.min = String(parameter.min);
    if (parameter.max !== undefined) input.max = String(parameter.max);
    if (parameter.step !== undefined) input.step = String(parameter.step);
    if (parameter.default !== undefined && parameter.default !== null) input.value = String(parameter.default);
    if (parameter.optional) input.placeholder = 'no limit';
    label.append(caption, input);
    return label;
  }

  function missionPayload(mission) {
    const payload = { mission: mission.id };
    for (const parameter of mission.parameters || []) {
      const input = document.getElementById(inputId(mission.id, parameter.name));
      if (!input) continue;
      if (parameter.type === 'checkbox') {
        payload[parameter.name] = input.checked;
        continue;
      }
      if (input.value === '' && parameter.optional) continue;
      const value = Number(input.value);
      if (Number.isFinite(value)) payload[parameter.name] = value;
    }
    if (mission.id === 'avoid_obstacle') {
      payload.turn_speed = payload.forward_speed;
      payload.reverse_speed = Math.max(10, Math.round((payload.forward_speed || 25) * 0.75));
    }
    return payload;
  }

  function formatProgress(mission) {
    const progress = mission?.progress || {};
    const details = [];
    if (progress.step !== undefined && progress.total_steps !== undefined) {
      details.push(`step ${progress.step}/${progress.total_steps}`);
    }
    if (progress.elapsed_s !== undefined) details.push(`${progress.elapsed_s} s`);
    if (progress.latest_map) details.push('map updated');
    return details.join(' · ');
  }

  function selectMission(missionId) {
    if (!catalog.some((mission) => mission.id === missionId)) return;
    selectedMissionId = missionId;
    document.querySelectorAll('[data-mission-select]').forEach((button) => {
      const selected = button.dataset.missionSelect === missionId;
      button.classList.toggle('active', selected);
      button.setAttribute('aria-selected', selected ? 'true' : 'false');
    });
    document.querySelectorAll('.mission-card').forEach((card) => {
      const selected = card.dataset.missionId === missionId;
      card.classList.toggle('selected', selected);
      card.hidden = !selected;
    });
  }

  function updateMissionState(mission) {
    activeMission = mission || null;
    const running = Boolean(activeMission?.running);
    statePillEl.textContent = running ? 'running' : 'idle';
    statePillEl.dataset.state = running ? 'running' : 'idle';
    summaryEl.classList.toggle('running', running);
    const missionTitle = catalog.find((item) => item.id === activeMission?.name)?.name || activeMission?.name;

    if (running) {
      const progress = formatProgress(activeMission);
      summaryEl.textContent = `${missionTitle}${progress ? ` · ${progress}` : ''}`;
    } else if (activeMission?.name && activeMission?.returncode !== null && activeMission?.returncode !== undefined) {
      const progress = formatProgress(activeMission);
      summaryEl.textContent = `${missionTitle} finished with code ${activeMission.returncode}${progress ? ` · ${progress}` : ''}`;
    } else {
      summaryEl.textContent = 'No mission running';
    }

    if (stopButton) stopButton.disabled = !running || requestInFlight;

    document.querySelectorAll('.mission-card').forEach((card) => {
      const isActive = running && card.dataset.missionId === activeMission.name;
      card.classList.toggle('active', isActive);
      const start = card.querySelector('.mission-start-button');
      if (start) start.disabled = running || requestInFlight;
    });

    document.querySelectorAll('[data-mission-select]').forEach((button) => {
      button.classList.toggle('running', running && button.dataset.missionSelect === activeMission.name);
    });
  }

  async function startMission(mission) {
    if (requestInFlight || activeMission?.running) return;
    requestInFlight = true;
    updateMissionState(activeMission);
    try {
      const state = await postJSON('/api/mission/start', missionPayload(mission));
      updateMissionState(state.mission);
    } catch (error) {
      summaryEl.textContent = error.message;
      summaryEl.classList.add('running');
    } finally {
      requestInFlight = false;
      updateMissionState(activeMission);
    }
  }

  async function stopMission() {
    if (requestInFlight) return;
    requestInFlight = true;
    try {
      const state = await postJSON('/api/mission/stop');
      updateMissionState(state.mission);
    } catch (error) {
      summaryEl.textContent = error.message;
    } finally {
      requestInFlight = false;
      updateMissionState(activeMission);
    }
  }

  function renderCatalog() {
    selectorEl.replaceChildren();
    listEl.replaceChildren();

    for (const mission of catalog) {
      const selector = document.createElement('button');
      selector.type = 'button';
      selector.className = 'mission-selector-button';
      selector.dataset.missionSelect = mission.id;
      selector.setAttribute('role', 'tab');
      selector.setAttribute('aria-selected', 'false');
      selector.textContent = mission.name;
      selector.addEventListener('click', () => selectMission(mission.id));
      selectorEl.append(selector);

      const card = document.createElement('article');
      card.className = 'mission-card';
      card.dataset.missionId = mission.id;
      card.hidden = true;

      const header = document.createElement('div');
      header.className = 'mission-card-header';
      const name = document.createElement('div');
      name.className = 'mission-name';
      name.textContent = mission.name;
      header.append(name);
      if (mission.map_enabled) {
        const badge = document.createElement('span');
        badge.className = 'mission-map-badge';
        badge.textContent = 'live map';
        header.append(badge);
      }

      const description = document.createElement('div');
      description.className = 'mission-description';
      description.textContent = mission.description;

      const parametersTitle = document.createElement('div');
      parametersTitle.className = 'mission-subtitle';
      parametersTitle.textContent = 'Mission parameters';

      const parameters = document.createElement('div');
      parameters.className = 'mission-parameters';
      for (const parameter of mission.parameters || []) {
        parameters.append(createParameterField(mission, parameter));
      }

      const actions = document.createElement('div');
      actions.className = 'mission-actions';
      const start = document.createElement('button');
      start.type = 'button';
      start.className = 'mission-start-button';
      start.textContent = 'Start mission';
      start.addEventListener('click', () => startMission(mission));
      actions.append(start);
      if (mission.map_enabled) {
        const maps = document.createElement('button');
        maps.type = 'button';
        maps.className = 'mission-map-button';
        maps.textContent = 'Open map';
        maps.addEventListener('click', () => { window.location.href = '/maps'; });
        actions.append(maps);
      }

      card.append(header, description, parametersTitle, parameters, actions);
      listEl.append(card);
    }

    const preferred = activeMission?.name && catalog.some((mission) => mission.id === activeMission.name)
      ? activeMission.name
      : selectedMissionId || catalog[0]?.id;
    if (preferred) selectMission(preferred);
    updateMissionState(activeMission);
  }

  async function loadCatalog() {
    try {
      const data = await getJSON('/api/missions');
      catalog = Array.isArray(data.missions) ? data.missions : [];
      activeMission = data.active || null;
      renderCatalog();
    } catch (error) {
      listEl.replaceChildren();
      const message = document.createElement('div');
      message.className = 'mission-error';
      message.textContent = `Cannot load missions: ${error.message}`;
      listEl.append(message);
      statePillEl.textContent = 'error';
    }
  }

  async function pollState() {
    try {
      const state = await getJSON('/api/state');
      updateMissionState(state.mission);
    } catch (_) {
      statePillEl.textContent = 'offline';
    }
  }

  stopButton = document.createElement('button');
  stopButton.type = 'button';
  stopButton.className = 'mission-stop-button';
  stopButton.textContent = 'Stop current mission';
  stopButton.addEventListener('click', stopMission);
  summaryEl.insertAdjacentElement('afterend', stopButton);

  loadCatalog();
  setInterval(pollState, 800);
})();
