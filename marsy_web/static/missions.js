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
    if (parameter.wide || parameter.type === 'textarea') label.classList.add('wide-field');

    if (parameter.type === 'checkbox') {
      const input = document.createElement('input');
      input.id = inputId(mission.id, parameter.name);
      input.dataset.parameter = parameter.name;
      input.dataset.parameterType = parameter.type;
      input.type = 'checkbox';
      input.checked = Boolean(parameter.default);
      label.append(input, document.createTextNode(parameter.label));
      return label;
    }

    const caption = document.createElement('span');
    caption.textContent = parameter.optional ? `${parameter.label} (optional)` : parameter.label;

    let input;
    if (parameter.type === 'textarea') {
      input = document.createElement('textarea');
      input.rows = Number(parameter.rows || 5);
      input.spellcheck = true;
    } else if (parameter.type === 'select') {
      input = document.createElement('select');
      for (const option of parameter.options || []) {
        const optionEl = document.createElement('option');
        if (typeof option === 'string') {
          optionEl.value = option;
          optionEl.textContent = option;
        } else {
          optionEl.value = String(option.value ?? '');
          optionEl.textContent = String(option.label ?? option.value ?? '');
        }
        input.append(optionEl);
      }
      const configuredDefault = String(parameter.default ?? '');
      if (configuredDefault && !Array.from(input.options).some((option) => option.value === configuredDefault)) {
        const customOption = document.createElement('option');
        customOption.value = configuredDefault;
        customOption.textContent = configuredDefault;
        input.append(customOption);
      }
    } else {
      input = document.createElement('input');
      input.type = parameter.type === 'text' ? 'text' : (parameter.type || 'number');
      if (parameter.type === 'text') input.autocomplete = 'off';
    }

    input.id = inputId(mission.id, parameter.name);
    input.dataset.parameter = parameter.name;
    input.dataset.parameterType = parameter.type;
    if (parameter.min !== undefined) input.min = String(parameter.min);
    if (parameter.max !== undefined) input.max = String(parameter.max);
    if (parameter.step !== undefined) input.step = String(parameter.step);
    if (parameter.default !== undefined && parameter.default !== null) input.value = String(parameter.default);
    if (parameter.placeholder) input.placeholder = String(parameter.placeholder);
    else if (parameter.optional && 'placeholder' in input) input.placeholder = 'no limit';
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
      const raw = input.value.trim();
      if (raw === '' && parameter.optional) continue;
      if (['text', 'string', 'textarea', 'select'].includes(parameter.type)) {
        payload[parameter.name] = raw;
        continue;
      }
      const value = Number(raw);
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
    if (progress.phase) details.push(String(progress.phase).replaceAll('_', ' '));
    if (progress.step !== undefined && progress.total_steps !== undefined && progress.total_steps > 0) {
      details.push(`step ${progress.step}/${progress.total_steps}`);
    }
    if (progress.skill) details.push(progress.skill);
    if (progress.elapsed_s !== undefined) details.push(`${progress.elapsed_s} s`);
    if (progress.latest_map) details.push('map updated');
    if (progress.status && progress.phase === 'finished') details.push(progress.status);
    return details.join(' · ');
  }

  function formatPlanValue(value) {
    if (value === null || value === undefined) return '—';
    if (typeof value === 'boolean') return value ? 'yes' : 'no';
    if (Array.isArray(value)) return value.join(', ');
    if (typeof value === 'object') return JSON.stringify(value);
    return String(value);
  }

  function updateAgentPlan(mission) {
    const panel = document.querySelector('[data-agent-plan-panel]');
    if (!panel) return;
    const meta = panel.querySelector('[data-agent-plan-meta]');
    const empty = panel.querySelector('[data-agent-plan-empty]');
    const list = panel.querySelector('[data-agent-plan-list]');
    const rawDetails = panel.querySelector('[data-agent-plan-details]');
    const raw = panel.querySelector('[data-agent-plan-json]');
    if (!meta || !empty || !list || !rawDetails || !raw) return;

    const isAgentMission = mission?.name === 'agent_mission';
    const progress = isAgentMission ? (mission.progress || {}) : {};
    const plan = progress.plan;
    const steps = Array.isArray(plan?.steps) ? plan.steps : [];

    list.replaceChildren();
    if (!plan || steps.length === 0) {
      panel.classList.add('empty');
      meta.textContent = '';
      raw.textContent = '';
      rawDetails.hidden = true;
      if (isAgentMission && progress.phase === 'planning') {
        empty.textContent = 'Generating and validating the Groq plan…';
      } else if (isAgentMission && progress.phase === 'error') {
        empty.textContent = progress.message || 'The plan could not be generated.';
      } else {
        empty.textContent = 'The generated plan will appear here after the mission starts.';
      }
      return;
    }

    panel.classList.remove('empty');
    empty.textContent = '';
    const sourceLabels = {
      groq: 'Groq',
      cache: 'plan cache',
      replan: 'Groq replan',
      mock: 'mock planner',
    };
    const source = sourceLabels[progress.plan_source] || progress.plan_source || 'planner';
    const revision = Number(progress.plan_revision || 0);
    const model = progress.model ? ` · ${progress.model}` : '';
    meta.textContent = `${source}${model} · ${steps.length} steps${revision > 0 ? ` · revision ${revision}` : ''}`;

    const currentStep = Number(progress.step || 0);
    steps.forEach((step, index) => {
      const item = document.createElement('li');
      item.className = 'mission-plan-step';
      if (progress.phase === 'step_started' && currentStep === index + 1) {
        item.classList.add('current');
      }

      const number = document.createElement('span');
      number.className = 'mission-plan-number';
      number.textContent = String(index + 1);

      const body = document.createElement('div');
      body.className = 'mission-plan-step-body';
      const skill = document.createElement('div');
      skill.className = 'mission-plan-skill';
      skill.textContent = String(step?.skill || 'unknown skill');
      body.append(skill);

      const argumentsObject = step?.arguments && typeof step.arguments === 'object'
        ? step.arguments
        : {};
      const argumentEntries = Object.entries(argumentsObject);
      if (argumentEntries.length > 0) {
        const argumentsEl = document.createElement('div');
        argumentsEl.className = 'mission-plan-arguments';
        for (const [key, value] of argumentEntries) {
          const argument = document.createElement('span');
          argument.className = 'mission-plan-argument';
          argument.textContent = `${key}: ${formatPlanValue(value)}`;
          argumentsEl.append(argument);
        }
        body.append(argumentsEl);
      }

      item.append(number, body);
      list.append(item);
    });

    raw.textContent = JSON.stringify(plan, null, 2);
    rawDetails.hidden = false;
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
    updateAgentPlan(activeMission);
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
      for (const parameter of mission.parameters || []) parameters.append(createParameterField(mission, parameter));

      let planPanel = null;
      if (mission.id === 'agent_mission') {
        planPanel = document.createElement('section');
        planPanel.className = 'mission-plan-panel empty';
        planPanel.dataset.agentPlanPanel = 'true';

        const planHeader = document.createElement('div');
        planHeader.className = 'mission-plan-header';
        const planTitle = document.createElement('div');
        planTitle.className = 'mission-subtitle';
        planTitle.textContent = 'Generated plan';
        const planMeta = document.createElement('div');
        planMeta.className = 'mission-plan-meta';
        planMeta.dataset.agentPlanMeta = 'true';
        planHeader.append(planTitle, planMeta);

        const planEmpty = document.createElement('div');
        planEmpty.className = 'mission-plan-empty';
        planEmpty.dataset.agentPlanEmpty = 'true';
        planEmpty.textContent = 'The generated plan will appear here after the mission starts.';

        const planList = document.createElement('ol');
        planList.className = 'mission-plan-list';
        planList.dataset.agentPlanList = 'true';

        const planDetails = document.createElement('details');
        planDetails.className = 'mission-plan-details';
        planDetails.dataset.agentPlanDetails = 'true';
        planDetails.hidden = true;
        const planDetailsTitle = document.createElement('summary');
        planDetailsTitle.textContent = 'Raw JSON';
        const planJson = document.createElement('pre');
        planJson.className = 'mission-plan-json';
        planJson.dataset.agentPlanJson = 'true';
        planDetails.append(planDetailsTitle, planJson);

        planPanel.append(planHeader, planEmpty, planList, planDetails);
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
      card.append(header, description, parametersTitle, parameters);
      if (planPanel) card.append(planPanel);
      card.append(actions);
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
