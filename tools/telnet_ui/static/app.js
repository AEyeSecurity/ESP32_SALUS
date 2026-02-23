const state = {
  connection: { connected: false },
  net: {},
  steer: {},
  pid: {},
  spid: {},
  speed: {},
  comms: {},
  drive_log: {},
  streams: { speed: { state: 'OFF' }, spid: { state: 'OFF' } },
  recent_logs: [],
};

let ws = null;
let commands = [];
const commandMap = new Map();
let pendingConfirm = null;

const el = {
  connectionBadge: document.getElementById('connectionBadge'),
  hostInput: document.getElementById('hostInput'),
  portInput: document.getElementById('portInput'),
  connectBtn: document.getElementById('connectBtn'),
  disconnectBtn: document.getElementById('disconnectBtn'),
  refreshStatusBtn: document.getElementById('refreshStatusBtn'),
  rawCommandInput: document.getElementById('rawCommandInput'),
  sendRawBtn: document.getElementById('sendRawBtn'),
  clearConsoleBtn: document.getElementById('clearConsoleBtn'),
  consoleOutput: document.getElementById('consoleOutput'),
  confirmModal: document.getElementById('confirmModal'),
  confirmText: document.getElementById('confirmText'),
  confirmYes: document.getElementById('confirmYes'),
  confirmNo: document.getElementById('confirmNo'),
};

function safeJson(value) {
  if (!value || Object.keys(value).length === 0) return 'Sin datos';
  try {
    return JSON.stringify(value, null, 2);
  } catch (err) {
    return String(value);
  }
}

function renderState() {
  document.getElementById('state-net').textContent = safeJson(state.net);
  document.getElementById('state-steer').textContent = safeJson(state.steer);
  document.getElementById('state-pid').textContent = safeJson(state.pid);
  document.getElementById('state-spid').textContent = safeJson(state.spid);
  document.getElementById('state-speed').textContent = safeJson(state.speed);
  document.getElementById('state-comms').textContent = safeJson(state.comms);
  document.getElementById('state-drive').textContent = safeJson(state.drive_log);
  renderConnectionBadge();
}

function renderConnectionBadge() {
  const connected = !!state.connection?.connected;
  const host = state.connection?.host || '-';
  const port = state.connection?.port || '-';
  const reason = state.connection?.reason || '';
  el.connectionBadge.textContent = connected
    ? `Conectado ${host}:${port}`
    : `Desconectado (${reason || 'idle'})`;
  el.connectionBadge.classList.toggle('online', connected);
  el.connectionBadge.classList.toggle('offline', !connected);
}

function mergePatch(patch) {
  Object.keys(patch || {}).forEach((key) => {
    state[key] = patch[key];
  });
}

function appendConsole(line, kind = 'info') {
  const row = document.createElement('div');
  row.className = `console-line ${kind}`;
  row.textContent = line;
  el.consoleOutput.appendChild(row);

  while (el.consoleOutput.children.length > 500) {
    el.consoleOutput.removeChild(el.consoleOutput.firstChild);
  }
  el.consoleOutput.scrollTop = el.consoleOutput.scrollHeight;
}

async function api(path, method = 'GET', body = undefined) {
  const resp = await fetch(path, {
    method,
    headers: { 'Content-Type': 'application/json' },
    body: body ? JSON.stringify(body) : undefined,
  });

  if (!resp.ok) {
    let detail = `HTTP ${resp.status}`;
    try {
      const json = await resp.json();
      detail = json.detail || detail;
    } catch (err) {
      // ignore parse error
    }
    throw new Error(detail);
  }

  return resp.json();
}

async function connectEsp() {
  try {
    const host = el.hostInput.value.trim();
    const port = Number(el.portInput.value);
    const result = await api('/api/connect', 'POST', { host, port });
    mergePatch({ connection: result });
    renderState();
    appendConsole(`[UI] Conectado a ${host}:${port}`, 'ok');
  } catch (err) {
    appendConsole(`[UI][ERROR] ${err.message}`, 'error');
  }
}

async function disconnectEsp() {
  try {
    await api('/api/disconnect', 'POST');
    appendConsole('[UI] Desconectado', 'warn');
  } catch (err) {
    appendConsole(`[UI][ERROR] ${err.message}`, 'error');
  }
}

async function refreshStatus() {
  const cmds = ['net.status', 'steer.status', 'pid.status', 'spid.status', 'speed.status', 'comms.status', 'drive.log'];
  for (const id of cmds) {
    try {
      await sendCommandById(id, {}, true, true);
    } catch (err) {
      appendConsole(`[UI][ERROR] ${id}: ${err.message}`, 'error');
    }
  }
}

function wsUrl() {
  const proto = window.location.protocol === 'https:' ? 'wss' : 'ws';
  return `${proto}://${window.location.host}/ws`;
}

function setupWs() {
  ws = new WebSocket(wsUrl());

  ws.onopen = () => {
    appendConsole('[WS] conectado', 'ok');
    const pingTimer = setInterval(() => {
      if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send('ping');
      } else {
        clearInterval(pingTimer);
      }
    }, 20000);
  };

  ws.onmessage = (event) => {
    try {
      const payload = JSON.parse(event.data);
      handleWsEvent(payload);
    } catch (err) {
      appendConsole(`[WS][ERROR] payload inválido`, 'error');
    }
  };

  ws.onclose = () => {
    appendConsole('[WS] desconectado, reintentando...', 'warn');
    setTimeout(setupWs, 1500);
  };

  ws.onerror = () => {
    appendConsole('[WS][ERROR] socket', 'error');
  };
}

function handleWsEvent(payload) {
  const type = payload.type;
  const data = payload.data || {};

  if (type === 'connection') {
    mergePatch({ connection: data });
    renderState();
    return;
  }

  if (type === 'state_patch') {
    mergePatch(data);
    renderState();
    return;
  }

  if (type === 'raw_line') {
    appendConsole(data.line || '', 'raw');
    return;
  }

  if (type === 'command_result') {
    appendConsole(`[CMD] ${data.rendered_command}`, 'ok');
    return;
  }

  if (type === 'error') {
    appendConsole(`[BACKEND][ERROR] ${data.message || 'Error'}`, 'error');
    return;
  }
}

function argInputType(argType) {
  if (argType === 'int' || argType === 'float') return 'number';
  return 'text';
}

function buildCommandRow(cmd) {
  const row = document.createElement('div');
  row.className = 'command-row';

  const title = document.createElement('div');
  title.className = 'command-title';
  title.textContent = cmd.label;
  row.appendChild(title);

  const controls = document.createElement('div');
  controls.className = 'command-controls';

  const inputs = [];
  (cmd.args || []).forEach((arg) => {
    const wrap = document.createElement('label');
    wrap.className = 'input-wrap';

    const span = document.createElement('span');
    span.textContent = arg.label;

    const input = document.createElement('input');
    input.type = argInputType(arg.type);
    input.placeholder = arg.name;
    input.dataset.argName = arg.name;

    if (arg.default !== null && arg.default !== undefined) input.value = arg.default;
    if (arg.min !== null && arg.min !== undefined) input.min = String(arg.min);
    if (arg.max !== null && arg.max !== undefined) input.max = String(arg.max);
    if (arg.step !== null && arg.step !== undefined) input.step = String(arg.step);

    wrap.appendChild(span);
    wrap.appendChild(input);
    controls.appendChild(wrap);
    inputs.push({ def: arg, input });
  });

  const btn = document.createElement('button');
  btn.className = 'btn';
  btn.textContent = 'Ejecutar';
  btn.addEventListener('click', async () => {
    const args = {};
    for (const entry of inputs) {
      const raw = entry.input.value.trim();
      if (!raw) {
        if (entry.def.required) {
          appendConsole(`[UI][ERROR] Falta argumento ${entry.def.name} para ${cmd.id}`, 'error');
          return;
        }
        continue;
      }
      if (entry.def.type === 'int') args[entry.def.name] = parseInt(raw, 10);
      else if (entry.def.type === 'float') args[entry.def.name] = parseFloat(raw);
      else args[entry.def.name] = raw;
    }

    try {
      await executeCatalogCommand(cmd, args);
    } catch (err) {
      appendConsole(`[UI][ERROR] ${cmd.id}: ${err.message}`, 'error');
    }
  });

  controls.appendChild(btn);
  row.appendChild(controls);
  return row;
}

async function sendCommandById(commandId, args = {}, confirmed = false, silent = false) {
  const payload = {
    command_id: commandId,
    args,
    confirmed,
  };
  const result = await api('/api/command', 'POST', payload);
  if (!silent) appendConsole(`[UI] -> ${result.rendered_command}`, 'ok');
  return result;
}

async function executeCatalogCommand(cmd, args) {
  if (cmd.requires_confirmation) {
    const ok = await confirmCommand(cmd.confirm_message || `¿Confirmas ejecutar ${cmd.id}?`);
    if (!ok) {
      appendConsole(`[UI] cancelado ${cmd.id}`, 'warn');
      return;
    }
    return sendCommandById(cmd.id, args, true);
  }
  return sendCommandById(cmd.id, args, false);
}

function confirmCommand(message) {
  return new Promise((resolve) => {
    pendingConfirm = resolve;
    el.confirmText.textContent = message;
    el.confirmModal.classList.remove('hidden');
  });
}

function closeConfirm(answer) {
  el.confirmModal.classList.add('hidden');
  if (pendingConfirm) {
    pendingConfirm(answer);
    pendingConfirm = null;
  }
}

async function loadCommands() {
  const response = await api('/api/commands');
  commands = response.commands || [];
  commandMap.clear();
  commands.forEach((cmd) => commandMap.set(cmd.id, cmd));

  document.querySelectorAll('.commands').forEach((container) => {
    container.innerHTML = '';
    const group = container.dataset.group;
    const groupCommands = commands.filter((cmd) => cmd.group === group);
    groupCommands.forEach((cmd) => {
      container.appendChild(buildCommandRow(cmd));
    });
  });
}

async function sendRawCommand() {
  const command = el.rawCommandInput.value.trim();
  if (!command) return;
  try {
    await api('/api/raw-command', 'POST', { command });
    appendConsole(`[RAW] -> ${command}`, 'ok');
    const history = JSON.parse(localStorage.getItem('rawHistory') || '[]');
    history.push(command);
    while (history.length > 50) history.shift();
    localStorage.setItem('rawHistory', JSON.stringify(history));
    el.rawCommandInput.value = '';
  } catch (err) {
    appendConsole(`[RAW][ERROR] ${err.message}`, 'error');
  }
}

async function loadInitialState() {
  try {
    const snapshot = await api('/api/status');
    mergePatch(snapshot);
    renderState();
  } catch (err) {
    appendConsole(`[UI][ERROR] status: ${err.message}`, 'error');
  }
}

function bindEvents() {
  el.connectBtn.addEventListener('click', connectEsp);
  el.disconnectBtn.addEventListener('click', disconnectEsp);
  el.refreshStatusBtn.addEventListener('click', refreshStatus);
  el.sendRawBtn.addEventListener('click', sendRawCommand);
  el.clearConsoleBtn.addEventListener('click', () => {
    el.consoleOutput.innerHTML = '';
  });
  el.rawCommandInput.addEventListener('keydown', (ev) => {
    if (ev.key === 'Enter') sendRawCommand();
  });

  document.getElementById('speedStreamOn').addEventListener('click', () => {
    const periodMs = Number(document.getElementById('speedStreamPreset').value);
    sendCommandById('speed.stream.on', { period_ms: periodMs });
  });
  document.getElementById('speedStreamOff').addEventListener('click', () => {
    sendCommandById('speed.stream.off', {});
  });

  document.getElementById('spidStreamOn').addEventListener('click', () => {
    const periodMs = Number(document.getElementById('spidStreamPreset').value);
    sendCommandById('spid.stream.on', { period_ms: periodMs });
  });
  document.getElementById('spidStreamOff').addEventListener('click', () => {
    sendCommandById('spid.stream.off', {});
  });

  document.getElementById('driveLogOn').addEventListener('click', () => {
    sendCommandById('drive.log.on', {});
  });
  document.getElementById('driveLogOff').addEventListener('click', () => {
    sendCommandById('drive.log.off', {});
  });

  el.confirmYes.addEventListener('click', () => closeConfirm(true));
  el.confirmNo.addEventListener('click', () => closeConfirm(false));
  el.confirmModal.addEventListener('click', (ev) => {
    if (ev.target === el.confirmModal) closeConfirm(false);
  });
}

async function start() {
  bindEvents();
  await loadCommands();
  await loadInitialState();
  setupWs();
}

start();
