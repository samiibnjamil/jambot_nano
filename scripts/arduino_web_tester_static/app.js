'use strict';

/* ---------- low-level API + console/serial-monitor helpers ---------- */

const consoleEl = document.getElementById('consoleOutput');
const autoscrollEl = document.getElementById('autoscroll');

function logLine(kind, text) {
  const time = new Date().toLocaleTimeString();
  const div = document.createElement('div');
  div.className = 'log-' + kind;
  div.textContent = `[${time}] ${text}`;
  consoleEl.appendChild(div);
  while (consoleEl.childElementCount > 500) consoleEl.removeChild(consoleEl.firstChild);
  if (autoscrollEl.checked) consoleEl.scrollTop = consoleEl.scrollHeight;
}

async function apiGet(path) {
  const res = await fetch(path);
  return res.json();
}

async function apiPost(path, body, signal) {
  const res = await fetch(path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body || {}),
    signal,
  });
  return res.json();
}

async function sendCommand(cmd, { log = true, signal } = {}) {
  if (log) logLine('sent', cmd);
  try {
    const res = await apiPost('/api/command', { cmd }, signal);
    if (res.error) {
      if (log) logLine('error', res.error);
      return null;
    }
    if (log) logLine('recv', res.reply);
    return res.reply;
  } catch (e) {
    if (e.name === 'AbortError') return null; // expected when STOP cancels an in-flight send
    if (log) logLine('error', String(e));
    return null;
  }
}

async function sendCmdVel(linearX, angularZ, { log = true, signal } = {}) {
  if (log) logLine('sent', `cmd_vel linear.x=${linearX} angular.z=${angularZ}`);
  try {
    const res = await apiPost('/api/cmd_vel', { linear_x: linearX, angular_z: angularZ }, signal);
    if (res.error) {
      if (log) logLine('error', res.error);
      return null;
    }
    return res;
  } catch (e) {
    if (e.name === 'AbortError') return null;
    if (log) logLine('error', String(e));
    return null;
  }
}

// Tracks whatever motor command is currently in flight from the joystick or
// sine-test loops, so STOP can cancel it immediately instead of letting it
// land on the Arduino after the stop command.
let motorAbortController = null;

/* ---------- connection state ---------- */

let connected = false;
let telemetryTimer = null;

function setConnected(isConnected, port, baud) {
  connected = isConnected;
  document.getElementById('statusDot').classList.toggle('on', isConnected);
  document.getElementById('statusText').textContent = isConnected
    ? `Connected: ${port} @ ${baud}` : 'Disconnected';
  document.querySelectorAll('.needs-connection').forEach(el => { el.disabled = !isConnected; });
  document.getElementById('disconnectBtn').disabled = !isConnected;
  document.getElementById('connectBtn').disabled = isConnected;

  if (isConnected) {
    startTelemetryPolling();
  } else {
    stopTelemetryPolling();
    joystickState.enabled = false;
    document.getElementById('joyEnable').checked = false;
  }
}

async function refreshPorts() {
  const data = await apiGet('/api/ports');
  const sel = document.getElementById('portSelect');
  const current = sel.value;
  sel.innerHTML = '';
  for (const p of data.ports) {
    const opt = document.createElement('option');
    opt.value = p; opt.textContent = p;
    sel.appendChild(opt);
  }
  if (data.ports.includes(current)) sel.value = current;
}

document.getElementById('refreshPortsBtn').addEventListener('click', refreshPorts);

document.getElementById('connectBtn').addEventListener('click', async () => {
  const port = document.getElementById('portSelect').value || document.getElementById('portManual').value;
  const baud = parseInt(document.getElementById('baudInput').value, 10) || 115200;
  if (!port) { logLine('error', 'pick or type a serial port first'); return; }
  logLine('info', `connecting to ${port} @ ${baud} (waiting for boot)...`);
  const res = await apiPost('/api/connect', { port, baud });
  if (res.error) {
    logLine('error', res.error);
    return;
  }
  setConnected(true, res.port, res.baud);
  for (const line of res.boot_log || []) logLine('recv', '[boot] ' + line);
  logLine('info', 'connected.');
});

document.getElementById('disconnectBtn').addEventListener('click', async () => {
  await apiPost('/api/disconnect', {});
  setConnected(false);
  logLine('info', 'disconnected.');
});

document.getElementById('stopBtn').addEventListener('click', () => {
  joystickState.enabled = false;
  document.getElementById('joyEnable').checked = false;
  sineTest.stop();
  stepTest.stop();
  if (motorAbortController) { motorAbortController.abort(); motorAbortController = null; }
  if (connected) sendCommand('m 0 0');
  sendCmdVel(0, 0, { log: false }); // harmless no-op if ROS2 mode isn't running
});

/* ---------- serial monitor free-text input ---------- */

document.getElementById('monitorForm').addEventListener('submit', (e) => {
  e.preventDefault();
  const input = document.getElementById('monitorInput');
  const cmd = input.value.trim();
  if (!cmd) return;
  sendCommand(cmd);
  input.value = '';
});

document.getElementById('clearConsoleBtn').addEventListener('click', () => {
  consoleEl.innerHTML = '';
});

/* ---------- battery / encoders / IMU one-shot reads ---------- */

document.getElementById('readBatteryBtn').addEventListener('click', () => sendCommand('b'));
document.getElementById('readEncodersBtn').addEventListener('click', () => sendCommand('e'));
document.getElementById('resetEncodersBtn').addEventListener('click', () => sendCommand('r'));
document.getElementById('readImuBtn').addEventListener('click', () => sendCommand('i'));
document.getElementById('togglePidBtn').addEventListener('click', () => sendCommand('x'));

document.getElementById('autonomousOnBtn').addEventListener('click', () => sendCommand('a 1'));
document.getElementById('autonomousOffBtn').addEventListener('click', () => sendCommand('a 0'));

/* ---------- buzzer ---------- */

document.getElementById('buzzerButtons').addEventListener('click', (e) => {
  const btn = e.target.closest('button[data-buzz]');
  if (!btn) return;
  sendCommand('n ' + btn.dataset.buzz);
});

/* ---------- LED: static color + patterns ---------- */

function currentRgb() {
  return [
    document.getElementById('ledR').checked ? 1 : 0,
    document.getElementById('ledG').checked ? 1 : 0,
    document.getElementById('ledB').checked ? 1 : 0,
  ];
}

document.getElementById('applyColorBtn').addEventListener('click', () => {
  const [r, g, b] = currentRgb();
  sendCommand(`l ${r} ${g} ${b}`);
});

document.querySelectorAll('button[data-preset]').forEach(btn => {
  btn.addEventListener('click', () => {
    const [r, g, b] = btn.dataset.preset.split(',').map(Number);
    document.getElementById('ledR').checked = !!r;
    document.getElementById('ledG').checked = !!g;
    document.getElementById('ledB').checked = !!b;
    sendCommand(`l ${r} ${g} ${b}`);
  });
});

document.querySelectorAll('button[data-pattern]').forEach(btn => {
  btn.addEventListener('click', () => sendCommand('l ' + btn.dataset.pattern));
});

/* ---------- motor control: raw PWM + closed-loop RPM, per motor ---------- */

function wireSliderNumberPair(sliderId, numberId) {
  const slider = document.getElementById(sliderId);
  const number = document.getElementById(numberId);
  slider.addEventListener('input', () => { number.value = slider.value; });
  number.addEventListener('input', () => { slider.value = number.value; });
}

['pwm1', 'pwm2'].forEach(id => wireSliderNumberPair(id + 'Slider', id + 'Num'));
['rpm1', 'rpm2'].forEach(id => wireSliderNumberPair(id + 'Slider', id + 'Num'));

document.getElementById('maxRpmInput').addEventListener('change', (e) => {
  const max = parseInt(e.target.value, 10) || 300;
  ['rpm1Slider', 'rpm2Slider'].forEach(id => {
    document.getElementById(id).min = -max;
    document.getElementById(id).max = max;
  });
});

document.getElementById('applyPwmBtn').addEventListener('click', () => {
  const p1 = document.getElementById('pwm1Num').value || 0;
  const p2 = document.getElementById('pwm2Num').value || 0;
  sendCommand(`o ${p1} ${p2}`);
});
document.getElementById('stopPwmBtn').addEventListener('click', () => {
  document.getElementById('pwm1Num').value = 0; document.getElementById('pwm1Slider').value = 0;
  document.getElementById('pwm2Num').value = 0; document.getElementById('pwm2Slider').value = 0;
  sendCommand('o 0 0');
});

document.getElementById('applyRpmBtn').addEventListener('click', () => {
  const r1 = document.getElementById('rpm1Num').value || 0;
  const r2 = document.getElementById('rpm2Num').value || 0;
  sendCommand(`m ${r1} ${r2}`);
});
document.getElementById('stopRpmBtn').addEventListener('click', () => {
  document.getElementById('rpm1Num').value = 0; document.getElementById('rpm1Slider').value = 0;
  document.getElementById('rpm2Num').value = 0; document.getElementById('rpm2Slider').value = 0;
  sendCommand('m 0 0');
});

/* ---------- PID tuning ---------- */

function wireGainPair(sliderId, numberId) {
  const slider = document.getElementById(sliderId);
  const number = document.getElementById(numberId);
  slider.addEventListener('input', () => { number.value = slider.value; });
  number.addEventListener('input', () => { slider.value = number.value; });
}
['kp', 'kd', 'ki', 'ko'].forEach(id => wireGainPair(id + 'Slider', id + 'Num'));

document.getElementById('applyPidBtn').addEventListener('click', () => {
  const kp = document.getElementById('kpNum').value;
  const kd = document.getElementById('kdNum').value;
  const ki = document.getElementById('kiNum').value;
  const ko = document.getElementById('koNum').value;
  sendCommand(`p ${kp} ${kd} ${ki} ${ko}`);
});

/* ---------- live telemetry polling + dashboard + chart feed ---------- */

function updateBatteryGauge(voltage) {
  const fill = document.getElementById('batteryFill');
  const label = document.getElementById('batteryLabel');
  label.textContent = voltage.toFixed(2) + ' V';
  const pct = Math.max(0, Math.min(100, ((voltage - 9.0) / (12.6 - 9.0)) * 100));
  fill.style.width = pct + '%';
  fill.style.background = voltage <= 9.9 ? 'var(--bad)' : voltage <= 10.3 ? 'var(--warn)' : 'var(--good)';
}

async function pollTelemetry() {
  try {
    const data = await apiGet('/api/telemetry');
    if (data.error) return;
    document.getElementById('encReadout').textContent = `${data.enc1} / ${data.enc2}`;
    document.getElementById('imuReadout').textContent = data.imu_ok
      ? `ax=${data.ax} ay=${data.ay} az=${data.az} gx=${data.gx} gy=${data.gy} gz=${data.gz}`
      : 'FAULT';
    updateBatteryGauge(data.battery);
    document.getElementById('pidFlagsReadout').textContent =
      `pid=${data.pid_enabled ? 'on' : 'off'} ramping=${data.ramping ? 'yes' : 'no'}`;
    document.getElementById('targetReadout').textContent =
      `${data.target_rpm1 ?? '-'} / ${data.target_rpm2 ?? '-'}`;
    document.getElementById('actualReadout').textContent =
      `${data.filtered_rpm1 ?? '-'} / ${data.filtered_rpm2 ?? '-'}`;
    document.getElementById('pwmReadout').textContent = `${data.pwm1 ?? '-'} / ${data.pwm2 ?? '-'}`;

    chart.push({
      target1: data.target_rpm1, actual1: data.filtered_rpm1,
      target2: data.target_rpm2, actual2: data.filtered_rpm2,
      pwm1: data.pwm1, pwm2: data.pwm2,
    });
  } catch (e) {
    // transient poll failure -- next tick will retry
  }
}

function startTelemetryPolling() {
  stopTelemetryPolling();
  const intervalMs = parseInt(document.getElementById('pollIntervalInput').value, 10) || 100;
  telemetryTimer = setInterval(pollTelemetry, intervalMs);
}
function stopTelemetryPolling() {
  if (telemetryTimer) clearInterval(telemetryTimer);
  telemetryTimer = null;
}
document.getElementById('pollIntervalInput').addEventListener('change', () => {
  if (connected) startTelemetryPolling();
});

/* ---------- live PID chart (dependency-free canvas line chart) ---------- */

class LiveChart {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.t0 = null;
    this.series = {
      target1: { color: '#4fc3f7', label: 'Target RPM1', data: [] },
      actual1: { color: '#0288d1', label: 'Actual RPM1', data: [] },
      target2: { color: '#ffb74d', label: 'Target RPM2', data: [] },
      actual2: { color: '#f57c00', label: 'Actual RPM2', data: [] },
      pwm1: { color: '#81c784', label: 'PWM1', data: [] },
      pwm2: { color: '#aed581', label: 'PWM2', data: [] },
    };
    this.visible = { target1: true, actual1: true, target2: true, actual2: true, pwm1: false, pwm2: false };
  }

  windowSec() {
    return parseFloat(document.getElementById('chartWindowInput').value) || 20;
  }

  push(sample) {
    const now = performance.now() / 1000;
    if (this.t0 === null) this.t0 = now;
    const t = now - this.t0;
    for (const key of Object.keys(this.series)) {
      if (sample[key] !== undefined && sample[key] !== null) {
        this.series[key].data.push([t, sample[key]]);
      }
    }
    const cutoff = t - this.windowSec();
    for (const s of Object.values(this.series)) {
      while (s.data.length && s.data[0][0] < cutoff) s.data.shift();
    }
    this.draw();
  }

  draw() {
    const { ctx, canvas } = this;
    const w = canvas.width = canvas.clientWidth;
    const h = canvas.height = canvas.clientHeight;
    ctx.clearRect(0, 0, w, h);

    const visibleSeries = Object.entries(this.series).filter(([k]) => this.visible[k]).map(([, s]) => s).filter(s => s.data.length);
    if (!visibleSeries.length) return;

    let minY = Infinity, maxY = -Infinity, minX = Infinity, maxX = -Infinity;
    for (const s of visibleSeries) {
      for (const [x, y] of s.data) {
        if (y < minY) minY = y;
        if (y > maxY) maxY = y;
        if (x < minX) minX = x;
        if (x > maxX) maxX = x;
      }
    }
    if (minY === maxY) { minY -= 1; maxY += 1; }
    const padY = (maxY - minY) * 0.1;
    minY -= padY; maxY += padY;
    if (maxX === minX) maxX = minX + 1;

    const marginL = 42, marginB = 6, marginT = 6, marginR = 6;
    const plotW = w - marginL - marginR;
    const plotH = h - marginT - marginB;
    const xToPx = x => marginL + ((x - minX) / (maxX - minX)) * plotW;
    const yToPx = y => marginT + plotH - ((y - minY) / (maxY - minY)) * plotH;

    ctx.strokeStyle = 'rgba(255,255,255,0.08)';
    ctx.fillStyle = 'rgba(255,255,255,0.45)';
    ctx.font = '10px monospace';
    const ySteps = 4;
    for (let i = 0; i <= ySteps; i++) {
      const y = minY + (maxY - minY) * i / ySteps;
      const py = yToPx(y);
      ctx.beginPath(); ctx.moveTo(marginL, py); ctx.lineTo(w - marginR, py); ctx.stroke();
      ctx.fillText(y.toFixed(0), 2, py + 3);
    }

    for (const s of visibleSeries) {
      ctx.strokeStyle = s.color;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      s.data.forEach(([x, y], i) => {
        const px = xToPx(x), py = yToPx(y);
        if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
      });
      ctx.stroke();
    }
  }
}

const chart = new LiveChart(document.getElementById('pidChart'));

document.querySelectorAll('#chartLegend input[type=checkbox]').forEach(cb => {
  cb.addEventListener('change', () => { chart.visible[cb.dataset.series] = cb.checked; chart.draw(); });
});

/* ---------- PID step test ---------- */

const stepTest = {
  timer: null,
  start() {
    this.stop();
    const r1 = parseFloat(document.getElementById('stepRpm1').value) || 0;
    const r2 = parseFloat(document.getElementById('stepRpm2').value) || 0;
    const duration = parseFloat(document.getElementById('stepDuration').value) || 3;
    logLine('info', `step test: rpm=(${r1}, ${r2}) for ${duration}s`);
    sendCommand(`m ${r1} ${r2}`);
    this.timer = setTimeout(() => { sendCommand('m 0 0'); logLine('info', 'step test done'); }, duration * 1000);
  },
  stop() {
    if (this.timer) { clearTimeout(this.timer); this.timer = null; }
  },
};
document.getElementById('runStepTestBtn').addEventListener('click', () => stepTest.start());
document.getElementById('stopStepTestBtn').addEventListener('click', () => { stepTest.stop(); sendCommand('m 0 0'); });

/* ---------- PID sine test ---------- */

const sineTest = {
  timer: null,
  stopTimeout: null,
  sendInFlight: false,
  start() {
    this.stop();
    const amp = parseFloat(document.getElementById('sineAmplitude').value) || 100;
    const period = parseFloat(document.getElementById('sinePeriod').value) || 4;
    const duration = parseFloat(document.getElementById('sineDuration').value) || 10;
    logLine('info', `sine test: amp=${amp} period=${period}s for ${duration}s`);
    const start = performance.now();
    this.timer = setInterval(() => {
      if (this.sendInFlight) return; // see joystickTick: never let sends queue up
      const t = (performance.now() - start) / 1000;
      const rpm = Math.round(amp * Math.sin((2 * Math.PI * t) / period));
      this.sendInFlight = true;
      motorAbortController = new AbortController();
      sendCommand(`m ${rpm} ${rpm}`, { log: false, signal: motorAbortController.signal })
        .finally(() => { this.sendInFlight = false; });
      document.getElementById('sineReadout').textContent = `${rpm} / ${rpm}`;
    }, 100);
    this.stopTimeout = setTimeout(() => { this.stop(); sendCommand('m 0 0'); logLine('info', 'sine test done'); }, duration * 1000);
  },
  stop() {
    if (this.timer) { clearInterval(this.timer); this.timer = null; }
    if (this.stopTimeout) { clearTimeout(this.stopTimeout); this.stopTimeout = null; }
  },
};
document.getElementById('runSineTestBtn').addEventListener('click', () => sineTest.start());
document.getElementById('stopSineTestBtn').addEventListener('click', () => { sineTest.stop(); sendCommand('m 0 0'); });

/* ---------- PID autotune (relay feedback) ---------- */

const atChart = new LiveChart(document.getElementById('atChart'));
atChart.series = {
  rpm1: { color: '#4fc3f7', label: 'RPM1', data: [] },
  rpm2: { color: '#ffb74d', label: 'RPM2', data: [] },
  pwm: { color: '#81c784', label: 'PWM', data: [] },
};
atChart.visible = { rpm1: true, rpm2: true, pwm: true };
// Fed directly via renderAtTail()/draw(), not push() -- draws the whole
// run's data every time rather than trimming to a rolling time window.

function renderAtTail(tail) {
  atChart.series.rpm1.data = tail.map(s => [s[0], s[1]]);
  atChart.series.rpm2.data = tail.map(s => [s[0], s[2]]);
  atChart.series.pwm.data = tail.map(s => [s[0], s[3]]);
  atChart.draw();
}

let atLogSource = null;
let atPollTimer = null;

function renderAtResult(result) {
  const box = document.getElementById('atResults');
  if (!result) { box.style.display = 'none'; return; }
  box.style.display = '';
  document.getElementById('atIdentified').textContent =
    `Ku=${result.ku.toFixed(4)} pwm/rpm, Pu=${result.pu.toFixed(3)}s, amplitude=${result.amplitude.toFixed(2)} rpm (${result.num_periods} periods)`;
  const zn = result.ziegler_nichols, tl = result.tyreus_luyben;
  document.getElementById('atZnValues').textContent =
    `Kp=${zn.kp.toFixed(4)} Ki=${zn.ki.toFixed(4)} Kd=${zn.kd.toFixed(5)}`;
  document.getElementById('atTlValues').textContent =
    `Kp=${tl.kp.toFixed(4)} Ki=${tl.ki.toFixed(4)} Kd=${tl.kd.toFixed(5)}`;
  document.getElementById('atUseZn').onclick = () => applyAtGains(zn, result.ko);
  document.getElementById('atUseTl').onclick = () => applyAtGains(tl, result.ko);
  document.getElementById('atFlashZn').onclick = () => saveAndFlashGains(zn, result.ko);
  document.getElementById('atFlashTl').onclick = () => saveAndFlashGains(tl, result.ko);

  const appliedEl = document.getElementById('atAppliedStatus');
  if (result.applied_live === true) {
    appliedEl.textContent = 'Tyreus-Luyben gains are live on the board right now (RAM only -- lost on reset unless you Save & Flash).';
  } else if (result.applied_live === false) {
    appliedEl.textContent = 'Identified successfully, but applying live failed -- check the log above.';
  } else {
    appliedEl.textContent = '';
  }

  if (result.samples && result.samples.length) renderAtTail(result.samples);
}

async function saveAndFlashGains(gains, ko) {
  const sketch = document.getElementById('sketchSelect').value;
  const fqbn = document.getElementById('fqbnSelect').value;
  const port = document.getElementById('portSelect').value || document.getElementById('portManual').value;
  if (!sketch) { logLine('error', 'pick a sketch in the Upload Firmware panel first'); return; }

  const ok = confirm(
    `This will overwrite the PID default constants in ${sketch} on disk ` +
    `(Kp=${gains.kp.toFixed(4)}, Kd=${gains.kd.toFixed(5)}, Ki=${gains.ki.toFixed(4)}, Ko=${ko}), then compile and flash it. Continue?`);
  if (!ok) return;

  const res = await apiPost('/api/save_pid', {
    sketch, kp: gains.kp, kd: gains.kd, ki: gains.ki, ko,
  });
  if (res.error) { logLine('error', 'save PID failed: ' + res.error); return; }
  logLine('info', `saved to ${sketch}: ${res.line}`);
  startUpload(sketch, fqbn, port, {
    onDone: () => logLine('info', 'firmware now boots with the autotuned PID gains'),
  });
}

function applyAtGains(gains, ko) {
  document.getElementById('kpNum').value = gains.kp.toFixed(4);
  document.getElementById('kpSlider').value = gains.kp.toFixed(4);
  document.getElementById('kdNum').value = gains.kd.toFixed(5);
  document.getElementById('kdSlider').value = gains.kd.toFixed(5);
  document.getElementById('kiNum').value = gains.ki.toFixed(4);
  document.getElementById('kiSlider').value = gains.ki.toFixed(4);
  document.getElementById('koNum').value = ko.toFixed(2);
  document.getElementById('koSlider').value = ko.toFixed(2);
  logLine('info', `Loaded into PID Tuning: Kp=${gains.kp.toFixed(4)} Kd=${gains.kd.toFixed(5)} Ki=${gains.ki.toFixed(4)} Ko=${ko}`);
}

function setAtRunning(running) {
  document.getElementById('atStartBtn').disabled = running || !connected;
  document.getElementById('atStopBtn').disabled = !running;
}

async function pollAtStatus() {
  try {
    const status = await apiGet('/api/autotune/status');
    document.getElementById('atStatus').textContent = status.error ? `error: ${status.error}` : status.phase;
    if (status.live_tail && status.live_tail.length) renderAtTail(status.live_tail);
    if (!status.running) {
      setAtRunning(false);
      clearInterval(atPollTimer);
      atPollTimer = null;
      renderAtResult(status.result);
    }
  } catch (e) { /* transient -- next tick retries */ }
}

document.getElementById('atStartBtn').addEventListener('click', async () => {
  document.getElementById('atLog').textContent = '';
  document.getElementById('atResults').style.display = 'none';
  const body = {
    setpoint_rpm: parseFloat(document.getElementById('atSetpoint').value) || 100,
    relay_pwm: parseFloat(document.getElementById('atRelayPwm').value) || 80,
    duration_s: parseFloat(document.getElementById('atDuration').value) || 10,
  };
  const res = await apiPost('/api/autotune/start', body);
  if (res.error) { logLine('error', 'autotune failed to start: ' + res.error); return; }
  setAtRunning(true);
  logLine('info', 'autotune started -- this holds the serial link exclusively until it finishes.');

  if (atLogSource) atLogSource.close();
  const atLog = document.getElementById('atLog');
  atLogSource = new EventSource('/api/autotune/log/stream');
  atLogSource.onmessage = (e) => { atLog.textContent += e.data + '\n'; atLog.scrollTop = atLog.scrollHeight; };
  atLogSource.addEventListener('done', () => { atLogSource.close(); atLogSource = null; });
  atLogSource.onerror = () => { if (atLogSource) { atLogSource.close(); atLogSource = null; } };

  if (atPollTimer) clearInterval(atPollTimer);
  atPollTimer = setInterval(pollAtStatus, 300);
});

document.getElementById('atStopBtn').addEventListener('click', async () => {
  await apiPost('/api/autotune/stop', {});
  logLine('info', 'autotune abort requested.');
});

/* ---------- joystick (Gamepad API) ---------- */

const joystickState = { enabled: false, gamepadIndex: null };
let joystickTimer = null;
let joystickSendInFlight = false;

// The 'gamepadconnected' event is unreliable for some Bluetooth
// controllers depending on OS/browser/driver -- rather than depending on
// it firing, joystickTick() polls navigator.getGamepads() every tick and
// adopts any connected pad it finds, which works regardless of whether
// the event ever fires. This listener is just for the earlier UI update;
// detection itself no longer depends on it.
window.addEventListener('gamepadconnected', (e) => {
  joystickState.gamepadIndex = e.gamepad.index;
  document.getElementById('gamepadName').textContent = e.gamepad.id;
  logLine('info', `gamepad connected: ${e.gamepad.id}`);
});
window.addEventListener('gamepaddisconnected', (e) => {
  if (joystickState.gamepadIndex !== e.gamepad.index) return;
  document.getElementById('gamepadName').textContent = '(none)';
  joystickState.enabled = false;
  joystickState.gamepadIndex = null;
  document.getElementById('joyEnable').checked = false;
  sendCommand('m 0 0');
  sendCmdVel(0, 0, { log: false });
  logLine('info', 'gamepad disconnected -- motors stopped');
});

function scanForGamepad() {
  if (joystickState.gamepadIndex !== null) return;
  const pads = navigator.getGamepads ? navigator.getGamepads() : [];
  for (const gp of pads) {
    if (gp) {
      joystickState.gamepadIndex = gp.index;
      document.getElementById('gamepadName').textContent = gp.id;
      logLine('info', `gamepad detected: ${gp.id}`);
      return;
    }
  }
}

document.getElementById('joyEnable').addEventListener('change', (e) => {
  joystickState.enabled = e.target.checked;
  if (!joystickState.enabled) sendCommand('m 0 0');
});

function drawStick(x, y) {
  const canvas = document.getElementById('stickCanvas');
  const ctx = canvas.getContext('2d');
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h);
  ctx.strokeStyle = 'rgba(255,255,255,0.2)';
  ctx.beginPath(); ctx.moveTo(w / 2, 0); ctx.lineTo(w / 2, h); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(0, h / 2); ctx.lineTo(w, h / 2); ctx.stroke();
  const px = w / 2 + (x * w / 2 * 0.9);
  const py = h / 2 + (y * h / 2 * 0.9);
  ctx.fillStyle = '#4fc3f7';
  ctx.beginPath(); ctx.arc(px, py, 6, 0, 2 * Math.PI); ctx.fill();
}

function applyDeadzone(v, dz = 0.08) {
  return Math.abs(v) < dz ? 0 : v;
}

// On-screen virtual joystick: drag directly on the stick canvas. Pointer
// Events unify mouse and touch, so this works on a laptop trackpad/mouse
// or a tablet/phone touchscreen. Self-centers to (0,0) on release --
// critical, since a stick that doesn't return to center on release would
// leave the robot driving after you let go.
const virtualStick = { x: 0, y: 0, dragging: false };

(function setupVirtualStick() {
  const canvas = document.getElementById('stickCanvas');
  const posFromEvent = (e) => {
    const rect = canvas.getBoundingClientRect();
    const x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    const y = ((e.clientY - rect.top) / rect.height) * 2 - 1;
    const mag = Math.hypot(x, y);
    const clampedMag = Math.min(mag, 1);
    const scale = mag > 0 ? clampedMag / mag : 0;
    return { x: x * scale, y: y * scale };
  };
  canvas.addEventListener('pointerdown', (e) => {
    if (document.getElementById('inputSourceOnscreen').checked === false) return;
    canvas.setPointerCapture(e.pointerId);
    virtualStick.dragging = true;
    Object.assign(virtualStick, posFromEvent(e));
  });
  canvas.addEventListener('pointermove', (e) => {
    if (!virtualStick.dragging) return;
    Object.assign(virtualStick, posFromEvent(e));
  });
  const release = () => {
    virtualStick.dragging = false;
    virtualStick.x = 0;
    virtualStick.y = 0;
  };
  canvas.addEventListener('pointerup', release);
  canvas.addEventListener('pointercancel', release);
})();

document.querySelectorAll('input[name=inputSource]').forEach(radio => {
  radio.addEventListener('change', () => {
    const isOnscreen = document.getElementById('inputSourceOnscreen').checked;
    document.getElementById('gamepadAxisConfig').style.display = isOnscreen ? 'none' : '';
    document.getElementById('stickCanvas').style.cursor = isOnscreen ? 'pointer' : 'default';
  });
});

function joystickTick() {
  const useOnscreen = document.getElementById('inputSourceOnscreen').checked;
  let throttle, turn;

  if (useOnscreen) {
    throttle = applyDeadzone(-virtualStick.y);
    turn = applyDeadzone(virtualStick.x);
  } else {
    scanForGamepad();
    const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
    const gp = joystickState.gamepadIndex !== null ? gamepads[joystickState.gamepadIndex] : null;
    if (!gp) return;

    const throttleAxis = parseInt(document.getElementById('throttleAxisSelect').value, 10);
    const turnAxis = parseInt(document.getElementById('turnAxisSelect').value, 10);
    const invertThrottle = document.getElementById('invertThrottle').checked;

    for (let i = 0; i < Math.min(gp.axes.length, 4); i++) {
      const bar = document.getElementById('axisBar' + i);
      if (bar) bar.style.width = (50 + gp.axes[i] * 50) + '%';
    }

    throttle = applyDeadzone(gp.axes[throttleAxis] || 0);
    turn = applyDeadzone(gp.axes[turnAxis] || 0);
    if (invertThrottle) throttle = -throttle;
  }

  drawStick(turn, -throttle);

  if (!joystickState.enabled) return;

  // Never let sends pile up behind a slow round-trip -- skipping a tick is
  // harmless (another follows in 100ms), but an unbounded backlog means a
  // STOP click can get buried behind several already-in-flight commands.
  if (joystickSendInFlight) return;

  const mode = document.querySelector('input[name=joyMode]:checked').value;

  if (mode === 'serial') {
    if (!connected) return;
    let left = throttle + turn;
    let right = throttle - turn;
    const mag = Math.max(1, Math.abs(left), Math.abs(right));
    left /= mag; right /= mag;
    const maxRpm = parseInt(document.getElementById('joyMaxRpm').value, 10) || 200;
    const rpm1 = Math.round(left * maxRpm);
    const rpm2 = Math.round(right * maxRpm);
    document.getElementById('joyReadout').textContent = `${rpm1} / ${rpm2}`;
    joystickSendInFlight = true;
    motorAbortController = new AbortController();
    sendCommand(`m ${rpm1} ${rpm2}`, {
      log: document.getElementById('logJoystick').checked,
      signal: motorAbortController.signal,
    }).finally(() => { joystickSendInFlight = false; });
  } else {
    if (!ros2Running) return;
    const maxLinear = parseFloat(document.getElementById('joyMaxLinear').value) || 0.2;
    const maxAngular = parseFloat(document.getElementById('joyMaxAngular').value) || 1.5;
    const linearX = throttle * maxLinear;
    const angularZ = turn * maxAngular; // REP103: +turn (left stick left / left axis) => +angular.z (left turn)
    document.getElementById('joyReadout').textContent = `${linearX.toFixed(2)} m/s / ${angularZ.toFixed(2)} rad/s`;
    joystickSendInFlight = true;
    motorAbortController = new AbortController();
    sendCmdVel(linearX, angularZ, {
      log: document.getElementById('logJoystick').checked,
      signal: motorAbortController.signal,
    }).finally(() => { joystickSendInFlight = false; });
  }
}

document.querySelectorAll('input[name=joyMode]').forEach(radio => {
  radio.addEventListener('change', () => {
    const isRos2 = document.getElementById('joyModeRos2').checked;
    document.getElementById('joySerialParams').style.display = isRos2 ? 'none' : '';
    document.getElementById('joyRos2Params').style.display = isRos2 ? '' : 'none';
  });
});

joystickTimer = setInterval(joystickTick, 100);

window.addEventListener('beforeunload', () => {
  if (connected) {
    navigator.sendBeacon('/api/command', new Blob([JSON.stringify({ cmd: 'm 0 0' })], { type: 'application/json' }));
  }
});

/* ---------- ROS2 mode: launch/stop the real control stack ---------- */

let ros2Running = false;
let ros2LogSource = null;

document.getElementById('ros2LaunchFile').addEventListener('change', (e) => {
  const isSensors = e.target.value === 'robot_jambot_with_sensors.launch.py';
  document.getElementById('ros2ArgsCore').style.display = isSensors ? 'none' : '';
  document.getElementById('ros2ArgsSensors').style.display = isSensors ? '' : 'none';
});

function collectRos2Args() {
  const launchFile = document.getElementById('ros2LaunchFile').value;
  const args = [];
  if (launchFile === 'robot_jambot.launch.py') {
    args.push(`enable_ekf:=${document.getElementById('ros2EnableEkf').checked}`);
  } else {
    args.push(`enable_control:=${document.getElementById('ros2EnableControl').checked}`);
    args.push(`enable_rviz:=${document.getElementById('ros2EnableRviz').checked}`);
    args.push(`enable_slam:=${document.getElementById('ros2EnableSlam').checked}`);
    args.push(`enable_camera:=${document.getElementById('ros2EnableCamera').checked}`);
  }
  return { launchFile, args };
}

function updateRos2Status(status) {
  ros2Running = status.running;
  document.getElementById('ros2Status').textContent = status.running
    ? `running (${status.launch_file})` : 'stopped';
  document.getElementById('ros2LaunchBtn').disabled = status.running;
  document.getElementById('ros2StopBtn').disabled = !status.running;
  // ros2_control owns the serial port while running -- direct connect would corrupt both sides.
  document.getElementById('connectBtn').disabled = status.running || connected;
}

async function pollRos2Status() {
  try {
    updateRos2Status(await apiGet('/api/ros2/status'));
  } catch (e) { /* transient -- next tick retries */ }
}
setInterval(pollRos2Status, 2000);
pollRos2Status();

document.getElementById('ros2LaunchBtn').addEventListener('click', async () => {
  const { launchFile, args } = collectRos2Args();
  const ros2Log = document.getElementById('ros2Log');
  ros2Log.textContent = '';
  const res = await apiPost('/api/ros2/start', { launch_file: launchFile, args });
  if (res.error) {
    logLine('error', 'ROS2 launch failed: ' + res.error);
    return;
  }
  logLine('info', `launching ${launchFile}...`);
  updateRos2Status(res);
  setConnected(false); // launching auto-disconnected the direct serial link server-side

  if (ros2LogSource) ros2LogSource.close();
  ros2LogSource = new EventSource('/api/ros2/log/stream');
  ros2LogSource.onmessage = (e) => {
    ros2Log.textContent += e.data + '\n';
    ros2Log.scrollTop = ros2Log.scrollHeight;
  };
  ros2LogSource.addEventListener('done', () => { ros2LogSource.close(); ros2LogSource = null; pollRos2Status(); });
  ros2LogSource.onerror = () => { if (ros2LogSource) { ros2LogSource.close(); ros2LogSource = null; } };
});

document.getElementById('ros2StopBtn').addEventListener('click', async () => {
  logLine('info', 'stopping ROS2 stack...');
  sendCmdVel(0, 0, { log: false });
  const res = await apiPost('/api/ros2/stop', {});
  updateRos2Status(res);
});

/* ---------- manual cmd_vel ---------- */

wireSliderNumberPair('cmdVelLinearSlider', 'cmdVelLinearNum');
wireSliderNumberPair('cmdVelAngularSlider', 'cmdVelAngularNum');

document.getElementById('cmdVelApplyBtn').addEventListener('click', () => {
  sendCmdVel(
    parseFloat(document.getElementById('cmdVelLinearNum').value) || 0,
    parseFloat(document.getElementById('cmdVelAngularNum').value) || 0);
});
document.getElementById('cmdVelStopBtn').addEventListener('click', () => {
  document.getElementById('cmdVelLinearNum').value = 0; document.getElementById('cmdVelLinearSlider').value = 0;
  document.getElementById('cmdVelAngularNum').value = 0; document.getElementById('cmdVelAngularSlider').value = 0;
  sendCmdVel(0, 0);
});

/* ---------- lidar (independent of the Arduino connection) ---------- */

function drawLidar(data) {
  const canvas = document.getElementById('lidarCanvas');
  const ctx = canvas.getContext('2d');
  const w = canvas.width = canvas.clientWidth;
  const h = canvas.height = canvas.clientHeight;
  ctx.clearRect(0, 0, w, h);
  const cx = w / 2, cy = h / 2;
  const maxRange = parseFloat(document.getElementById('lidarMaxRange').value) || 6;
  const scale = (Math.min(w, h) / 2) * 0.9 / maxRange;

  ctx.strokeStyle = 'rgba(255,255,255,0.15)';
  ctx.fillStyle = 'rgba(255,255,255,0.4)';
  ctx.font = '10px monospace';
  for (let r = 1; r <= maxRange; r++) {
    ctx.beginPath();
    ctx.arc(cx, cy, r * scale, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.fillText(r + 'm', cx + 3, cy - r * scale);
  }
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx, 4); ctx.stroke();
  ctx.fillText('fwd', cx + 4, 12);

  if (data && data.available && data.ranges) {
    ctx.fillStyle = '#4fc3f7';
    const { angle_min: angleMin, angle_increment: inc, range_min: rangeMin } = data;
    data.ranges.forEach((r, i) => {
      if (r === null || r < rangeMin || r > maxRange) return;
      const angle = angleMin + i * inc;
      // REP103: angle 0 = forward (+x), positive = left (+y). Forward is
      // drawn up, left is drawn toward screen-left, matching a bird's-eye
      // view looking the same direction as the robot.
      const px = cx - r * Math.sin(angle) * scale;
      const py = cy - r * Math.cos(angle) * scale;
      ctx.beginPath(); ctx.arc(px, py, 2, 0, 2 * Math.PI); ctx.fill();
    });
  }

  ctx.fillStyle = '#ef5350';
  ctx.beginPath(); ctx.arc(cx, cy, 4, 0, 2 * Math.PI); ctx.fill();
}

async function pollLidar() {
  try {
    const data = await apiGet('/api/lidar');
    const status = document.getElementById('lidarStatus');
    if (!data.available) {
      status.textContent = data.error || 'no data';
    } else {
      status.textContent = data.stale ? 'stale -- driver may have stopped' : `live (${data.ranges.length} pts)`;
    }
    drawLidar(data);
  } catch (e) {
    // transient poll failure -- next tick retries
  }
}
setInterval(pollLidar, 200); // independent of Arduino connect state -- lidar is a separate ROS2 topic

/* ---------- firmware upload ---------- */

async function loadUploadOptions() {
  const sketches = await apiGet('/api/sketches');
  const sketchSel = document.getElementById('sketchSelect');
  sketchSel.innerHTML = '';
  for (const s of sketches.sketches) {
    const opt = document.createElement('option'); opt.value = s; opt.textContent = s;
    sketchSel.appendChild(opt);
  }
  const fqbns = await apiGet('/api/fqbns');
  const fqbnSel = document.getElementById('fqbnSelect');
  fqbnSel.innerHTML = '';
  for (const [value, label] of Object.entries(fqbns)) {
    const opt = document.createElement('option'); opt.value = value; opt.textContent = label;
    fqbnSel.appendChild(opt);
  }
}

function startUpload(sketch, fqbn, port, { onDone } = {}) {
  if (!sketch || !fqbn || !port) {
    logLine('error', 'pick a sketch, board, and port before uploading');
    return;
  }
  const uploadLog = document.getElementById('uploadLog');
  uploadLog.textContent = '';
  document.getElementById('uploadBtn').disabled = true;
  document.getElementById('savePidFlashBtn').disabled = true;

  const url = `/api/upload/stream?sketch=${encodeURIComponent(sketch)}&fqbn=${encodeURIComponent(fqbn)}&port=${encodeURIComponent(port)}`;
  const es = new EventSource(url);
  es.onmessage = (e) => {
    uploadLog.textContent += e.data + '\n';
    uploadLog.scrollTop = uploadLog.scrollHeight;
  };
  es.addEventListener('done', () => {
    es.close();
    document.getElementById('uploadBtn').disabled = false;
    document.getElementById('savePidFlashBtn').disabled = false;
    logLine('info', 'upload finished -- check the upload log panel');
    if (onDone) onDone();
  });
  es.onerror = () => {
    es.close();
    document.getElementById('uploadBtn').disabled = false;
    document.getElementById('savePidFlashBtn').disabled = false;
  };
}

document.getElementById('uploadBtn').addEventListener('click', () => {
  const sketch = document.getElementById('sketchSelect').value;
  const fqbn = document.getElementById('fqbnSelect').value;
  const port = document.getElementById('portSelect').value || document.getElementById('portManual').value;
  startUpload(sketch, fqbn, port);
});

/* ---------- save tuned PID gains into the sketch, then reflash ---------- */

document.getElementById('savePidFlashBtn').addEventListener('click', () => {
  const gains = {
    kp: parseFloat(document.getElementById('kpNum').value) || 0,
    kd: parseFloat(document.getElementById('kdNum').value) || 0,
    ki: parseFloat(document.getElementById('kiNum').value) || 0,
  };
  const ko = parseFloat(document.getElementById('koNum').value) || 1.0;
  saveAndFlashGains(gains, ko);
});

/* ---------- init ---------- */

(async function init() {
  await refreshPorts();
  await loadUploadOptions();
  const status = await apiGet('/api/status');
  if (status.connected) {
    setConnected(true, status.port, status.baud);
  } else {
    setConnected(false);
  }
  requestAnimationFrame(function raf() { chart.draw(); requestAnimationFrame(raf); });
})();
