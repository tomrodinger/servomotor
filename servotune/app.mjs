import { COMMANDS as C, ServoClient, WebSerialTransport, MotorError, parseAddress, firmwareCapabilities } from './protocol.mjs';
import { DemoTransport } from './demo.mjs';
import { scopeTrace } from './scope.mjs';

const $ = id => document.getElementById(id);
let client = null, connecting = false, demo = false, immediateSupported = false, pidSupported = false;
let countsPerRotation = 3276800;
let scopeGeneration = 0, motionGeneration = 0, samples = [];
let scopeRunning = false, motionRunning = false;
let transactionTimer = null;
const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));

function notice(text, error = false) { $('notice').textContent = text; $('notice').className = error ? 'error' : ''; }
function controls() {
  for (const element of document.querySelectorAll('[data-connected]')) element.disabled = !client;
  for (const element of document.querySelectorAll('[data-target]')) element.disabled = !client || !immediateSupported;
  $('read-pid').disabled = $('write-pid').disabled = !client || !pidSupported;
  $('demo').disabled = $('connect').disabled = $('address').disabled = !!client || connecting;
  $('disconnect').disabled = !client;
  $('scope-start').disabled = !client || scopeRunning;
  $('scope-stop').disabled = !client || !scopeRunning;
  $('motion-start').disabled = !client || !immediateSupported || motionRunning;
  $('motion-stop').disabled = !client || !motionRunning;
}
function showStatus(status) {
  $('status').textContent = status.error ? `Fatal error ${status.error} · reset required` :
    `${status.flags & 2 ? 'Outputs enabled' : 'Outputs disabled'} · ${status.flags & 4 ? 'Closed-loop position mode' : 'Open-loop mode'}`;
}
function stopTransactionStats() {
  clearInterval(transactionTimer); transactionTimer = null;
  $('transaction-rate').textContent = '0.0';
  $('transaction-time').textContent = '—';
}
function startTransactionStats(link) {
  stopTransactionStats();
  let previousCount = link.completed, previousTime = performance.now();
  $('transactions').textContent = String(previousCount);
  $('transaction-rate').textContent = '—';
  transactionTimer = setInterval(() => {
    if (client !== link) return;
    const now = performance.now(), completed = link.completed;
    const count = completed - previousCount, elapsed = now - previousTime;
    $('transactions').textContent = String(completed);
    $('transaction-rate').textContent = (count * 1000 / elapsed).toFixed(1);
    $('transaction-time').textContent = count ? `${(elapsed / count).toFixed(2)} ms` : '—';
    previousCount = completed; previousTime = now;
  }, 1000);
}
function sample(state) {
  const now = performance.now();
  samples.push({ t: now, measured: Number(state.measured) * 360 / countsPerRotation,
    target: Number(state.target) * 360 / countsPerRotation });
  samples = samples.filter(s => s.t >= now - 5500);
  const rate = samples.length > 1 ? (samples.length - 1) * 1000 / (now - samples[0].t) : 0;
  $('sample-count').textContent = `${samples.length} samples in view · ${rate.toFixed(0)} samples/s${demo ? ' · simulated' : ''}`;
  draw();
}
function showState(state) {
  const target = Number(state.target) * 360 / countsPerRotation;
  const measured = Number(state.measured) * 360 / countsPerRotation;
  $('state-target').textContent = `${target.toFixed(3)}°`;
  $('state-measured').textContent = `${measured.toFixed(3)}°`;
  $('state-error').textContent = `${(Number(state.target - state.measured) * 360 / countsPerRotation).toFixed(3)}°`;
  if (scopeRunning) sample(state);
}
function draw() {
  const canvas = $('scope'), ratio = devicePixelRatio || 1;
  canvas.width = Math.round(canvas.clientWidth * ratio); canvas.height = Math.round(canvas.clientHeight * ratio);
  const ctx = canvas.getContext('2d'), w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h);
  const now = performance.now();
  const scale = Math.max(1, ...samples.flatMap(s => [Math.abs(s.target), Math.abs(s.measured)])) * 1.15;
  ctx.strokeStyle = '#53616c'; ctx.lineWidth = ratio;
  ctx.beginPath(); ctx.moveTo(0, h / 2); ctx.lineTo(w, h / 2); ctx.stroke();
  ctx.fillStyle = '#dce5eb'; ctx.font = `${12 * ratio}px sans-serif`; ctx.fillText(`±${scale.toFixed(2)}°`, 10 * ratio, 20 * ratio);
  for (const [key, color] of [['target', '#72dc99'], ['measured', '#75ceff']]) {
    ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = 2 * ratio;
    scopeTrace(samples, key).forEach(point => {
      const x = w * (1 - (now - point.t) / 5000), y = h / 2 - (point.value / scale) * h * .45;
      if (point.move) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }); ctx.stroke();
    // A single snapshot, or points separated by a long capture gap, must still
    // be visible without drawing a fictitious response between them.
    ctx.fillStyle = color;
    for (const s of samples) {
      ctx.fillRect(w * (1 - (now - s.t) / 5000) - ratio,
        h / 2 - (s[key] / scale) * h * .45 - ratio, 2 * ratio, 2 * ratio);
    }
  }
}
async function readPid() {
  const gains = await client.pid();
  ['p', 'i', 'd'].forEach((id, n) => { $(id).value = String(gains[n]); $(id + '-range').value = String(gains[n]); });
  notice('PID constants read from ' + (demo ? 'the demo motor.' : 'the motor.'));
}
async function inspectConnection() {
  const version = await client.request(C.VERSION, undefined, 5);
  if (version[4]) throw new Error('Device is in its bootloader; wait for firmware startup before connecting');
  const numeric = [version[3], version[2], version[1], version[0]];
  const capabilities = firmwareCapabilities(numeric);
  immediateSupported = capabilities.immediate; pidSupported = capabilities.pid;
  client.compactTargetPosition = capabilities.compactTargetPosition;
  const specs = await client.request(C.SPECS, undefined, 8), view = new DataView(specs.buffer);
  const frequency = view.getUint32(0, true); countsPerRotation = view.getUint32(4, true);
  if (!frequency || !countsPerRotation) throw new Error('Invalid product specifications');
  if (pidSupported) await readPid();
  showStatus(await client.status());
  $('banner').className = demo ? 'demo' : '';
  $('banner').textContent = `${demo ? 'DEMO — simulated motor, no hardware connected' : 'Serial motor connected'} · Firmware ${numeric.join('.')}${immediateSupported ? '' : ' · Immediate targets unavailable on this firmware'}`;
}
async function connect(useDemo) {
  if (client || connecting) return;
  const address = parseAddress($('address').value);
  connecting = true;
  $('demo').disabled = $('connect').disabled = true;
  let transport;
  try {
    transport = useDemo ? new DemoTransport({ compactTargetPosition: startupParameters.get('demo-legacy') !== '1' }) : await WebSerialTransport.open();
    demo = useDemo; client = new ServoClient(transport, address);
    startTransactionStats(client);
    samples = []; $('sample-count').textContent = '0 samples'; draw();
    await inspectConnection();
  } catch (error) {
    stopTransactionStats();
    if (client) await client.close().catch(() => {});
    else if (transport) await transport.close().catch(() => {});
    client = null; throw error;
  } finally {
    connecting = false; controls();
  }
}
async function disconnect() {
  stopScope(); stopMotion();
  stopTransactionStats();
  if (client) $('transactions').textContent = String(client.completed);
  const old = client; client = null; controls();
  if (old) await old.close();
  $('status').textContent = 'Not connected'; $('banner').className = '';
  $('banner').textContent = 'Disconnected. Demo is available without a motor.';
}
function targetCounts(degrees) {
  if (degrees === undefined) {
    if (!$('target').value.trim()) throw new Error('Enter a target angle');
    degrees = Number($('target').value);
  }
  if (!Number.isFinite(degrees)) throw new Error('Enter a finite target angle');
  const counts = Math.round(degrees * countsPerRotation / 360);
  if (!Number.isSafeInteger(counts)) throw new Error('Target is too large for an exact degree conversion');
  return BigInt(counts);
}
async function sendTarget(withPosition, degrees) {
  if (!immediateSupported) throw new Error('This firmware does not support immediate targets');
  const position = targetCounts(degrees), positionReply = await client.target(position, withPosition);
  if (positionReply) showState(positionReply);
  notice(withPosition ? 'Target applied; measured position received.' : 'Target applied.');
}
function stopScope() { scopeGeneration++; scopeRunning = false; }
function stopMotion() { motionGeneration++; motionRunning = false; }
function reportError(error) {
  stopScope(); stopMotion();
  if (error instanceof MotorError) showStatus({ flags: 0, error: error.code });
  notice(error.message, true); controls();
}
async function startScope() {
  if (scopeRunning) return;
  const generation = ++scopeGeneration, link = client;
  scopeRunning = true; controls();
  try {
    while (client === link && generation === scopeGeneration) {
      const state = await link.positions();
      if (client !== link || generation !== scopeGeneration) break;
      sample(state);
      await sleep(5); // Schedule after the complete reply, never before it.
    }
  } catch (error) {
    // Stop/reset/disconnect may cancel a queued poll. It must not stop a newer
    // capture or overwrite the result of the user's stop/reset action.
    if (client === link && generation === scopeGeneration) throw error;
  } finally {
    if (generation === scopeGeneration) { scopeRunning = false; controls(); }
  }
}
function action(id, callback) {
  $(id).addEventListener('click', async () => {
    const element = $(id); element.disabled = true;
    try { await callback(); }
    catch (error) { reportError(error); }
    finally { controls(); }
  });
}
action('demo', () => connect(true));
action('connect', () => connect(false));
action('disconnect', disconnect);
action('read-pid', readPid);
action('write-pid', async () => {
  const gains = ['p', 'i', 'd'].map(id => {
    if (!$(id).value.trim()) throw new Error('Enter all three PID constants');
    return Number($(id).value);
  });
  await client.setPid(...gains); await readPid();
});
action('get-status', async () => showStatus(await client.status()));
action('closed-loop', async () => { await client.request(C.CLOSED_LOOP); showStatus(await client.status()); });
action('drive-off', async () => { stopMotion(); stopScope(); client.cancelQueued(); await client.request(C.DISABLE); showStatus(await client.status()); });
action('estop', async () => { stopMotion(); stopScope(); client.cancelQueued(); await client.request(C.ESTOP); showStatus(await client.status()); notice('Emergency stop acknowledged.'); });
action('reset', async () => { stopMotion(); stopScope(); client.cancelQueued(); await client.reset(); await inspectConnection(); notice('Reset complete; PID startup defaults restored.'); });
action('target-only', () => sendTarget(false));
action('target-position', () => sendTarget(true));
action('scope-start', startScope);
action('scope-stop', stopScope);
action('motion-start', async () => {
  targetCounts();
  const angle = Number($('target').value); let sign = 1;
  const generation = ++motionGeneration, link = client;
  motionRunning = true; controls();
  // Starting just the alternating test must capture the entire response, not
  // only the command-50 snapshot at each reversal.
  if (!scopeRunning) void startScope().catch(reportError);
  try {
    while (client === link && generation === motionGeneration) {
      await sendTarget(true, sign * angle); sign = -sign;
      await sleep(250);
    }
  } catch (error) {
    if (client === link && generation === motionGeneration) throw error;
  } finally {
    if (generation === motionGeneration) { motionRunning = false; controls(); }
  }
});
action('motion-stop', () => {
  stopMotion();
  notice('Test stopped; holding the last accepted target.' +
    (scopeRunning ? ' Scope continues to capture settling.' : ' Scope is paused.'));
});
for (const id of ['p', 'i', 'd']) {
  $(id + '-range').addEventListener('input', () => { $(id).value = $(id + '-range').value; });
  $(id).addEventListener('input', () => { $(id + '-range').value = $(id).value; });
}
addEventListener('resize', draw);
controls(); draw();
const startupParameters = new URL(location.href).searchParams;
if (startupParameters.has('address')) $('address').value = startupParameters.get('address');
if (startupParameters.get('target')?.trim() && Number.isFinite(Number(startupParameters.get('target')))) {
  $('target').value = startupParameters.get('target');
}
if (startupParameters.get('demo') === '1') {
  connect(true).catch(error => notice(error.message, true));
}
