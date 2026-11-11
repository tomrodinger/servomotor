// Real-motor QA through a test-only UART bridge. This exercises the production
// browser UI, WebSerialTransport stream handling, framing and transaction queue.
// Native Chrome permission/adapter selection is left to the user's browser.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const readline = require('node:readline');
const { spawn } = require('node:child_process');
const { chromium } = require(process.env.PLAYWRIGHT_MODULE || 'playwright');

(async () => {
  const port = process.env.SERVOTUNE_PORT, uid = process.env.SERVOTUNE_UID;
  const firmwareVersion = process.env.SERVOTUNE_FIRMWARE_VERSION || '0.17.0.0';
  assert(port && /^[0-9a-f]{16}$/i.test(uid), 'Set SERVOTUNE_PORT and a 16-digit SERVOTUNE_UID');
  const child = spawn(process.env.PYTHON || 'python3', [path.join(__dirname, 'test_hardware_serial_bridge.py'), '--port', port, '--uid', uid, '--firmware-version', firmwareVersion]);
  const pending = new Map(), trace = []; let serial = 0, page;
  const rpc = operation => new Promise((resolve, reject) => {
    const id = ++serial; pending.set(id, { resolve, reject });
    child.stdin.write(JSON.stringify({ id, ...operation }) + '\n');
  });
  child.stderr.on('data', data => process.stderr.write(data));
  const lines = readline.createInterface({ input: child.stdout });
  lines.on('line', line => {
    const message = JSON.parse(line);
    if (message.event === 'data') {
      trace.push({ event: 'rx', t_ms: performance.now(), bytes: message.bytes });
      if (page && !page.isClosed()) page.evaluate(bytes => window.hardwareReceive?.(bytes), message.bytes).catch(() => {});
    } else if (message.event === 'error') {
      if (page && !page.isClosed()) page.evaluate(text => window.hardwareFailure?.(text), message.message).catch(() => {});
    } else {
      const waiter = pending.get(message.id); if (!waiter) return;
      pending.delete(message.id); message.error ? waiter.reject(new Error(message.error)) : waiter.resolve(message.result);
    }
  });
  child.on('exit', code => {
    for (const waiter of pending.values()) waiter.reject(new Error('Serial bridge exited: ' + code));
    pending.clear();
  });
  const browser = await chromium.launch({ headless: true });
  const report = { port, unique_id: uid, passed: false, transport: 'Test-only UART bridge; native Web Serial chooser not automated' };
  try {
    await rpc({ operation: 'prepare' });
    report.requested_current_limits_internal_units = [50, 50];
    page = await browser.newPage({ viewport: { width: 1440, height: 1150 } });
    const errors = [];
    page.on('pageerror', error => errors.push(error.message));
    await page.exposeBinding('hardwareSerial', async (_, operation) => {
      if (operation.operation === 'write') trace.push({ event: 'tx', t_ms: performance.now(), bytes: operation.bytes });
      return rpc(operation);
    });
    await page.addInitScript(() => {
      let input;
      window.hardwareReceive = bytes => input?.enqueue(Uint8Array.from(bytes));
      window.hardwareFailure = text => input?.error(new Error(text));
      Object.defineProperty(navigator, 'serial', { configurable: true, value: {
        async requestPort() {
          return {
            async open({ baudRate }) {
              if (baudRate !== 230400) throw new Error('Unexpected baud rate');
              this.readable = new ReadableStream({ start(controller) { input = controller; }, cancel() { input = null; } });
              this.writable = new WritableStream({ write(data) { return window.hardwareSerial({ operation: 'write', bytes: Array.from(data) }); } });
              await window.hardwareSerial({ operation: 'open' });
            },
            async close() { await window.hardwareSerial({ operation: 'close' }); }
          };
        }
      } });
    });
    await page.goto(process.env.SERVOTUNE_URL);
    await page.getByLabel('Motor address', { exact: true }).fill(uid);
    await page.getByLabel('Target (degrees)', { exact: true }).fill('1');
    await page.getByRole('button', { name: 'Connect serial', exact: true }).click();
    await page.getByText('Serial motor connected · Firmware ' + firmwareVersion, { exact: true }).waitFor();
    assert.equal(await page.locator('#p').inputValue(), '2000');
    assert.equal(await page.locator('#i').inputValue(), '5');
    assert.equal(await page.locator('#d').inputValue(), '175000');
    await page.getByRole('button', { name: 'Enable closed loop', exact: true }).click();
    await page.getByText('Outputs enabled · Closed-loop position mode', { exact: true }).waitFor();
    await page.getByRole('button', { name: 'Send target', exact: true }).click();
    await page.getByText('Target applied.', { exact: true }).waitFor();
    await page.getByRole('button', { name: 'Send target + read position', exact: true }).click();
    await page.locator('#state-target').filter({ hasText: '1.000°' }).waitFor();
    assert.match(await page.locator('#state-measured').innerText(), /^-?\d+\.\d{3}°$/);
    await page.locator('#p').fill('2001');
    await page.getByRole('button', { name: 'Write PID', exact: true }).click();
    await page.getByText('PID constants read from the motor.', { exact: true }).waitFor();
    // Starting only the alternating test must capture the complete response.
    await page.getByRole('button', { name: 'Start alternating targets', exact: true }).click();
    await page.waitForFunction(() => parseInt(document.querySelector('#transactions').textContent, 10) >= 500);
    assert.ok(parseInt(await page.locator('#sample-count').innerText(), 10) > 100);
    report.completed_transactions = Number(await page.locator('#transactions').innerText());
    const metrics = () => page.evaluate(() => ({
      completed: Number(document.querySelector('#transactions').textContent),
      transactions_per_second: Number(document.querySelector('#transaction-rate').textContent),
      milliseconds_per_transaction: parseFloat(document.querySelector('#transaction-time').textContent),
    }));
    report.active_metrics = await metrics();
    assert.ok(report.active_metrics.transactions_per_second > 10);
    assert.ok(Math.abs(report.active_metrics.transactions_per_second * report.active_metrics.milliseconds_per_transaction - 1000) < 5);
    await page.screenshot({ path: process.env.SERVOTUNE_SCREENSHOT || '/tmp/servotune-real-motor.png', fullPage: true });
    // Scope pause must not freeze communication statistics or add sparse
    // command-50 snapshots to the paused plot while retargeting continues.
    await page.getByRole('button', { name: 'Stop scope', exact: true }).click();
    const pausedSamples = await page.locator('#sample-count').innerText();
    await page.waitForFunction(() => {
      const rate = Number(document.querySelector('#transaction-rate').textContent);
      return rate > 0 && rate <= 6;
    });
    report.paused_scope_metrics = await metrics();
    assert.equal(await page.locator('#sample-count').innerText(), pausedSamples);
    await page.getByRole('button', { name: 'Stop test', exact: true }).click();
    await page.waitForFunction(() => document.querySelector('#transaction-rate').textContent === '0.0');
    report.idle_metrics = await metrics();
    assert.equal(await page.locator('#transaction-time').innerText(), '—');
    // Starting another alternating test resumes capture automatically. Stopping
    // only the test keeps capturing the physical settling response.
    await page.getByRole('button', { name: 'Start alternating targets', exact: true }).click();
    await page.waitForFunction(() => Number(document.querySelector('#transaction-rate').textContent) > 10);
    assert.equal(await page.locator('#scope-start').isEnabled(), false);
    await page.getByRole('button', { name: 'Stop test', exact: true }).click();
    const settlingSamples = await page.locator('#sample-count').innerText();
    await page.waitForFunction(previous => document.querySelector('#sample-count').textContent !== previous, settlingSamples);
    report.scope_pause_resume_and_settling_passed = true;
    await page.getByRole('button', { name: 'Stop scope', exact: true }).click();
    await page.locator('#p').fill('2000');
    await page.getByRole('button', { name: 'Write PID', exact: true }).click();
    await page.getByText('PID constants read from the motor.', { exact: true }).waitFor();
    await page.getByRole('button', { name: 'Emergency stop', exact: true }).click();
    await page.getByText('Emergency stop acknowledged.', { exact: true }).waitFor();
    await page.getByRole('button', { name: 'Send target + read position', exact: true }).click();
    await page.getByText('Fatal motor error 55:', { exact: false }).waitFor();
    await page.getByRole('button', { name: 'Reset motor', exact: true }).click();
    await page.getByText('Reset complete;', { exact: false }).waitFor();
    // The now-healthy firmware reset is silent, unlike the fatal-state reset.
    await page.getByRole('button', { name: 'Reset motor', exact: true }).click();
    await page.waitForFunction(() => !document.querySelector('#reset').disabled);
    assert.match(await page.locator('#notice').innerText(), /Reset complete/);
    await page.getByRole('button', { name: 'Disconnect', exact: true }).click();
    await page.getByText('Not connected', { exact: true }).waitFor();
    assert.deepEqual(errors, []);
    report.passed = true; report.script_errors = errors;
  } catch (error) {
    report.error = String(error);
    if (page && !page.isClosed()) {
      report.last_notice = await page.locator('#notice').innerText().catch(() => '');
      report.last_status = await page.locator('#status').innerText().catch(() => '');
    }
    throw error;
  }
  finally {
    await browser.close(); page = null;
    await rpc({ operation: 'cleanup' }).catch(error => { report.cleanup_error = String(error); report.passed = false; });
    await rpc({ operation: 'shutdown' }).catch(() => {});
    child.stdin.end();
    report.request_count = trace.filter(t => t.event === 'tx').length;
    report.trace = trace;
    if (process.env.SERVOTUNE_REPORT) fs.writeFileSync(process.env.SERVOTUNE_REPORT, JSON.stringify(report, null, 2) + '\n');
  }
  assert(report.passed, 'Browser hardware test failed');
  console.log('PASSED: actual motor + production browser UI/stream/transaction code, using the documented test-only UART bridge');
})().catch(error => { console.error(error); process.exitCode = 1; });
