// Optional browser QA: npm install --no-save playwright, then run with the
// local server running. PLAYWRIGHT_MODULE may point at an existing install.
const assert = require('node:assert/strict');
const { chromium } = require(process.env.PLAYWRIGHT_MODULE || 'playwright');

(async () => {
  const browser = await chromium.launch({ headless: true });
  try {
    const page = await browser.newPage({ viewport: { width: 1440, height: 1150 } });
    // Observe the paths actually sent to the canvas, rather than checking a
    // second implementation of the plot or only the existence of samples.
    await page.addInitScript(() => {
      globalThis.scopePaths = {};
      for (const method of ['beginPath', 'moveTo', 'lineTo', 'stroke']) {
        const original = CanvasRenderingContext2D.prototype[method];
        CanvasRenderingContext2D.prototype[method] = function (...args) {
          if (this.canvas.id === 'scope') {
            if (method === 'beginPath') this.scopePoints = [];
            else if (method === 'moveTo' || method === 'lineTo') {
              this.scopePoints.push({ x: args[0], y: args[1], move: method === 'moveTo' });
            } else if (method === 'stroke') {
              globalThis.scopePaths[this.strokeStyle] = this.scopePoints.slice();
            }
          }
          return original.apply(this, args);
        };
      }
    });
    const errors = [];
    page.on('pageerror', error => errors.push(error.message));
    await page.goto(process.env.SERVOTUNE_URL || 'http://127.0.0.1:8765/?demo=1');
    await page.getByText('DEMO — simulated motor, no hardware connected', { exact: false }).waitFor();
    await page.getByText('Firmware 0.17.0.0', { exact: false }).waitFor();
    assert.equal(await page.locator('#p').inputValue(), '2000');
    assert.equal(await page.locator('#i').inputValue(), '5');
    assert.equal(await page.locator('#d').inputValue(), '175000');
    await page.getByRole('button', { name: 'Start alternating targets', exact: true }).click();
    await page.getByText('Fatal motor error 13:', { exact: false }).waitFor();
    assert.equal(await page.locator('#scope-stop').isEnabled(), false);
    assert.equal(await page.locator('#motion-stop').isEnabled(), false);
    await page.getByRole('button', { name: 'Reset motor', exact: true }).click();
    await page.getByText('Reset complete;', { exact: false }).waitFor();
    await page.getByRole('button', { name: 'Enable closed loop', exact: true }).click();
    await page.getByText('Outputs enabled · Closed-loop position mode', { exact: true }).waitFor();
    await page.getByRole('button', { name: 'Send target + read position', exact: true }).click();
    await page.locator('#state-target').filter({ hasText: '10.000°' }).waitFor();
    assert.match(await page.locator('#state-measured').innerText(), /^-?\d+\.\d{3}°$/);
    for (const id of ['state-counter', 'state-mode', 'state-queue']) assert.equal(await page.locator('#' + id).count(), 0);
    await page.getByLabel('Target (degrees)', { exact: true }).fill('-10');
    await page.getByRole('button', { name: 'Send target', exact: true }).click();
    await page.getByText('Target applied.', { exact: true }).waitFor();
    await page.locator('#p').fill('2050');
    await page.locator('#i').fill('6');
    await page.locator('#d').fill('175032');
    await page.getByRole('button', { name: 'Write PID', exact: true }).click();
    await page.getByText('PID constants read from the demo motor.', { exact: true }).waitFor();
    // User regression: do not press Start scope separately.
    await page.getByRole('button', { name: 'Start alternating targets', exact: true }).click();
    await page.waitForFunction(() => parseInt(document.querySelector('#sample-count').textContent, 10) >= 220);
    assert.equal(await page.locator('#scope-start').isEnabled(), false);
    assert.equal(await page.locator('#motion-start').isEnabled(), false);
    await page.waitForFunction(() => Number(document.querySelector('#transactions').textContent) > 220);
    const rate = Number(await page.locator('#transaction-rate').innerText());
    const time = parseFloat(await page.locator('#transaction-time').innerText());
    assert.ok(rate > 20, 'Show the rate of completed transactions');
    assert.ok(time > 0 && Math.abs(rate * time - 1000) < 5, 'Show milliseconds per completed transaction');
    const paths = await page.evaluate(() => globalThis.scopePaths);
    const target = paths['#72dc99'], measured = paths['#75ceff'];
    assert.ok(target.length > 200 && measured.length > 200, 'Continuous capture throughout the move');
    let steps = 0;
    target.forEach((point, i) => {
      if (!point.move) {
        const previous = target[i - 1];
        assert.ok(point.x === previous.x || point.y === previous.y, 'No diagonal target ramps');
        if (point.x === previous.x && point.y !== previous.y) steps++;
      }
    });
    assert.ok(steps >= 6, 'Several held target steps must be visible');
    assert.ok(new Set(measured.map(p => p.y.toFixed(3))).size > 100, 'Measured response has intermediate samples');
    await page.screenshot({ path: process.env.SERVOTUNE_SCREENSHOT || '/tmp/servotune-demo.png', fullPage: true });
    // Pausing the scope must also suppress the sparse command-50 snapshots,
    // even while alternating targets continue to be sent.
    await page.getByRole('button', { name: 'Stop scope', exact: true }).click();
    const paused = await page.locator('#sample-count').innerText();
    const completedWhilePaused = Number(await page.locator('#transactions').innerText());
    const previousTarget = await page.locator('#state-target').innerText();
    await page.waitForFunction(previous => document.querySelector('#state-target').textContent !== previous, previousTarget);
    await page.waitForTimeout(550);
    assert.equal(await page.locator('#sample-count').innerText(), paused);
    await page.waitForFunction(previous => Number(document.querySelector('#transactions').textContent) > previous, completedWhilePaused);
    await page.getByRole('button', { name: 'Stop test', exact: true }).click();
    // Starting a fresh test resumes a paused scope; stopping the test then
    // leaves continuous sampling running to capture the settling response.
    await page.getByRole('button', { name: 'Start alternating targets', exact: true }).click();
    await page.waitForFunction(previous => document.querySelector('#sample-count').textContent !== previous, paused);
    assert.equal(await page.locator('#scope-start').isEnabled(), false);
    await page.getByRole('button', { name: 'Stop test', exact: true }).click();
    const settling = await page.locator('#sample-count').innerText();
    await page.waitForFunction(previous => document.querySelector('#sample-count').textContent !== previous, settling);
    // Drive off must also cancel an active capture without a stale polling
    // error overwriting the result of the disable action.
    await page.getByRole('button', { name: 'Drive off', exact: true }).click();
    await page.getByText('Outputs disabled · Closed-loop position mode', { exact: true }).waitFor();
    await page.getByRole('button', { name: 'Send target + read position', exact: true }).click();
    await page.getByText('Fatal motor error 55:', { exact: false }).waitFor();
    await page.getByRole('button', { name: 'Reset motor', exact: true }).click();
    await page.getByText('Reset complete;', { exact: false }).waitFor();
    assert.equal(await page.locator('#p').inputValue(), '2000');
    assert.equal(await page.locator('#i').inputValue(), '5');
    // With no scope or alternating requests, the next complete idle interval
    // must show zero throughput and no division-by-zero transaction time.
    await page.waitForFunction(() => document.querySelector('#transaction-rate').textContent === '0.0');
    assert.equal(await page.locator('#transaction-time').innerText(), '—');
    await page.getByRole('button', { name: 'Disconnect', exact: true }).click();
    await page.getByText('Not connected', { exact: true }).waitFor();
    assert.equal(await page.locator('#transaction-rate').innerText(), '0.0');
    assert.equal(await page.locator('#transaction-time').innerText(), '—');
    // Existing 0.17.0.2 motors still return 24 bytes. Verify the production UI
    // selects that schema from the version and retains working scope capture.
    const legacyUrl = new URL(process.env.SERVOTUNE_URL || 'http://127.0.0.1:8765/?demo=1');
    legacyUrl.searchParams.set('demo', '1'); legacyUrl.searchParams.set('demo-legacy', '1');
    await page.goto(legacyUrl.href);
    await page.getByText('Firmware 0.17.0.2', { exact: false }).waitFor();
    await page.getByRole('button', { name: 'Enable closed loop', exact: true }).click();
    await page.getByText('Outputs enabled · Closed-loop position mode', { exact: true }).waitFor();
    await page.getByLabel('Target (degrees)', { exact: true }).fill('1');
    await page.getByRole('button', { name: 'Send target + read position', exact: true }).click();
    await page.locator('#state-target').filter({ hasText: '1.000°' }).waitFor();
    assert.equal(await page.locator('#state-target').innerText(), '1.000°');
    await page.getByRole('button', { name: 'Start alternating targets', exact: true }).click();
    await page.waitForFunction(() => parseInt(document.querySelector('#sample-count').textContent, 10) >= 40);
    await page.getByRole('button', { name: 'Drive off', exact: true }).click();
    await page.getByText('Outputs disabled · Closed-loop position mode', { exact: true }).waitFor();
    await page.getByRole('button', { name: 'Disconnect', exact: true }).click();
    await page.getByText('Not connected', { exact: true }).waitFor();
    assert.deepEqual(errors, []);
    console.log('PASSED: stable compact 0.17.0.0 and legacy 0.17.0.2 command 50, PID round trips, alternating-test automatic capture, actual step paths, scope pause/resume, settling capture, transaction rate/time, fatal errors, reset and disconnect; no browser script errors');
  } finally { await browser.close(); }
})().catch(error => { console.error(error); process.exitCode = 1; });
