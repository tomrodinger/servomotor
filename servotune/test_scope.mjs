// Regression for the sparse, opposite-looking triangle traces reported on the
// real motor. These tests check the actual path builder used by the canvas.
import assert from 'node:assert/strict';
import { scopeTrace, MAX_SAMPLE_GAP_MS } from './scope.mjs';

assert.deepEqual(scopeTrace([], 'target'), []);
const sparse = Array.from({ length: 6 }, (_, i) => ({
  t: i * 250, target: i % 2 ? -10 : 10, measured: i % 2 ? 10 : -10,
}));
// Command-50 snapshots alone are too far apart to establish the motor response.
// Leave isolated points instead of manufacturing opposite triangle waves.
for (const key of ['target', 'measured']) {
  assert.ok(scopeTrace(sparse, key).every(p => p.move));
}

const measured = [0, 2, 6, 10, 12, 11, 10, 5, -3, -10, -12, -10];
const dense = measured.map((position, i) => ({
  t: i * 15, target: i < 6 ? 10 : -10, measured: position,
}));
const before = structuredClone(dense);
const target = scopeTrace(dense, 'target');
target.forEach((point, i) => {
  if (!point.move) assert.ok(point.t === target[i - 1].t || point.value === target[i - 1].value,
    'Every target segment must be horizontal or vertical');
});
assert.deepEqual(target.filter(p => p.t === 90).map(p => p.value), [10, -10]);
assert.deepEqual(scopeTrace(dense, 'measured').map(p => p.value), measured,
  'Preserve measured overshoot, direction and amplitude without filtering');
assert.deepEqual(dense, before);
const gap = [dense[0], { ...dense[1], t: MAX_SAMPLE_GAP_MS },
  { ...dense[2], t: 2 * MAX_SAMPLE_GAP_MS + 1 }];
assert.deepEqual(scopeTrace(gap, 'measured').map(p => p.move), [true, false, true]);
console.log('PASSED: stepped references, unmodified measured overshoot, and gaps instead of sparse triangle interpolation');
