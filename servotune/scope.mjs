// Scope paths use browser receipt times. Do not invent motion across missing
// samples, or interpolate a ramp between two accepted position references.
export const MAX_SAMPLE_GAP_MS = 100;

export function scopeTrace(samples, key) {
  const points = [];
  let previous;
  for (const sample of samples) {
    const move = !previous || sample.t - previous.t > MAX_SAMPLE_GAP_MS;
    if (!move && key === 'target' && sample.target !== previous.target) {
      points.push({ t: sample.t, value: previous.target, move: false });
    }
    points.push({ t: sample.t, value: sample[key], move });
    previous = sample;
  }
  return points;
}
