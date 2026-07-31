# Zero-crossing gated integral (fw 0.15.4.0–0.15.4.2): REJECTED AND REVERTED

**Date:** 2026-07-13. **Outcome:** reverted to the committed 0.15.3.4. Nothing from
0.15.4.x ships. **Read this before attempting the "full torque at small error" goal again.**

## What we were trying to solve

Tom's requirement: the motor should apply *maximum torque even when the position error is
tiny* (0.1% of a revolution = 3,277 counts), the way a classical integrator would — wind up
until the error is gone. The 25% integral clamp shipped in 0.15.3.4 (the anti-windup that
cured the kI>10 limit cycle) caps sustained torque at ~31% of stall at that error, so the
motor parks short under load (measured: 1,991 counts short at current 25 — see plan §17).

## What we built (and why it looked right)

A **zero-crossing-gated integral**: the integral may exceed the 25% rail toward *full*
authority once the error has kept one sign (outside a ±1,000-count deadband) for 0.1 s;
on each zero crossing it is set to the midpoint of its last two crossing values (self-tunes
to the true hold torque, since the stick-slip friction band is symmetric about it) and
integration freezes for 0.1 s.

It passed *everything we could measure*:
- Load holding: standing error 9.3k → ~600 counts in sim; **on the bench, a 0.1%-rev step
  that 0.15.3.4 parks 1,991 counts short of converged to 1 count** (integral reached 317%
  of the old rail). The stated goal was objectively achieved.
- Snap overshoot, no-load quiet, dormancy, release behavior: all within budget.
- Differential 7.0M samples bit-exact, UBSan overflow 3×1.42B ticks clean, battery/sweep
  green, ISR cost ≤1–2 µs, mutation testing killing every planted defect.

## Why it failed — the thing no test caught

**Tom's hand.** With the gate active he felt, at every tuning tried: a **whine/vibration**
when holding the shaft slightly off-position, and **jerkiness/notchiness** when rocking the
shaft through the setpoint. Decisive experiment: **at kI=0 (integral and gate machinery
inert) all complaints vanished instantly** — smooth, quiet, no overshoot — but the return
became weak/slow and parked short. So the roughness *is* the gate machinery, not the
mechanics, not the D term, not the disk.

Two repair attempts, both instructive:

1. **Slew-limited corrections (0.15.4.2).** Diagnosis: crossings reassigned the integral
   *instantly* — up to a full rail-to-rail torque reversal in one 32 µs tick. Fixed by
   slewing (one-tick |ΔI| 103M → 6.6M, bound airtight). **Bench verdict: no better.**
   Lesson: a 0.5 ms ramp and a 32 µs step are indistinguishable to a human hand. Smoothing
   at tick timescales cannot fix a feel problem that lives at 0.1–1 s timescales.
2. **Above-rail windup rate cap (designed, never flashed).** Adversarial verification
   **broke it before hardware**: the cap equilibrates against the back-calculation drain and
   silently pins holding torque at ~68% of rail for loads ≥ 0.26 N·m (parks ~8° off,
   permanently, 10/10 seeds). Do not resurrect a wind cap without solving that interaction.

Also tried and rejected on the bench: kI=5 (slower integral tempo) — still bad.

## The real lesson (the one to keep)

**The gate makes the loop's behavior depend on interaction history.** Its state machine
(same-sign counter, crossing midpoints, freeze windows, staleness) operates on exactly the
timescales a human hand works at — 0.1–1 s — so a hand interacting with the motor
continuously re-triggers it: the integral winds against the hand, crossings fire corrections,
freezes gate the response, and the torque *trend* keeps shifting under the fingers. That is
felt as whine and notchiness, and it is **structural to any state-machine-gated integrator**,
not a tuning or smoothing defect. Our entire validation suite measured *hands-off* scenarios
(steps, profiled moves, constant loads, releases) and every one of them passed; the failure
mode lived in a dimension we never instrumented — and even after we *did* instrument it
(forced-rock/forced-hold sim scenarios with per-tick torque-smoothness metrics), the metric
we chose (per-tick ΔI) turned out to be the wrong proxy for what a hand perceives.

**If this goal is attempted again, do not re-derive this design.** Directions that do NOT
carry the same flaw, in rough order of promise:

1. **Feedforward, not gated feedback.** The torque needed to hold a load is a *plant*
   property, not a control-history property: estimate it (e.g. a slow, unconditional
   low-pass of the output during steady state) and add it as a feedforward term. No state
   machine, no history dependence, no crossing events — the smoothness problem cannot arise.
2. **Raise the honest ceiling instead of gating.** The 25% clamp exists because integral
   torque above breakaway friction re-arms the stick-slip limit cycle (I-rail torque 0.094
   N·m vs 0.13 N·m breakaway — plan §17). Reduce the *stiction* (mechanics) or reduce the
   integral's *stick-slip coupling* (e.g. integral acting only below a velocity threshold)
   and the clamp can rise safely without any gating.
3. **Accept the standing error and fix it at the trajectory level** — profiled moves already
   land clean; the error only appears against sustained external loads.
4. **Velocity-based D with int64 math** (plan §21) — orthogonal, still promising for
   heavy-inertia loads, unrelated to this failure.

## Bench truths worth keeping (independent of the gate)

- **Return speed law:** cruise speed ≈ authority/kD counts/tick — kD is simultaneously the
  landing brake *and* a cruise governor. "Fast return + soft landing" cannot be tuned
  independently with fixed PID gains.
- **Braking window:** the PID error input is clamped at authority/kP, so the loop is blind
  to distance beyond it; that clamp is also the emergent brake trigger (widening it made
  overshoot 5× worse — plan §21). Lowering kP is the per-load knob.
- **Heavy inertia:** commanded snap moves are ballistic (J·ω²/2τ); at 20× rotor inertia a
  0.2-rot snap overshoots 18° at *any* gains, while the same move over 0.2 s overshoots
  0.05°. Trajectory shaping, not tuning.
- **Fleet caution:** the 25% clamp's quiet margin depends on breakaway friction; sim says
  units with stiction ≤ 0.10 N·m could hum even at 25%. Unmeasured across the fleet.

## Artifacts preserved here

- `rejected_source/` — the full 0.15.4.2 firmware (`motor_control.c.bak`, `main.c.bak`),
  the complete firmware diff, the gated simulator, and the harness-mirror diffs.
- `rejected_firmware_binaries/` — all 0.15.4.x `.firmware` builds (M17 + M23), moved out of
  `firmware_releases/` so they can never be flashed by accident.
- Study data: `../2026-07-11_load_holding_study/`, `../2026-07-13_per_load_tuning/`,
  `../2026-07-13_smoothness_0.15.4.2/`.
- Plan doc §17–§22 tell the full story chronologically (§22 ends with this reversal).
