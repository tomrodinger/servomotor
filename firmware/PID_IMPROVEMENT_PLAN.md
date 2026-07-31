# PID Control Loop Improvement Plan

**Status (2026-07-10): firmware 0.15.3.4 — anti-windup + rounding fixes +
integer-overflow hardening + the ultracode-review fixes of §16, all
bench-verified. The kI windup limit cycle is fixed. Read §6 (simulation
findings), §8 (bench verification), §9 (overflow hardening), §13-§14
(overshoot studies), §15 (D-filter study), §16 (final adversarial review
pass). Remaining roadmap: gain-resolution shifts, velocity feedforward,
retune shipped defaults.** Baseline measurement was done 2026-07-06 on stock
0.15.3.0.

> NOTE on line numbers: code citations in §1-§11 refer to the 0.15.3.0 source
> and are stale by ~+100 lines in the current file; section content reflects
> the state at the time each study was done. §1 describes the OLD (pre-fix)
> algorithm.

Target: M17 first (bench motor available), but `PID_controller()` in
`Src/motor_control.c` is shared by all products (M1, M2, M17, M23), so every
change must be evaluated against all four. The goal is a bug-free release:
one change at a time, each validated on hardware before the next.

Constraints:

- MCU is a 64 MHz STM32G0 (Cortex-M0+), **no FPU** — integer math only.
- The control ISR (`TIM16_IRQHandler`) runs at **31.25 kHz** (every 32 µs,
  TIM16: PSC=0, ARR=2048 @ 64 MHz) and already has little headroom
  (`ERROR_CONTROL_LOOP_TOOK_TOO_LONG` watchdog). Improvements must add at
  most a handful of cycles; several actually remove work.

Related files:

- `Src/motor_control.c` — `PID_controller()` (~line 2198), its caller
  `motor_movement_calculations()` (M17 branch ~line 2455), `compute_velocity()`
  (~line 1981), `recompute_pid_parameters_and_set_pwm_voltage()` (~line 2117).
- `Src/motor_control.h` — per-product `PROPORTIONAL/INTEGRAL/DERIVATIVE_CONSTANT_PID`,
  `PID_SHIFT_RIGHT`, `DERIVATIVE_CONSTANT_AVERAGING_SCALAR_SHIFT`,
  `PWM_VOLTAGE_VS_COMMUTATION_POSITION_FUDGE_SHIFT`.
- `tools/pid_debug_capture.py` — measurement tool (see "Measurement tooling" below).
- `firmware/pid_baseline_captures/2026-07-06/` — baseline captures from the
  bench M17 on stock firmware, for before/after comparison.

---

## 1. How the M17 PID worked as of 0.15.3.0 (SUPERSEDED -- kept as the baseline record)

Units: positions in microsteps, **3,276,800 counts/rev**; one electrical
cycle = 65,536 counts; 90° electrical = `HALL_TO_POSITION_90_DEGREE_OFFSET`
= 16,384 counts. The sensor is 3 analog hall sensors, 150 segments/rev
(21,845 counts/segment), interpolated from the 12-bit ADC with 4×
oversampling.

Every 32 µs, `PID_controller(error)` is called with
`error = current_position_i64 - hall_position_i64` (trajectory minus
measured). With M17 defaults `kP=2000, kI=5, kD=175000, PID_SHIFT_RIGHT=11`:

```
D:  error_change = error - previous_error            (clamped to ±max_error_change)
    lpf = lpf*31 >> 5;  lpf += error_change          (leaky integrator, DC gain 32, τ ≈ 1 ms)
    d_term = lpf * (kD >> 5)

I:  i_term += clamp(error, ±max_error) * kI          (clamped to ±max_integral_term)

P:  p_term = clamp(error, ±max_error) * kP

    output = (i_term + p_term + d_term) >> 11
```

The clamps are recomputed from the max PWM voltage (default 200) in
`recompute_pid_parameters_and_set_pwm_voltage()`:

| quantity            | formula                       | value @ defaults |
|---------------------|-------------------------------|------------------|
| `max_error`         | `(200 << 19) / kP`            | 52,428           |
| `max_integral_term` | `200 << 19`                   | 104,857,600      |
| `max_error_change`  | `(200 << 19) / kD`            | 599              |

The M17 caller then uses `output` **twice** (motor_control.c:2474-2493):

1. As a **commutation (load) angle offset**, clamped to ±16,384 (±90°e).
2. `output >> 8` (the "fudge shift") as the **PWM voltage magnitude**,
   clamped to `max_motor_pwm_voltage`.

So full authority = output 51,200 (full voltage; angle already saturated at
16,384). Note `max_integral_term >> 11 = 51,200`: **the I term alone is
allowed 100% of full authority.** The effective P gain is `2000/2048 ≈ 0.98`
output counts per count of error.

The PID itself never learns that its output was clamped downstream — the
saturation happens in the caller.

Two observations recorded for later (not part of this work, verify before
touching): (a) `TIME_STEPS_PER_SECOND` in motor_control.h is
`PWM_FREQUENCY >> 1` = 15,625, but the ISR and the published unit
conversions (`"seconds": 31250` in unit_conversions_M3.json) both use
31,250 — the define only feeds the internal `MAX_VELOCITY` /
`MAX_ACCELERATION` limits, which may therefore be 2× off; (b) M2 has an
undocumented `error >>= 3` at the top of `PID_controller`.

---

## 2. Measured baseline (bench M17, stock fw, 2026-07-06)

All data in `firmware/pid_baseline_captures/2026-07-06/`, captured with
`tools/pid_debug_capture.py` (~240 Hz effective sampling of the 31.25 kHz
loop; struct semantics verified — see "Measurement tooling").

**Standstill, default gains** (`standstill_test2.*`): after the closed-loop
entry transient, error mean ≈ 9, std ≈ 37–147 counts. The I term ramps to
≈ 8.4 M (≈ 4,100 counts of output) and holds the static load; error mean
stays near 0 — I is doing its job at small kI.

**Standstill D-term noise floor**: `d_term` std ≈ 206k–230k pre-shift
(≈ ±100–110 counts on the output) while the useful P term at this error
level is comparable — D is mostly re-amplified sensor noise. Crucially,
**error std at standstill is identical (37.2 counts) with kD=175,000 and
kD=0** (`kd0_test.csv` vs `ki0_test.csv`): today's D adds output noise and
measurably zero regulation benefit at standstill.

**Sticky negative D bias — rounding bug confirmed**: `d_term` mean is
consistently ≈ **−80,000** pre-shift (≈ −40 counts DC on the output) across
every capture including pure standstill. Cause: the filter decay
`(lpf * 31) >> 5` is an arithmetic shift (floor); for any lpf value in
−31..−1 the result equals lpf, so **negative filter values in that band
never decay**, while positive ones decay to 0. The filter wanders in the
−31..−1 band between noise kicks (−31×5468 ≈ −170k; observed mean sits
mid-band). The final `>> 11` has the same negative-truncation asymmetry.

**Move transient, default gains** (`move_test1.*`, 2 rot / 2 s): peak error
≈ +10,600 counts at acceleration, oscillating tracking error ±2,500 during
the move, −4,700 spike at the stop, clean settling. I peaks at 18 M — 17%
of the rail. Defaults are stable for this move.

**Windup limit cycle reproduced** (`ki25_move_test.*`, kI=25 + same move):
the move triggers a sustained **≈ 33 Hz limit cycle, ±10,000 counts
(±0.003 rev), that never settles** — still full amplitude 5 s after the
move ended. The I term swings +43 M / −36 M (±40% of rail) out of phase
with the error; P and D are passengers. This is the user-visible "I above
~10 just oscillates" symptom, root-caused.

Why the integrator dominates: at 31.25 kHz, `i_term` grows by
`error × kI` every 32 µs. With error at its clamp (52,428):

| kI | time from 0 to full-authority rail |
|----|-----------------------------------|
| 5  | 12.8 ms                           |
| 10 | 6.4 ms                            |
| 25 | 2.6 ms                            |

Any transient longer than a few ms fully winds the integrator; unwinding
requires an equal error·time product of opposite sign → overshoot →
re-wind in the other direction → limit cycle.

---

## 3. Problems and proposed fixes

Ordered by expected impact. Estimated ISR cost is for the M0+ (no FPU),
all int32.

### P1. Integrator windup (the "I > 10 oscillates" bug)

There is no anti-windup: the only bound is a clamp equal to 100% of full
output authority.

**Fix A (primary): back-calculation anti-windup.** Make the PID compute the
same saturation the caller applies (±51,200 = full-voltage-equivalent
output for M17, or derive from `max_motor_pwm_voltage`), and bleed the
excess back out of the integrator:

```c
raw = (i_term + p_term + d_term) >> PID_SHIFT_RIGHT;
sat = clamp(raw, ±output_limit);
i_term -= (raw - sat) << PID_SHIFT_RIGHT;   // or a >>1 fraction of it
output = sat;
```

Cost ≈ 2 compares + 1 subtract + 1 shift. The caller's own clamps stay as
a safety net (they become no-ops).

**Fix B (also do): separate, smaller integral clamp.** The I term's job is
canceling friction / static load; it does not need 100% authority. Baseline
data: holding the bench load takes ≈ 8.4 M ≈ 8% of the rail. Default the
clamp to ~25% of full scale and make it a compile-time per-product constant
(a gravity-loaded axis may need more; consider exposing it via a command
later). Cost: zero (changes a constant).

**Fix C (optional, later):** freeze integration while the trajectory is
moving fast (the `moving` flag / `current_velocity_i64` exist), if A+B
still leave visible windup on long moves. Only if measurements demand it.

### P2. D term differentiates sensor noise over a 32 µs baseline

`error - previous_error` per tick is dominated by hall-interpolation noise;
the leaky integrator that suppresses the noise adds ~1 ms of lag
(cutoff ≈ 155 Hz → 20–60° phase loss in the 50–300 Hz band where damping
is needed) and carries the sticky-bias bug (P3). Evidence: D noise floor
and the zero-benefit kD=0 comparison in §2.

**Fix (primary): replace the D path with velocity-error damping.**
The firmware already computes a measured velocity every tick over a 20-tick
FIR window (`compute_velocity()`, 640 µs baseline — ~20× better SNR, linear
phase), and the trajectory generator knows the exact desired velocity
(`current_velocity_i64`). Replace `d_term` with:

```c
d_term = (desired_velocity - measured_velocity) * kD_scaled;
```

This is mathematically the same damping (derivative of error = velocity
error) with far better signal quality, no derivative kick, and it
**removes** the LPF multiply+shift from the ISR. Care points:
`velocity` from `compute_velocity()` is scaled by
`COMPUTED_VELOCITY_CALIBRATION_CONSTANT (3000) >> 16` — the desired-velocity
term must be brought to the same scale; and `current_velocity_i64` is in
velocity-shifted internal units (`VELOCITY_SHIFT_LEFT`). Work out the
scaling on paper, then verify with the capture tool (d_term should be
near-zero during perfect tracking, large during the accel/decel phases).

**Fallback (if velocity scaling proves messy): N-tick FIR difference on the
error** — `error - error_ring[i-32]` with a 32-entry int32 ring buffer
(128 B RAM). Same DC gain as today (the existing `>>5` scaling already
matches N=32), ~32× SNR improvement, linear phase, no sticky bias. Cost:
1 subtract + ring index vs. today's multiply+2 shifts.

### P3. Rounding bugs (sticky negative D bias, output truncation bias)

Confirmed in data (§2). Even if P2 replaces the LPF, fix the pattern
everywhere it remains:

```c
// symmetric round-toward-zero decay / rounded shift:
x = (x * 31 + ((x < 0) ? 31 : 0)) >> 5;          // or add (1 << (s-1)) before >> s
output = (sum + (1 << (PID_SHIFT_RIGHT - 1))) >> PID_SHIFT_RIGHT;
```

Cost: 1 add each. Fix in the same step as P2 (D path) and P1 (output path)
respectively, and measure that `d_term` mean at standstill goes to ≈ 0.

### P4. Gain granularity (why usable kI is only single digits)

At 31.25 kHz, kI=1 already adds `error × 31250` per second to the
accumulator — enormous. Give the I and D paths extra fixed shifts so users
can tune finely:

```c
i_term += error * kI;             // accumulate as today
i_contrib = i_term >> I_EXTRA_SHIFT;   // e.g. 4 → user range ~0..160 instead of 0..10
```

Choose shifts so that current default gains map to round new values
(document the mapping: old kI=5 → new kI=80 at I_EXTRA_SHIFT=4). This
changes the meaning of the "Set PID constants" (cmd 43) values — release
notes must say so, and the mapping must be exact so existing tuned setups
can convert. Cost: 1 shift per term.

### P5. No feedforward on M17

The M23 closed-loop branch adds a back-EMF voltage term; the M17 branch
computes nothing from the known trajectory velocity. Add
`v_ff = (desired_velocity * K_VFF) >> shift` to the output (voltage path).
This directly reduces the steady tracking error during moves, which is
exactly what winds the integrator today (I peaked 17% of rail on a gentle
move). K_VFF can be estimated from the baseline: during the 1 rev/s move
the I term settled around 13–18 M ≈ 6,000–9,000 output counts ≈ 23–35 PWM
volts-units per rev/s. Cost: 1 multiply + 1 shift + 1 add. Do this AFTER
P1–P3 so its effect is measurable in isolation.

### P6. PID unaware of downstream saturation

Fixed as a side effect of P1 Fix A (the PID becomes the place where
saturation is computed). When implementing, decide the M17 output limit as
`max_motor_pwm_voltage << PWM_VOLTAGE_VS_COMMUTATION_POSITION_FUDGE_SHIFT`
so it tracks the user's current-limit setting, exactly like the existing
`max_*` recomputation does.

---

## 4. Implementation sequence (one step = one change + full validation)

Rules for every step:

- Bump `DEVELOPMENT_FIRMWARE_VERSION`, build with `make PRODUCT_NAME=M17`
  **and compile-check M1, M2, M23** (shared function!).
- Flash the bench M17, run the capture suite (below), compare against
  `pid_baseline_captures/`.
- Run the per-command test `python_programs/test_read_multipurpose_buffer.py`
  (validates the debug path still works) plus a normal move test
  (`test_go_to_position.py`).
- Check ISR headroom before/after with the "Get profiled times" debug
  command (`get_profiled_times()`), since the loop-too-long watchdog is the
  hard limit.
- Full `run_all_tests.py` only at the checkpoints marked ⏸ (suite takes
  ~10 min; batch it).
- Where a change is separable, keep the old behavior reachable via a
  `#define` during development so A/B captures are trivial.

Steps:

1. **Rounding fixes only** (P3, output shift + LPF decay). Smallest
   possible diff. Gate: standstill capture shows `d_term` mean ≈ 0 (was
   ≈ −80k); everything else unchanged; identity checks in the capture tool
   still PASS (note: the tool's `output == (P+I+D) >> 11` check must be
   updated in the same commit if rounding is added to the final shift —
   it becomes `(P+I+D + 1024) >> 11`).
2. **Back-calculation anti-windup + smaller I clamp** (P1 A+B, P6). Gate:
   kI=25 + move capture no longer limit-cycles; kI sweep (5 / 25 / 50 /
   100) settles monotonically better, no oscillation; default-gain move
   capture unchanged or better; standstill holding still works (I still
   reaches the ~8.4 M holding value). ⏸ full suite.
3. **D path replacement with velocity-error damping** (P2, includes
   scaling analysis). Gate: standstill error std with the new D ≤ old;
   move-transient overshoot reduced vs. baseline at same kD-equivalent;
   d_term tracks accel phases (visible in capture) instead of noise;
   sweep kD to find where improvement saturates. Keep the FIR fallback
   branch in the commit history if the scaling fights back.
4. **Gain-resolution shifts** (P4) with an exact documented mapping of old
   → new constants for all four products. Gate: default behavior bit-per-bit
   equivalent at mapped gains (capture identity: same clamps, same output
   for same stimulus), then verify fine-tuning actually helps (kI sweep in
   new units). ⏸ full suite.
5. **Velocity feedforward** (P5), M17 first. Gate: during the standard
   2 rot / 2 s move the mean I contribution during cruise drops
   substantially (target: <half of baseline's 13–18 M); no standstill
   regression.
6. **Retune defaults** for M17 with everything in, using capture sweeps;
   re-derive recommended ranges for the datasheet. ⏸ full suite, then
   release per the normal firmware release flow.

Steps 1–2 are low-risk and fix the reported I-term bug. Steps 3–5 are the
quality improvements; each is independently revertible.

---

## 5. Measurement tooling

`tools/pid_debug_capture.py` (written and hardware-validated 2026-07-06).

- Uses test mode 3 (`READ_PID_DEBUG_DATA_TEST_MODE`): in closed loop the
  PID writes `{error, p_term, i_term, d_term, output}` (5 × int32) into the
  multipurpose buffer whenever it is empty; polling "Read multipurpose
  buffer" (cmd 35) drains it at ~240 Hz. `error` is captured post-clamp.
- **Never clears test mode with `test_mode(0)`** — on current firmware that
  locks the MCU (`set_led_test_mode(0)` → `while(1)`, see
  WORK_CHECKLIST TODO #8). Cleanup is always `system_reset()`, which also
  reverts `--set-pid` constants to firmware defaults.
- Built-in validation on every capture (all PASSED on stock firmware):
  - `output == (P+I+D) >> s` exactly, with `s` auto-detected (= 11 on M17,
    confirming `PID_SHIFT_RIGHT` and the struct layout);
  - `p_term == error × kP` exactly, kP recovered from data (= 2000 default);
  - `kI=0 → i_term ≡ 0` when requested;
  - sampling monotonicity/rate, timeout counting, I-rail reporting.
- Usage:

```bash
# standstill / noise floor
python3 tools/pid_debug_capture.py -p /dev/tty.usbserial-210 -a X --duration 5

# move transient (2 rotations over 2 s, sent at t=1 s)
python3 tools/pid_debug_capture.py -p /dev/tty.usbserial-210 --duration 6 --move 2

# temporary gains for a sweep point (reverted by the final reset)
python3 tools/pid_debug_capture.py -p /dev/tty.usbserial-210 --duration 8 \
    --move 2 --set-pid 2000 25 175000

# re-plot a saved capture, optionally trimmed
python3 tools/pid_debug_capture.py plot capture.csv --start 2.9 --end 3.5
```

Note the tool undersamples the 31.25 kHz loop (~240 Hz, RS485-limited);
that is sufficient for windup/settling/noise-floor work (the observed limit
cycle is ~33 Hz). If per-tick resolution is ever needed, extend the
firmware's `fast_capture_data` path instead.

---

## 6. Simulation findings (2026-07-07) — what changed vs the plan above

With the bench offline for a day, the algorithm was developed against a new
headless simulator, `c_test_programs/pid_algorithm_sim.c` (build:
`cc -O2 -o pid_algorithm_sim pid_algorithm_sim.c -lm`), plus a test driver
`c_test_programs/run_pid_sim_battery.py` and a differential test
`c_test_programs/test_pid_differential.sh`. Results, figures and the final
battery report live in `firmware/pid_baseline_captures/2026-07-07_sim/`.

**The simulator.** Firmware-exact integer control path (PID + the M17
caller block + the STEP/DIR 64-step quantizer with its 1-step hysteresis and
1-step/tick slew), against a continuous plant: stepper torque curve
`tau_max*(V/200)*sin(load angle)`, inertia, viscous + Coulomb friction with
**stiction** (static 0.13 N*m > kinetic 0.022 N*m — the bench motor is a
stiff-shaft QC reject and the stiction is what makes it interesting), and
white hall-sensor noise. Calibrated until the OLD algorithm reproduced the
2026-07-06 bench baselines: standstill error std ~33, D bias ~−85k, D std
~180k, kI=5 move peak ~10k with settling, kI=25 → sustained ±9k limit cycle
(fig5). NOTE: your existing `servo_simulator` has no plant dynamics
(hall = commanded position), which is why a new one was needed.

**Findings that overturned parts of the original plan:**

1. **The kI limit cycle is integrator-fueled STICK-SLIP**, and the effective
   cure is the **integral clamp at 25% of authority** — it bounds the stale
   integrator "debt" after each slip event. Back-calculation AW at the output
   limit almost never fires in normal operation (kept anyway: it correctly
   handles genuine saturation, e.g. absurd gains). Clamps of 37.5% or more let
   the cycle return; 25% still holds 1.4x the largest measured static effort.
2. **The old algorithm is bistable even at the default kI=5** on the
   high-stiction plant: it limit-cycles on 4 of 10 noise seeds (the bench
   just got lucky). With the fix, no seed limit-cycles for kI 5..100 and
   kI 25-50 settles TIGHTER than today's default (fig6). The original
   single-seed comparisons that suggested "step 1 destabilizes defaults"
   were this bistability, not a real effect.
3. **Step 3 (FIR-32 / velocity-error D) is REJECTED.** Measured and then
   derived analytically: the existing leaky filter's noise gain is ~1.0
   sensor-sigma (the FIR difference is sqrt(2) — WORSE, not ~20x better as §3/P2
   originally claimed); the FIR variant destabilizes at kD ~700k+ where the
   LPF topology stays graceful to 1M+; no measurable benefit anywhere else.
   The LPF topology with the decay-rounding fix is Pareto-better. The §3/P2
   analysis above is retained for history but superseded by this.
4. **kD=0 is unusable** on a stiction plant (post-move RMS in the tens of
   thousands even with anti-windup) — D provides the essential damping of the
   stick-slip cycle. Document "do not set kD to 0" for users.
5. An integration deadband was prototyped and showed no reliable benefit
   (dropped).

**What shipped into 0.15.3.1 (all in `PID_controller` /
`recompute_pid_parameters_and_set_pwm_voltage`):**

- LPF decay rounds toward zero (kills the sticky −80k D bias).
- Output shift rounds to nearest.
- `max_integral_term` reduced by `INTEGRAL_TERM_AUTHORITY_SHIFT` (2 → 25%).
- Back-calculation anti-windup at `pid_output_limit` (= `max_pwm_voltage <<
  FUDGE_SHIFT`; on M23/M1/M2 this limit sits above their callers' own clamps,
  so for them the patch is a near-noop by design). The back-calculation
  preserves the capture tool's exact identity `output == (P+I+D+1024) >> 11`
  even while saturated.
- Verified: `test_pid_differential.sh` extracts the two firmware functions
  verbatim at build time and asserts bit-identical behavior vs the simulated
  design over 4.5M random samples across 9 gain sets — PASSED. M17 and M23
  builds green. Battery: 60/62 checks pass; the 2 exceptions are corner cases
  on the pathological plant (static load ≥ 0.05 N*m with kI≥25 is bistable —
  though still far better than old in median; moves smaller than one slip
  quantum bounce — old is 2.5x worse there). Real cure for the load corner is
  velocity feedforward (still planned, §3/P5).

**Revised remaining roadmap:** bench-verify 0.15.3.1 (checklist in the task
list / §4 step 1-2 gates) → gain-resolution shifts (§3/P4) → velocity
feedforward (§3/P5) → retune defaults, likely kI in the 25-50 range → release.

---

## 7. Assumptions verified / still open

Verified on hardware or in source (2026-07-06):

- ISR rate 31.25 kHz (TIM16 PSC=0, ARR=2048; unit conversions agree:
  `"seconds": 31250`).
- Debug struct layout, `PID_SHIFT_RIGHT=11`, `kP=2000` defaults (recovered
  from live data, exact).
- `test_mode(0)` lockup still present in current source (main.c:880).
- Windup limit cycle at kI=25 (33 Hz, never settles); defaults stable on a
  2 rot / 2 s move.
- Sticky negative D bias exists and matches the floor-shift explanation.
- D term provides no measurable standstill benefit (error std identical
  with kD=0).

Still to verify before/while implementing:

- Exact scaling chain for velocity-error damping (P2):
  `compute_velocity()` calibration constant vs. `current_velocity_i64`
  units (`VELOCITY_SHIFT_LEFT=12`), including sign conventions.
- ISR cycle headroom today (run `get_profiled_times()` on the bench before
  step 1 to record the baseline max time).
- Whether M1/M2/M23 default gains interact badly with any shared-code
  change (at minimum compile + M17-equivalent capture on an M23 if
  available; M23 has its own back-EMF path in the caller).
- The M2 `error >>= 3` special case and the meaning of
  `DERIVATIVE_CONSTANT_AVERAGING_SCALAR` for products other than M17
  (M1/M2 use PID_SHIFT_RIGHT=18 — check int32 headroom for every clamp
  formula when gains change).
- `max_error` currently depends on kP (`(V<<19)/kP`) — after anti-windup,
  revisit whether the error clamp is even needed at its current value or
  should be decoupled from kP.

---

## 8. Bench verification of 0.15.3.1 (2026-07-07)

Captures + fig7 in `firmware/pid_baseline_captures/2026-07-07_bench_fw0.15.3.1/`.
All captures passed the exactness checks (identity now `(P+I+D+1024)>>11`,
detected automatically as "round-nearest"; p_term == error*kP exact; the
integral visibly rails at the new 26,214,400 clamp = 25% authority).

| check | old fw (2026-07-06) | new fw 0.15.3.1 |
|---|---|---|
| standstill D-term mean | −80k…−96k (sticky bias) | **−7k ≈ 0** |
| standstill error std | 37.2 | 37.4 (no regression) |
| kI=5 move (defaults) | 38 RMS, peak err 10.6k | 33–37 RMS, peak err ~5k |
| kI=10 move | (user-reported edge of stability) | 37–40 RMS |
| kI=25 move, kD=175k | 6,865 RMS limit cycle, forever | bistable: 254…3,042 RMS (bounded at half amplitude) |
| kI=25 move, kD=350k | (sim: still cycles — kD alone can't fix old) | **29–39 RMS, 3/3 runs** |
| kI=50 move, kD=700k | — | **38 RMS, 2/2 runs, peak err 3.2k (3x better tracking)** |
| kI=100 move, kD=350k | — | 1,581–1,607 RMS (needs more kD, untested higher) |
| per-command tests | — | test_read_multipurpose_buffer.py, test_go_to_position.py PASS |
| ISR budget | — | loop-too-long watchdog never fired over ~25 min closed loop |

**New tuning rule discovered on the bench (matches the sim):** the remaining
stick-slip hunt on a high-stiction motor dies when **kD is scaled with kI,
roughly kD ≈ 14,000 × kI**. With that, kI=25 and kI=50 settle to ~38 counts
RMS every run — the same settled quality the old firmware could only reach at
kI=5 with luck — while cutting the move tracking lag 2–3x. The windup cycle
itself (the original bug) no longer occurs at any tested kI; what remains at
under-damped settings is a bounded, half-amplitude bistable hunt.

Candidate new M17 defaults for the retune step (validate on a healthy-friction
motor first): kP=2000, kI=25, kD=350000.

---

## 9. Integer-overflow hardening + permanent overflow test (0.15.3.2, 2026-07-07)

**Question answered: can the PID integer math overflow? It could — 15 distinct
undefined-behavior sites were found and fixed; a permanent test now guards them.**

`c_test_programs/test_pid_overflow.sh` (+ `test_pid_overflow.c`) extracts
`PID_controller()` and `recompute_pid_parameters_and_set_pwm_voltage()`
**verbatim from the firmware source at run time** and compiles them under clang
UBSan (`signed-integer-overflow,shift,integer-divide-by-zero`), so any signed
overflow, invalid shift, or div-by-zero is reported at the exact source line.
A meta-test first proves the sanitizer catches a known overflow (no silent
false PASS). The driver is adversarial: a boundary gain grid (kP/kI/kD across
all u32 including sign-flip values; V across every product's shift limit),
17 hostile error waveforms (int64 extremes, alternating, ramps, triangles over
6 decades, spike trains, windup pumps, random soak), mid-flight constant
changes against hot state, and an int64 shadow model that re-verifies the
output identity and all state bounds on every tick.
**Run it after ANY change to the PID code:** `cd c_test_programs && ./test_pid_overflow.sh`
(~2 min; `--quick` for a 15 s smoke). Full run: **1.08 billion ticks, 52,650
config/waveform pairs, 0 overflows, 0 invariant mismatches.**

What it found on 0.15.3.1 (all fixed in 0.15.3.2):

1. `V << (PID_SHIFT_RIGHT+FUDGE)` in recompute overflows int32 for V ≥ 4096
   (M17), ≥ 512 (M23), ≥ 32 (M1/M2 — their defaults overflowed this for years).
2. u32 gains > 2^31 flip sign when stored to int32 → positive feedback.
3. `lpf * 31` decay multiply overflows for kD in [32..48] (V=200).
4. Stale `lpf` × new derivative constant after a mid-flight gain change.
5. `error * kI` and `integral + increment` overflow for large kI × large error.
6. The P+I+D+half sum overflows once the inputs are already out of range.
7. `error - previous_error` overflows for giant errors (loosened deviation limit).
8. Left-shifting negative values (the 0.15.3.1 anti-windup and negation edge
   cases) — technically UB even when it "works".

The fixes (all in `motor_control.c`, verified bit-identical against the sim by
`test_pid_differential.sh`, 6.5M samples including mid-stream constant changes):

- `PID_controller` now takes the callers' **int64** difference and clamps it to
  ±2^29 counts on entry (`PID_ERROR_INPUT_CLAMP`).
- `recompute_pid_parameters_and_set_pwm_voltage` computes in int64 and caps:
  authority ≤ 800e6 (`PID_MAX_AUTHORITY_PRE_SHIFT`), max_error_change ≤ 2e6
  (`PID_MAX_ERROR_CHANGE`), gains ≤ INT32_MAX, and the effective kI such that
  one integration step from a railed integrator still fits int32. The bound
  derivation is in the comment block above those defines.
- Constants changes reset the derivative filter state and re-clamp the
  integral (stale-state hazard).
- Anti-windup corrections use int64 multiply instead of shifting a negative.

None of this changes behavior at sane settings: the sim battery is unchanged
(60/62, same two pathological-plant corners), bench standstill and
kI=25/kD=350k regressions match 0.15.3.1 (RMS 37.1), and a live hardware smoke
test with u32-max gains, the kD=40 corner, and sign-flip kI ran with zero
fatal errors and full recovery. The ISR hot path gained only the entry clamp
(~4 cycles); the 64-bit math runs only when constants change.

---

## 10. Performance impact of the PID changes (2026-07-07)

Two checks: instruction-level assembly comparison (0.15.3.0 vs 0.15.3.2, both
-O3, arm-none-eabi-gcc 10-2020-q4) and on-hardware ISR timing via the firmware's
always-on TIM14 profiler (1 us resolution; identical instrumentation in both
versions, so the comparison is clean — the DO_DETAILED_PROFILING sub-profilers
remain disabled as before).

Function sizes (instructions in the .elf; "common path" = instructions actually
executed on a typical closed-loop tick, traced by hand through the disassembly):

| function | old size | new size | old common path | new common path |
|---|---|---|---|---|
| PID_controller | 128 | 264 | ~70 | ~110 |
| motor_movement_calculations | 152 | 154 | — | — |
| TIM16_IRQHandler (rest of ISR) | 208 | 208 | — | — |
| motor_phase_calculations | 135 | 135 | — | — |
| get_sensor_position | 259 | 259 | — | — |
| recompute_pid_parameters... (COLD: runs only on constants change) | (inlined) | 203 | n/a | n/a |

Where the ~40 added common-path instructions come from: int64 error entry
clamp incl. high-register saves (~11), anti-windup not-saturated compare
branches (~8), register-pressure shuffles in the I path (~5), the half-LSB
rounding constant (~4), the D decay rounding-fix branch (~3, +2 more only when
the filter value is negative), branch restructuring (rest). Much of the +136
size delta is the saturation branch bodies, which do not execute unless the
output rails. The recompute function grew to 203 instructions of 64-bit math
but is cold path (gain/voltage changes only, never in the ISR).

Measured ISR duration on the bench M17 (closed loop, default gains; "window
max" = max over ~62,500 consecutive ISRs):

| condition | old 0.15.3.0 | new 0.15.3.2 | delta |
|---|---|---|---|
| standstill, typical | 21-23 us | 22-24 us | ~+1 us |
| standstill, window max | 23 us | 25 us | +2 us |
| during move, typical | 22-23 us | 23-25 us | ~+1.5 us |
| during move, window max | 26 us | 27 us | +1 us |

Verdict: the changes cost **+1 to +2 us of the 32 us tick (~3-6% of budget)**,
consistent with the static instruction count (~40 instructions ~= 0.7-1 us at
64 MHz plus branch/load cycles). Worst-case headroom is 5 us to the tick period
(was 6 us) and the loop-period fatal watchdog threshold is 100 us. Measurement
scripts: scratchpad profile_isr.py procedure documented here; the asm
comparison is reproducible via `git show HEAD:firmware/Src/motor_control.c`
swap + `arm-none-eabi-objdump -d`.

---

## 11. ISR speed study: entry clamp anatomy + full-loop breakdown (2026-07-07)

### Where the PID slowdown actually came from (and the recovery)

Disassembly analysis showed the +1-2 us was NOT mainly the int64 entry clamp:
the real cost was the **int64 anti-windup arithmetic**, which forced GCC to
allocate high registers (r8-r11) — 25 high-register moves plus a double
push/pop pair on every call, entry and exit.

With the overflow caps of §9 in place, the anti-windup correction provably
fits in int32 (|I|+|P|+|D|+half <= ~1.8e9 → |output| <= ~880e3 → excess *
2^11 <= ~1.81e9 < 2^31), so the int64 there was redundant belt-and-braces.
Converting it to int32 ("V2", applied in the working tree):

| metric | 0.15.3.0 | 0.15.3.2 (int64 AW) | V2 (int32 AW) |
|---|---|---|---|
| PID_controller size (instr) | 128 | 264 | 208 |
| high-register moves | 0 | 25 | **0** |
| common-path (typical tick) | ~70 | ~110 | ~90 |
| bench ISR max, moving | 26 us | 27 us | 27 us (typicals improved ~0.3 us, below 1 us quantization) |

V2 is bit-identical in behavior (differential test: 6.5M samples, 0
mismatches) and UBSan-clean across the full overflow battery. The remaining
~20-instruction delta over 0.15.3.0 is the 64-bit entry clamp (~7 executed),
the AW not-saturated compares (~4), the rounding constant and decay fix (~7).
Removing the entry clamp itself was considered and REJECTED: the alternative
(bounding max_allowable_position_deviation and trusting the main-loop watchdog
cadence) trades a local, provable guarantee for a cross-module timing
assumption, to save only ~0.2 us.

### Measured ISR breakdown (diagnostic build with DO_DETAILED_PROFILING)

Section times at 1 us resolution (the diagnostic build itself inflates the
whole-ISR number by ~5 us of instrumentation; section values are tight):

| section | standstill | moving | notes |
|---|---|---|---|
| get_sensor_position | 8 us | 8-10 us | includes one software division (__divsi3, 229-instr shift-subtract loop, ~1.5-2 us for these operand sizes) + variable-length normalization loop + 9 weight multiplies |
| compute_velocity | 1 us | 1-2 us | fine |
| motor_movement_calculations | 10-11 us | 11-12 us | handle_queued_movements (255 instr: queue state machine, int64 velocity, 96-bit position accumulate) + safety compares + PID (~2.5 us) + M17 output stage |
| motor_phase_calculations | 2-4 us | 2-4 us | contains TWO volatile spin-delay loops (~1 us each) executed on EVERY tick that issues a step: DIR setup delay + STEP pulse-width delay |
| (remainder: hall delta stats, fatal checks, profiler, entry/exit) | ~3-4 us | ~3-4 us | |

### Candidate optimizations (proposals — not implemented except V2)

| # | change | est. saving | risk | recommendation |
|---|---|---|---|---|
| 1 | V2: int32 anti-windup | ~0.3 us every tick | none (bit-identical, all gates green) | **applied; released as 0.15.3.3** |
| 2 | motor_phase_calculations: only run the DIR-setup spin delay when the direction actually CHANGED since the last step (currently it sets DIR + waits on every step) | ~0.3-1 us on every stepping tick (i.e., during all motion) | very low — DIR setup time still honored on every change | do next, after checking the AT5833/GC6609 DIR-to-STEP setup spec in datasheets/ |
| 3 | STEP pulse width by work overlap: raise STEP, do remaining bookkeeping, drop STEP at ISR end (instead of spin-waiting) | ~0.3-1 us on stepping ticks | low-medium — must respect the driver's min STEP high time and max frequency; check datasheet | worth doing with #2 |
| 4 | __builtin_expect / branch-layout hints on the PID + ISR hot branches (M0+ taken branch = 2 cycles, no predictor) | ~0.2-0.3 us | very low | bundle with #2 |
| 5 | get_sensor_position: replace the variable normalization while-loop with a 3-compare binary shift | ~0.2-0.5 us, removes jitter | low | optional |
| 6 | get_sensor_position: replace __divsi3 with a Newton-Raphson reciprocal (16-bit, 2 iterations) | ~1-1.5 us | medium — changes interpolation rounding slightly; needs bench noise-floor re-validation | future work, biggest remaining single win |
| 7 | handle_queued_movements: early-out when queue empty and velocity zero | ~0.3-0.6 us at standstill only | low | optional |

Realistic near-term bundle (#1-#4): recovers ~1-2 us during motion — i.e. all
of what the PID work added — bringing worst-case ISR back to ~25-26 us of the
32 us budget. Item #6 could buy another 1-2 us later if margin is ever needed.

(Historical note: V2 was initially benched under the 0.15.3.2 label, then
released as 0.15.3.3; the current firmware is 0.15.3.4 — see §16.)

---

## 12. V2 extended verification (0.15.3.3, 2026-07-07)

V2 (int32 anti-windup) was kept and released as **0.15.3.3** (M17 + M23 built;
bench flashed). Since the simulator uses a copy of the PID logic rather than
the same .c file, the link that makes sim results transfer is
`test_pid_differential.sh`, which extracts the shipping functions verbatim and
asserts bit-identity against the sim model — green at 6.5M samples.

Off-hardware:
- Full overflow battery: 1.08 billion ticks, 0 UBSan findings, 0 invariant
  mismatches.
- New permanent extended sweep `c_test_programs/run_pid_sim_extended_sweep.sh`
  (82 checks): 20-seed robustness at kI=25/kD=350k and kI=50/kD=700k, noise
  sweep (sigma 10..100), load sweep, healthy-plant no-regression, 120 s
  standstill soaks, slew-limit-violent moves, extreme-gain runs — 82/82.

Bench (0.15.3.3):
- Standstill capture: identity (round-nearest, shift 11) exact, all checks PASS.
- Six move regressions: post-move RMS 32-43 across kI=5/25/50 with scaled kD;
  tracking peaks even lower than the previous session (kI=50/kD=700k: peak
  error ~600 counts).
- **Anti-windup exercised on real hardware**: with the current limit lowered to
  50 (output limit 12800) and kP=8000 + a fast move, the output measurably
  railed; the capture identity held EXACTLY on all 1444 samples including the
  saturated ones — the int32 back-calculation behaves bit-exactly as designed —
  and the integral railed at precisely the scaled clamp (6,553,600). Move
  completed to 3.0000 rotations, no fatal error.
- 100-move endurance soak: no faults, worst sampled position error 88 counts.
- Extreme-gains smoke (u32-max, kD=40 corner, kI/kP sign-flip): all clean with
  full recovery.
- Full hardware suite `run_all_tests.py`: **63/63 PASSED**, total test time 816 s (~13.6 min), in line with the historical ~13:33 baseline.

Sharpened known-corner data (sim, pathological stiction 0.13 plant only):
with a continuous static load of 0.04-0.05 N*m (>=10% of stall) and kI>=25,
the new algorithm is bimodal (some seeds settle to ~40 RMS, others hunt at
~5-6.5k) where the old one hunted consistently at ~2-3.3k; raising kD does not
rescue this corner. On a healthy-friction plant the same load settles
identically for old and new (~440-480 RMS). Planned velocity feedforward
(§3/P5) is the proper cure; until then, heavily-loaded axes on high-stiction
units should prefer moderate kI.

### Supply-voltage sweep (0.15.3.3, 2026-07-07, bench M17)

Identical battery at 12.6 V, ~20 V (earlier session), and 24.0 V (supply
confirmed via Get supply voltage). Captures in
`pid_baseline_captures/2026-07-07_bench_fw0.15.3.3_voltages/`.

| metric | 12 V | 20 V (earlier session) | 24 V |
|---|---|---|---|
| standstill error std / D bias | 39.7 / ~0 | 37.4 / ~0 | 31.6 / ~0 |
| post-move RMS (all gain sets) | 31-37 | 32-43 | 34-37 |
| move peak err, kI=25/kD=350k | ~2770 | 1032-1913 | ~2920 |
| move peak err, kI=50/kD=700k | ~1780 | ~600 | ~1520 |
| aggressive 3 rot / 0.8 s + reversal | clean | clean | clean (incl. hard regen decel, no overvoltage trip) |
| 30-100 move soak | clean, 22 counts | clean, 88 counts | clean, 1 count |
| capture identity / no fatal errors | exact / none | exact / none | exact / none |

Conclusions: settled accuracy and stability are voltage-independent across
12-24 V; every configuration settles to the same ~31-43 counts RMS with zero
faults. Move tracking peaks measured back-to-back at 12 V and 24 V are nearly
identical (~2800-2900 at kI=25), while the earlier 20 V session showed lower
peaks — so the peak variation tracks motor condition (temperature/stiction
drift across the day) more than supply voltage; only the kI=50 set shows the
expected mild headroom improvement at 24 V vs 12 V (1780 -> 1520). No
control-loop behavior change is needed across the supported supply range.

**Theory check (same day, ~20 V re-measured back-to-back with the 12/24 V runs):**
kI=25 peaks 2,214-2,775 and kI=50 peaks 1,631-1,722 — i.e. in the same band as
the 12 V and 24 V afternoon measurements, NOT the morning 20 V session's
1,032-1,913 / ~600. Confirmed: move-peak variation is dominated by motor
condition drift (this high-stiction unit changes through the day), not supply
voltage. Settled RMS remained 35.6-38.6 as at every other voltage.

---

## 13. Snap-back overshoot study (2026-07-08)

Question: after a large disturbance (hand-twist and release), how do the PID
constants shape overshoot and ringing? Commanded emulation: queue
move_with_velocity(0, 0.3 s lead-in) + (15 rev/s, 40 ms) + (0, stop) — the
setpoint jumps 0.6 rotation essentially instantly, the rotor snaps from rest
toward a stationary target at full drive authority. 9 configurations x 5
repeats, fig8 + all CSVs in `pid_baseline_captures/2026-07-08_overshoot_study/`.

Method notes (hard-won): a LONG velocity burst confounds the landing with
kinetic pass-through (rotor still moving at setpoint stop); a tiny step
(<0.01 rot) completes faster than the ~240 Hz debug sampling can see; a
sub-ms burst outruns RS485 command delivery unless the queue is pre-loaded
with a zero-velocity lead-in. The 0.6-rot / 40 ms step avoids all three. The
velocity command ceiling is ~19.5 rev/s (i32 internal units), below the
34 rev/s motor limit.

Results (median overshoot of 5 / settle time, after a 1.97M-count snap):

| config | overshoot (counts) | settle | notes |
|---|---|---|---|
| kD=87.5k | 8,077 | 38 ms | under-damped, ring |
| kD=175k (old default) | 6,418 | 45 ms | under-damped |
| **kD=350k** | **2,435** | **52 ms** | **optimum at kP=2000/kI=25** |
| kD=700k | 3,362 | 124 ms | noise-ring grows |
| kD=1.4M | 6,496 | never | sustained hunt (D-noise cliff) |
| kP=1000 | 3,518 | 63 ms | gentler but softer hold |
| kP=4000 | 8,134 | 47 ms | BIMODAL: 112/169 vs 8k-20k across repeats — stiff + stiction = lottery landings |
| kI=5 | 6,582 | 84 ms | slow undershoot recovery |
| kI=100 | 3,622 | 45 ms | fine — kI barely affects the landing (anti-windup working) |

Conclusions: kD is the overshoot knob and has an optimum (~350k at kP=2000,
i.e. the kD ~= 14,000 x kI rule found earlier also lands here); overshoot after
a full-authority snap cannot be fully eliminated on a high-stiction motor
(the rotor arrives with momentum), but 2,435 counts = 0.27 degrees after a
216-degree snap, settling in ~50 ms with no oscillation, and raising kD
further makes things worse, not better. kI is free to choose for holding
stiffness. Raising kP for a harder landing backfires (bimodal). Repeat spread
is stiction-state variability — judge medians.

---

## 14. Root cause of the snap-back overshoot floor (2026-07-08/09)

Question: classical PID theory says an overdamped/critically-damped no-overshoot
tune should exist — why couldn't §13 find one? Answer, proven in simulation and
confirmed on the bench: **the integral term charges during the approach and is
still pushing forward through the landing.** The loop is NOT underdamped — at
kD=350k the P-D pair alone corresponds to a damping ratio of ~2.4.

Evidence (fig9 + `overshoot_analysis.json`, sim instrumented with `--step`,
`--no-slew`, `--linear-torque` ablations and landing metrics):

1. **Tick-level dissection** (fig9-A, friction and noise removed): during the
   approach the error keeps one sign, so I charges to ~+9,000 output counts
   (its clamp). At the landing D brakes hard, but I cancels it — the TOTAL
   output turns positive again while the rotor is still short of the target.
   The controller pushes when it should brake; the rotor crosses at speed.
2. **The floor is flat vs step size** (5k..2M counts → same ~2-3k overshoot;
   fig9-B). Classical damping predicts overshoot ∝ size. A flat floor is the
   signature of a stored, clamped quantity being "paid off" at the landing —
   the integral rails on any approach longer than a few ms.
3. **More kD makes it worse, not better**: slower approach → MORE time for I
   to charge. This is why §13 found an optimum instead of monotonic
   improvement, and why kI=5 overshot more than kI=100 (slower unwind).
4. **Ablations rank stiction/slew/torque-shape as secondary** (they change the
   number but not the floor's existence; stiction actually ABSORBS overshoot).
5. **BENCH CONFIRMATION**: same 0.61-rot snap, kI=0 vs kI=25 →
   kI=25: 4,756/7,808/8,967 counts; **kI=0: 0/0/0.** QED.

What eliminates it without giving up I (fig9-C):

- **Profiled moves**: a 0.15 s trapezoid over the same distance → 650 counts
  (vs 2,883). Error never grows large, I never charges. Users who care about
  overshoot should command trapezoids, not rely on disturbance recovery.
- **Integral separation** (freeze integration while |error| > ~5,000 counts):
  ZERO overshoot on both bench-calibrated and healthy plants, normal moves
  unaffected (37 RMS), standstill unaffected. Textbook conditional-integration.
  **Not yet in firmware.** Caveat found in sim: on a heavily loaded axis
  (0.05 N*m) 3 of 6 seeds got worse (integrator starved during breakaway when
  the load needs it). Needs a load-aware design (e.g. freeze only when error
  and velocity have the same sign, or a settable threshold) + the full test
  pipeline (differential, overflow, battery, bench) before shipping.

Sim additions for this study (permanent): `--step` instant setpoint jump,
`--no-slew` / `--linear-torque` plant ablations, `--i-freeze-above` prototype,
and vel_at_linear_entry / peak_vel / overshoot_counts metrics.

---

## 15. The D-term noise filter: deep analysis (2026-07-09)

Question (tom): the LPF on the error-change was put in "fairly arbitrarily" —
is it needed, is it right, would a different filter be better? Study artifacts:
fig10 + raw captures + sweep JSONs in
`pid_baseline_captures/2026-07-09_dfilter_study/`.

**What it is.** `lpf = decay(lpf) + Δerror; d_term = lpf * (kD >> SHIFT)` with
decay = (SHIFT=5) 31/32 per tick — a leaky integrator / EMA, DC gain 2^SHIFT,
τ = 2^SHIFT ticks ≈ 1.0 ms, cutoff ≈155 Hz. The kD >> SHIFT rescaling keeps
the effective derivative gain constant if SHIFT changes. This is precisely the
standard industrial "derivative filter" (typically τD/8..τD/20) — the
"arbitrary" structure is the textbook one. The only change ever made to it was
the 0.15.3.1 rounding fix (decay now rounds toward zero; previously negative
values −31..−1 never decayed → the ≈−85k standstill bias). Topology untouched.

**Measured noise (the filter's reason to exist).** Per-tick hall position at
standstill, rotor magnetically locked (cmd 7, capture type 2, decimated 16:1
to fit RS485): σ = 38.5 counts/tick and WHITE — sum-of-16 and sum-of-64
variance ratios 1.00/0.99 vs the white prediction; spectrum flat 1–977 Hz.
Consequences: (a) differentiating this without a filter is hopeless; (b) white
noise has no structure to exploit, so no notch/matched filter helps and a
higher-order low-pass only buys steeper rolloff at the cost of more lag —
the time constant is the ONLY knob that matters.

**Sweep (sim `--d-shift` + bench builds via
`EXTRA_DEFINES=DERIVATIVE_CONSTANT_AVERAGING_SCALAR_SHIFT=n`, kP=2000/kI=25/
kD=350k).** D-noise halves per +1 shift exactly as theory predicts (bench:
1.85M / 869k / 415k / 201k / 113k for shifts 3..7). Control quality:

| SHIFT | τ | bench move settle RMS | bench snap overshoot (median of 3) |
|---|---|---|---|
| 0 (no filter) | 32 µs | 1,181 (sim; cannot settle) | — |
| 3 | 0.26 ms | 41 | 5,631 |
| 4 | 0.51 ms | 39 | 4,901 |
| **5 (current)** | **1.02 ms** | **41** | **2,306** |
| 6 | 2.05 ms | 41 | 7,493 |
| 7 | 4.10 ms | **27,911 — sustained hunt** | bimodal |

Control quality is FLAT across shifts 3–6 (the loop absorbs the leftover
noise), then collapses at shift 7 where τ≈4 ms approaches the loop's own
dynamics: D brakes 4 ms late and sustains the stick-slip cycle. The sim had
predicted the cliff one notch later (shift 7 mild, favoring 6) — the bench
overrules it; snap medians vary within the motor's stiction day-spread and no
shift beats 5 outside that spread.

**Recommendation: keep SHIFT=5, change nothing.** It sits mid-plateau with two
full steps of margin to the instability cliff, and its residual D noise
(±200 output counts) measurably costs nothing in error, settle, or landing.
The filter is required (no-filter cannot settle a move), and the structure is
already optimal for white noise. The define now carries an explanatory comment
+ #ifndef guard for future studies; NOTE for any future shift increase: the
overflow cap PID_MAX_ERROR_CHANGE must shrink to 2^31/(2^s(2^s−1)) (533k for
s=6, 131k for s=7) and the overflow battery must be rerun.

---

## 16. Ultracode final review pass -> 0.15.3.4 (2026-07-10)

Before committing, a multi-agent adversarial pass reviewed every changed file
(9 lenses x refuter verification, 105 agents), constructed new offline test
dimensions, and re-ran everything. 46 raw findings -> 26 confirmed after
adversarial verification. The important ones, all fixed:

**Firmware (all in `recompute_pid_parameters_and_set_pwm_voltage`):**

1. **Low-current-limit torque regression (major).** `pid_output_limit = V << 8`
   sat BELOW the 90-degree commutation angle (16,384) whenever the max PWM
   voltage was set under 64, so the anti-windup clamp cut the achievable lead
   angle (45 deg at V=32 -> 71% torque; 22.5 deg at V=16 -> 38%) vs old
   firmware. Fixed: the saturation point is now
   `max(V << 8, HALL_TO_POSITION_90_DEGREE_OFFSET)`. Proven on hardware
   (H4 in the campaign: at V=25 the output demonstrably exceeds the old
   V<<8 ceiling).
2. **Derivative kick on constants changes (minor).** recompute zeroed
   `previous_error`, so the next tick's error_change equaled the full current
   position error (up to ~3% of authority) — an audible jerk on every
   Set-PID-constants / Set-max-current during motion. Fixed: previous_error is
   kept (provably overflow-safe: always a previously-input-clamped value).
3. **Over-conservative kI cap at tiny kP (minor).** The increment-safety cap
   divided by max_error even when the +/-2^29 input clamp is the smaller true
   bound. Fixed: divides by min(max_error, PID_ERROR_INPUT_CLAMP).

**Test-harness hardening (findings + the new-tests fleet):**

- `test_pid_overflow.sh` now loops ALL product parameter sets by default
  (M17 shift 11 / M23 shift 14 / M1-M2 shift 18 with the M2 error>>3 prescale
  compiled in and mirrored in the shadow model) — full run = 3.24 BILLION
  UBSan-checked ticks, zero findings. It also extracts
  INTEGRAL_TERM_AUTHORITY_SHIFT and DERIVATIVE_CONSTANT_AVERAGING_SCALAR_SHIFT
  from the firmware sources instead of trusting stub copies, and both
  harnesses accept PID_SRC=<copy> for mutation testing.
- **Mutation testing** (new, permanent technique): 8 single-bug mutants of
  motor_control.c (rounding, AW sign, clamp removals, state-reset removal,
  entry-clamp removal, authority-shift change) — **16/16 killed** by the
  differential + overflow harnesses. The suite demonstrably catches real bugs.
- Capture-tool self-test fleet found and we fixed: false-FAIL on saturated
  anti-windup samples whose corrected integral also clamps (identity provably
  cannot hold; now recognized via --output-limit), same exemption for the
  kI=0 check, integral-rail report updated to the 25% clamp, corrupted-CSV
  rows now fail with a clean error, cleanup no longer masks exceptions when
  the port never opened, detect_shift([]) no longer returns vacuous matches.
- Sim: CSV 'error' column now logs the post-clamp value exactly like the
  firmware debug struct; --help documents all flags; step2 mirrors all three
  firmware fixes (differential bit-identity re-verified: 6.5M samples, 0
  mismatches).

**Re-verification of 0.15.3.4:** M17 + M23 + M17-JIG builds green; overflow
3x1.08B ticks clean; differential 6.5M bit-identical; battery 60/62 (the two
documented pathological-plant corners); extended sweep 82/82; mutation
spot-check still kills; hardware campaign (below) green; full hardware suite
63/63.

Notable NON-issues confirmed during review: servo_simulator's build failure is
pre-existing at git HEAD (its Makefile passes -DPRODUCT_NAME_M3, rejected by
the firmware sources — TODO for another day) and is unaffected by this work;
the working tree also contains an unrelated docs-accuracy work stream
(error_codes.json / motor_commands.json / API_documentation /
BUGS_FOUND_2026-07-09.md) that must be committed separately from the PID work.

## 17. Load-holding / torque-at-small-error study (2026-07-11)

**Tom's question:** can the loop apply maximum torque when the position error is
only ~0.1% of a rotation (3,277 counts), via integral windup? **Answer: no — and
this is the one real capability the anti-windup clamp traded away.** Study is
sim-only (motor was disconnected); 1,157 runs across 4 independent analyses,
overflow violations 0; data + fig11 + zero-cross prototype archived in
`pid_baseline_captures/2026-07-11_load_holding_study/`.

**Where we stand (V=200, kp=2000, ki=25, kd=350000):**
- NEW fw (25% I clamp): sustained output at e=3,277 is P(3,200)+I-rail(12,800)
  = 16,000 of 51,200 → 0.124 N·m = **31% of stall** (V=80 default: 28.6% of that
  ceiling). Full torque needs e ≥ 39,322 counts = 1.20% rev. Windup DELAY is a
  non-issue (I rails in 2–13 ms at ki=5..100); the CEILING is the issue.
  Holding a constant load leaves a permanent standing error: closed form
  e_ss ≈ 1.024*((tau_L−0.022)/7.8125e-6 − 12,800) counts; measured 9.3k/23.5k
  counts at 0.20/0.30 N·m (0.29%/0.72% rev). ki changes none of this.
- OLD fw (100% windup): does reach 100% torque at any error after ~41 ms, BUT
  hunts perpetually at 17–22 Hz (±3k counts) for EVERY load in 0.14–0.26 N·m
  (= wherever tau_max − load > static friction) at every ki, and above 0.27 N·m
  it "holds" at a bimodal 0.9k–18k standing error while driving permanent
  full-rail power (heating). Old was never actually good at this either.
- Raising the fixed clamp is a dead end: the no-load post-move limit cycle
  returns at **31.25%** (one notch up; 5/5 seeds at ki=10/100) — tighter than
  the earlier ≥37.5% note. Physics: I-rail torque must stay under breakaway
  friction (25% → 0.094 < 0.13 N·m; the margin exists only for the stiff bench
  unit — at fs ≤ 0.10 even 25% hums in sim, so fleet stiction spread must be
  measured before ANY clamp increase). Fracs 5..10/16 buy ZERO standing-error
  improvement at 0.20 N·m anyway (rotor stuck inside the friction cone).
- Other rejected mechanisms (all sim-tested): two-tier slow windup (hunts under
  load, 24k snap-back overshoot, never settles at 0.30); integral deadband +
  100% clamp (cycle amplitude ~3k >> any sane deadband); high kP (kp=32000
  actually meets the 0.1% spec instantly with no I at all, but latches a
  285–450 Hz full-rail relay scream after steps, 5/5 seeds — usable only as a
  supervised special mode, not a default).

**The one mechanism that achieved both goals in sim: zero-crossing-gated
integral ("avg" mode).** I may wind past the 25% rail toward FULL authority only
after the error has kept one sign (outside a ±1,000-count deadband) for 0.1 s;
on each zero crossing, I is set to the midpoint of its values at the last two
crossings (self-tunes to the true hold torque, since the friction band is
symmetric about it) and integration freezes 0.1 s. Naive clamp/freeze/bleed
variants all fail (4.5 Hz relaxation cycles or step2-equivalent error) — the
midpoint rule is the load-bearing invention. Sim scorecard: no-load bit-quiet
(gate dormant, identical to step2, ki 5–100); load 0.20 error 9.5k → ~600
counts settling in 0.1–0.6 s; old-style hunting never returns; snap-back
overshoot 22% BETTER than step2 (2.2k vs 2.9k) and fixes step2's hidden
−2.7k post-step park; Tom-scenario 3,277-count step: old hunts forever, step2
parks −2.0k..−2.8k forever, gate −915 (db500 variant: −350..−420 but needs
ki ≥ 25). Accuracy floor = deadband (±1,000 counts = 0.031% rev, 3x inside the
0.1% spec). Cost ~10–15 instr/tick. NOT IMPLEMENTED in firmware.

**Port prerequisites if adopted** (from the independent overflow audit):
letting I reach 100% of authority breaks the int32 proof at the current
PID_MAX_AUTHORITY_PRE_SHIFT=800e6 (worst |P|+|I|+|D| = 2.4e9 > 2^31). Lower the
cap to ≤600e6 — **M17-specific #ifdef only**: on M17 legal V ≤ 390 → real
authority ≤ 204.5M so the change is free, but on M1/M2/M23 the 800e6 cap BINDS
(raw authority 4.3–20e9) and lowering it globally would silently retune them.
ki cap must use I_FULL; back-calc clamps to ±I_FULL. Rerun test_pid_overflow.sh
(3 product sets) + differential + battery + sweep; then bench: re-measure
breakaway jump quantum (deadband ≥ jump/2), constant-load rig (weight over
pulley), spring load (sign-reversing — unmodeled in sim, I unwind lag could
re-enable hunting), sudden load REMOVAL (excursion ~I_excess/kp ≈ 0.8% rev
worst case — not expressible in the sim), sustained-stall thermal soak.

**Recommendation:** ship 0.15.3.4 as-is now (quiet + predictable; standing
error under load is honest and documented), treat the gated integral as the
next firmware feature behind its own full verification pipeline + bench
campaign. Fallback for users needing stiffness today: raise kP (documented
relay-scream risk) or raise max PWM voltage (more absolute torque at the same
% ceiling).

## 18. Rounding-code necessity re-examination (2026-07-11)

Tom challenged whether the two rounding changes matter, suspecting the
anti-windup was the only meaningful fix. A 3-agent ablation/math/cost analysis
(archived in `pid_baseline_captures/2026-07-11_rounding_necessity/`; sim gained
`--floor-output` / `--sticky-decay` ablation flags for this) confirmed his
headline: **the anti-windup (25% clamp + back-calculation) alone reproduces the
entire limit-cycle fix** — AW-only matches full step2 at every kI within seed
noise, while old blows up (e.g. kI=25 post-move RMS: old 3.7k, AW-only 53,
full 50). R1 (output round-to-nearest): proven ≤1 output count, non-accumulating
through the AW path (exhaustive over all 2^32 sums + 5e6-tick saturated dual
sim) — cosmetic. R2 (D-decay round-toward-zero): NOT a 1-count issue — it
removes a real, kD-proportional stuck-filter bias (−85k pre-shift at kD=175k,
matching bench) — but the bias never reached the plant (0/187,500 tick position
divergence even at 8x kD; DC push sits under stiction), so its value is
truthful debug data + removing a one-sided standstill drive. **Decision: keep
both** (cost ~7 instructions ≈ 0.1 µs/tick; removal would force weakening the
H10 hardware gate and two battery checks plus a full re-verification cycle for
strictly-equivalent code).

## 19. Zero-crossing-gated integral: implementation + release-overshoot fix (0.15.4.0/0.15.4.1, 2026-07-12/13)

**0.15.4.0 — the gate (plan §17) implemented in firmware.** M17-only (#ifdef):
authority cap lowered to 500e6 on M17 only (int32 proof with full-authority I;
non-M17 keeps 800e6 — cap BINDS there), new `max_integral_term_full` used by the
ki cap / back-calc clamps / constants-change re-clamp, gate state (4 words)
reset at all PID-reset sites. Verified: differential 6.5M samples bit-exact
(harness now #ifdef-aware — TRAP: it previously compiled extracted code without
PRODUCT_NAME_M17, which would have silently tested the gateless path; 6/6
mutations killed incl. lost-define + inverted-cap), UBSan overflow 3 sets x
1.42B ticks with proven gate coverage (152M above-rail ticks), sim port
bit-identical to prototype 14/14, battery gained gate section (77/79), review
agent 0 must-fix. Bench (fw flashed same day): ISR window-max 26/27-28 µs vs
26/27 baseline (cost ≤ 1 µs resolution; PID_controller static size 208→296);
capture identity exact; **objective A/B at low current: settled error after a
0.1%-rev step at current 25: 0.15.3.4 parks 1,991 counts short forever (I pinned
at 25% rail), 0.15.4.1 settles at 1 count (I reached 317% of rail)**; at
current 15: 2,289 → 791 (I at full authority).

**Bench-found design flaw (Tom, hand test): release-after-sustained-hold
overshoot.** Any hand/load interaction ≥ 0.1 s opens the gate and charges I
toward full authority; on release the rotor flies home, and the post-crossing
midpoint (computed against a stale sample) left a large frozen I pushing
through the target for the whole 0.1 s freeze. Reproduced in sim after adding
`--load-until` (load vanishes mid-run): 13.4k-count overshoot vs 2.9k gate-off.
NOT the freeze duration (short holdoff made 0.30-load releases WORSE — gate
reopens during the excursion).

**Fix (0.15.4.1) — "freeze-unwind + stale-crossing clamp"**, selected from 5
sim candidates (bleed/cross-to-rail/midpoint-down rejected with data — bleed
defeats the self-tune and hunts at low stiction):
1. During the post-crossing freeze, integration steps that REDUCE |I| are
   applied, floored at 0 (freeze only blocks re-charging).
2. A crossing whose previous crossing is > INTEGRAL_GATE_STALE_TICKS (2x
   holdoff) old is a release/first event: its midpoint is clamped to the 25%
   rail (new saturating counter `integral_gate_ticks_since_cross`, INT32_MAX =
   no history).
Sim results: release overshoot 13.4k → 5.3-5.6k (settle 0.11 s; residual is
rotor momentum), load-hold accuracy IMPROVES (662 → 285-400 counts), Tom-step
scenario improves (-914..755 → ~250), all battery/no-regress gates pass.
Adversarial verification (19 seeds, timed releases every 0.05 s, pulsed loads,
ki/kd/load extremes): CONFIRMED-SHIPPABLE in the recommended envelope
(ki ≤ 100); winner also CLEANS unfixed-gate hunt pockets at ship gains
(L0.14/ki5/kd175k hunted 6/10 seeds unfixed → 0/10) and is strictly better at
catastrophic extremes (ki=5000: unfixed runs away to the input clamp, fixed
stays bounded). **Documented out-of-envelope corners (firmware accepts these
gains; candidates for a future bounded-unwind remedy — do NOT ship untested):**
(a) sustained-hold hunt at ki 200-300/kd175k and ki ≥ 500/kd350k (std
600-1,300 vs 33); (b) load exactly == stiction with ki5/kd350k hunts 10/10
seeds at 3 Hz (narrow pocket; ship gains clean); (c) cyclic loads: ~40% better
release overshoot but ~40% worse re-application sag (I drains between cycles);
(d) release from saturated (≥0.24 N·m) holds still ~9-12k (charge accrues
before any crossing exists — uncatchable by crossing rules).
Harness mirrors updated + mutation-proven (differential 7.0M samples 0
mismatches, 3/3 new mutations detected incl. a probe pinning the stale
threshold from both sides; overflow adds release-stress phase + 4 mandatory
coverage counters, full 3-set run clean). PID_controller static 296 → 372
instructions; bench ISR unchanged (26/28 µs window-max).

**Status: 0.15.4.1 on the bench motor, held for Tom's subjective re-test.
UNCOMMITTED. Remaining before commit/release: Tom's hand verdict; remaining
bench items: disable→displace→enable transient (review §16 finding), spring
(sign-reversing) load, sustained-stall thermal soak, fleet stiction spread;
decide whether the out-of-envelope corners need the bounded-unwind remedy
first. The mislabeled fw0.15.4.0 artifacts (rebuilt with fix code before the
version bump) were moved to .bak; only 0.15.4.1 binaries are real.**

## 20. Large-inertia (plastic disk) overshoot characterization (2026-07-13, bench)

Tom mounted a large plastic disk (large J) and saw big overshoot. Measured
programmatically via hall-position polling (~280 Hz; the PID debug error field
clamps at max_error and pegged — do not use it for large-overshoot work):
snap (0.05 s) trapezoid steps at current 200, kp2000/ki25/kd350k: 0.05 rot →
10.2k counts (1.1°); 0.20 rot → 150k (16.5°); 0.61 rot → 214k (23.5°).
**Mechanism is BALLISTIC, not loop tuning:** output railed (peak ~400% of I-rail
= full saturation) through the catch-up, so stopping distance = J·ω²/2τ.
Evidence: ki=0 vs ki=25 within 15% (integral not the driver); kd sweep 350k→5.6M
barely helps (150k→90k) and kd ≥ 2.8M hunts with the disk (settle never);
move-time series collapses it: 0.2-rot move in 0.05 s → 18.5°, in 0.2 s →
**0.05°** (448 counts), 1.0 s → 0.02°. The cliff sits where demanded accel
crosses achievable τ/J. Conclusion: with big inertia, command profiles the
torque can follow (or raise the current limit); no gain change fixes a
saturated approach. Hand-release overshoot with the disk is the same physics
(momentum) — the 0.15.4.1 fix removes the integral contribution only.
Scripts: scratchpad measure_overshoot2.py / measure_movetime.py (hall-polling
method worth promoting to tools/ if reused).

## 21. Per-load PID tunability study (2026-07-13, sim; 3,762 runs + adversarial validation)

**Tom's requirement:** a no-overshoot tuning for the bare rotor, another for a
loaded shaft — "the PID should be tunable for any kind of load." Bench with the
loose plastic disk was irreproducible (backlash; 0.75° vs 7.5° same config), so
the quantitative answer comes from the sim (rigid --inertia), fw 0.15.4.1
semantics, snap 0.2-rot step, V=200. Results archived in
pid_baseline_captures/2026-07-13_per_load_tuning/.

**VALIDATED TUNING TABLE (snap-step overshoot; 9,102 counts = 1°):**
| load (× rotor J) | tuning | snap overshoot | caveats |
|---|---|---|---|
| ×1 (bare) | kp2000 / ki25 / **kd700k** | 1.1–1.3k counts (0.13°) | robust seeds 1–12, ki 5–100, low stiction; at 2× hall noise prefer kd350k (default) |
| ×5 | **kp500 / ki25 / kd1.4M** | 1.2–1.4k (0.14°) | settle 0.2–4 s (parks inside ±1000 deadband); fallback kp250/kd1.4M = 5.8k but 0.23 s |
| ×20 | **none good** | best 1.8–2.0k (kp500/kd1.4M) | permanent 1.5k-count 3.8–4 Hz limit cycle at EVERY ki 1–100; the "quiet" cell (kp2000/kd350k, 65.6k ov) is stick-slip parking — hunts at low stiction/small steps/releases. Use profiled moves (default gains track 2-rot/2-s at postrms ~47). |
| ×50 | kp250 / ki25 / kd1.4M "best available" | 15–18k (1.7–1.9°) | hunts in ~20% of seeds; kp250/kd5.6M REJECTED (1/24-seed 24° tail at low stiction — noise-saturated D fails to brake) |
Amplitude bound: snap steps ≥ 0.6 rot at J ≥ 20 = 25°+ for EVERY tuning
(saturated ballistics). Universal: standstill dormancy safe everywhere; the
0.15.4.1 release fix never re-breaks; at J ≥ 20 the ki-driven stick-slip cycle
is the residual limiter (ki 1–100 all sustain it — ki/deadband sweep at heavy J
is the top follow-up).

**Architecture findings (H1/H2 quantified on scratch sims):**
- H1 braking window CONFIRMED: error clamp = authority/kp blinds the loop
  beyond 52k counts at kp2000; kp2000 heavy-J overshoot matches the
  stopping-distance-minus-window bound. Lowering kp widens the window — the
  legitimate in-firmware per-load knob (knee at kp~500/J5–20, kp~250/J50).
- **Widening the error clamp REJECTED**: the clamp is the architecture's
  emergent braking trigger (P railed at +authority cancels against mec-railed D
  at −authority exactly at the window edge). An 8× window made J50 overshoot
  47° (P un-cancellable + gain-1 back-calc slams I to the wrong rail).
- H2 D-noise ceiling CONFIRMED: mec = authority/kd ≈ hall-noise σ at kd~2.8M →
  stock usable kd caps at ~1.4M, J-independent. **Velocity-D (unclamped int64
  error_change + int64 D/LPF/sum) is the worthwhile future firmware option**:
  its noise ceiling scales WITH inertia (J50 quiet at kd=22.4M = 16× more
  damping authority) → J50 snap overshoot 0.2° quiet (9× better than stock
  best), release peak 3–4× better, and it needs a per-load mode switch anyway
  (at bare J it is WORSE — noise). Cost: int64 LPF/D/sum in the tick (ISR
  budget re-check required), keep PID_MAX_ERROR_CHANGE=2e6 as wrap protection.
  NOT implemented; candidate §22 after the heavy-J ki/deadband sweep.

## 22. Hands-on smoothness: slewed gate corrections (0.15.4.2, 2026-07-13)

**Bench findings (Tom, disk rig):** on 0.15.4.1 with kI active: (1) buzz when
holding the shaft slightly off-position, (2) jerkiness when rocking it through
the setpoint, (3) release overshoot. Killer diagnostic: at kI=0 (gate machinery
inert) ALL THREE vanish — but return is weak/slow and it parks short. So the
roughness is the gate itself under human interaction — never measured before
(all prior scenarios were hands-off).

**Made measurable:** sim gained --forced-rock/--forced-hold (rotor externally
prescribed) + per-tick |ΔI|/|Δoutput| smoothness metrics. Quantified on
0.15.4.1: every rocking crossing is "stale" and the midpoint clamp slams the
integral by 79–105M pre-shift IN ONE 32 µs TICK = a full rail-to-rail torque
reversal (the jerk); holds ramp rail→full at raw error·ki (3.4–20 ms; the buzz,
plus stick-slip and disk backlash).

**Fix shipped in 0.15.4.2 — crossing-correction SLEW (N4):** corrections become
a target the integral approaches at (full authority >> 4) per tick during the
freeze, with the release unwind running in parallel (no snap-back; abandoned
without a jump at freeze expiry; crossing sample stays pre-correction). One-tick
|ΔI| 103M → 6.58M (bound airtight in a 630-run adversarial rock grid), rock
output slam 51k → 3.6–4.9k counts. Slew shift 4 is a hard cliff: N≥5 hunts
d-shift-7 holds (9/10 seeds). Bonus: cures the §19 ki≥200 hold-hunt corners
(5/5→0/5) at their original cells.

**REJECTED companion — above-rail windup rate cap (M11):** designed to soften
the hold-fight ramp and initially selected as part of the winner; adversarial
verification BROKE it: the cap equilibrates against the back-calculation drain
and pins holding torque at ~68% of rail for loads ≥ 0.26 N·m (parks 7.9° off,
10/10 seeds, permanent) + 26 new hunt pockets (13 in-envelope). Firmware ships
SLEW-ONLY; the hold-fight ramp stays raw (buzz may partially remain during
deliberate sustained holds — bench verdict pending). Any future wind cap must
solve the back-calc interaction first. Also honest: the §19-corner "cures" are
partly relocations (same gains hunt at other stiction/J cells); pocket-hopping
is a pre-existing gate trait.

**Slew-only re-verification:** heavy-load hold cell that killed M11 now back at
peak_i=100% with base-class parks; rock ΔI bound 6.58M; release 2.8–8.6k;
Tom-step park −6..−42. Battery 160/162 (2 documented corners) + sweep 82/82
(regression agent, post-sim-edit). Differential + overflow mirrors updated for
slew-only with mutation proofs (see workflow results).

**Also this session: per-load return-speed law.** Return cruise speed ≈
authority/kD counts/tick (P-rail vs D-rail balance) — kD is both the landing
brake AND a cruise governor; kI adds up to 2× more. At heavy J, "10 rev/s
return + soft landing" is impossible for fixed gains (commanded ≥0.5-rot snaps
at J≥20 land ballistic at 8–38° with ANY gains once kI drives the approach);
that requires trajectory-aware braking (§21 velocity-D / cascade — future).
Bench default for the disk rig: kp500/ki25/kd1.4M @ current 390 ≈ 2.8 rev/s
cruise; kp2000/ki25/kd350k gives ~10 rev/s hand-release returns with fine
small-step behavior but crashes commanded multi-turn snaps at heavy J.

## 23. VERDICT: the gated integral (0.15.4.x) is REJECTED AND REVERTED (2026-07-13)

> **Note added 2026-07-31, when this record was committed to git.** The two sentences below
> were true on 2026-07-13 and are kept as written. They are NOT the current state: the tree now
> ships fw **0.15.9.0**, and the version numbers 0.15.4.0–0.15.4.2 were later **reused** for real,
> shipped releases that have nothing to do with the gate. So two different builds share the name
> `servomotor_M17_fw0.15.4.0_scc3_hw1.5.firmware` — the real released one in
> `firmware/firmware_releases/` (36,929 bytes) and the rejected gate build preserved here as
> `...firmware.bak` (37,025 bytes). Do not confuse them. For what the real 0.15.4.0 onwards
> actually changed, see the behaviour-by-firmware-version table in
> `API_documentation/autogeneration/knowhow.md`.

**Sections 17–22 describe an experiment that FAILED and was reverted. The shipping
firmware is 0.15.3.4 (commit 858544d). No 0.15.4.x code exists in the tree.**

Tom's bench verdict on 0.15.4.2, after every repair attempt: the **whine/vibration when
holding the shaft off-position** and the **jerkiness when rocking it through the setpoint**
were still there and "quite bad". Decisive diagnostic: at **kI=0 (gate machinery inert) all
complaints vanish**. The roughness is the gate itself.

Root cause (the lesson to keep): **the gate makes the loop's behavior depend on interaction
history**, and its state machine (same-sign counter, crossing midpoints, freeze windows,
staleness detection) runs on exactly the 0.1–1 s timescales a human hand works at — so a
hand continuously re-triggers it and feels the torque *trend* shifting. This is structural
to any state-machine-gated integrator, not a tuning or smoothing defect. The slew fix
(0.15.4.2) reduced per-tick torque steps 15× and changed nothing a hand could feel: a
0.5 ms ramp and a 32 µs step are the same thing to a finger. Our whole validation suite —
which the design passed, including the objective goal (a 0.1%-rev step that 0.15.3.4 parks
1,991 counts short of converged to 1 count on the bench) — measured only hands-off
scenarios.

**Full post-mortem, preserved rejected source, rejected binaries, and the list of
directions that do NOT carry this flaw (feedforward hold-torque estimation being the most
promising):**
`pid_baseline_captures/2026-07-13_gate_experiment_REVERTED/POSTMORTEM.md`.

Everything else in §17–§22 remains valid and useful: the load-holding physics (§17), the
per-load tuning table and the braking-window / D-noise-ceiling architecture findings (§21),
the large-inertia ballistics (§20), and the return-speed law (cruise ≈ authority/kD; §22).
