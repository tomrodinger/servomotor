# PID research campaign, 2026-07-14 — a different approach

**Status: RESEARCH ONLY. No firmware, no repo code, and no simulator source was modified.**
All experiments were run on scratchpad copies of `c_test_programs/pid_algorithm_sim.c`.

**Read this before the next PID attempt.** It reframes the problem: the biggest levers found are
*not* in `PID_controller()`. Three of them are not code changes at all.

Method: ~57 agents across three fleets — a 30-topic prior-art/control-theory sweep, 4 simulator
experiments (torque linearisation, PTOS braking curve, disturbance observer, cascade+feedforward),
4 hardware deep-dives, and an architecture design panel — followed by adversarial verification
(a refuter, a hand-feel reviewer, and an ISR-budget reviewer), which **killed or downgraded most of
the headline results, including the one this campaign started from.**

---

## 1. Eight facts, verified in source, that change the picture

### 1.1 We have been reasoning about the wrong driver chip
M17 hw1.5 is **SCC=3** (`firmware/VERSIONS`: `M17 1.5 3`) → `motor_control.h:7-9` includes
**`AT5833.h`**, not `GC6609.h`. The link map confirms every `GC6609.o` section is at address 0x0
(garbage-collected away); `init_GC6609_through_UART()` is not in the shipped image.

The board carries an **ATD5833** — an A4988/DRV8825-class fixed-off-time chopper with **no UART and
no registers**. So: **no StealthChop, no SpreadCycle choice, no INTPOL/MicroPlyer, no MRES.**
MS1/MS2 are static GPIO straps driven high in `AT5833.c` → 1/16 microstepping, which is **already the
chip's finest setting**.

⇒ **The 5.625°-electrical angle quantum is a permanent property of this board.** Any plan that
depends on 1/32, 1/64 or 1/256 microstepping needs a board respin. (Two of the research agents
recommended exactly that before this was caught.)

### 1.2 The current knob is a 1.6 kHz actuator, not an instantaneous one
The schematic *is* in the repo (`PCB/Servomotor3/1.5/`, plus `production/bom.csv` and the netlist):

```
PA8 (TIM1_CH1 PWM) ──[ R22 = 10 kΩ ]──┬── ATD5833 VREF (pin 17)
                                      └──[ C1 = 10 nF ]── GND
```
τ = **100 µs = 3.1 control ticks**, f_c = **1592 Hz** (KiCad annotates the box "Cutoff frequency:
1592 Hz"), 10-90 % rise = 220 µs ≈ 7 ticks.

Current scale (ATD5833: `I_trip = VREF / (8·Rs)`, Rs = 150 mΩ): **1 count = 5.36 mA**;
`max_motor_pwm_voltage` 200 → 1.07 A; 390 → 2.09 A.

### 1.3 TIM1 has no CCR preload, and its period beats against the control tick
`PWM.c` sets PWM mode but never sets `OC1PE`, so `TIM1->CCR1 = motor_pwm_voltage` (written every tick
at `motor_control.c:2813`) lands **asynchronously mid-period** → occasional double/extended pulses.
And f_TIM1/f_TIM16 = 124.76 kHz / 31.23 kHz = **3.9942**, not an integer, so the write instant walks
through the PWM period once every ~5.5 ms → a **~181 Hz beat** on the VREF ripple. That is squarely
in the audible/tactile band.
Fix: `TIM1->CCMR1 |= TIM_CCMR1_OC1PE;` at init. One line, zero runtime cost.
(Also: `PWM.h`'s comment says the M17 TIM1 PWM is "about 250 kHz". It is **124.76 kHz**.)

### 1.4 `#define HYSTERESIS 1` in the step engine is dead code
`motor_control.c:2828-2834`: a `steps_to_target` of ±1 fails both `> HYSTERESIS` and `< -HYSTERESIS`,
so it passes through unchanged and **a step is issued**. The real deadband — `if ((steps_to_target >=
-1) && (steps_to_target <= 1)) break;` — is commented out at line 2825. Confirmed independently in
the disassembly by two agents.

⇒ The step engine chases hall noise across the 1024-count boundary. In simulation it emits
**~14,000 STEP pulses/s at standstill**, flipping the electrical angle ±5.625° at ~14 kHz.

### 1.5 The torque map is quadratic in the PID output
The M17 output stage (`motor_control.c:2627-2646`) drives **both** actuators from the same number:

```
lead angle θ = clamp(u, ±16384 counts)     (16384 = 90° electrical)
current    I = clamp(|u| >> 8, 0..V_max)
torque     ∝ I · sin θ   ⇒   T ∝ u²  for |u| < 16384 (i.e. below 32 % of stall)
```

| u | I | θ | torque | small-signal gain |
|---|---|---|---|---|
| 1,000 | 3.9 | 5.5° | 0.19 % of stall | 1× |
| 4,000 | 15.6 | 22° | 2.9 % | 4× |
| 16,384 | 64 | 90° | 32 % | 10× (peak) |
| 51,200 | 200 | 90° | 100 % | 5× |

The model reproduces the plan's own measurements exactly (§17's "31 % of stall at e = 3,277" → 31.2 %).
Consequences, computed from the shipped constants:
- Nudge the shaft 100 counts → u = 98 → `I = 98>>8 = ` **0** → literally zero current, zero torque.
- The motor cannot break away from stiction until e ≈ 4,600-5,000 counts ⇒ **the shipping firmware
  has a ~0.5° dead band — wider than the 0.36° (0.1 %-rev) spec it is meant to hold.**
- Incremental stiffness at e = 3,277 is 4.2 N·m/rad; the goal needs ~64. Factor 15.

**But — and this is why the obvious fix fails — the quadratic map is also an accidental
quantisation squelch.** The felt torque staircase per microstep is `I·Δsin(θ)`, and because I ∝ |u| ∝ |θ|
that product is *bounded at ~1.1 % of stall* and goes to **zero at the origin**. The map's "defect"
is what hides the 5.625° quantum and the 14 kHz dither. **Anything that wakes the actuator up near
zero error must pay for that**, and on this board there is no finer microstepping to pay with.

### 1.6 The 1-step-per-tick bit-bang is the product's speed limit
`for (i = 0; i < 1; i++)` at `motor_control.c:2816` ⇒ 31,250 steps/s ÷ 3,200 steps/rev =
**586 RPM**. The datasheet says **"Maximum Speed: 560 RPM."** The advertised top speed is an artifact
of the ISR's step generator, not of the motor. (It also means that at speed, rotor tracking consumes
the entire step budget, leaving no rate authority to change the lead angle.)

### 1.7 Every PID study to date ran the motor at half its permitted current
`DEFAULT_MAX_MOTOR_PWM_VOLTAGE = 200`, but `set_max_motor_current()` accepts up to
`MAX_PWM_VOLTAGE_AT_ZERO_VELOCITY = 390`. `pid_algorithm_sim.c:60` hardcodes `MAX_PWM_V 200`, so
**every simulation in this project — and every bench capture — has been measuring a motor running at
51 % of its allowed current.**
(⚠️ 390 counts = 2.508 V VREF, which is 8 mV **over the ATD5833's absolute-maximum 2.5 V**. Use ≤ 388.)

### 1.8 The bench motor is a QC reject, and it has been driving the whole research programme
The simulator's calibrated friction is **0.13 N·m static / 0.022 N·m kinetic = 5.9:1**. A normal
stepper is 1.3-2:1. Sweeping it:

| static friction (N·m) | 0.13 (bench) | 0.08 | 0.05 | 0.035 | 0.025 |
|---|---|---|---|---|---|
| limit-cycle frequency (Hz) | **28** | 22 | 14 | **0** | **0** |
| standing error std (counts) | 4,331 | 2,411 | 1,216 | 574 | **39** |

**The stick-slip limit cycle that forced the 25 % integral clamp vanishes entirely on a normal-friction
motor.** Everything downstream of that clamp — including the entire zero-cross-gate saga — may be an
artifact of one bad motor. **Measure the stiction spread across a few units before tuning anything else.**

---

### 1.9 M17's output stage is the anomaly — M1, M2 and M23 already ship the linear map
This is the finding the whole campaign converges on, and it was found independently by five agents
plus the prior-art sweep. **M1/M2 (`motor_control.c:2587-2593`) and M23 (`:2697-2703`) do:**

```c
if (motor_pwm_voltage >= 0) commutation_position += HALL_TO_POSITION_90_DEGREE_OFFSET;
else { commutation_position -= HALL_TO_POSITION_90_DEGREE_OFFSET; motor_pwm_voltage = -motor_pwm_voltage; }
```
Fixed ±90° electrical lead, sign folded into the current magnitude ⇒ **torque ∝ u. Linear.**
That is the Mechaduino law, the SimpleFOC law, the VESC/ODrive law, the MTPA (max-torque-per-amp)
law — *every* reference implementation. **Only M17 ramps the angle and the current together.**

And there is a good reason M17 is different: **M1/M2/M23 drive the phases directly, so a ±90° sign
flip is instantaneous. On M17 it is 32 STEP pulses = 32 ticks = 1.02 ms.** The proportional-angle
map is (almost certainly) a workaround for the step engine's 1-step/tick slew limit — and the
quadratic torque map is the price that was paid for it, probably without anyone noticing.

Two independent prior-art hits say the M17 law is *nearly* right and is missing one thing:
- **Oriental Motor AlphaStep** (patent US6121744A, claim 1) reduces to
  `theta_excite = theta_rotor + clamp(theta_cmd − theta_rotor, ±90°)` — **which is exactly our angle
  law.** What we are missing is their current floor.
- **Trinamic TMC4361A** closed-loop: same angle law, but the current ramps from a **non-zero
  `CL_IMIN` (recommended ¼·CL_IMAX)** up to CL_IMAX. That floor is precisely what makes the
  small-signal gain non-zero at the origin.

⇒ **A current floor turns our torque map from `I·sin θ ∝ u²` into `I_floor·sin θ ∝ u`. One clamp.**
The hand-feel reviewer rejected it because at `I = 78` one microstep off centre is a 3.8 %-of-stall
torque quantum sitting *at* the setpoint (8× today's worst case). That objection is real but it is
**not decisive**, because a 1.8° hybrid stepper's own **detent torque is typically 5-10 % of holding
torque** — the same size or bigger, and always present. **This one belongs on the bench, not in a
spreadsheet.** Try a small floor first (I ≈ 25-40, quantum 1.2-2 %).

### 1.10 The stick-slip hunt may be an ACTUATOR SLEW-RATE failure, not a gain failure
The architecture panel derived this and nobody else did. A slip event lasts ~0.5 ms. Reversing the
torque requires traversing the lead angle, and **at 1 step/tick a 180° reversal takes 32 ticks =
1.02 ms — twice the slip duration.** The brake arrives *after* the rotor has already crossed the
breakaway threshold on the far side ⇒ it re-slips ⇒ limit cycle at 17-22 Hz. If this is right, **no
control law fixes it and no gain fixes it** — the plan has been tuning around an actuator that cannot
brake in time.

The relevant code is `for (int32_t i = 0; i < 1; i++)` (`motor_control.c:2816`) — **a loop bound.**
The catch: each step burns ~1.2 µs in two `volatile` spin delays, so `i < 4` costs ~8 µs/tick, which
we do not have. **Un-crippling the step rate therefore requires paying for it first** (T1.2), and/or
generating steps in hardware (a timer channel + DMA). It would also raise the product's top speed
above 560 RPM (fact 1.6).

## 2. What to try, in order

### Tier 0 — free, do these first (no control-law change)

**T0.1 — Raise the current limit and re-measure everything.** `set_max_motor_current(300, 300)`.
Sim, shipping PID, 8 seeds, with τ_max scaled conservatively (τ ∝ I^0.85):

| metric | V = 200 (today) | **V = 300** | V = 388 |
|---|---|---|---|
| **0.1 %-rev step, standing error** | **2,128** | **88** | 100 |
| hold error @ 0.30 N·m | 23,444 | 19,162 | 16,186 |
| standstill error std | 33 | 33 | 33 |
| snap overshoot, bare rotor | 2,928 | 5,267 ← the cost | 7,364 |

**The stiffness goal that the gated integral was built for, and rejected for, is met by a settings
command.** The mechanism is a threshold effect: at e = 3,277 the shipping firmware makes 0.124 N·m and
breakaway is 0.13 N·m — it is losing a coin flip by 5 %. More current wins it.
The cost (more ballistic energy → more snap overshoot) is exactly what T1.1 fixes.

**T0.2 — Scope the STEP pin.** Standstill, closed loop, no load: count STEP pulses over 1 s; then nudge
the rotor a few hundred counts and repeat, 8×. Prediction: **10,000-15,600 pulses/s at some park
positions and ~0 at others.** This is the cheapest, highest-information measurement available, and it
tests the leading new hypothesis for the whine (below).

**T0.3 — Measure the stiction spread** over several motors (breakaway torque, and the "jump quantum"
when it releases). Fact 1.8 says most of our tuning pain may be one motor.

**T0.4 — Free hygiene:** set `OC1PE` on TIM1 (fact 1.3); cap the current setting at 388, not 390
(fact 1.7); fix the stale 250 kHz comment in `PWM.h`.

### Tier 1 — cheap, memoryless control changes

**T1.1 — The P-clamp (2 lines, ~6 cycles, and the strongest verified control result in the batch).**

```c
/* config time (recompute_pid_parameters...) */
max_p_error = (max_error * 11) >> 4;          /* beta = 5/16 */
/* ISR */
int32_t p_error = clamp(error, ±max_p_error);
proportional_term = p_error * proportional_constant_pid;   /* I and D keep the un-narrowed error */
```

*Why it works:* with a static setpoint, `error_change = −v_rotor`, so the shipping PID **already is a
velocity loop** — the P term is the velocity command and the D term is the velocity feedback
(`v_cmd = (kP/kD)·e`). Clamping the error into P therefore **caps the implied approach velocity**,
which caps the ballistic stopping distance (overshoot ∝ v_coast²).

Measured (8 seeds): snap overshoot **7.3× / 4.9× / 16× better at 5× / 20× / 50× rotor inertia**, and
**bit-identical** on standstill error, standstill jitter, profiled moves, hold under load, and
forced-rock smoothness. Robust to ¼ and 1/10 friction, 5× hall noise, and −25 % torque.

Honest limits (from the refuter): **it does nothing at the bare rotor** (2,928 → 2,922), the "16×"
figure is a 2-rev snap at 50× J whose baseline coasts at 94 % of the STEP-rate ceiling, and at
realistic inertias the win is **1-5×**. The full PTOS `sqrt` braking curve was tested and **rejected**:
it adds 0-3 % over the clamp, and mis-setting its acceleration knob by 2× in either direction changes
nothing. Keep the clamp; skip the curve.

*Bonus:* β is a natural user-facing "maximum closed-loop return speed" knob — which is what §22's
"return cruise ≈ authority/kD" law was really groping for.

**T1.2 — Recover ISR time to pay for anything else.** The budget is tighter than the plan assumes:
27 µs of 32 is the real deadline (overrun ⇒ back-to-back ISRs and RS485 starvation), so ~3.5 µs is
spendable. Available:
- `__divsi3` in `get_sensor_position`: **2.2-2.5 µs, every tick** (140-160 cycles of libgcc
  shift-subtract). The single biggest recoverable block.
- The two `volatile` spin delays in the step engine: 38 cycles each = **1.19 µs**, on ~45 % of
  standstill ticks (because of the dither) and most moving ticks. ⚠️ Their required length must be
  re-derived from the **ATD5833** datasheet, not the TMC2209 one (an agent's "we waste 17×" claim was
  based on TMC timings and does not apply to an A4988-class part).
- Watch out: on M0+, `x * CONSTANT` compiles to a 5-10-cycle shift-add chain, not a 1-cycle `MULS`.
  Any new gains must be `static int32_t`, not `#define`s.

**T1.3 — A better velocity estimate for the D term (nearly free, and it unlocks damping).**
Our D filter is *algebraically* `e[k] − EMA₃₂(e[k−1])`, i.e. already a 32-tick differentiator with a
noise gain of 1.008·σ — so the old "an FIR would be 20× better" idea was correctly rejected. But the
Gauss-Markov bound for a linear velocity estimator at our lag is **47**, and ours sits at **214 —
4.55× off it.** A cascade of **three 1-pole EMAs (a = 7/8)** hits 47.0 exactly, using shifts only.
⇒ ~4× less D-term noise ⇒ ~4× more usable kD ⇒ substantially more damping, which is the thing that
currently caps overshoot performance (the D-noise ceiling of plan §21).

**T1.4 — Take the derivative of the MEASUREMENT, not the error.** VESC ships `kd_proc` = derivative
of the *measured* angle, and its position-PID defaults are **kI = 0 and kD = 0**; Mechaduino made the
same call. Differentiating the error (as we do) injects a derivative kick on every setpoint change.
This is the standard 2-DOF/I-PD move and it is free.

**T1.5 — Input shaping, at zero firmware cost.** `handle_queued_movements()` integrates a
piecewise-constant acceleration from the queue, and convolving a piecewise-constant signal with a
ZV/ZVD/EI impulse shaper yields a signal that is *still piecewise constant*. So a shaped trapezoid is
**at most 6-9 queue items** (of 32) — the movement queue is already an input shaper's native data
structure, and the shaping can be done entirely **host-side, in Python, with no firmware change at
all.** This is the right answer for a resonant load (Tom's plastic disk).

### Tier 2 — real research, needs the bench first

**T2.1 — Disturbance observer / linear ADRC** (3rd-order ESO on [position, velocity, disturbance],
disturbance fed forward, integrator deleted). In sim it solved goal 4 without any state machine
(0.1 %-rev step → 124 counts vs 2,301; 33× better under a 0.20 N·m load; **2.9-33× smoother than
shipping under a rocking hand**, and *far* smoother than the rejected gate, whose commanded current
had a std of 24.8 vs shipping's 0.9 — the gate's failure is visible in the data as a *wandering*
current). Structurally it cannot have the gate's flaw: it is LTI, with no timers, counters, crossings
or freeze windows.

**But it is not ready**, for two reasons the refuter found:
1. `b0 ∝ τ_max/J` is a compile-time constant with a tolerance of only **[0.25×, 2×]**. A 5-50× inertia
   load puts it far outside, and the DOB then becomes **worse than shipping**. (The bench M17 has a
   metal pulley on it *right now* — it is already in the mismatch regime.) Declared per-load it is
   spectacular: 5× J snap overshoot **49 counts** vs shipping's 34,685.
2. Its observer bandwidth lands **on top of the VREF pole** (1592 Hz). A 3rd-order ESO tuned for an
   instantaneous actuator has no 22° of phase to give away, and it feeds `d` forward at 100 %
   authority — a ringing `d` at ~600 Hz with full current behind it is **a 600 Hz whine the simulator
   structurally cannot see.**

**T2.2 — Cascade + feedforward.** Snap overshoot 65× better at the bare rotor (2,928 → 45) and 7.6× at
20× J with **one** tuning; standing error under load 45× better with a **full-authority** integral and
no hunting. Cost only +0.3-0.6 µs. Risk: standstill torque jitter 3.9× higher and +53 % STEP
transitions — an unresolved audible/tactile risk on a board whose angle quantum cannot be reduced.

---

## 3. Ideas that were tested and REJECTED (do not re-propose)

- **Torque linearisation by reciprocal-sine current compensation** (`I = |T| / sin(θ_actually_applied)`,
  read back from `actual_step_position`). It does work as an actuator fix — delivered torque comes
  within 2-4 % of ideal-linear across a 200× range, vs 28 % of ideal at Tom's spec point today.
  **Rejected as originally formulated, by proof:** with a 1-step (5.625°) angle floor, 1/sin = 10.2
  and the current must swing 2× between *adjacent* steps; step-boundary crossings happen at up to
  15.6 kHz; and the VREF network is a 1.59 kHz single pole that closes only 27 % of a gap per tick.
  **The compensation can never reach the coil.** It also made snap overshoot 3-13 % *worse* —
  deleting the quadratic map deletes the accidental soft landing (fact 1.5).
  *Caveat worth recording:* the architecture panel's version escapes the proof by using a **large lead
  floor (≥22.5°)**, where the adjacent-step current ratio is only 1.23× — which the RC *can* follow.
  So the idea is not dead in principle; it is dead in the form that would have been implemented. It
  would need τ_VREF measured on the bench (M1 below) before anyone spends a week on it.
- **A soft torque deadband** (which the linearisation experiment's own ablation identified as the real
  winner: standstill step dither 14,058 Hz → 0). **Refuted:** it is a *park-position lottery*. It
  silences the dither only when the rotor parks mid-step; when it parks within ~30 counts of a step
  boundary (~6 % of positions — and the sim's default is exactly on one) it makes the dither **worse**
  (14,581 → 15,624/s). Field symptom would be "1 unit in 20 hums, depending on where it stopped".
- **A current floor** (TMC4361A's `CL_IMIN` ≈ ¼·`CL_IMAX`; recommended independently by two agents,
  from control prior-art and from chopper physics). **Refuted on hand-feel:** at I = 78, one microstep
  off centre is a torque quantum of **3.8 % of stall (15 mN·m) sitting AT the setpoint** — 8× today's
  worst case, in the one place today has none — and the mitigation both agents assumed (1/64
  microstepping) does not exist on this board (fact 1.1).
- **Mechaduino's ±90°-sign law** (angle = 90°·sgn(u), current = |u|). Correct for a DAC-driven
  H-bridge; wrong for us: a sign flip is 32 microsteps at 1 step/tick = **1.02 ms** to reverse the lead
  angle.
- **The PTOS sqrt braking curve** (as distinct from the P-clamp). Contributes 0-3 %; its knob is
  irrelevant; the loop never rides the curve, it just rails against the clamp.

---

## 4. The leading new hypothesis for the whine — and why it is not the gate

Tom's decisive diagnostic was "at kI = 0 the whine and notchiness vanish", and the post-mortem
attributed that to the gate's state machine. There is a simpler explanation that also fits, and that
would still be present in the shipping 0.15.3.4:

1. The step engine has **no working hysteresis** (fact 1.4) and dithers its 5.625° quantum at ~14 kHz,
   chasing hall noise, whenever the rotor parks near a step boundary.
2. That dither is nearly silent when the current is small — and with the **quadratic map** (fact 1.5)
   the current at a small error *is* essentially zero (u = 98 → I = 0).
3. **kI is what puts current into that dither.** The integrator rails in 10-37 ms at hand-scale errors,
   raising I from ~0 to 53+, so the *same* 14 kHz angle dither suddenly has real current behind it.
4. ⇒ **kI ≠ 0 + hand holding the shaft off-position = a 14 kHz mechanical/acoustic carrier.** Setting
   kI = 0 removes the current, and the whine with it — exactly the observed diagnostic, with no state
   machine required.

If T0.2 (scope the STEP pin) confirms this, then the whine is a **step-engine** problem, not a
**controller** problem, and the gate was very likely convicted on circumstantial evidence.

Note the trap: the obvious fix (restore the commented-out `break` to give a real ±1-step deadband)
trades the dither for a static 5.625° torque staircase at the setpoint — smooth-but-notchy instead of
quiet-but-buzzy. On a board that cannot microstep finer, **that is the fundamental trade of this
hardware**, and it should be decided with the scope and a hand, not in simulation.

---

## 5. What the simulator cannot see (fix before trusting the next round)

- **No VREF lag.** The current knob is modelled as instantaneous; it is a 1.59 kHz single pole. This is
  what made the linearisation look feasible.
- **No back-EMF / voltage limit.** Adding one shows the 50× J coast self-limits at 12 V (a supported
  rail) — the P-clamp's win drops from 16× to ~5.8× there.
- **`MAX_PWM_V` hardcoded at 200** (line 60) — half the permitted current (fact 1.7).
- **The noise model is wrong in kind.** `get_sensor_position` is a segment-switched ratiometric
  estimator (150 segments/rev) whose denominator collapses near segment boundaries, so the noise is
  **position-dependent, not stationary white** — and across a segment boundary the accumulator's
  telescoping breaks, injecting a **random walk** at 150 places per revolution. Every experiment ran
  white σ = 33 (the brief said 38.5; both understate it).
- **No detent/cogging torque**, no compliance, no backlash.

---

## 5b. Bench measurements, in priority order (all before any firmware)

| # | measurement | decides |
|---|---|---|
| **M1** | **Scope STEP (PA1) at standstill, closed loop; count pulses/s. Nudge the rotor a few hundred counts, repeat 8×.** | Whether the whine is the step engine dithering at ~14 kHz (§4). Cheapest, highest-information test we have. |
| **M2** | `set_max_motor_current(300,300)`, then re-run the 0.1 %-rev step test | Whether goal 4 is already solved by a settings command (sim says 2,128 → 88 counts) |
| **M3** | Breakaway torque + slip jump quantum, **on 3+ motors** | Whether the 25 % clamp, the limit cycle, and the whole gate saga are artifacts of one 5.9:1 QC-reject bench unit |
| **M4** | Scope VREF while square-waving `CCR1` 40↔160; also sweep `CCR1` = 2…60 and measure coil current | τ_VREF (predicted 100 µs from R22/C1) and whether the chopper is monotone at low current. Decides the current-floor and sine-inversion options. |
| **M5** | τ_max and J directly (lever + scale; torque step + hall log), bare and with the pulley | Every gain in every candidate scales off τ_max/J, and it was *guessed* by every agent |
| **M6** | Top speed: command increasing velocity until it stalls | Confirms (or refutes) the 586 RPM step-engine ceiling |

## 5c. Coverage note (be honest about what this campaign did NOT cover)

The monthly spend limit killed **11 of the 37** literature agents mid-flight, including
`stiction-hunting`, `stiffness-vs-quiet`, `deadband-quantization`, `anticog-ripple`,
`integral-velocity-gate`, `inertia-id`, `notch-resonance`, `academic-stepper`, `m0-fixedpoint`, and
that fleet's synthesis step. 26 topics completed; the experiments, hardware deep-dives, adversarial
critiques and the architecture panel all completed. The gaps most worth revisiting are
**anti-cogging/detent** (directly relevant to the current-floor question of §1.9) and
**stiction/hunting prior art** (directly relevant to §1.8).

## 6. One-paragraph summary

The control law is not where the biggest wins are. **M17's output stage is the anomaly** — it ramps the
lead angle *and* the current with the same number, making torque quadratic in the controller output, so
the motor is nearly dead near zero error (a ~0.5° breakaway dead band). M1, M2 and M23 — and Mechaduino,
SimpleFOC, ODrive, VESC, AlphaStep and Trinamic — all use the linear map instead. M17 is different
because its STEP/DIR engine cannot flip the lead angle quickly (32 ticks), which is also why it hunts:
**a 180° torque reversal takes 1.02 ms and a stiction slip lasts 0.5 ms, so the brake arrives too late.**
That reframes the limit cycle as an actuator slew-rate failure rather than a gain failure — and no gain
was ever going to fix it. Meanwhile the quadratic map's "defect" is what hides a 5.625° angle quantum
and a ~14 kHz step dither, so naively linearising it backfires; the right small fix is probably
AlphaStep's/Trinamic's **current floor**, decided on the bench against the motor's own detent torque.
And the stiffness goal that the rejected gated integral was built to reach appears to be met by
**raising the current limit from 200 to 300** (a settings command — we have been running every test at
51 % of the permitted current), while the high-inertia overshoot goal is largely met by a **two-line
clamp on the error feeding the P term**, which caps the implied approach velocity and is bit-identical
everywhere else. Both are memoryless, so neither can fail the way the gate did. Before any of it: scope
the STEP pin, and measure breakaway torque on more than one motor — because the bench unit is a 5.9:1
QC reject, and on a normal-friction motor the limit cycle that started this whole programme does not
exist at all.
