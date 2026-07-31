# PID whine campaign, 2026-07-15 → 2026-07-20 — full record

**Status: PAUSED at a dead end (2026-07-20). Nothing from this campaign ships.** The shipping
PID algorithm remains the one released in 0.15.3.4 (commit 858544d), carried unchanged through
the current firmware line (0.15.9.0 as of 2026-07-31 — its later changes are validation/error
handling, not control math). Beware: version numbers 0.15.4.x were REUSED by real releases
unrelated to the rejected gate experiment (see the 2026-07-31 note in PID_IMPROVEMENT_PLAN.md
§23), and this campaign's 0.15.3.88–.94 experiment builds exist ONLY on the branch. All
experiment code is preserved on branch **`pid-current-floor`** (commit 58a4b28) — do not merge
it; it is a lab notebook in code form.

**Read `PID_RESEARCH_2026-07-14.md` and `PID_IMPROVEMENT_PLAN.md` §17–23 first** (the
gated-integral history). This document covers what came after: the "whine" campaign — the
attempt to make the motor hold position with high damping (kD≈230k+) without the audible
440–620 Hz whine that appears under a compliant load.

---

## 1. The problem, in one paragraph

Tom's goals: no overshoot at any load, no jerkiness, no vibration/whine, stiff holding, fast
smooth return after hand release, ISR well under 32 µs. High kD is needed for the first goals;
with a compliant load (a loose 3D-printed PLA disc on the bench motor), kD ≥ ~200k produces an
audible tonal whine at hold, 440–620 Hz, loudest when the disc is held slightly off-center.
The whole campaign is the fight between those two requirements. The bench whine rig: motor
unique ID 04245C407D26CCD0 with the PLA disc, on the 39-motor rack at /dev/cu.usbserial-110.
The disc has always had significant play/backlash on its hub (confirmed by Tom 2026-07-20 —
it was never snug). A second reference motor, metal pulley (snug), UID 3ED0F91FAADBD9AF,
~2.3× rotor inertia, does NOT whine in normal use.

## 2. The measured laws (bench facts — these survived everything and are the campaign's
   most valuable output)

- **L1 (onset law): whine onset at kD × I ≈ 28.6M ± 10%** (kD in firmware units, I in current
  units 0–200 ≈ 0–1.07 A). Measured by automated binary search of kD against the 250–900 Hz
  band-RMS of a decimated hall capture (threshold ≈ 40), across floors 60–150. Examples:
  floor 80 → onset kD ≈ 336k; kD=230k → onset at I ≈ 124.
- **L2 (frequency law): the mode sits ~590–610 Hz at onset (all currents 60–150) and softens
  to 450–490 Hz as the oscillation develops** (amplitude-dependent). It also wanders with park
  position (±20–40 Hz). The resonator = disc+rotor on the motor's MAGNETIC spring (stiffness
  ∝ current) in series with the compliant PLA hub. Pinning the floor at 150 moved the mode to
  550–600 Hz (≈ √(I ratio)).
- **L3 (observer cost law): the alpha–beta velocity observer's output costs whine budget at
  ~1.2× kD per gain unit; ONE extra EMA pole (155 Hz) on its output cuts that cost ~15×**
  (bench-measured: obs 360k + pole keeps onset ≈ 319k vs 63k without the pole at obs 270k).
- **Hall noise: σ ≈ 38.6 counts/tick white (4-scan); 8-scan averaging measured σ = 21–22**
  (better than the predicted √2) — and the whine did not change at all, which together with
  L1/L2 proves **sensor noise is NOT what sets the whine's loudness** (it at most seeds it).
- Deterministic sensor artifact: the hall interpolation has a **±393-count ripple, 75
  cycles/rev** (= one interpolation segment; the exact period variable is the interpolator's
  internal `fraction`, 3×fraction ≈ 2^17) = ±5.6% position-dependent loop-gain modulation.
  Believed responsible for the park-position wander of L2; NOT the sustainer.
- Whine amplitude at the rotor is small (the standstill hunt is ~57 counts p2p at the liked
  config); the audible energy comes from the disc acting as a soundboard.

## 3. What was BUILT and is preserved on branch `pid-current-floor` (58a4b28)

Runtime-switchable knob system on `set_maximum_motor_current`'s 2nd (regen) arg — all gated
by `CURRENT_FLOOR_EXPERIMENT` + M17, all default-off, stock behavior at bits clear:

| Bits | Feature | Bench verdict |
|---|---|---|
| 0–7 | current floor (clamped ≤150 by firmware) | **KEEPER** — linearizes the quadratic torque map, fixes the universal ~0.5° dead band; floor 80 is part of the liked config |
| 0x100 | step deadband (kills standstill STEP dither) | kills dither, does NOT change the whine (bench 07-15); hurts fine settling |
| 0x200 | extra ~320 Hz pole on the damping signal | **WORSE** (07-20): whine unchanged off-center, ADDED a center whine |
| 0x400 | dnorm — damping gain × floor/I(est) | **NULL** (07-20): no audible change |
| 0x800 | observer output pole at 78 Hz (vs 155) | untested on bench |
| 0x1000 | alpha–beta velocity observer (110 Hz, shifts 5/11) | **KEEPER** — the campaign's one real win, see §4 |
| 0x2000 | current-scheduled notch, center from I via LUT (was: active damper — deleted) | untested on bench |
| 0x4000 | ihyg 155 Hz current-command LP (was: current dither — refuted in sim, deleted) | untested on bench |
| 0x8000 | observer output pole 155 Hz | **KEEPER** — part of the liked config (L3) |

test_mode extensions (RAM-only, safe): 77/78/79 = hall-ripple compensation off/on/readback
(table populated only by a fresh cmd-6 calibration; never bench-exercised), 80–88 = HFD
parallel 420 Hz damping branch, gain (v−80)×256 (bench: **NULL**, see §5).

Compile-time features (all reverted in the final build; kill-switch defines exist):
D-slide (D_SLIDE_RDES_LIMIT), smooth kI boost (PID_KI_BOOST_M1), current sigma-delta
(IHYG_NO_SD reverts). See §7 for why they were reverted.

Also on the branch: the ripple-compensation subsystem (settings-flash table + calibration
piggyback + runtime lerp correction of `fraction` — fully implemented, never bench-validated),
and an updated overflow harness mirror. Firmware versions 0.15.3.88–.93 are committed binaries;
**0.15.3.94** (the last-flashed build) = .93 + `DISABLE_8SCAN` guards on the two 8-scan sites
(ADC.h buffer-size conditional and hall_sensor_calculations.c summation conditional — add
`&& !defined(DISABLE_8SCAN)` to both `PRODUCT_NAME_M17 && CURRENT_FLOOR_EXPERIMENT` guards)
+ version bump; its source edits post-date commit 58a4b28 and were lost with the scratchpad,
but the recipe here reproduces it exactly. Build line for all of these:
`make PRODUCT_NAME=M17 EXTRA_DEFINES='CURRENT_FLOOR_EXPERIMENT INTEGRAL_TERM_AUTHORITY_SHIFT=2
OBSERVER_VELOCITY_CONSTANT=360000 OBS_VE4_LIM=4096 OBSERVER_ALPHA_SHIFT=5 OBSERVER_BETA_SHIFT=11
[D_SLIDE_RDES_LIMIT=0 PID_KI_BOOST_M1=0 IHYG_NO_SD DISABLE_8SCAN]'.

**As of 2026-07-20 the whole 39-motor rack runs 0.15.3.94** (flashing is broadcast). The
separate bench motor on /dev/cu.usbserial-210 was not touched by this campaign.

## 4. The one real win: the "liked config"

`regen = 36944` (floor 80 + observer 0x1000 + observer-pole 0x8000), current 200, kP=2000,
kI=25, kD=230000, with the sliding-surface landing law (always on with the observer).
Tom's verdict (fw .73, re-confirmed through .87): *"feels quite nice, no overshoot, return to
center is fast, whine only slightly off-center and not too loud."* Measured (07-17, fw .87):
cruise 4.98 rev/s, overshoot 56 counts (0.006°), settle 1.95 s (integral-rate-limited
last-mile creep), post-settle oscillation p2p ≈ 96 counts. This is the configuration to beat.
The landing law history matters: a graded octave ladder (.78) and an allowance deadzone (.79)
both failed by hand-feel; the lesson — **any deadzone/magnitude-subtraction in the damping
path removes small-signal damping at the origin → overshoot + center whine return.** The
sliding surface (rdes = −(error>>7), vex = ve4−rdes clamped ±2048, slope exactly 1 at the
origin) fixed the perceptible 0.25-rot slowdown without those regressions.

## 5. The complete tried-and-failed record (the expensive knowledge)

Everything below was sim-motivated, and most of it was sim-"validated" before failing on the
bench. Order roughly chronological.

1. **Fixed notch filters** (D-path −60 dB at the measured frequency; output notch; 3
   attempts): DEAD. The mode wanders with current and park position (L2), and a notch's
   phase skirt destabilized the bare shaft. Structural, not tuning: see the frontier proof, §6.
2. **2nd-order D low-pass (D-LP2, 311 Hz)**: made the whine LOUDER (added phase lag).
3. **Step-quantizer sigma-delta** (noise-shaped STEP output): works as designed, zero
   acoustic effect.
4. **Step deadband**: kills the 10k+/s standstill STEP dither entirely, whine unchanged —
   proved the whine is not the step engine.
5. **Zero-cross gated integral** (0.15.4.x, pre-campaign): rejected on hand-feel — see
   PID_IMPROVEMENT_PLAN.md §23 and the preserved record commit.
6. **Active resonance damper** (Chamberlin SVF band-pass 520 Hz on error, added to u):
   bench NULL. Post-mortem (two independent analyses agreed): (a) at bench amplitudes its
   output was below every actuator quantum (current LSB = 256 u-units, microstep = 1024,
   lead clamp ±16384 saturated off-center) — it never physically reached the motor; (b) its
   phase was ~90° off at 450–520 Hz (a weak negative spring, not a damper) because the sim
   it was tuned in effectively contained the coil L/R pole that the driver's current chopper
   bypasses in reality.
7. **8-scan hall averaging** (σ÷√2): σ improvement REAL (38.6→21), whine unchanged → noise
   is not the loudness-setter. **WARNING: the 8-scan buffer change also intermittently
   corrupts the temperature and supply-voltage ADC slots** — spurious fatal error 40
   (overheat) on cold motors and vbus readings of ~13–15 V at a true 24.0 V. Root cause
   never isolated (suspect: DMA/sequencer alignment with the doubled circular buffer).
   This is why `DISABLE_8SCAN` exists and is set in .94. Do not re-enable without fixing it.
8. **Extra ~320 Hz damping-path pole (dpole, 0x200)**: bench WORSE — off-center whine
   unchanged, new center whine appeared. (Sim had predicted −96%.)
9. **HFD** — parallel band-limited 420 Hz damping branch injected into u (the sprint's most
   sophisticated design; sim: whine floors ÷100, onset ×3–4.5, better landing): bench NULL
   at G1024 and G2048, no audible change to the off-center whine, no chatter either.
10. **dnorm (0x400)** — damping gain normalized by operating current: bench NULL.
11. **Current dither** (±32 counts, 244 Hz update — psychoacoustic tone spreading): killed
    in sim review before reaching the bench (−2..−6 dB best case in the recalibrated model,
    sometimes negative); deleted from the firmware.
12. **Current cap 110** (bound kD×I under onset by capping I): switched on 2026-07-20 but
    the session ended before Tom's verdict — **genuinely untested; the highest-prior
    untested lever** since it acts directly on the proven law L1.
13. Never reached the bench at all: obs-pole-78 (0x800), scheduled notch (0x2000), ihyg
    current LP (0x4000), the ripple-compensation table (incl. its calibration), the
    park-position/onset/seed discriminator experiments (X1/X2/X3), and the low-kD
    model-falsification probe.

**The pattern across 6+ nulls**: every intervention injected into the controller OUTPUT path
(u-domain filters, dampers, dnorm scaling, HFD) produced no audible change off-center. The
only things that ever changed the whine on the bench were the REAL loop-gain terms: kD, the
operating current I, and the observer bandwidth/pole (L1–L3). Working hypothesis for the
nulls: off-center the lead angle is saturated at the ±16384 clamp and current moves in 5.36 mA
LSB steps, so small additive u-signals are clipped/quantized away — matching the damper
forensics arithmetic. Any future fix should act on gains/current/observer structure (or the
mechanics), not on u.

## 6. Theory results (paper-valuable, bench-humbled)

- **LTI frontier proof** (sim+analysis): no fixed u-domain compensator can be whine-free
  off-center AND damped at center, because the whine condition scales with operating current
  I, which the integral winds up off-center; the liked config crosses onset at I ≈ 116–124 —
  exactly the measured off-center whine. This proof correctly predicted failures 1, 2, 8.
- **Mechanism theory (confidence was ~0.8, now ~0.4)**: the whine is a self-excited limit
  cycle — the D/observer path's accumulated phase makes its torque anti-damping at the
  current-stiffened mode; onset when kD×I×|H(f_mode)| exceeds the hub's mechanical damping
  (reproduces L1), frequency from f = (1/2π)√((k_hub+β·I)/J) with β ≈ 0.34 (reproduces L2 at
  both measured currents). The theory still explains the laws, but every intervention it
  blessed failed on metal, so treat its quantitative predictions as decoration.
- **Sim credibility: near zero for fix validation.** A recalibrated model passed seven
  bench-truth validation gates and still mis-predicted dpole (−96% predicted, worse in
  reality) and HFD (÷100 predicted, null). Sims remained useful for MECHANISM reasoning and
  for overflow/regression harnesses only. **Rule for the next campaign: no fix gets more
  than one build's effort before a bench A/B; the bench is the only validator.**

## 7. The return-dynamics side quest (also reverted, with an unresolved mystery)

Two sim-validated features for Tom's "return faster + kill the 2 s creep" request were built
into .92 and reverted in .93 after bench testing on 2026-07-20:
- **D-slide** (subtract the landing-surface reference from the kD error-change path before
  its clamp): predicted cruise 4.85→7.7 rev/s; measured cruise UNCHANGED (4.87). With the
  observer disabled entirely (regen 80) the same motor cruised ~8 rev/s, so the D path was
  not the binding governor that day. Note the historical 10 rev/s returns were at current
  390, not 200 — cruise at 200 may simply be torque-limited.
- **Smooth kI boost ×2 near zero error**: implicated in settle failures the same day.

**UNRESOLVED MYSTERY (biggest open item):** on 2026-07-20 the whine-rig motor's release
overshoot inflated monotonically across the day — 56 counts (Friday, fw .87) → ~2,000
(.92/.93 AND cold) → ~15,600 (.94) — across FOUR different firmware builds including
Friday's own .87 binary (which by then only produced spurious fatal-40s from the 8-scan ADC
bug, so it never completed a clean control run). Supply was confirmed 24 V (the 13–15 V
readings were the ADC bug). Disc play is old, per Tom. Candidate explanations, none proven:
(a) some real mechanical/magnetic change on that specific motor (hall magnet slip? stale
calibration?) — a fresh cmd-6 calibration was never tried; (b) the ADC corruption also
degrading hall quality subtly on the 8-scan builds — but .94 (4-scan) was the worst
measurement; (c) a backlash-state lottery with the loose disc making the metric itself
unstable at high slam counts. The metal-pulley motor measured ~5,100 counts overshoot at the
same gains (2.3× inertia — plausibly its normal ballistic behavior, no Friday baseline to
compare). **Before ANY future landing-quality tuning: re-baseline return_metrics on a freshly
calibrated motor and establish run-to-run variance first.**

Also from 2026-07-20: violent repeated slam tests leave motors in a sustained audible hunting
state (the metal-pulley motor "whined" only because a test left it hunting) and can trip the
real overheat protection. Rest the motor and system_reset between slam batches.

## 8. Where everything lives

- **Branch `pid-current-floor` (58a4b28)**: all experiment source, binaries .88–.93, the
  knob system, ripple compensation, HFD, the overflow-harness mirror. The .94 delta is the
  recipe in §3.
- **`PID_RESEARCH_2026-07-14.md`**: the research campaign that reframed the problem
  (quadratic torque map, dead band, actuator slew, VREF RC = 1.6 kHz, driver identification).
- **`PID_IMPROVEMENT_PLAN.md`** §1–23 + `pid_baseline_captures/`: the pre-campaign PID work
  and the gated-integral record.
- The campaign's simulators, playbooks and fleet reports lived in a /private/tmp scratchpad
  and were lost to the temp cleaner (only their conclusions survive, in this file). The
  useful tool that survives by being re-runnable: `return_metrics.py` (reconstructable from
  §5 of the plan-era tooling; quantifies cruise/knee/overshoot/settle from an over-fast
  commanded move) — recreate it in tools/ if needed.
- Agent memory (Claude): `pid-improvement-work.md` in the project memory dir mirrors this
  document with additional operational detail.

## 9. If/when this resumes — the shortest sensible path

1. **Recalibrate the whine-rig motor (cmd 6) and re-baseline** return_metrics + whine at the
   liked config. Do not trust any old absolute numbers until the §7 mystery is resolved.
2. Run the cheap discriminators BEFORE any new fix: (a) the **current-cap 110 verdict** (the
   one proven-lever test that was interrupted); (b) **X1 park-position spectroscopy** at two
   spatial scales (microstep vs interpolation segment) to identify the seed; (c) the
   **low-kD probe** (kD 40–60k at floor 80: the model predicts a never-observed whine band —
   10 minutes to calibrate all model trust).
3. If the cap works: ship it as an "anti-whine mode" knob (bounded holding torque is an
   honest, explainable trade) alongside the liked config as default.
4. The untested-but-built knobs in priority order: ripple table (valuable for accuracy
   regardless of whine; run calibration, A/B with test_mode 77/78), scheduled notch (0x2000,
   needs the X2 frequency-law fit for its LUT), ihyg (0x4000, only if evidence points
   current-borne), obs-pole-78 (0x800).
5. The mechanical lever was never tried and is probably the largest untouched effect: a
   stiffer/damped disc hub (the whine needs the compliant load; the fleet's rigid loads
   never whine). Even just a TPU washer or a snug hub would move the resonance and its
   damping more than any firmware knob tested.
6. Keep the discipline that emerged: one knob at a time; 3–4 park positions per verdict;
   Tom's ear + phone analyzer as the arbiters; sims for mechanism only, never for validation.
