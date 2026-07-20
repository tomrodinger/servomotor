# Product Improvement Ideas

Collected during hands-on exploration (2026-07-16, fw 0.15.4.0). Separate from
bug reports (see BUGS_FOUND_2026-07-09.md): these are usability, capability,
and performance ideas, not defects.

---

## 1. A motion-complete indication (status bit or dedicated query)

Today NO status bit reflects ordinary motion (exploration E7), and
`get_n_queued_items()==0` is not sufficient either: when the commanded profile
outruns the motor, the rotor keeps gliding to the commanded position for up to
seconds after the queue empties (E5). Every user must invent the same two-part
recipe (queue empty + position settled). A "motion in progress / position
settled" status bit — set while queue is non-empty OR |commanded − hall| >
threshold — would make wait-for-completion a one-liner and eliminate a whole
class of "measured too early" bugs.

## 2. A non-fatal "command saturated / lagging" warning flag

Commanding an unattainable velocity (supply-voltage limited, measured
~8.6 rot/s at 20 V free shaft, E4) produces no feedback at all: the deviation
just grows silently until either the fatal deviation limit trips or the motor
quietly arrives late (E5). A sticky, readable-and-clearable warning bit
("deviation exceeded X% of the limit" or "velocity saturated") would let hosts
detect marginal operation before it becomes a fatal error — valuable in
production machines where loads change slowly over time (wear, lubrication).

## 3. Queue slots: degenerate trapezoid phases still cost a slot

A trapezoid/go_to_position always consumes 3 queue items even when coast or
both ramps are degenerate — a zero-displacement dwell costs 3 of 32 slots
(E6). Compacting degenerate phases (dwell = 1 item) would triple the queue
depth for dwell-heavy and short-segment choreography.

## 4. Enable the independent watchdog (IWDG)

Exploration found a firmware deadlock (BUG-17) that leaves the device
permanently silent until power cycle — in a machine this means physically
reaching the motor. The STM32's independent watchdog costs nothing at runtime
and would convert any such hang (present and future) into an automatic reboot
with the standard fatal-error breadcrumbs. Arguably the single highest-value
robustness feature available.

## 5. Validate parameters that can only ever be wrong

'Set safety limits' accepts lower > upper (BUG-18), which no caller can ever
mean, and the result is an instant fatal (or worse). A general firmware pass
rejecting logically-impossible parameter combinations at the command handler
(with an error response) turns latent traps into immediate, debuggable
feedback. Candidates: inverted limits, zero/negative durations (done in
0.15.4.0), current limits of 0, deviation limit of 0.

## 6. An "enable and hold here" re-engagement option

Re-enabling MOSFETs after a disable resumes closed loop at the OLD commanded
position (exploration E17): if the shaft sagged or was moved while unpowered,
the PID yanks it back — potentially violently, or straight into a deviation
fault. A variant of 'Enable MOSFETs' (or a flag) that first rebases the
commanded position to the current hall position would make torque-release/
re-engage cycles safe under load with one command. (Workaround today:
zero_position or a go_to_position to the measured position before enabling —
but both need the queue empty and host round trips.)

## 7. Document/spec a regen budget, or add a brake-energy clamp option

Decelerating even the free rotor pumps +0.4 V into a 20 V bench supply (E2);
with real load inertia and a supply that cannot sink current, regen is the
path to the 26.5 V overvoltage fatal. A datasheet "regen energy per stop"
figure plus (longer term) a firmware-switchable dissipative clamp mode (PWM
the windings as a brake resistor below the overvoltage threshold) would let
integrators size supplies confidently.

## 8. Make the error-45 crash forensics actually readable

The firmware carefully records position-deviation forensics into
debugValue1-4 when fatal 45 trips — but they are unreachable, because 'Get
debug values' is not answered in the fatal-error state (exploration E31).
Either answer that one reader in the fatal state, or append the four values
to the error reply itself. Zero-cost diagnosis for the most common
real-world fault (collision/stall) instead of a lost black box.

## 9. Fleet characterization as a production test step

The characterization tool (characterize_motors.py) measured 39 motors in 9
minutes with 0.17% unit-to-unit velocity spread — tight enough that a
per-unit pass band (e.g. vmax 8.5-8.8 rot/s, amax within 15% of the batch
median at fixed voltage) would catch bearing drag, magnet defects, and
mis-assembled shafts at end-of-line with no extra fixturing beyond the
existing rack. Attachment outliers stood out unmistakably (metal pulley
read half the median acceleration).
