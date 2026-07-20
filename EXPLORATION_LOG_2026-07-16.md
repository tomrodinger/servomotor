# Motor Exploration Log — 2026-07-16

Open-ended hardware exploration to find undocumented behavior, gotchas, and
bugs, and to improve the documentation. Bench: single M17 (unique ID
99856389A2B46555, alias 'X') on /dev/cu.usbserial-210, firmware 0.15.4.0,
supply ~20 V, free shaft (no load). Firmware upgrade and calibration excluded
per instructions.

Format per experiment: hypothesis → method → outcome → learned → documentation action.

---

## E1 — Stopping physics: how far does the rotor travel for each stop method?

**Hypothesis:** an 'Emergency stop' (MOSFETs off → freewheel) lets the rotor
coast farther than a commanded stop (MOSFETs on, commanded velocity forced to
zero), because the latter brakes magnetically; disabling MOSFETs mid-move
behaves like the emergency stop.

**Method:** run at a steady 5 rot/s (long 'Move with velocity'), then interrupt
with (A) emergency_stop, (B) reset_time (clears the queue and steps commanded
velocity to 0 with MOSFETs still on), (C) disable_mosfets. Measure hall
position from the moment of the command until standstill. Repeated in open
loop and closed loop. Free shaft, current limit 390.

**Outcome (travel after the stop command, from 5 rot/s):**

| Stop method | open loop | closed loop |
| :-- | :-- | :-- |
| A emergency_stop (freewheel) | +0.043 rot | +0.048 rot |
| B reset_time (commanded stop, MOSFETs on) | +0.022 rot | +0.019 rot |
| C disable_mosfets mid-move (freewheel) | +0.050 rot | +0.052 rot |

**Learned:** (1) a commanded stop halts in roughly HALF the distance of a
freewheel stop even with an unloaded rotor; with a real load's inertia the gap
grows — so for the shortest physical stop, command zero velocity (or
reset_time) and keep the MOSFETs on; 'Emergency stop' removes torque, it does
not brake. (2) NEW: the closed-loop mode flag (status bit 2) SURVIVES
disable_mosfets — the controller stays in closed-loop mode with the drivers
off (status read [4, 0] after C in closed loop).

**Documentation action:** add stopping-physics guidance to knowhow ("choosing
how to stop"); document that disable does not exit closed-loop mode.

---

## E2 — Does hard accel/decel disturb the supply rail (sag or regen spike)?

**Hypothesis:** a hard velocity step to 10 rot/s sags the 20 V supply, and the
hard stop pumps energy back (regen) producing a voltage spike.

**Method:** closed loop, current limit 390. Queue an instantaneous
move_with_velocity 0→10 rot/s then 10→0, while polling 'Get supply voltage' as
fast as the bus allows (372 samples in 1.2 s ≈ 3.2 ms/sample).

**Outcome:** idle 20000 mV; minimum during the burst 20000 mV (NO sag at all);
maximum 20400 mV (+0.4 V regen blip during the deceleration). Note the returned
value's default unit is millivolts (first item of the voltage units list).

**Learned:** at this current limit the accel draw is invisible at 3 ms sampling
resolution, but regen on deceleration is real and measurable even with a free
shaft (+2%). With a high-inertia load and a stiff/diode-isolated supply this is
the mechanism that would push the rail toward the 26 V overvoltage fault, and
why the firmware carries overvoltage protection at all.

**Documentation action:** knowhow: note that decelerating a load pumps energy
into the supply rail (measured +0.4 V on a free shaft) — power supplies that
cannot sink current may need a clamp with large decelerating inertia; error 14
(overvoltage) can fire on regen, not just on a wrong supply. [Correction
2026-07-20: originally written as error 41; ERROR_OVERVOLTAGE is code 14.]

---

## E3 — End-position accuracy: open loop vs closed loop, identical trapezoid

**Hypothesis:** for an identical 2-rotation / 1-second trapezoid on a free
shaft, both modes land close to target, but closed loop lands tighter, and
'Get max PID error' quantifies the worst in-flight tracking error.

**Method:** fresh reset each; trapezoid_move(2.0 rot, 1.0 s); wait for queue
empty + 0.4 s settle; compare hall position to target; in closed loop also read
get_max_pid_error over the move window.

**Outcome:** open loop landed −504 counts from target (−0.015% of the 2-rot
move, ≈0.055°); closed loop landed +37 counts (≈0.004°). Max PID error during
the closed-loop move: −0.00043/+0.00153 rotations (≈−14/+50 counts... NOTE:
returned in the selected position unit, rotations here; +0.00153 rot ≈ +5026
counts ≈ 0.55° worst-case lag during accel).

**Learned:** (1) closed loop is ~14× more accurate at the endpoint on a free
shaft (37 vs 504 counts). (2) open-loop error (~500 counts) is the
commutation-step quantization, not sensor noise. (3) get_max_pid_error returns
values in the library's selected POSITION unit — easy to misread as counts.

**Documentation action:** knowhow: state typical free-shaft endpoint accuracy
figures (they answer "how accurate is it?" directly); Get max PID error doc:
make explicit that the library converts the two values into the current
position unit.

---

## E4 — Physical top speed at 20 V vs the firmware velocity ceiling

**Hypothesis:** the firmware ceiling (~34 rot/s from 2040 RPM) is far above
what the motor can physically do at 20 V; commanding an unattainable velocity
in closed loop produces no error if the deviation limit is wide.

**Method:** closed loop, current 390, deviation limit 50 rot. For targets 8,
10, 12, 14 rot/s: move_with_velocity(target, 1.5 s) then 0; read hall position
at t=1.05 s to get achieved average speed; read status after.

**Outcome:** target 8 → achieved ~8.1 rot/s. Targets 10, 12, 14 → hall position
at 1.05 s IDENTICAL (9.04 rot → ~8.6 rot/s) for all three; no error latched.

**Learned:** (1) physical top speed at 20 V / current 390 / free shaft is
~8.6 rot/s (~520 RPM) — the voltage/back-EMF limit, not the firmware limit,
governs; a loaded motor will be slower still. (2) commanding beyond it is NOT
an error: the commanded position races ahead while the motor runs flat-out,
so the deviation grows silently (up to 5.7 rot here) until either the
deviation limit trips or the profile ends — see E5 for what happens next.

**Documentation action:** knowhow: add "attainable speed is supply-voltage
limited (measured ~8.6 rot/s at 20 V free shaft); the firmware max-velocity
ceiling will not protect you from commanding unattainable speeds."

---

## E5 — GOTCHA: queue empty does not mean motion finished (catch-up glide)

**Hypothesis:** (from E4) when a commanded velocity is physically unattainable,
the commanded position races ahead of the rotor; after the profile ends and the
queue is EMPTY, the closed-loop controller keeps the motor moving until it
catches up to the commanded end position.

**Method:** closed loop, wide deviation limit. move_with_velocity(12 rot/s,
1.0 s) + stop; poll queue count and hall position every 0.1 s.

**Outcome:** queue empty at t=1.11 s with hall at 9.71 rot; the motor KEPT
MOVING for another ~0.35 s (9.71 → 10.76 → 11.84 → 12.00) and settled exactly
at the commanded 12.0 rot.

**Learned:** "wait until get_n_queued_items()==0" is NOT a motion-complete
test when the profile outran the motor. The commanded target is faithfully
reached (nothing is lost), but up to seconds later than the profile claims.
A program that queue-polls, then immediately measures, grabs a stale position;
one that disables MOSFETs "after the move" drops the load mid-glide.

**Documentation action:** knowhow: extend the wait-for-completion recipe to
"queue empty AND position settled" (two consecutive equal hall reads, or
commanded-vs-hall within tolerance); cross-reference from move_with_velocity.

---

## E6 — Queue accounting: every trapezoid/go_to_position takes 3 slots, always

**Hypothesis:** (fresh-eyes Q1/Q2/Q16) trapezoid_move uses 3 slots with coast,
2 without, maybe 1 for a zero-displacement dwell; the executing item still
counts in get_n_queued_items.

**Outcome:** measured immediately after queueing from rest: trapezoid 5 rot/3 s
→ 3 slots; trapezoid 0.05 rot/0.2 s (triangular) → 3; trapezoid 0 rot/1 s
(dwell) → 3; go_to_position → 3; three move_with_velocity segments → 3 (1
each). Decrement pattern of trapezoid 2 rot/4 s: count 3 during the accel
phase, 2 during coast (0.12→3.94 s), 1 during decel, 0 at end — the executing
sub-phase IS included in the count.

**Learned:** the docs' "2 slots if there is no coast phase" is WRONG on
0.15.4.0 — a trapezoid/go_to_position always consumes exactly 3 queue items
(degenerate phases still occupy a slot), so the 32-slot queue holds at most 10
such moves; move_with_velocity/move_with_acceleration are exactly 1 each. A
zero-displacement dwell also costs 3, not 1. get_n_queued_items counts the
item being executed.

**Documentation action:** fix Trapezoid move / Go to position descriptions
("adds three items" unconditionally, delete the 2-slot claim); knowhow golden
rule 8: "budget 3 slots per trapezoid → max 10 queued trapezoid moves."

---

## E7 — No status bit indicates ordinary motion

**Hypothesis:** (fresh-eyes Q7) the "busy" status bit is NOT set during normal
queued moves — only during calibration/homing — so get_status cannot detect
motion.

**Method:** poll get_status at 20 Hz during a 2 s trapezoid; OR all flag bits
together; compare with idle.

**Outcome:** union of all status flags during the move = 0b00000110 (MOSFETs +
closed loop) — IDENTICAL to idle after the move. No bit ever indicated motion.

**Learned:** confirmed: motion is invisible in Get status. The only motion
indicators are get_n_queued_items > 0 plus position-settled (E5).

**Documentation action:** Get status doc: note bit 6 (busy) is never set by
ordinary queued moves — do not poll status to detect motion; knowhow
wait-for-completion recipe covers the right way.

---

## E8 — Position-deviation fatal 45 vs instantaneous velocity steps

**Hypothesis:** (fresh-eyes Q5) move_with_velocity's instantaneous velocity
step creates a transient commanded-vs-measured gap; with a tight deviation
limit (0.2 rot) a large-enough step should trip fatal 45 even though the speed
is attainable.

**Method:** closed loop, deviation limit 0.2 rot, free shaft; step 0→V for
0.4 s then stop, for V = 2, 4, 6, 8, 10 rot/s; read status/fatal code.

**Outcome:** V = 2, 4, 6, 8 rot/s: no error — the free-shaft transient lag
stays under 0.2 rot even for an instantaneous 8 rot/s step. V = 10 rot/s
(unattainable at 20 V, E4): fatal 45 as the gap grows without bound. After
fatal 45 the status flags read 0 — MOSFETs off AND closed-loop bit cleared.

**Learned:** on a free shaft the deviation watchdog is hard to trip with
attainable commands — its real function is catching stalls/overload/
unattainable commands. A fatal error clears the closed-loop mode bit (compare
E1: plain disable_mosfets does NOT clear it).

**Documentation action:** knowhow: position the deviation limit as a
stall/overload detector, with the E4/E5 unattainable-speed case as the
free-shaft way to trip it; note fatal errors clear the closed-loop bit.

---

## E9 — Stale-planner advice after Emergency stop is obsolete on 0.15.4.0

**Hypothesis:** (fresh-eyes Q6 contradiction) the command descriptions still
say "send 'System reset' before queueing further moves" after Emergency stop /
mid-motion Reset time, but the BUG-1 fix (0.15.4.0) should make that
unnecessary.

**Method:** on fw 0.15.4.0 (verified via get_firmware_version → 0.15.4.0):
start a 5 rot/s move, interrupt at 1 s with (a) emergency_stop → enable → zero,
(b) reset_time → zero; then queue move_with_acceleration(+20, 0.4 s) and
(−20, 0.4 s) — peak 8 rot/s, well inside the 20 rot/s limit — with NO system
reset. Control: same sequence from a clean state.

**Outcome:** control OK; (a) OK, fatal=0, end position 3.200 rot exactly as
predicted; (b) OK, fatal=0, 3.200 rot. No spurious 16/26/27/28.

**Learned:** the fix holds under a user-realistic sequence; the reset advice in
'Emergency stop'/'Reset time' descriptions is stale for ≥0.15.4.0 (still valid
for older firmware). Also a self-inflicted lesson worth keeping: my first E9
run used ±50 rot/s² × 0.5 s (peak 25 rot/s > 20 limit) and got error 28 —
correct rejection that LOOKS like the old bug. Error 28 on an accel move means
"predicted velocity exceeds the limit"; check the math before blaming state.

**Documentation action:** motor_commands.json: rescope the "System reset
first" sentences in Emergency stop / Reset time to "firmware older than
0.15.4.0"; knowhow error-handling section already version-scopes BUG-1 —
cross-check wording.

---

## E10 — Entering closed loop barely disturbs the shaft

**Hypothesis:** (fresh-eyes Q9) go_to_closed_loop nudges the rotor to align
commutation, shifting position by a measurable amount — which is why examples
zero AFTER entering.

**Method:** from a fresh reset (MOSFETs off), read comprehensive position and
hall; go_to_closed_loop; wait for the mode bit + 0.3 s; read again.

**Outcome:** commanded position unchanged (0.0); hall moved by only +79 counts
(~0.009 degrees). No visible/meaningful shaft motion.

**Learned:** closed-loop entry really is gentle — the "zero after entering"
ritual guards against microscopic shifts, not big snaps. Compare E11.

**Documentation action:** knowhow already says entry "may even give a gentler
engagement" — add the measured figure (~tens of counts) for confidence.

---

## E11 — enable_mosfets (open loop) snap measured at ~4 degrees

**Hypothesis:** (fresh-eyes Q17) the open-loop commutation snap on enable is a
fraction of a commutation step — well under 0.1 rotation but maybe enough to
matter for tight tolerances.

**Method:** from disabled state (rotor wherever the previous experiment left
it), read hall; enable_mosfets; poll hall every 20 ms for 0.5 s; record peak
excursion from the starting point.

**Outcome:** peak excursion 38268 counts = 4.2 degrees (~0.012 rotation).

**Learned:** the enable twitch can exceed 4 degrees — 50x larger than the
closed-loop entry shift (E10, 0.009 deg). For twitch-sensitive mechanisms,
enter closed loop directly instead of enable_mosfets; always zero AFTER
enabling, never before.

**Documentation action:** knowhow golden rule 3: add the measured magnitudes
(up to ~4 deg open-loop enable vs ~0.01 deg closed-loop entry) and the
"prefer go_to_closed_loop for gentle engagement" recommendation with numbers.

---

## E12 — 'Get max PID error' windows: resets on every read, never empty in closed loop

**Hypothesis:** (fresh-eyes Q18) each read returns min/max since the previous
read; a back-to-back second read returns the empty-window sentinels.

**Method:** in closed loop at rest: read twice back-to-back; run a 1-rot
trapezoid; read; read again immediately.

**Outcome:** idle back-to-back reads return small nonzero values (about
-470..+210 counts) — the PID dither of an actively holding servo, NOT
sentinels. After the move: [-1498, +4469] counts; immediate re-read: back to
dither level. Values are returned in the library's selected position unit.

**Learned:** the window resets on every read as documented, but in closed loop
a window is never empty (the PID runs every tick), so the min>max sentinel
case essentially cannot be observed while holding. Idle dither = roughly
±500 counts (±0.05 deg) — a useful baseline figure for "how still is it?".

**Documentation action:** Get max PID error doc: note the sentinel case only
appears when the PID has not run at all in the window (e.g. open loop);
document typical idle dither magnitude so users can set expectations.

---

## E13 — Reversal near the fence trips fatal 27 (not 26) at queue time

**Hypothesis:** (fresh-eyes Q20) chaining a move that ends at nonzero velocity
with a go_to_position back past the start forces an overshoot turn point; with
the fence set just beyond, the planner rejects with error 26
(TURN_POINT_OUT_OF_SAFETY_ZONE).

**Method:** closed loop, limits [-1.0, +1.85]; move_with_velocity(2 rot/s,
0.9 s) (ends at 1.8 rot still moving +2 rot/s); immediately queue
go_to_position(0.5, 1.0). Contrast: same go_to_position from rest.

**Outcome:** the chained reversal raised FatalError 27
(PREDICTED_POSITION_OUT_OF_SAFETY_ZONE), not 26; from rest the same command
ran clean.

**Learned:** the planner's predicted-position check catches the excursion
before the turn-point check does (both exist; which fires depends on
internals). For users the lesson is the same: chaining a reversal near the
fence faults at QUEUE time with 26 OR 27 — treat them as one family.

**Documentation action:** go_to_position/knowhow: mention 26 and 27 together
as "reversal/overshoot near the safety fence" rejections with this concrete
trigger example.

---

## E14 — Safety limits live in the CURRENT frame: 'Zero position' physically moves your fence

**Hypothesis:** safety limits are absolute physical bounds; zeroing the
position re-expresses them.

**Method:** closed loop; limits [-0.5, +3.0]; move to 2.5 rot; zero_position;
command go_to_position(2.0) — physically 4.5 rot, 1.5 rot PAST the original
fence.

**Outcome:** ACCEPTED and executed — the motor drove to physical 4.5 rot
without complaint. The limits are interpreted in whatever frame is current;
zeroing at 2.5 shifted the physical fence by +2.5 rot.

**Learned:** hypothesis WRONG: safety limits do not protect a fixed physical
zone across zero_position calls. Any workflow that zeroes mid-session (e.g.
after homing) MUST re-send Set safety limits afterward. Corollary: set limits
AFTER homing+zeroing, never before. (Credit where due: the 'Zero position'
description already warns "safety limits are not rebased" — the experiment
confirms it physically; 'Set safety limits' and knowhow did not carry the
warning.)

**Documentation action:** add the frame-relativity cross-reference to Set
safety limits and a knowhow rule (set limits after homing/zeroing); Zero
position doc already correct.

---

## E15 — CRITICAL BUG FOUND: fatal error racing a reply can HARD-HANG the motor (BUG-17/18/19)

**Hypothesis:** (follow-up to an anomaly) responses around a fatal-error latch
looked unreliable — one system_reset in fatal state timed out with partial
bytes. Initial hypothesis: the reset reply is truncated by the reboot.

**Method:** repeatedly latch fatal 25 via set_safety_limits(2.0, -2.0)
(inverted bounds — intended as an input-validation probe) and capture raw
TX/RX bytes around the triggering command and the recovery reset.

**Outcome:** three runs, three different behaviors:
1. limits stored, success reply complete, fatal latched silently; NEXT command
   got an error packet (host attributed it to the wrong command).
2. set_safety_limits itself returned a clean error packet (FatalError 25).
3. set_safety_limits reply TRUNCATED mid-packet, then the device went
   PERMANENTLY SILENT — get_status, ping, System reset (alias, broadcast, and
   unique-ID), parser-resync byte floods: nothing answers. Power cycle
   required. The documented "Get status and System reset always answered in
   fatal state" guarantee is broken.

**Learned (mechanism, confirmed in source):** the safety-zone check runs in
the 32 µs motor-control ISR (motor_control.c:2557), so inverted limits fatal
within one tick of the success reply STARTING to transmit. fatal_error() does
__disable_irq() with transmitCount > 0 (truncation — only the bytes already
in the 8-deep UART FIFO drain). The respond-flag set at command start
(main.c:233) is still set, so the fatal loop immediately calls
rs485_transmit_error_packet() -> rs485_transmit() -> `while(transmitCount>0);`
(RS485.c:481) with interrupts off and no manual UART pumping yet = infinite
spin. Which of the three outcomes you get depends on where the reply TX was
when the ISR hit. Filed as BUG-17 (deadlock), BUG-18 (no validation of
inverted limits), BUG-19 (unsolicited error packet desyncs the host), BUG-20
(code-inspection: reset silently dropped in fatal state while TX busy) in
BUGS_FOUND_2026-07-09.md.

**Documentation action:** knowhow: NEVER send inverted safety limits on
<=0.15.4.0 (can hard-hang the device until power cycle); note that a fatal
error arriving mid-reply can truncate that reply (host should treat a timeout
after a state-changing command as "possibly faulted", flush, and probe with
get_status). Product ideas: firmware watchdog + limit validation.

**Session note:** the bench motor is hard-hung in outcome 3 as of this entry;
hardware experiments are paused until it is power cycled.

---

## E16 — BUG-17/18/19/20 fixed in fw 0.15.5.0 and hardware-verified

**Context:** Tom confirmed the red LED was SOLID ON during the hang (matches
the mechanism exactly: the deadlock hits inside the fatal loop's first
half-cycle delay, right after the first red_LED_on()), power cycled the motor
(full recovery), and asked for the fix — with MOSFET disabling kept as the
very first priority in the fatal path.

**Fix (firmware 0.15.5.0):** (1) new bounded `rs485_drain_transmit()` — at
fatal_error() entry, AFTER disable_mosfets(), any in-flight reply is clocked
out by hand so it reaches the host intact and the fatal loop cannot deadlock
on it; (2) the deferred error reply is suppressed whenever a reply was
already issued (respond-flag now cleared at every reply-issuing function in
RS485.c — kills the unsolicited-packet desync); (3) the fatal loop's response
path is guarded against transmit-busy; (4) 'Set safety limits' rejects
lower > upper with fatal 34 deterministically; (5) System reset in the fatal
state always latches reset_requested (fires when TX drains) instead of being
silently droppable.

**Verification (all on the real motor):** 10/10 inverted-limit commands
returned a clean FatalError 34 with readable status (previously: coin-flip
between silent latch / clean error / truncated reply + permanent hang);
15/15 rounds of deviation-fatal-45 firing at random moments during
continuous status polling delivered the error cleanly — zero timeouts, zero
desyncs; motion regression clean (trapezoid landed exactly 2.0000 rot,
post-emergency-stop accel sequence fine). Also compile-checked against the
bootloader target and simulator flags (shared source files).

**Bonus lesson (half-duplex bus physics):** a deliberately "pipelined" test —
get_status + system_reset in ONE host write while in the fatal state —
fails with garbage on the wire. That is not a firmware bug: RS485 here is
half duplex, so the device's status reply collides with the host still
transmitting the reset request. Captured collision bytes confirmed it; the
same two packets 5 ms apart work perfectly. Rule: NEVER transmit a second
command before the previous reply has fully arrived (or timed out).

**Documentation action:** motor_commands.json 'Set safety limits' + error 34
entry + knowhow warnings rescoped to "fixed in 0.15.5.0"; knowhow
communication section gains the never-pipeline rule (half-duplex collision).

---

(Exploration resumed on fw 0.15.5.0 after the full suite passed 63/63.)

## E17 — Re-enabling MOSFETs after a disable resumes closed loop gently

**Hypothesis:** since the closed-loop mode bit survives disable_mosfets (E1),
a later enable_mosfets should resume closed-loop control without another
go_to_closed_loop, but perhaps with an ugly transient.

**Method:** closed loop, move to 1.0 rot, settle; disable_mosfets (status
00000100 — CL bit kept); enable_mosfets; measure hall shift; run another
trapezoid and check landing + max PID error. Also: with a fresh reset (never
in closed loop), read get_max_pid_error twice.

**Outcome:** re-enable restored status 00000110 with a snap of only +542
counts (~0.06 deg — 70x gentler than the 38k-count open-loop enable snap in
E11, because the PID re-engages at the held commanded position). The
follow-up move landed at exactly 2.0000 rot with normal tracking error
(±1600 counts). Open-loop sentinel check: get_max_pid_error returns the
sentinel pair, which the library converts to position units — it reads as
[+655.36, -655.36] rotations (min > max), NOT as obviously-invalid raw ints.

**Learned:** (1) disable→enable is a legitimate "temporary torque release"
recipe in closed loop — no go_to_closed_loop rerun needed. CAVEAT untested
under load: if the shaft sags while disabled, the PID will yank it back to
the still-held commanded position on re-enable (and may trip the deviation
limit); zero or re-command before re-enabling if the shaft may have moved.
(2) the "no data" sentinel from get_max_pid_error looks like ±655.36
shaft_rotations after unit conversion — recognize it by min > max, not by
magnitude.

**Documentation action:** knowhow closed-loop section: add the
disable/enable resume recipe + load caveat; Get max PID error: mention the
±655.36-rotation appearance of the sentinels in default units.

---

## E18 — The trapezoid planner's minimum duration follows an exact formula

**Hypothesis:** (fresh-eyes Q3) the shortest accepted duration for a given
displacement is governed by max velocity and max acceleration together:
d_min = displacement/maxVelocity + maxVelocity/maxAcceleration (time to
cover the area plus one full ramp).

**Method:** maxV=10 rot/s, maxA=100 rot/s^2, displacement 2 rot; 12-step
binary search on duration between accepted and rejected.

**Outcome:** minimum accepted duration = 0.3000 s, matching the formula
(2/10 + 10/100 = 0.300) to four decimal places. Rejection below is fatal
error 15 (ERROR_ACCEL_TOO_HIGH).

**Learned:** the planner sizes the ramps from maxVelocity/maxAcceleration
exactly as 'Set maximum velocity' describes, and the acceptance boundary is
predictable in closed form — programs can pre-validate durations instead of
try/catch. Error 15 (not 16/28) is what fires at this boundary.

**Documentation action:** knowhow motion patterns: add the d_min formula as
the planning rule of thumb; Trapezoid move description: mention the closed-
form minimum duration.

---

## E19 — Validation edge probes (equality, mid-motion zero, idempotence, maxV=0)

**Method/outcomes (fresh state each):**
- Velocity limit equality: move_with_velocity at EXACTLY maxV (5.0) is
  ACCEPTED (limit is inclusive); 0.2% above -> fatal 16 at queue time.
- zero_position mid-motion -> fatal 8 (ERROR_QUEUE_NOT_EMPTY), as documented
  (note it is FATAL and latches, not a polite rejection).
- go_to_closed_loop when already in closed loop: harmless no-op, status
  unchanged (idempotent).
- set_maximum_velocity(0): ACCEPTED silently (firmware stores a 0 limit),
  and then every trapezoid/go_to_position is silently planned as a
  ZERO-MOTION dwell: success response, queue drains over the commanded
  duration, commanded position NEVER advances, hall never moves, no error.
  move_with_velocity commands correctly reject with fatal 16. Filed as
  BUG-21 -- a program that sets maxV=0 by unit-conversion accident (any
  value under half an internal unit rounds to 0!) loses all position moves
  silently.

**Learned:** limits are inclusive; the maxV=0 hole is the velocity twin of
the already-fixed maxA=0 hole (BUG-6) but fails silently instead of
crashing; unit rounding can produce it accidentally.

**Documentation action:** motor_commands.json 'Set maximum velocity': warn
that 0 is accepted and silently zeroes all position-move motion (until
fixed); BUGS file: BUG-21.

---

## E20 — Capture semantics: per-byte timeout, and a fatal mid-capture truncates the stream

**Hypothesis:** (fresh-eyes Q13) a capture longer than the library's 1.2 s
read timeout still completes (the timeout must be per-byte, not total); a
queued move that underruns (fatal 18) during the capture blackout kills the
capture and is only discoverable afterward.

**Method:** capture of 200 points x 16 timesteps x 25 sums = 2.56 s. Run
once at rest; run once with a doomed move queued (move_with_velocity ending
at nonzero velocity 0.5 s in -> fatal 18 mid-capture).

**Outcome:** at rest: completed cleanly in 2.55 s, all 1200 bytes. With the
doomed move: TimeoutError (stream stopped mid-payload when the fatal killed
the sampling engine), and get_status afterwards showed fatal 18. Notably the
device stayed fully responsive after a fatal fired DURING a streaming
transmission — on 0.15.4.0 this was another arming of the BUG-17 deadlock.

**Learned:** the library timeout restarts per received byte, so long
captures are safe; end all motion at rest before capturing (already a doc
rule) because a mid-capture fatal is invisible until the capture dies; a
truncated capture = timeout, then probe Get status.

**Documentation action:** Capture hall sensor data description: add the
truncation-on-fatal note; knowhow reading-state: per-byte timeout fact.

---

## E21 — 'Detect devices' during motion does not disturb queued moves

**Hypothesis:** (fresh-eyes Q14) motion executes autonomously, so the ~1 s
detect deaf-window only endangers hosts that need to STREAM more segments.

**Method:** queue trapezoid_move(10 rot, 4 s); at t=0.5 s send an addressed
'Detect devices'; wait out the deaf window; verify completion.

**Outcome:** detect replied (after 2.1 s — the random reply delay plus
library retry), the move continued unperturbed and landed at exactly
10.000 rot with no error.

**Learned:** as predicted. Also measured: the detect reply can take >1.2 s
end-to-end, so a single TimeoutError on detect is normal — retry.

**Documentation action:** Detect devices/knowhow: note queued motion is
unaffected; only streaming hosts must pause feeding during the window.

---

## E22 — Protocol robustness spot checks

**Method/outcomes:**
- Unknown command ID (200): silently dropped — no response, no error, host
  times out at 1.2 s; device completely unaffected afterward. Timeout is the
  ONLY symptom of an unsupported command (same signature as misaddressing).
- Multimove moveTypes stray bits: setting ALL bits above moveCount
  (0xFFFFFFF8|0b101) executes identically to clean 0b101 (landed 1.500 rot
  both ways) — bits beyond moveCount are ignored, not validated.
- Unique-ID addressing (16-hex-digit string in the M3 constructor):
  enable/zero/move/status all work identically to alias addressing.

**Documentation action:** knowhow addressing: fold in "unsupported command
IDs are silently dropped (timeout), indistinguishable from misaddressing".

---

## E23 — Homing: paced over maxDuration; both busy+homing bits; dwell at zero distance

**Hypothesis:** free-shaft homing (no obstacle) travels maxDistance and ends
clean; maxDistance=0 acts as a timed no-motion dwell in the homing state.

**Method:** closed loop, current 150. homing(2.0 rot, 5.0 s) with status/
queue tracing; then homing(0, 2 s); then homing(2 rot, 0 s).

**Outcome:** (1) homing internally queues a 3-item move and paces it over
the FULL maxDuration — 2 rot took ~5 s, i.e. speed = maxDistance/maxDuration
(~0.4 rot/s). Status during: 0b1010110 — homing (bit 4) AND busy (bit 6) both
set. No obstacle -> ends at cmd=hall=maxDistance (+36 counts), flags clean,
zero+move works immediately. (2) homing(0, 2 s): accepted; homing+busy bits
set for exactly the 2 s with zero motion — a "homing dwell", harmless.
(3) homing(anything, 0 s): fatal 34, consistent with the zero-duration rule.

**Learned:** the homing PRESS SPEED is chosen by the caller via
maxDistance/maxDuration — a "generous" maxDuration is not just a timeout, it
directly slows the approach (gentler collision). This is the missing
intuition in the homing recipe. Homing is when bit 6 (busy) is actually
observable.

**Documentation action:** Homing description + knowhow homing recipe: state
speed = maxDistance/maxDuration explicitly; note busy+homing bits; note the
zero-distance dwell curiosity.

---

## E24 — Round-trip latency: ~3.4 ms median, unaffected by motion

**Method:** 200 get_status round trips at rest, 200 during a 5 rot/s move
(perf_counter, sorted).

**Outcome:** idle: median 3.41 ms, p95 5.55 ms, max 12.5 ms. Moving: median
3.38 ms, p95 4.79 ms, max 15.0 ms. Statistically identical.

**Learned:** the docs' "up to ~100 ms worst case" is far too pessimistic for
a healthy single-device bus: budget ~3.5 ms typical, ~15 ms outlier
(host-USB jitter dominated). ~290 status reads/second are attainable, idle
or moving.

**Documentation action:** knowhow performance envelope: replace the vague
worst-case with measured figures.

---

## E25 — BUG-21 fixed in fw 0.15.6.0 and verified

**Fix:** `set_max_velocity()` rejects 0 with fatal 34, mirroring the existing
zero check in `set_max_acceleration()` (BUG-6 pattern); the command handler
is its only caller, so no internal firmware path can trip it.

**Verification:** flashed 0.15.6.0 to the bench motor; 3/3
set_maximum_velocity(0) attempts returned a clean FatalError 34 with
readable status and normal reset recovery; a tiny nonzero limit (0.001
rot/s) is still accepted; a normal trapezoid landed at exactly 1.0000 rot
with no error. test_set_maximum_velocity.py extended with a zero-rejection
scenario (PASSED standalone, all three scenarios).

**Documentation action (done):** motor_commands.json 'Set maximum velocity'
+ error_codes.json entry 34 + knowhow rescoped to "rejected as of 0.15.6.0";
BUGS file BUG-21 marked fixed; docs regenerated.

---

## E26 — Multipurpose buffer, vibrate, debug values, ping edges, mild thermal

**Method/outcomes (fw 0.15.6.0):**
- Test mode 3 (PID snapshot -> buffer): works in closed loop; the response is
  tag byte 4 + five packed i32 [error, P, I, D, output]. GOTCHA: mode 3 is
  CONTINUOUS — after every read clears the buffer, the still-active mode
  refills it within one control tick, so back-to-back reads return fresh
  snapshots, never the empty tag. In open loop (PID not running) the buffer
  stays empty (single 0x00). Also: sending test_mode(4) while mode 3 is
  active produced a corrupt 1-byte read (tag 4, no payload) — the mode-4
  entry wipes the shared size field while a mode-3 dataset is still tagged.
  One test mode at a time, and read the buffer before switching.
- Test mode 4 (GC6609 register dump): NEVER completes on the M17 bench —
  buffer stays empty after 10 s of polling, with MOSFETs enabled or
  disabled. The docs claim it stores a register dump; on M17/fw 0.15.6.0 it
  does not. Filed as BUG-22 (developer-facing).
- Vibrate: confirmed no-op on M17 (peak hall excursion 98 counts = noise
  floor; success reply anyway), matching the docs.
- Get debug values: 30 fields; per-read-window reset confirmed live (hall
  delta max/min and one profiler field changed/reset between back-to-back
  reads).
- Ping payload edges: exact echo for 10x 0x00, 10x 0xFF, and high-byte
  patterns — no framing sensitivities to payload content.
- Mild thermal: 40 C at idle and unchanged after 90 s of continuous small
  moves at current limit 390; supply steady at 20.2 V. The PCB temperature
  moves on a scale of minutes, not seconds — short bursts are thermally
  invisible.

**Documentation action:** Read multipurpose buffer / Test mode descriptions:
add the continuous-refill nuance, the one-mode-at-a-time rule, and the
mode-4-broken-on-M17 caveat; knowhow reading-state: PID-snapshot recipe
(closed loop only).

---

## E27 — Metrology: repeatability, hysteresis, resolution floor, long-travel closure

**Method/outcomes (closed loop, current 390, free shaft):**
- Repeatability at target 1.0 rot, 6 approaches per direction: spread 73
  counts (from below) / 113 counts (from above); bidirectional hysteresis
  (mean difference) only -40 counts (-0.004 deg) — within the noise floor,
  i.e. no measurable backlash/hysteresis on a free shaft.
- Resolution floor: commanded displacements of 3277 and 328 counts are
  cleanly visible in single hall reads (+3356/+342); at 33 counts and below
  the single-read hall noise (~+/-100 counts) swamps the measurement —
  commanded position moves exactly, but verifying sub-0.0001-rotation moves
  needs averaged reads.
- Long travel: 50 rotations out and back (attainable speed, settled waits)
  closed to +53 counts of zero — no accumulation error over 100 rotations.
- SELF-INFLICTED LESSON, worth documenting loudly: the first attempt
  commanded 50 rot in 4 s = 12.5 rot/s — above the ~8.6 rot/s supply
  ceiling — and my "queue empty + 0.35 s" wait then measured the motor
  +10 ROTATIONS from home, mid catch-up glide (E5's gotcha biting its own
  documenter). Attainability + position-settled waiting are not optional
  for correctness at high commanded speeds.

**Documentation action:** knowhow performance envelope: add the
repeatability/hysteresis/closure figures and the "sub-330-count moves are
real but need averaged reads to verify" note.

---

(An Opus 12-lens idea fleet proposed ~150 experiments; a triage agent
deduplicated them against E1-E27 into a 20-experiment plan in 5 batches.
Batches run safest-first below.)

## E28 — Batch A: at-rest telemetry, wire constants, profiler, timebase

**A1 Hall sensor statistics (cmds 32/33, first exercise):** fresh boot reads
ALL ZERO (gotcha: looks like three dead sensors — it means "statistics never
started", they only accumulate after 'Control hall sensor statistics' 1).
Healthy M17 baseline: at rest means ~[9557, 9992, 4927] ADC units; across
one full slow rotation every channel sweeps peak-to-peak ~8200-8340 with
max ~12400 / min ~4000. Undocumented control values (5) are ACCEPTED
silently (no validation). measurementCount ~31.4k samples/s.

**A2 Wire scaling verified end-to-end:** setting 1.0 rot/s reads back
450,359,963,648 raw = wire(2^20 counts/timestep) << 12 — i.e. internal
velocity storage is counts/timestep x 2^32; acceleration analogously wire
(2^24) << 8. A/B test: a raw-wire 'Move with velocity' [219902326, 15625]
produced exactly the same 1.0000-rot travel as the wrapper's 2 rot/s x0.5 s
call — the documented 2^20 velocity scaling is EXACT.

**A2+ BUG-23 (review-and-decide): velocity/accel ceilings are 2x/4x their
intended values.** Boot default max velocity reads back as 68.0 rot/s (=4080
RPM) though the firmware constant chain derives it from MAX_RPM 2040; boot
max acceleration reads 2000 rot/s^2 though derived from 500 rot/s^2. Root
cause area: motor_control.h computes TIME_STEPS_PER_SECOND = PWM_FREQUENCY
>> 1, which evaluates to half the real 31,250/s tick for this product, so
MAX_VELOCITY inflates 2x and MAX_ACCELERATION (divided by TSPS^2) 4x.
Consequence: the silent clamp and boot defaults permit double/quadruple the
design envelope (physically unreachable at 20 V, but the guard is off).
Docs previously repeated the intended 2040 RPM — corrected to effective
values with the caveat.

**A3 Control-loop CPU + reset-on-read table:** profiler times are in
microseconds (loop period reads 32 = the tick). Motor-control calculations:
17 us open-loop idle, 21 us closed-loop idle, 24-27 us during a move — up
to ~84% of the 32 us budget (max-jitter loop period 36-43 us). The *MaxTime
fields and hall-delta max/min/avg reset per read; the plain *Time fields do
not.

**A4 Timebase:** free-running clock measured +4058 ppm (+0.41%) fast vs the
host over 120 s (previous doc figure "~0.3%" — updated). 'Time sync' at
10 Hz converged a -3.4 ms initial offset to within +/-200 us in ~40 s
(documented ~5000 us bound comfortably met); HSITRIM only moved 62->63.
timeError REPORTS unclamped offsets at least to +/-5 s.

**Documentation action:** hall-statistics baseline + all-zero gotcha into
motor_commands.json/knowhow; ceilings corrected (BUG-23 caveat); CPU-budget
and profiler-reset table into Get debug values; clock figures into knowhow
clock section.

---

## E29 — Batch B: PID dynamics (step captures, P/D sweeps, live PID math, anti-windup)

**B1 Step response via concurrent capture (captureType 2 + queued move):**
0.1 rot / 0.15 s trapezoid, default gains: 0.3% overshoot, settled ~130 ms —
BEFORE the profile even ends. The free-shaft response is profile-dominated:
the loop tracks the commanded trajectory so tightly that classic step-
response tuning metrics barely discriminate gains. Note: captureType-2
position samples are NOT encoder counts (measured ~4.1x counts per unit).

**B2 P sweep (I=5, D=175000), 1-rot/0.6-s test move:** tracking error
improves 11.5k -> 5.0k counts from P=500 to P=4000, then JUMPS to ~50k at
P>=8000 (in-motion oscillation onset between 4000 and 8000); idle dither
stays ~300 counts up to P=32000 and explodes to ~47k at P=64000 (buzzing at
rest). NO fatal error anywhere — instability manifests as buzz/large PID
error, not a fault. Default P=2000 is near-optimal; 4000 slightly tighter.

**B3 D sweep at P=8000 (oscillatory regime):** D = 0/16/31/32/64 all
indistinguishable (~63-66k tracking) — the documented D>>5 quantization is
real AND small D values are ineffective anyway; damping becomes visible at
175000 (46k) and saturates ~700k-1.4M (33k). No upper noise knee up to
1.4M on a free shaft.

**B4 Live PID arithmetic verified via test-mode-3 snapshot during a
sustained lag (P=2000, I=0, D=0, commanded 11 rot/s):** error=102,236
counts, P-term = 204,472,000 = EXACTLY kP x error, output = 99,840 =
EXACTLY (P+I+D) >> 11. The M17 PID output scaling (>>11) and term math are
now hardware-confirmed.

**B4b Anti-windup:** overshoot past the commanded end after a 1.0 s vs
1.5 s unattainable-velocity lag: kI=0 stops short (-4.4k counts residual);
kI=5 lands +0.2k/+1.2k; kI=50 essentially perfect (+15/+9); kI=500
overshoots ~23k counts but IDENTICALLY for both lag durations — the
integral clamp bounds windup independent of duration (the 0.15.3.4
anti-windup works as designed).

**Documentation action:** knowhow closed-loop section: gain-sweep findings
(safe P range, D effectiveness threshold, no-fatal instability signature);
Get max PID error / capture docs already updated; PID-snapshot math note.

---

## E30 — Batch C: current limit as the authority knob; PWM/back-EMF telemetry

**C1 current sweep (top speed / brisk-move tracking / standing offset):**
current 50: 9.78 rot/s top speed, 9.1k tracking, 152-count rest offset;
100: 9.82 / 7.6k / 31; 200: 9.41 / 9.3k / 35; 390: 8.76 / 9.0k / 39.
GOTCHA at current 20: the motor runs (4.9 rot/s) but parks with a PERMANENT
~0.63-ROTATION standing error and NO error flag — too weak to overcome its
own friction, the servo just stalls short silently. Also: top speed is
mildly INVERSE to the current cap (9.8 at 50 vs 8.8 at 390) — the cap does
not throttle free-shaft speed; do not raise current expecting speed.
'Set maximum motor current' with regen exceeding the firmware table range
correctly rejects with fatal 23.

**C2 the current units ARE PWM duty:** open-loop holding, motorPwmVoltage
(Get debug values field 29) reads EXACTLY setting/2 across 50..390. In
closed loop at steady speed on a free shaft the PWM effort is tiny (20-50)
regardless of speed — the controller uses only what the (near-zero) load
demands. motorPwmVoltage is the drive-effort telemetry channel.

**Documentation action:** knowhow current guidance: add the too-low-current
silent-standing-error trap and the "current does not buy speed" note;
Get debug values: motorPwmVoltage meaning.

---

## E31 — Batch D: planner algebra, queue-overflow blast radius, deviation default

- **Superimposition CONFIRMED numerically:** MWV(1 rot/s, 1 s) then
  trapezoid(2 rot, 1 s) then stop landed at exactly 4.000 rot (naive
  expectation 3.0): a trapezoid queued after motion ending at v0 lands at
  v0 x T + d and ends at v0.
- **Queue overflow kills everything:** with the queue genuinely full at 32,
  the 33rd item raises fatal 17 AND aborts the entire in-flight
  choreography (MOSFETs drop, flags clear). Overfilling is not a polite
  rejection. Exactly 10 trapezoids fit (30 slots); the 11th faults.
- **Multimove:** a 5-entry mixed velocity/accel plan executed to exactly
  the hand-integrated 1.500 rot. A zero-duration entry inside a Multimove
  is DROPPED SILENTLY and consumes no slot (n=4 after queuing 5 entries) —
  INCONSISTENT with standalone moves, which reject zero durations with
  fatal 34 since 0.15.4.0. (First-run mask bug also confirmed: an
  acceleration-0 segment MAINTAINS current velocity — a trailing "stop"
  entry must be a VELOCITY-type 0, or the plan ends at nonzero velocity and
  fatals 18.)
- **Deviation default bracketed:** commanded advance with MOSFETs off:
  1.95 rot -> no fault, 2.01 rot -> fatal 45. Default limit = 2.0 rot
  exactly as documented.
- **Error-45 forensics are UNREACHABLE:** the documented crash forensics in
  debugValue1-4 cannot be read because 'Get debug values' is not answered
  in the fatal-error state (error reply instead). Doc contradiction; filed
  as a product-improvement item (expose forensics via the error reply or
  allow this one reader in fatal state).

**Documentation action:** Multimove description (zero-duration drop +
accel-0-maintains-velocity warning); knowhow queue rules (overflow blast
radius); error-45 forensics caveat.

---

## E32 — Batch E: fatal-state table, wire malformations, resync, bootloader window

- **Wire CRC32 is standard CRC-32 (zlib-compatible)** — verified against a
  captured packet. Useful for implementers on other platforms.
- **Definitive fatal-state table:** with fatal 45 latched, of 13 probed
  commands ONLY 'Get status' answered normally; ping, all readers,
  enable/disable MOSFETs, emergency stop, zero, reset time, detect devices
  all receive the error reply; NONE of them un-latches the fault; 'System
  reset' alone recovers.
- **Counter-to-cause mapping:** first byte with LSB=0 -> firstBitErrorCount
  +1 exactly; corrupted CRC -> crc32ErrorCount +1 and the NEXT command
  works immediately (no resync needed); a TRUNCATED frame jams the parser
  until bus silence — measured threshold between 95 and 105 ms: the
  documented 100 ms receiver resync is exact.
- **Bootloader window measured:** probing 150 or 200 ms after 'System
  reset' pinned the device in the bootloader 5/5 times; at 250 ms and
  beyond, 0/5. The app-launch boundary sits between 200 and 250 ms, and
  early probes pin RELIABLY, not occasionally. The 0.5 s golden rule has
  2x margin.

**Documentation action:** knowhow: sharpen golden rule 1 with the measured
window; comm section: counter mapping + CRC standardness + truncation-vs-CRC
recovery difference; error handling: the fatal-state table.

---

## E33 — BUG-23 fixed in fw 0.15.7.0 (Tom's ruling) + BUG-22 root cause confirmed

**Tom's rulings (remote):** the GC6609 chip is no longer used (explains
BUG-22: the scc>=2 M17 build includes AT5833.h, so the mode-4 code is
compiled out — docs updated to "defunct"); and the max velocity default
should be the motor's actual rated maximum, with higher values allowed for
experimentation.

**Fix:** TIME_STEPS_PER_SECOND corrected from PWM_FREQUENCY>>1 to
PWM_FREQUENCY (the ISR runs once per PWM period — the stale halving was
BUG-23's root cause), making the boot defaults the true rated 34 rot/s /
500 rot/s^2; new EXPERIMENTAL_MAX_VELOCITY (2x) and
EXPERIMENTAL_MAX_ACCELERATION (4x = the old effective limits) are used only
in the set-command clamps so experimenters can exceed the rating.

**Verified on hardware (0.15.7.0):** boot readbacks exactly 34.00 rot/s and
500.0 rot/s^2; limit settable to 39 rot/s / 1500 rot/s^2; accel clamps at
2000; a predicted-36-rot/s acceleration move rejects with error 28 at
default and executes after raising the limit to 39; trapezoid regression
exact; test_set_maximum_velocity.py PASSED.

**Bonus wire-format discoveries (now documented):** the velocity LIMIT
field is u32 -> largest settable value ~39.06 rot/s (2343 RPM; the 68
firmware clamp is unreachable over the wire); MOVE commands carry velocity
as SIGNED i32 -> largest commandable velocity ~19.5 rot/s either direction.

---

## E34 — Fleet characterization tool: max velocity and max acceleration per motor

**Task (Tom):** characterize every motor on the bus (1 now, 39-motor rack
coming) for maximum velocity (rot/s) and maximum acceleration (rot/s^2), and
plot histograms.

**Tool:** `python_programs/characterize_motors.py` — discovers all motors
(detect_devices_iteratively, addressed by unique ID so duplicate rack
aliases are harmless), and per motor: closed loop at current 390,
self-calibrates the capture position scale against a known 0.5-rot move,
then 3 trials x both directions of an instantaneous unattainable velocity
step (15 rot/s) captured at 128 us/sample. Max velocity = median of the
speed plateau; max acceleration = 10-90% rise-time acceleration of the
spin-up ramp with sub-sample interpolation. Outputs JSON + histogram PNG.

**Method pitfalls found on the way (all fixed in the tool):**
(1) at 1 ms/sample the u16 capture stream ALIASES above ~3 rot/s (per-sample
delta exceeds the 32768 unwrap limit) — velocities read ~6x low; 128 us
sampling keeps the delta at ~17k at top speed. (2) window-fit slope
estimators are unstable because the unloaded spin-up completes in UNDER
1 MS — integer-sample band-crossing quantizes into discrete clusters;
sub-sample interpolation fixes it. (3) a position-lag cross-check
(plateau-line back-extrapolation) confirmed the sub-ms ramp is real motion,
not a fused-sensor artifact.

**Bench motor result (fw 0.15.7.0, 20.2 V, current 390, free shaft):**
max velocity 8.651 rot/s (519 RPM), trials spread only +/-0.003;
max acceleration ~9,260 rot/s^2, trials 9,049-9,628 (+/-3%). The unloaded
rotor reaches full speed in well under a millisecond — free-shaft
acceleration is torque/inertia-limited and will drop dramatically under
load inertia.

**Documentation note:** the aliasing pitfall and the 128-us recipe belong in
the Capture hall sensor data guidance (added to the capture description
work queue); characterization results JSON kept alongside the tool for the
rack run.

---

## E35 — 39-motor rack characterization: the velocity ceiling is INTRINSIC, not supply-limited

**Run:** all 39 rack motors (/dev/cu.usbserial-110, 24.0-24.5 V supply)
discovered on the first 5-round detection pass and characterized in 9
minutes with zero failures (rack_characterization_results.json/.png).

**Population statistics (current 390):**
- Max velocity: median 8.607 rot/s, range 8.585-8.646, stdev 0.015
  (0.17% — astonishing unit-to-unit consistency).
- Max acceleration: median 11,007 rot/s^2, stdev ~10% excluding outliers.
- Outliers exactly as predicted from the shaft attachments Tom described:
  3ED0F91FAADBD9AF at 5,009 rot/s^2 (~half the median -> the LARGE METAL
  PULLEY, added inertia roughly equal to the rotor's own) and
  04245C407D26CCD0 at 8,682 rot/s^2 (-21% -> the plastic disk). Both have
  NORMAL max velocity (8.646/8.640) — as physics predicts, added inertia
  costs acceleration, not top speed.

**DISCOVERY — E4's interpretation falsified:** the rack runs at 24 V, the
bench at 20 V, yet all 40 motors top out at the same ~8.6 rot/s. The
velocity ceiling is NOT supply-voltage/back-EMF limited as E4 assumed — it
is intrinsic to the drive (most plausibly the firmware/driver commutation
step-rate; ~8.6 rot/s = ~900 counts per 32-us tick). Supply voltage buys
ACCELERATION instead: median unloaded spin-up rose from ~9,300 rot/s^2 at
20 V (bench) to ~11,000 at 24 V (rack), roughly proportional to voltage.
Docs corrected (knowhow golden rule 7 + attainable-speed bullet, Move with
velocity description).

**Documentation action (done):** corrected the three "supply-voltage
limited" claims; recorded population spread as the expected unit-to-unit
variation figure.

---

## E36 — fw 0.15.8.0: defaults aligned to the product spec (Tom's ruling)

**Ruling:** default max velocity = the datasheet's Maximum Speed (560 RPM =
9.333 rot/s, slightly above the ~8.6 measured); default max acceleration =
12,000 rot/s^2 (slightly above the ~11,000 measured); both raisable and
lowerable by the user, with errors enforcing whatever is set.

**Implementation:** MAX_RPM 2040 -> 560 (non-M1) with the /60 moved to the
end of the macro chain (560 is not divisible by 60); acceleration chain
rewritten to a direct 12,000 rot/s^2 with overflow-safe ordering (12000 x
MICROSTEPS_PER_ROTATION would overflow 64 bits — divide by the tick rate
first). Experimental clamps remain 2x/4x the defaults: velocity raisable to
18.67 rot/s, acceleration to 48,000.

**Verified on hardware (0.15.8.0, bench motor):** boot readbacks exactly
9.3333 rot/s and 12,000 rot/s^2; move at 9.5 -> fatal 16, at 9.3 ->
accepted; accel move at 12,500 -> fatal 15, at exactly 12,000 -> accepted
and executed (first attempt hit MY known gotcha, not a firmware fault: two
0.5-ms moves streamed one-at-a-time underrun the queue -> error 18; queue
them behind a dwell); raised limits (15 / 20,000) accepted and honored;
lowered limits (3 / 100) enforce with 16/15; clamps confirmed at 18.67 and
48,000. test_set_maximum_velocity.py PASSED.

**Note:** the 39-motor rack still runs its older firmware; flash it with
0.15.8.0 when convenient if the new defaults should apply there too.

---

## E37 — BUG-24 found and verified: INT64_MIN deviation limit = instant fatal 45

**Hypothesis:** (idea-fleet honorable mention) the handler takes
llabs(input); llabs(INT64_MIN) is undefined behavior and stays negative on
ARM, so the stored limit goes negative and the per-tick check
|deviation| > limit fires on every tick.

**Method:** set_max_allowable_position_deviation(-2^63) with
position_unit='encoder_counts' (identity conversion), 3 trials, fresh state
each.

**Outcome:** 3/3 exactly as predicted — the set command returns a CLEAN
success reply, and fatal 45 is latched by the very next status read.
System reset recovers fully. Bonus: this is a brand-new fatal-races-reply
trigger, and the 0.15.5.0 BUG-17 fix held perfectly (no truncation, no
desync, no lock-up — on 0.15.4.0 this would have been deadlock roulette).

**Learned:** every negative value EXCEPT INT64_MIN takes its absolute value
as documented; INT64_MIN alone is a landmine. Filed as BUG-24 (minor,
suggested fix: clamp or reject in the handler).

**Documentation action (done):** INT64_MIN exception added to the 'Set max
allowable position deviation' description.

---

## E38 — Documentation audit pass 3 (Fable finders + Opus verify/apply)

**Run (2026-07-20, Tom's request):** 12 Fable audit lenses over all doc
sources (find phase completed before Tom's token-limit stop; results fully
salvaged from the workflow journal) -> 109 raw findings -> 87 deduped ->
87 Opus adversarial verifiers -> 84 survived (74 CONFIRMED, 3 refuted) ->
5 Opus per-file apply agents -> 61 direct corrections + 3 residuals fixed
by hand + 2 of my queued additions. One finding overridden by owner ruling
(the datasheet's 560 RPM stays, per Tom).

**Highlights of what the audit caught:** leftover splice artifacts from my
own earlier edits ('bug., and' in Set maximum velocity); parameter texts
missed while descriptions were updated (maximumAcceleration still said 0
was accepted; upperLimit still said inverted limits unvalidated); stale
0.15.7.0-era text presented as current (500 rot/s^2 cap 'equals the
default'); DANGEROUS advice in error entry 53 (claimed System reset
recovers LED test modes 10-13 — only a power cycle can); wrong overvoltage
error number (41 -> 14) that I had propagated from my own E2 note; test
mode 4 still presented as functional; 6 example files with pre-0.15.4.0
comments; ten structural knowhow improvements (golden rules now a clean
1-10 with the safety-limit frame rule promoted to rule 10, measurement
parentheticals moved after the rules they evidence); the firmware-upgrade
doc section no longer shows a synthesized placeholder-value stub (now
points at upgrade_firmware.py).

**Validation:** motor_commands.json + error_codes.json parse and (where
required) encode Latin-1; all touched examples compile; host suite 7/7;
regenerated docs grep-verified free of all fixed stale phrases.

---

## E39 — 39-motor rack: fleet upgrade to 0.15.8.0 + multi-motor experiments

**Fleet upgrade (2026-07-20, Tom's request, firmware of my choosing =
0.15.8.0):** rack census found all 39 motors on a uniform DEVELOPMENT build
0.15.3.94 (the other agent's), all sharing alias 88 — per-unique-ID
addressing mandatory. Flashed one at a time with per-motor verification:
39/39 OK in 231 s (~6 s/motor), zero retries. Fleet census after: 39/39 on
0.15.8.0, zero fatals, spec defaults (9.3333 rot/s / 12000 rot/s^2)
confirmed on sampled motors, supply 24.0-24.4 V.

**Deployment gate:** full suite on the bench motor at 0.15.8.0: 61/63, with
2 failures that are the INTENTIONAL spec change biting — both high-speed
tests command >560 RPM (580, and a sweep that died at exactly the 560
boundary — a live confirmation the new default enforces at precisely the
datasheet value). Both updated to raise the limit first (the documented
path) and re-passed standalone. The characterization tool needed the same
one-line change (its 15 rot/s step). MIGRATION LESSON documented in
knowhow: pre-0.15.8.0 programs commanding >560 RPM need one explicit
'Set maximum velocity' call.

**E39a broadcast epoch (fresh-eyes Q19 answered):** one broadcast 'Reset
time' aligned all 39 clocks within a 543 us total spread (median offset
+379 us ~= read latency).

**E39b fleet time sync:** round-robin syncing all 39 as fast as the bus
allows ran at 244 syncs/s bus-wide (~6.3/s per motor); after 60 s the whole
fleet held within +/-248 us of the master (spread 430 us). Fleet-wide
sub-millisecond synchronization is practical over a single 230400-baud bus.

**E39c broadcast simultaneous motion:** broadcast go_to_closed_loop ->
39/39 entered closed loop; broadcast zero + broadcast trapezoid(1 rot, 2 s)
-> all 39 moved simultaneously and landed at median -2 counts, worst +75
counts, zero fatals; broadcast reset -> 39/39 clean baseline.

**E39d fleet re-characterization on 0.15.8.0:** 39/39 (one transient
capture timeout, motor healthy, re-measured clean): velocity median 8.740
rot/s, range 8.702-8.795 (0.25% stdev); accel median 11,323 rot/s^2; same
two attachment outliers (metal pulley 5,330; plastic disk 8,877). Note the
fleet reads ~1.5% faster than on 0.15.3.94 (8.740 vs 8.607) — small but
consistent; plausibly commutation/control differences between the dev build
and mainline. Outputs: rack_characterization_01580_results.json /
_histogram.png.

**Rack left in a clean state: all motors reset, no fatals.**

---

## E40 — Rack session 2: bus scale, detect statistics, fleet health census

- **Fleet telemetry rate:** polling every hall position of all 39 motors
  takes 150 ms per full sweep (259 reads/s) — full-fleet position telemetry
  runs at ~6.7 Hz over one 230400-baud bus; budget accordingly (or use
  broadcast choreography instead of read-modify loops).
- **RTT uniformity:** per-motor median round trip across all 39 spans just
  3.59-3.71 ms — bus position and device count do not measurably affect
  latency; the single-motor ~3.4 ms figure holds at fleet scale.
- **Detect collisions quantified:** 15 single-round 'Detect devices' runs
  against 39 responders found all 39 in 13 rounds; worst round found 35.
  The random-delay collision avoidance works better than the docs imply —
  the 3-round iterative default is comfortably sufficient even at 39
  devices.
- **Fleet health census:** supply across the rack 24.00-24.50 V (0.5 V
  harness drop, healthy); temperatures 38-45 C after the day's work; comm
  error counters ZERO on all 39 motors since boot — despite the detect
  collisions (collisions corrupt the host's reception, not the devices').

---

## E41 — Fleet oscillator census: a 5000-ppm spread justifies Time sync

100-second free-run measurement of all 39 clocks against the host (single
broadcast epoch): median +773 ppm, range -1473 to +3614 ppm, stdev 1313 ppm.
Two arbitrary motors can therefore drift apart by up to ~5 ms PER SECOND
when free-running — the concrete, population-measured reason 'Time sync' (or
generous timing margins) is mandatory for multi-motor coordination, and a
ready-made factory pass band for oscillator health (|ppm| < ~4000).

---

## E42 — Rack session 2: wave choreography, fault isolation, fleet power

**E42a Epoch-scheduled wave (the documented coordination recipe, executed at
n=39):** broadcast 'Reset time' epoch, then per-motor queues of
[dwell-until-scheduled-slot, trapezoid(0.5 rot, 0.5 s)] with dwells computed
against the shared epoch (compensating command-stagger). All 39 motors fired
in order at 150 ms spacing; measured start errors were bounded by the
150 ms/sweep detection granularity of fleet position polling (the underlying
epoch alignment is sub-millisecond per E39a/b). The dwell-as-scheduler
pattern works and is now a proven recipe.

**E42b Fault isolation and re-integration:** latched fatal 45 on one motor
(via the BUG-24 INT64_MIN probe), then ran a broadcast fleet move: all 38
healthy motors executed exactly; the faulted one held position, stayed
readable ([0, 45]); NEIGHBOR comm counters remained zero (a fatal device
does not pollute the shared bus). A solo reset + closed-loop re-entry
re-integrated it into the very next broadcast move. One bad actor is
containable and recoverable without touching the fleet.

**E42c Fleet inrush:** broadcast 'Enable MOSFETs' on all 39 at once dips the
24.5 V rail by 300 mV; enabling one at a time keeps the dip at 100 mV.
Modest either way on this rack; on a marginal supply, stagger enables.

**E42d Synchronized regen and a HOST-side gotcha:** all 39 motors hard-stop
from 5 rot/s simultaneously -> the rail rises only ~300 mV (free rotors
carry little energy) and no motor faults. But in BOTH runs exactly one
host-side read that was in flight during the stop transient came back
corrupted (partial reply -> TimeoutError), while every DEVICE counter stayed
zero: the synchronized electrical transient glitches the host
adapter/receiver, not the motors. Production guidance: wrap telemetry reads
that straddle synchronized fleet transients in retry+flush, and prefer
reading after, not during, a mass stop.

**Rack left clean: final broadcast reset -> 39/39 at [0, 0].**

---

## E43 — Rack session 3: collision forensics, fleet jam, hall screen, CRC hazard

**E43a Shared-alias collision (all 39 answer one alias-88 'Get status'):**
5/5 trials the host received only garbage (TimeoutError or
CommunicationError — never a parseable reply), and for the first time all
day the DEVICES logged noise too (counters 5-10 per sampled motor: each
receiver heard 38 other transmitters fighting). UID-addressed probes worked
immediately afterward. RULE: never use alias-addressed queries on a bus
with duplicate aliases; assign unique aliases or address by unique ID.

**E43b Truncated broadcast jams the whole fleet:** one 4-byte fragment of a
valid broadcast frame made UID probes TIME OUT fleet-wide (3/3 trials);
150 ms of bus silence restored every motor (verified on motors at both ends
of the chain). Exactly one decode-error count per motor per jam. One
glitchy master transmission deafens all 39 receivers for the same ~100 ms
window — size host retries above that.

**E43c Hall-signature fleet screen (production pass band):** one rotation
per motor, statistics from all 117 hall channels: p-p amplitude median
11,797 ADC units (range 8,675-13,581, stdev 1,144); midlines uniform
(8,064-8,275); worst intra-motor channel imbalance 911 (median 359). A
factory screen of "p-p in 8,000-14,000, midline 8,000-8,300, imbalance
< 1,500" would have passed all 39 units; data kept in
scratchpad rack_hall_signatures.json.

**E43d Detect-backoff fairness (30 single rounds):** median round found all
39 (worst 29); the missed motors scatter (max 2 misses for any one unit) —
the backoff delay is genuinely re-randomized per round, no chronic losers,
no seeded-PRNG defect.

**E43e Mixed-CRC hazard — WORSE than expected:** disabling CRC on ONE motor
('CRC32 control' 0) then broadcasting normal CRC'd traffic latched FATAL 51
on that motor instantly: the 4 CRC bytes parse as excess payload on a
no-CRC device, and 'Get go to closed loop' became a wrong-size command. The
device sat fatal-latched (readable as [0, 51] only via no-CRC framing)
while the other 38 followed the choreography — silent fleet divergence plus
a latched fault from a single mode mismatch. RULE: never mix CRC modes on a
shared bus; if a device is switched to no-CRC, every frame it hears
(including broadcasts meant for others) must be no-CRC. Recovery: no-CRC
'System reset' to the odd device (boot restores CRC).

**Rack left clean: 39/39 at [0, 0].**

---

## E44 — BUG-24 fixed in fw 0.15.9.0

INT64_MIN now saturates to INT64_MAX in the 'Set max allowable position
deviation' handler (main.c) before the llabs() call — the documented
absolute-value contract completed rather than a new rejection error, since a
maximal negative plausibly means "maximal limit" and INT64_MAX is exactly
the boot-default watchdog-off value. Verified on the bench (0.15.9.0):
INT64_MIN accepted, no fatal, acts as maximal limit 3/3; -0.5 rot still
takes its absolute value and trips correctly; default 2.0-rot enforcement
regression intact. Docs rescoped. The rack remains on 0.15.8.0 (carries the
one-value landmine until flashed).

---
