# Bugs found during documentation ground-truth verification (2026-07-09)

## STATUS UPDATE 2026-07-14 — fix session results

Fixed in **firmware 0.15.4.0** (built, flashed to the bench motor, each fix
hardware-verified; full 47-example suite passed afterwards):

| Bug | Fix | Hardware verification |
| :-- | :-- | :-- |
| BUG-1 stale velocity planner | `clear_the_queue_and_stop_no_disable_interrupt()` now zeroes `velocity_after_last_queue_item` | Reproduced both symptoms on 0.15.3.3 (spurious error 28 on a legal move; accepted overspeed + runtime error 16 on an illegal one); same script shows correct behavior on 0.15.4.0 |
| BUG-2 Multimove stack overflow | count validated BEFORE the copy | 33-move packet → clean fatal error 24, device alive |
| BUG-3 Test mode 0 lockup | value 0 clears motor/overtemperature test modes + restores the overvoltage threshold; never enters the LED-test path | test_mode(0) returns success, device responsive, no error |
| BUG-4 ADD_TO_QUEUE_TEST desync | dry-run no longer commits planner state | code-reviewed (no host tooling drives cmd 120) |
| BUG-5 int64 overflow hole | overflow guard before the velocity prediction (both copies) | pathological accel×duration → clean fatal error 28 |
| BUG-6 zero-parameter family | duration 0 on all four move commands and max-acceleration 0 now raise fatal error 34 | all five cases verified |
| BUG-7 frozen status flags | `fatal_error()` refreshes flags to 0 | flags read 0x00 with error code intact in fatal state |
| BUG-11 silent empty buffer | empty-buffer read replies with data_type byte 0 | returns b'\x00' instead of timing out |
| BUG-16 LED comment | RS485.c comment now says green | n/a (comment) |

Fixed in the **Python library / codegen** (host tests + hardware-verified):

| Bug | Fix |
| :-- | :-- |
| BUG-8 counts_per_timestep factor | generator emits 2^20; unit_conversions_M3.json + Arduino files regenerated; 1 rot/s commanded in counts_per_timestep lands at exactly 2.000 rotations in 2 s |
| BUG-9 (TODO #8) time conversion | time tables are rescaled per the parameter's InternalUnit (microsecond-wire commands now convert correctly in every unit; velocity/acceleration tables deliberately untouched) |
| BUG-10 default time unit | units list reordered in the generator — 'seconds' is first, so it is the data-driven default (per Tom: defaults stay first-item-of-list) |
| BUG-12 hex unique-ID strings | M3 constructor parses 4+-char strings as hex unique IDs |
| BUG-13 M3 CRC control | `motor.set_crc32_enabled(False/True)` threads through to execute_command |
| BUG-14 auto-flush | detect_devices_iteratively() flushes the receive buffer before returning |

Consequential updates: tests test_time_sync / test_reset_time / test_get_current_time /
test_read_multipurpose_buffer updated to the corrected specifications; examples and
generated documentation updated with version-scoped notes.

Still open: **BUG-15** (stress-read reliability investigation), the datasheet
`indicators.py` red-LED-traffic staleness (datasheet regen pending), and the
temperature/broadcast-consistency review items.

## Post-fix code review round (2026-07-14/15, 8-angle review of the full diff)

Additional defects found and FIXED (firmware rebuilt as 0.15.4.0, re-flashed,
full battery + 47-example suite re-passed):

- fatal_error()'s new flags refresh would have erased the in-bootloader status
  bit on bootloader fatal errors (error_handling.c is linked into the
  bootloaders) — now preserves bit 0.
- The int64 overflow guard covered the velocity prediction but not the
  quadratic position term (accel × n(n+1)/2) — now one shared helper bounds
  both terms (velocity overflow → error 28, position overflow → error 27).
- The `ran_the_add_to_queue_test` flag permanently suppressed the
  ERROR_POSITION_DISCREPANCY safety check after any cmd-120 use; with the
  dry-run no longer corrupting the planner, the suppression was removed and
  the check runs unconditionally.
- add_to_queue_test read a stale queue slot for its position prediction
  (wrong-variable copy-paste, pre-existing) and silently accepted zero-duration
  moves — both fixed (parity with the real commands). NOTE: the dry-run no
  longer chains multi-segment predictions (its only consumer was the obsolete
  test_safety_limit.py).
- M3 constructor string parsing tightened: exactly 16 hex digits = unique ID
  (rejects IDs that collapse into the one-byte alias range, e.g. "00FF" would
  have become broadcast 255); other strings parse as decimal (restores the old
  behavior for "0100" etc.); reserved aliases 252/253/254 rejected everywhere.
- MicroPython _SimpleEnum now iterates in insertion order, so the
  first-item-is-default rule holds on the ESP32 too.
- New public servomotor.set_serial_port(port) API; all 47 examples use it
  instead of poking the communication.serial_port module global.
- The doc generator's unit-reference sections are now DERIVED from
  unit_conversions_M3.json (they had already drifted: claimed timesteps was the
  default and counts_per_timestep was broken — both stale after this session's
  fixes). The Arduino command methods were regenerated per the documented
  pipeline (their embedded comments carried pre-rewrite descriptions).
- motor_commands.json: the empty-buffer reply is a normal short-format packet
  (the extended-size format applies only to non-empty replies) — description
  corrected. test_time_sync.py verbose-mode NameError fixed.

Compatibility notes for the next library release (intentional breaking changes,
confirmed by Tom's rulings): the default time unit changed from 'timesteps' to
'seconds', and time_unit='timesteps' now converts truly (µs/32) for the clock
commands instead of passing raw microseconds through — scripts using the old
identity-passthrough workaround must switch to 'seconds' or 'microseconds'.

---

While verifying `error_codes.json` and `motor_commands.json` against the firmware
source (every claim traced to code), a number of real bugs surfaced. This file
documents them for fixing; **nothing here has been fixed yet** except where noted.

> **Line-number caveat:** `firmware/Src/main.c` and `firmware/Src/motor_control.c`
> references are against the working tree of 2026-07-09, which carries uncommitted
> PID/anti-windup edits. Line numbers in those two files may shift; function names
> and code excerpts are the stable reference.

---

## Firmware — high priority

### BUG-1: Stale `velocity_after_last_queue_item` after a mid-motion queue clear

**Component:** `firmware/Src/motor_control.c`
**Symptom:** after an 'Emergency stop' or 'Reset time' issued while the planned
end-of-queue velocity was nonzero (e.g. during a `Move with velocity`, or a
multimove whose last segment ends at nonzero velocity), every subsequently queued
acceleration-type move (trapezoid move, go-to-position, move-with-acceleration,
homing's internal trapezoid) is validated from a wrong velocity baseline. The
runtime then executes a *different* profile than was validated: the motor can
physically overspeed by the stale amount and trip fatal error 16
(`ERROR_VEL_TOO_HIGH`) mid-move, or raise spurious errors 26/27/28 at queue time,
and go-to-position moves land at offset positions.

**Root cause:** `clear_the_queue_and_stop_no_disable_interrupt()`
(motor_control.c:421-429) zeroes `current_velocity_i64` and re-syncs
`position_after_last_queue_item`, but never resets `velocity_after_last_queue_item`
(declared at :211). That variable is written only in `add_to_queue` (:1794) and
`add_to_queue_test` (:1889); the reset at :3584 exists only inside
`#ifdef MOTOR_SIMULATION`.

**Reachable via:** `EMERGENCY_STOP_COMMAND` → `emergency_stop()` (:3264-3270),
`RESET_TIME_COMMAND` → `reset_time()` (:3255-3262), homing-collision path (:1415).

**Fix candidate:** add `velocity_after_last_queue_item = 0;` inside
`clear_the_queue_and_stop_no_disable_interrupt()` (mirrors what the simulator init
already does).

**Workaround (documented in error_codes.json entry 16):** send 'System reset'
after an emergency stop during motion before queueing new moves.

### BUG-2: Multimove stack buffer overflow when moveCount > 32

**Component:** `firmware/Src/main.c`, `MULTIMOVE_COMMAND` handler
**Symptom:** a size-consistent Multimove packet declaring more than 32 moves is
`memcpy`'d into a fixed 260-byte stack buffer (sized for 32 moves) **before** the
`n_moves > MAX_MULTIMOVES` check runs — the copy at main.c:673 precedes the check
at main.c:678. The stack is smashed before `fatal_error(ERROR_MULTIMOVE_MORE_THAN_32_MOVES)`
can fire. Behavior after that is undefined (crash, corruption).

**Fix candidate:** validate `n_moves_in_this_command <= MAX_MULTIMOVES` (and that
`payload_size` is consistent with it) *before* the `memcpy`.

### BUG-3: 'Test mode' value 0 hangs the device (documented as "clear all test modes")

**Component:** `firmware/Src/main.c`, `TEST_MODE_COMMAND` handler
**Symptom:** sending 'Test mode' with value 0 routes into `set_led_test_mode(0)`
(main.c:880-883), which disables all interrupts, disables the MOSFETs, and enters
`while(1)` (main.c:178-188) **before the success response is sent**. The device
goes completely dead on the bus; only a power cycle recovers it. The historical
documentation said "0 = clear all test modes", which is exactly the value a
well-meaning user (or AI) would send.

**Related:** LED test modes 10-13 send the success response, then enter the same
interrupts-off `while(1)` forever — also power-cycle-only recovery. If that is
intended for production LED testing it should at least not disable the ability
to exit; consider making them time-limited or resettable.

**Fix candidate:** make value 0 clear `test_mode` (and not enter the LED-test
path); the safe clear today is only 'System reset'. The new motor_commands.json
description warns about this loudly, but the firmware behavior itself remains.

---

## Firmware — medium priority

### BUG-4: `ADD_TO_QUEUE_TEST` (opcode 120) desyncs the motion planner

**Component:** `firmware/Src/motor_control.c` `add_to_queue_test()` (:1802-1897)
**Symptom:** the debug command runs the full validation math and then **commits**
`velocity_after_last_queue_item` and `position_after_last_queue_item`
(:1888-1889) without queueing anything. Every later real move is validated from
this fictitious baseline (same failure class as BUG-1). It also sets
`ran_the_add_to_queue_test`, which suppresses the position-discrepancy check
(:2037-2041) but not the velocity check. Exposed over RS485 to anyone.

**Fix candidate:** compute the predictions into locals and do not commit them to
the planner state (or restore the previous values afterwards).

### BUG-5: int64 overflow hole in queue-time velocity prediction

**Component:** `firmware/Src/motor_control.c` `add_to_queue()` (:1727-1731)
**Symptom:** `predicted_final_velocity = velocity_after_last_queue_item +
acceleration * n_time_steps` can wrap int64 with pathological inputs
(acceleration up to ~2^34.75 internal units × n_time_steps up to 2^32-1 →
~1.2e20 > INT64_MAX). A crafted (accel, n) pair passes the final-value check
while the per-tick runtime accumulation crosses `max_velocity` and trips fatal
error 16 mid-move. Requires an absurd command (hours-long full-acceleration
move), so it is a validation hole rather than a normal-operation hazard.

**Fix candidate:** saturating/checked multiply, or bound `n_time_steps` per item.

### BUG-6: Divide-by-zero family

1. `compute_trapezoid_move()` divides by 0 when `duration == 0`
   (motor_control.c:453-456) — reached from 'Go to position' / 'Trapezoid move'
   with duration 0. On this platform the int64 software division returns 0 and
   the move becomes a silent no-op, but it is relying on undefined behavior.
2. `set_max_acceleration(0)` is accepted without error; any subsequent
   trapezoid-planned command then computes `max_velocity / max_acceleration`
   with a zero divisor (motor_control.c:449).
3. 'Get debug values' computes `averageHallPositionDelta` by dividing by a
   sample count that is zero if no samples accumulated since the last read
   (motor_control.c:1579-1581) — the read also resets the stats (:1585-1588),
   so two back-to-back reads always hit this.

**Fix candidates:** reject duration 0 / acceleration 0 with a parameter error,
or early-out; guard the average division.

### BUG-7: 'Get status' flags are stale in the fatal-error state

**Component:** `common_source_files/error_handling.c` (:90-101)
**Symptom:** in the fatal-error state, Get status is answered by the minimal
handler which returns the *stored* device_status without recomputing flags; the
flags are frozen at whatever the last normal-mode Get status computed (or 0 if
never queried since boot). Example: fatal_error() disables the MOSFETs, but the
"MOSFETs enabled" bit can still read 1. Only `fatalErrorCode` is trustworthy in
that state.

**Fix candidate:** clear (or recompute) the flags field inside `fatal_error()`
when the error code is stored.

---

## Firmware — minor / review-and-decide

- **Broadcast inconsistencies.** Most success-response commands execute on
  broadcast and only suppress the reply. Exceptions found: 'Control hall sensor
  statistics' skips the *action* too (main.c:750-764 — everything inside
  `if(!is_broadcast)`); 'Time sync' parses but never calls `time_sync()` on
  broadcast (main.c:390-400) — an AI told to "sync 10x/second" that broadcasts
  gets silently zero synchronization; 'Get communication statistics' never
  resets counters on broadcast (main.c:1114-1122). Decide which of these are
  intended and align the rest.
- **'Capture hall sensor data' responds even when broadcast** — the handler has
  no `is_broadcast` guard (main.c:301-360), so broadcasting it with multiple
  devices causes bus collisions. It also blocks ALL packet processing until the
  capture completes (main.c:1431-1434), with no abort mechanism.
- **Vibrate (M1 only): 'vibrate 0' faults if the queue is non-empty** — the
  queue check (:1176) runs before the turn-off branch (:1179), so even turning
  vibration *off* can raise fatal ERROR_QUEUE_NOT_EMPTY.
- **Temperature conversion clamps to 0 outside ~33..307 °C**
  (temperature.c:27-46) — a motor at room temperature reads exactly 0 °C, and
  the i16 can never go negative. A range sentinel distinct from 0 °C would be
  less misleading.

---

## Library / tooling bugs

### BUG-8: `counts_per_timestep` velocity conversion factor is wrong (~104.86×)

**Component:** `autogeneration/autogenerate_unit_conversions.py:289` →
`python_programs/servomotor/unit_conversions_M3.json` → Arduino
`AutoGeneratedUnitConversions.h/.cpp`
**Symptom:** the generator assigns `'counts_per_timestep': rps_factor`
(109951162.7776) instead of 2^20 = 1048576. Commanding a velocity in the unit
literally named `counts_per_timestep` sends ~104.86× the intended speed or trips
fatal error 16. Cross-check: `counts_per_second` = 33.554432 × 31250 = 1048576
is correct, and the acceleration variant `counts_per_timestep_squared` =
16777216 = 2^24 is correct — only the velocity one is wrong.
**Fix:** one line in the generator, then regenerate `unit_conversions_M3.json`
and the Arduino unit-conversion files (both Python and Arduino libraries are
affected).

### BUG-9: M3.py wrapper time-unit conversion ~32× off (known, TODO #8)

Verified during this effort: the wire format of 'Get current time' / 'Time sync'
is true microseconds (TIM14 at 1 MHz; no conversion in the handlers), and the
JSON `InternalUnit: microseconds` is correct. The ~32× discrepancy therefore
lives in the Python wrapper's conversion layer (the conversion-factors file uses
the 32 µs timestep as the internal time unit: `timesteps: 1`,
`microseconds: 0.03125`), i.e. the M3 wrapper applies timestep-based factors to
microsecond-based wire values for these commands. Already tracked as TODO #8.

### Minor tooling items

- `python_programs/convert_python_motor_commands_to_json.py` writes the key
  `"data_type_id:"` (trailing colon); if ever re-run it would emit a
  `data_types.json` the loader cannot read. Stale legacy generator.
- `unit_conversions_M3.json`'s header says "modify autogenerate_unit_conversions.py"
  but the copy next to it exists only as `.bak`; the live generator is
  `autogeneration/autogenerate_unit_conversions.py`.
- Generated Arduino file banners cite `generate_command_code_new2.py`, which no
  longer exists (the orchestrator runs `generate_command_code.py`). Cosmetic.
- `Get comprehensive position`'s third output (`externalEncoderPosition`)
  carries a position `UnitConversion`, but it is a raw external quadrature count
  (EXTI counter on PB4/PB5, GPIO_interrupts.c:36-65); applying motor position
  units to it yields meaningless values. Removing the metadata changes the
  generated Python/Arduino APIs — needs a decision. (The prose now explains it.)

---

## Documentation drift (non-firmware, found during the repo survey)

- `servomotor_datasheets/package.txt` still uses stale "M3-60/M3-48/M3-40" model
  names and a height spec inconsistent with `specs.py`.
- `marketting_page/specs.py` rated power (38/32/25 W) disagrees with
  `servomotor_datasheets/specs.py` (26.4/26.4/24 W) — the two spec sources drifted.
- `API_documentation/servomotor_api_documentation.pdf` (Aug 2025) is superseded
  by the M17-split docs and should probably be moved to `.bak`.

---

## Added 2026-07-10 (from Tom's documentation review + bench measurements)

### BUG-10: Library default time unit is 'timesteps'; should be 'seconds'

**Component:** `python_programs/servomotor/M3.py` (defaults come from the first
entry of each units list in `unit_conversions_M3.json`)
**Symptom:** `M3(alias)` without `time_unit=` interprets all durations as 32 µs
timesteps — `trapezoid_move(1.0, 2.0)` means 2 timesteps, not 2 seconds. Tom:
the default should be seconds. (The Arduino library already defaults to SECONDS.)
**Fix candidate:** explicit default `time_unit='seconds'` in the M3 constructor
(safer than reordering the JSON units list, which would change the generated
Arduino enums' first entry / documentation of internal units).

### BUG-11: Reading an empty multipurpose buffer gets no response

**Component:** `firmware/Src/main.c` READ_MULTIPURPOSE_BUFFER_COMMAND handler
(entire transmit path inside `if (data_type != 0)`)
**Symptom:** the host times out (1.2 s) and must treat TimeoutError as "buffer
empty". Tom: this is a bug — the device should respond.
**Fix candidate:** send a response with data_type 0 and no payload when empty.

### BUG-12: M3 constructor does not parse 16-hex-digit unique-ID strings

**Component:** `python_programs/servomotor/M3.py` (~lines 311-329)
**Symptom:** `M3("0123456789ABCDEF")` raises ValueError (or, worse, an all-digit
ID string is silently misparsed as decimal), even though the constructor's own
error message mentions hex strings. `communication.string_to_alias_or_unique_id()`
already implements the correct parsing.
**Fix candidate:** use `string_to_alias_or_unique_id()` inside the constructor.

### BUG-13: M3 wrapper methods have no CRC32 control

**Component:** `python_programs/servomotor/M3.py`
**Symptom:** high-level methods always append a CRC32; after `crc32_control(0)`
the wrapper can no longer talk to the device — users must drop to
`execute_command(..., crc32_enabled=False)`.
**Fix candidate:** a per-object flag (e.g. `motor.set_crc32_enabled(False)`)
threaded through to execute_command.

### BUG-14: Receive-buffer flush after detection should be automatic

**Component:** `python_programs/servomotor/` (detection / command layer)
**Symptom:** users currently must call `servomotor.flush_receive_buffer()` after
device detection or risk stale bytes corrupting the next response parse.
**Fix candidate:** flush inside detect_devices()/detect_devices_iteratively()
after the response-collection loop completes (and possibly before every
send_command when the RX buffer is unexpectedly non-empty).

### BUG-15 (investigated 2026-07-15 — NOT REPRODUCIBLE on current firmware/library/bench)

Evidence from the investigation session (fw 0.15.4.0, current library, single M17
on /dev/cu.usbserial-210 with adapter-motor ground connected):

- Dedicated stress harness: 3 minutes of aggressive random velocity segments
  (up to ±6 rot/s, motor current 390) while hammering get_status / get_position /
  ping with no inter-read sleeps: **49,360 reads, 729 motion segments, ZERO
  failures**, and all six device-side communication counters (CRC32, decode,
  first-bit, framing, overrun, noise) read 0 afterwards.
- `test_random_speed_stress.py` run twice end-to-end: PASSED both times with no
  retry paths firing.

Conclusion: the "this step is unreliable" comments date from older
firmware/library revisions (before the receive-buffer flush and related fixes)
and possibly a different bench configuration (multiple motors, or no ground
between adapter and motors). If read failures ever reappear, the first
diagnostic is 'Get communication statistics' — nonzero framing/noise/CRC
counters point at the electrical path; clean counters with host-side timeouts
point at the firmware response path. The retry loops in the two stress tests
are harmless and were left in place.

### BUG-15 original report: occasional read failures under stress implied by tests

**Component:** unknown (firmware RS485 path, library parsing, or USB adapter)
**Evidence:** `test_random_speed_stress.py` retries get_status in up to 5 passes
with the comment "I found that this step is unreliable", and
`test_get_temperature.py` retries reads 3x. Tom: this should not be true; likely
a bug. Bench validation of the 47 examples saw zero failures under light load.
**Action:** reproduce under high-speed motion + polling, capture the failing
bytes, and determine whether the loss is firmware-side (e.g. TX while busy),
library-side, or adapter-side.

### BUG-16 (docs/comments): LED "communication traffic" indicator is GREEN, not red

**Truth (confirmed in firmware + by Tom):** `common_source_files/RS485.c:454-460`
lights the GREEN LED while a packet is being received. The red LED is the
fatal-error blinker (plus an M1/M2-only overcurrent indicator).
**Stale artifacts to fix:** the code comment at RS485.c:454 still says "control
the red LED"; `servomotor_datasheets/indicators.py` still tells users the red
LED shows communication traffic (the datasheet PDF therefore documents the old
behavior). PROTOCOL_SPEC.md already says green and is correct.

### Bench measurements affecting documented timings (not bugs)

- Post-'System reset' minimum quiet wait: 0.5 s is sufficient (bootloader window
  is at most ~250 ms); the 1.5 s used across the test suite is just margin.
- Post-'Set device alias' wait: device answered in application mode at the new
  alias 0.3 s after the command on the bench; docs now say 0.5-1 s.
- Broadcast 'Time sync' no-op CONFIRMED by experiment (30 broadcast syncs: clock
  rate unchanged within 161 ppm; same 30 addressed: shifted ~6800 ppm).
- Confirmed by Tom as wanted fixes: BUG-6 (accel-0 / duration-0 validation) and
  BUG-7 (status flags frozen in fatal-error state).

---

## Added 2026-07-16 (hands-on exploration session, fw 0.15.4.0 on the bench)

Full experiment narrative in `EXPLORATION_LOG_2026-07-16.md` (E15 covers the
lock-up, E16 the fix verification).

**STATUS UPDATE 2026-07-16 (later): BUG-17/18/19/20 ALL FIXED in firmware
0.15.5.0** (built, flashed to the bench motor after Tom power cycled it, and
hardware-verified):
- BUG-17/19: `fatal_error()` now hand-drains any in-flight reply before the
  fatal loop runs (new bounded `rs485_drain_transmit()` in RS485.c; MOSFETs
  are still disabled FIRST, per Tom's priority) and suppresses the deferred
  error reply when a reply was already issued (flag now cleared at every
  reply-issuing point in RS485.c, closing the desync window); the fatal
  loop's response path is also guarded against transmit-busy.
- BUG-18: `Set safety limits` rejects lower > upper with fatal error 34.
- BUG-20: System reset in the fatal state always sets `reset_requested`
  (executes once TX drains) even when the reply must be skipped.
- Verification on hardware: 10/10 inverted-limit sends -> clean FatalError 34
  with readable status (this exact sequence previously truncated/hung);
  15/15 rounds of fatal-45 firing during continuous get_status traffic ->
  clean error delivery, zero timeouts, zero desyncs; sequential
  status-then-reset in fatal state works. NOTE: a "pipelined" two-packets-in-
  one-write test fails by design — this is a half-duplex bus, so the second
  request collides with the first reply on the wire (captured collision
  garbage confirms); not a firmware defect.
- Also compile-verified against the bootloader target (shared files) and the
  simulator (MOTOR_SIMULATION) flags. The c_test_programs simulator Makefile
  itself is PRE-EXISTINGLY broken (hardcodes -DPRODUCT_NAME_M3, rejected by
  gpio.h) — unrelated to this change, flagged for later.

### BUG-17 (CRITICAL): async fatal error during reply transmission hard-hangs the device — power cycle required

**Component:** `common_source_files/error_handling.c` fatal loop +
`common_source_files/RS485.c` transmit path.
**Symptom:** if a fatal error fires from the 32 µs motor-control ISR while a
command reply is still draining out of the UART, the reply is truncated
mid-packet and the device goes permanently silent: Get status, ping, System
reset (addressed, broadcast, and unique-ID) are all ignored. Only a power
cycle recovers it. This violates the documented guarantee that Get status and
System reset are always answered in the fatal-error state. Reproduced on the
bench: `set_safety_limits(2.0, -2.0)` (inverted bounds, see BUG-18) makes
fatal 25 fire within one 32 µs tick of the success reply starting to
transmit; three runs produced three different outcomes (clean error packet /
truncated reply but functional fatal state / truncated reply + hard hang)
depending on the interleaving.
**Root cause (chain of three):** (1) `fatal_error()` executes `__disable_irq()`
while `transmitCount > 0`, stalling interrupt-driven TX with a partial packet
on the wire (only what already sat in the 8-deep UART FIFO drains — hence the
intermittency). (2) `main.c` sets `if_fatal_error_then_respond_flag` at
command start and clears it only at the end of `process_packet` (main.c:1145),
so the flag is still set. (3) The fatal loop's first action
(`systick_half_cycle_delay_plus_handle_commands`, error_handling.c:123-126) is
`rs485_transmit_error_packet()` with NO transmit-idle guard and BEFORE any
manual `USART1_IRQHandler()` pump — `rs485_transmit()` enters
`while(transmitCount > 0);` (RS485.c:481) which can never decrement with IRQs
off. Infinite spin; RX dead; reset command unreachable.
**Suggested fix:** in `fatal_error()` before the loop (or in the flag branch),
manually pump `USART1_IRQHandler()` until `!rs485_transmit_not_done()` (with a
timeout), or abort TX (`transmitCount = 0` + flush FIFO) before transmitting
the error packet. Defense in depth: enable the independent watchdog (IWDG) so
any future deadlock self-recovers.
**Exposure:** any ISR-raised fatal (25, 41, 45, 16, ...) landing in a ~300-500 µs
reply-TX window. Rare for genuinely asynchronous errors, but BUG-18 makes it
nearly deterministic.

### BUG-18: 'Set safety limits' accepts inverted bounds (lower > upper), guaranteeing fatal 25 one tick later

**Component:** `firmware/Src/main.c` SET_SAFETY_LIMITS handler (no validation),
`firmware/Src/motor_control.c:2557-2560` (per-tick check).
**Symptom:** sending lower=2.0, upper=-2.0 stores the values unvalidated; with
lower > upper EVERY position is outside the zone, so the ISR check
`(pos > upper || pos < lower)` raises fatal 25 within 32 µs — racing the
command's own success reply, i.e. directly arming BUG-17. Observed all three
BUG-17 outcomes from this one trigger, including the hard hang.
**Suggested fix:** validate `lower <= upper` in the handler and reject with an
error response instead of storing; also consider validating that the CURRENT
position is inside the new zone.

### BUG-19: unsolicited error packet after a completed reply desynchronizes the host

**Component:** `firmware/Src/main.c` flag lifetime + fatal loop response.
**Symptom:** when a fatal fires in the microseconds after a command's reply
was fully sent but before `if_fatal_error_then_dont_respond()` (end of
`process_packet`), the fatal loop transmits an EXTRA error packet that the
host never asked for. The host library then reads this stale packet as the
reply to its NEXT command, shifting every subsequent response by one packet
(observed on the bench as a FatalError raised against the wrong command and a
subsequent partial-read timeout).
**Suggested fix:** clear the respond-flag immediately after each handler
queues its reply (not at the end of `process_packet`), or record a
"reply already sent" marker the fatal loop checks. Library side (defensive):
`flush_receive_buffer()` before transmitting each command would self-heal
one-packet desyncs.

### BUG-20 (code inspection, not yet hardware-verified): System reset silently dropped in fatal state while a reply is draining

**Component:** `common_source_files/error_handling.c` `handle_packet()`
SYSTEM_RESET branch.
**Symptom (predicted):** in the fatal-error state, a System reset that arrives
while the previous reply (e.g. a Get status response) is still transmitting is
completely ignored — `reset_requested` is never set because the whole action
sits inside `if ((payload_size == 0) && !rs485_transmit_not_done())`. A host
that pipelines get_status + system_reset back-to-back never gets the reset and
sees a timeout. Same pattern in the GET_STATUS branch (dropped status reads).
**Suggested fix:** set `reset_requested = 1` unconditionally (the reset
already waits for TX-done before executing); only the reply transmission needs
the busy guard. Could not be hardware-verified because BUG-17 hung the bench
motor first.

### BUG-21 (FIXED in 0.15.6.0): 'Set maximum velocity' accepts 0, silently turning every position move into a no-motion dwell

**Fix status (2026-07-16):** fixed in firmware 0.15.6.0 — `set_max_velocity()`
now rejects 0 with fatal 34, mirroring the BUG-6 pattern in
`set_max_acceleration()` (its only caller is the command handler, so no
internal path is affected). Hardware-verified: 3/3 clean FatalError 34 with
readable status and normal recovery; tiny nonzero limits still accepted;
motion regression clean. `test_set_maximum_velocity.py` gained a
zero-rejection scenario (passes). Docs rescoped in motor_commands.json,
error_codes.json (error 34 trigger list), and knowhow.

**Component:** `firmware/Src/motor_control.c` `set_max_velocity()` (:3259, no
validation) + trapezoid planner.
**Symptom:** after `set_maximum_velocity(0)` (accepted with a success reply),
every 'Trapezoid move'/'Go to position' is accepted, occupies the queue for
the commanded duration, returns success — and moves NOTHING: the commanded
position never advances and no error is ever raised (verified on 0.15.5.0,
hall and commanded both frozen). 'Move with velocity' correctly rejects with
fatal 16. The velocity twin of the fixed maxA=0 hole (BUG-6), but it fails
SILENTLY instead of crashing. Reachable by accident: the library rounds any
requested limit below half an internal unit down to 0.
**Suggested fix:** reject 0 in the SET_MAXIMUM_VELOCITY handler with fatal 34
(ERROR_PARAMETER_OUT_OF_RANGE), matching the 0.15.4.0 treatment of
'Set maximum acceleration' 0.

### BUG-22 (root cause confirmed by Tom; docs fixed): 'Test mode' 4 (GC6609 register dump) is defunct on current hardware

**Update 2026-07-16:** Tom confirmed the GC6609 chip is no longer used. The
M17 scc>=2 build includes AT5833.h instead (motor_control.h product/scc
gate), so the `#ifdef GC6609` mode-4 code is compiled out entirely — the
command stores nothing and test_mode stays latched at 4 until reset. Docs
updated to mark mode 4 defunct. Suggested cleanup (not yet done): remove or
scc-gate the mode-4 path and the stale "make sure to uncomment" comments;
consider an AT5833 register-dump equivalent if production still needs one.

**Component:** `firmware/Src/motor_control.c` TIM16 handler GC6609 branch
(:2995-3002) / `GC6608_UART_bit_bang_read_registers`.
**Symptom:** after test_mode(4), 'Read multipurpose buffer' returns the empty
tag forever (polled 10 s, MOSFETs enabled and disabled, fw 0.15.6.0 bench
M17) — the bit-bang register read never reports finished, so
MULTIPURPOSE_DATA_TYPE_GC6609_REGISTERS is never stored and test_mode stays
latched at 4 until reset. Docs claimed the dump works; doc caveat added.
Developer-facing severity (production/debug tool).
**Also observed:** switching to mode 4 while mode 3 is active corrupts the
pending read (mode-4 entry wipes the shared `multipurpose_data_size` while a
mode-3 dataset is still tagged → 1-byte tag-only response). Suggested fix:
investigate the bit-bang read's finished condition on M17 hardware 1.5, and
have set_motor_test_mode clear `multipurpose_data_type` when wiping the size.

### BUG-24 (FIXED in 0.15.9.0): 'Set max allowable position deviation' with INT64_MIN latched fatal 45 on the next tick

**Fix status (2026-07-20):** the handler in `firmware/Src/main.c` now
saturates INT64_MIN to INT64_MAX before the `llabs()` call, completing the
documented absolute-value contract (a maximal negative input now means a
maximal limit, the same value the boot default uses). Hardware-verified on
0.15.9.0: INT64_MIN accepted with no fatal 3/3 and behaves as
watchdog-at-maximum (a 3-rotation MOSFETs-off commanded advance no longer
trips); ordinary negatives keep the abs contract (-0.5 rot limit trips at
1 rot advance); default enforcement regression intact (2.01 rot trips 45).
Docs rescoped in motor_commands.json and error_codes.json.

**Component:** `firmware/Src/main.c:997` — `set_max_allowable_position_deviation(llabs(new_...))`.
**Symptom (hardware-verified 3/3 on 0.15.8.0):** the wire value INT64_MIN is
accepted with a clean success reply, but `llabs(INT64_MIN)` is undefined
behavior and stays negative on ARM, so the per-tick check
`llabs(position_deviation) > max_allowable_position_deviation`
(motor_control.c:3136) is true on every tick and fatal 45 latches
immediately. All other negative values take their absolute value as
documented. Recoverable with System reset; no lock-up (the 0.15.5.0
reply-race fix held under this brand-new trigger — clean success reply,
fatal delivered on the next command). Found 2026-07-20 while the doc-audit
fleet ran.
**Suggested fix:** special-case INT64_MIN (clamp to INT64_MAX) or reject it
with fatal 34 in the handler.

### BUG-23 (FIXED in 0.15.7.0): velocity/acceleration ceilings were 2x/4x their intended design values

**Fix status (2026-07-16, per Tom's ruling "default = actual motor maximum,
configurable higher for experiments"):** `TIME_STEPS_PER_SECOND` corrected to
`PWM_FREQUENCY` (the motor-control ISR runs once per PWM period; the stale
`>> 1` halved it), so MAX_VELOCITY/MAX_ACCELERATION now compute the rated
34 rot/s (2040 RPM) and 500 rot/s^2 and are the boot defaults. New
EXPERIMENTAL_MAX_VELOCITY (2x) / EXPERIMENTAL_MAX_ACCELERATION (4x = the
old effective limits) are used only in the set-command clamps, so
experimenters can still exceed the rating. Hardware-verified on 0.15.7.0:
boot readbacks exactly 34.00 / 500.0; limit settable to 39 rot/s and 1500
rot/s^2, clamped at 2000; predicted-36-rot/s move rejects with 28 at
default and runs after raising the limit; regression + suite velocity test
PASSED. Bonus wire-format facts documented: the u32 limit field caps
settable velocity at ~39.06 rot/s (so the 68 clamp is unreachable over the
wire) and move commands (i32) cap at ~19.5 rot/s.

**Component:** `firmware/Src/motor_control.h` MAX_VELOCITY / MAX_ACCELERATION
derivation chain.
**Symptom (hardware-measured, fw 0.15.6.0 M17):** boot-default max velocity
reads back 68.0 rot/s (4080 RPM) although the chain starts from MAX_RPM 2040;
boot-default max acceleration reads 2000 rot/s^2 although derived from
10000 mm/s^2 / 20 mm-per-rot = 500 rot/s^2. The silent clamp in
set_max_velocity/set_max_acceleration therefore also allows users 2x/4x the
intended envelope.
**Root cause area:** `TIME_STEPS_PER_SECOND` is defined as
`PWM_FREQUENCY >> 1` in the same header; for this product it evaluates to
half the true 31,250 steps/s control rate, inflating MAX_VELOCITY by 2x and
MAX_ACCELERATION (which divides by TSPS^2) by 4x. ONE_REVOLUTION_MICROSTEPS
(3,276,800) matches the encoder counts, ruling out a microstep-unit factor.
**Impact:** low at 20 V (physics caps at ~8.6 rot/s) but the envelope guard
is not protecting the design intent; the runtime fatal-16 check with boot
defaults only fires at 68 rot/s. Needs Tom's intent ruling: fix the macro
(halving the effective ceilings) may CHANGE behavior for users who relied on
the current defaults.

- **Trapezoid slot count:** 'Trapezoid move' description says "2 [items] if
  there is no coast phase" — measured: ALWAYS 3 items, even for triangular and
  zero-displacement (dwell) moves (E6). Also 'Get n queued items' output text
  still claims "Zero-duration moves are dropped silently" — zero-duration is
  now REJECTED with fatal 34 on ≥0.15.4.0.
- **Stale post-stop advice:** 'Emergency stop' / 'Reset time' descriptions
  still say to send 'System reset' before queueing further moves; verified
  unnecessary on ≥0.15.4.0 (E9) — rescope to older firmware.
- **Safety limits follow the zeroed frame:** 'Zero position' shifts the
  physical location of the safety fence (limits are interpreted in the current
  frame, E14a) — must re-send 'Set safety limits' after zeroing if the fence
  protects real hardware. Not currently documented.

---

## Already fixed during this effort (for context)

- `error_codes.json`: was missing `ERROR_STREAMING_OVERFLOW = 54`; all 55 entries
  rewritten from traced `fatal_error()` call sites and adversarially verified
  (original in `error_codes.json.bak`).
- `motor_commands.json`: all 48 command descriptions and 78 parameter texts
  rewritten from byte-verified firmware handlers (original in
  `motor_commands.json.bak`); structure/types/UnitConversion untouched.
- `API_documentation/autogeneration/error_handling.txt`: enriched with the
  verified fatal-error/LED semantics, communication statistics counters, and
  test-mode notes.

---

## STATUS UPDATE 2026-07-27 — Arduino library bugs FIXED + verified

All Arduino-library bugs below are **FIXED** (generator-side changes, then regenerated;
never hand-edited the generated files) and **verified on BOTH hardware targets** (ESP32-S3
bench + 35-motor rack), 63/63 tests, 1043 assertions, 0 failures:

| Bug | Fix |
| :-- | :-- |
| **BUG-25/26/27** (variable-length responses) | Generator now detects `size:null` output types and emits a CALLER-OWNED-BUFFER API: `void <cmd>([inputs,] buf, bufferSize, actualSize*)` — fail-fast guard on null/zero buffer (no bus traffic), strings null-terminated (actualSize = strlen), blobs raw, required size reported on `BUFFER_TOO_SMALL`. `Communication::getResponse` now reports the required size on too-small (it already drains the full packet). The 3 fixed-size response structs were removed. |
| **BUG-28** (32× time) | Generator is now `InternalUnit`-aware: microsecond-wire time params (cmd 9 Get current time output, cmd 10 Time sync input) are scaled by `CONVERSION_FACTOR_MICROSECONDS` at the conversion call site. Timestep-based move durations unchanged. |
| **BUG-29** (new, default units) | Arduino default units are now data-driven = the first unit listed per type in `unit_conversions_M3.json` (matching Python). Current default AMPS→`internal_current_units`; voltage list reordered so both libraries default to `volts` (firmware internal unit is decivolts / volts×10 — confirmed in ADC.c). |

The test suite's 8 formerly-failing "expected-fail" assertions now PASS; those tests'
known-bug annotations were cleared so any regression is flagged as unexpected.

---

## Arduino library — variable-length response bugs (found 2026-07-21)

Found while validating the Arduino library after the doc/codegen work (multi-agent
static audit + regenerate-and-diff + hardware confirmation). **NOT fixed yet — logged
for a later fix session.** All three share ONE root cause and are pre-existing generator
limitations, not regressions.

**Root cause:** `autogeneration/generate_command_code.py` emits, for EVERY command response,
a fixed-size C++ struct plus a `receivedSize == sizeof(struct)` **exact-match** check. That
is wrong for response fields whose data type is **variable-length** (`string_null_term`,
`general_data`, both `"size": null` in `data_types.json`). The firmware returns a payload
whose length is not `sizeof(struct)`, so the equality check always fails, the method sets
`COMMUNICATION_ERROR_DATA_WRONG_SIZE` and returns a zeroed/empty struct. The command can
never deliver its data on Arduino. The **Python reference library handles these correctly**
(`communication.py` ~747-816: null-terminator search for `string_null_term`, actual-length
read for `general_data`).

| Bug | Command | JSON output type | Generated struct | Runtime result on Arduino |
| :-- | :------ | :--------------- | :--------------- | :------------------------ |
| **BUG-25** | cmd 24 Get product description | `string_null_term` | `char[32]` (Servomotor.h:149) | firmware sends 11 B `"Servomotor\0"` → `11 != 32` → always `DATA_WRONG_SIZE`, returns empty. Empirically confirmed: Python returns `[['Servomotor\x00']]`; Arduino `getProductDescription()` never succeeds. |
| **BUG-26** | cmd 7 Capture hall sensor data | `general_data` (stream) | `uint8_t data` (1 B, Servomotor.h:66-69) | multi-point capture is `nPoints*channels*2` bytes → never `== 1` → can never receive the capture. |
| **BUG-27** | cmd 35 Read multipurpose buffer | `general_data` | `uint8_t bufferData` (1 B, Servomotor.h:217-220) | variable buffer contents → exact-1-byte check fails → always empty. |

**Impact:** none are core motion/telemetry, so all runtime motion tests still pass; these are
informational/diagnostic reads that are silently non-functional on Arduino.

**Suggested fix (later):** teach `generate_command_code.py` to detect null-size data types and
emit a variable-length receive path — a large/bounded buffer, accept `receivedSize <= bufsize`,
and (for `string_null_term`) null-terminate — mirroring the Python `communication.py` logic.

**Verification also done 2026-07-21 (no bugs):** regenerate-and-diff of the command code showed
the committed `Servomotor.cpp`/`.h`/`Commands.h` are byte-identical to generator output for the
current JSON across all 48 commands, except `//` description comments + timestamp (0 functional
diffs). The datasheet Arduino example compiles verbatim for ESP32-S3 and runs correctly (moves
exactly 1.0 rotation).

## Arduino library — BUG-28: 32× time error for microsecond-wire commands (found 2026-07-21)

Found by the expanded Arduino test suite (new `test_ard_time.cpp`) and confirmed against the
Python library on the same motor. **NOT fixed yet — logged for later.**

**Symptom:** the Arduino `getCurrentTime()` reads **32× too large** in every time unit (over a real
1 s sleep it reported 32.36 s); `timeSync()` puts a value **32× too small** on the wire. The Python
library reads the same device correctly (seconds≈1.0, µs≈1e6, timesteps≈31250).

**Root cause:** the Arduino code generator is NOT `InternalUnit`-aware. It routes every `Type=="time"`
parameter through one generated `convertTime()` whose table is normalized to **timesteps** (31250 Hz,
`CONVERSION_FACTOR_SECONDS=31250`). Two parameters actually have `InternalUnit=="microseconds"` (1 MHz)
on the wire, so they are off by exactly `1,000,000 / 31,250 = 32×`. The Python library fixes this via
`M3.py` `_rescale_for_internal_unit` (keys factor tables on `Type_InternalUnit`); the Arduino generator
has no equivalent. Root-cause locations: `autogeneration/generate_command_code_module/maps.py`
(`UNIT_CONVERSION_MAP` maps Type→func only, ignores InternalUnit), `generate_command_implementations.py`
(emits `convertTime` keyed on Type only), `generate_unit_conversion_code.py` (single timestep table).

**Scope — exactly two parameters** (all 48 commands enumerated):

| Bug | Cmd | Param | Dir | InternalUnit | Effect |
| :-- | :-- | :---- | :-- | :----------- | :----- |
| **BUG-28a** | 9 Get current time | `currentTime` | Output | microseconds | reads 32× too large |
| **BUG-28b** | 10 Time sync | `masterTime` | Input | microseconds | sent 32× too small (huge timeError) |

All timestep-`InternalUnit` time params (move durations: Trapezoid/Go-to-position/Homing duration,
Move-with-velocity/acceleration timing, Multimove) are CORRECT — verified by passing motion tests.
`updateFrequency` (raw Hz) and capture `timeStepsPerSample` (raw timesteps) carry no UnitConversion and
are unaffected. **Suggested fix:** make the Arduino codegen `InternalUnit`-aware (mirror Python's
`_rescale_for_internal_unit`), then regenerate.

## Arduino test-suite expansion (2026-07-21)

Added 14 new host-emulator `test_ard_*.cpp` files (~250 assertions) mirroring the Python per-command
tests, covering ~all 48 commands. Run against the bench M17 (uid 99856389A2B46555) via an ESP32-S3
USB↔RS485 bridge. Everything passes EXCEPT the four known bugs above (BUG-25 product description,
BUG-26 capture hall data, BUG-27 read multipurpose buffer, BUG-28 time). Motion, settings, safety,
PID, closed-loop, homing, addressing/set-alias, CRC32 control, detect, status, telemetry, diagnostics,
vibrate/identify/test-mode all correct. (One initial homing failure was a test-only unit-clobber bug,
fixed; homing library code is correct.)
