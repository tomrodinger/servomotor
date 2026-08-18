# Test expansion campaign, August 2026

**Goal (Tom, 2026-08-04):** roughly DOUBLE the number of test cases. Think creatively about
what is NOT yet covered. Run everything on real hardware — the 35-motor rack AND the
ESP32-S3. When a firmware bug is found: investigate, establish the root cause, write it up
here, and **do NOT fix it**.

**This file is the durable record.** It survives context clears. Read it first to resume.

## HOW TO RESUME — current state and exact commands (2026-08-18)

**Hardware right now.** Always `ls /dev/cu.*` and probe first; the numbers move.

| what | port | state |
|---|---|---|
| 35-motor rack | `/dev/cu.usbserial-110` | all fw 0.15.12.0, all alias 88 → address by unique ID, left clean |
| bench M17 `99856389A2B46555` | `/dev/cu.usbmodem2101` | fw 0.15.12.0 (upgraded 2026-08-06), reached THROUGH the bridge |

**The ESP32-S3 currently holds the RS485 BRIDGE, not the on-device test suite.** That is why host
tools can reach the bench motor at all. To go back to on-device runs:
`Arduino_library/Servomotor_TestSuite_ESP32S3/flash_suite.sh`.

```bash
# build the host runner (needed after adding or editing any module)
cd Arduino_library/Servomotor_TestSuite_ESP32S3 && ./build_host_suite.sh

# run ONE module against the rack, or against the bench motor via the bridge
./host_suite /dev/cu.usbserial-110  5E2E8161CF1D2624 <module>
./host_suite /dev/cu.usbmodem2101   99856389A2B46555 <module>

# after writing a new tm_*.cpp — never hand-edit test_registry.{h,cpp}
./register_modules.py --apply

# which commands are thinly covered?
./coverage_report.py --gaps 8

# hardware-free suites (fast, run these before committing)
cd ../../python_programs && python3 run_host_tests.py          # 12 tests
python3 test_fleet_consistency.py -p /dev/cu.usbserial-110     # all 35 motors agree
```

**BEFORE trusting any hardware result:** `pgrep -f host_suite`. Two processes on one RS485 bus
corrupt each other silently and the failures look exactly like device faults.

**Upgrading the bench motor** (or any motor behind a microcontroller): flash the bridge, then use
`test_firmware_upgrade.py -p <port> -a X`, which is alias-addressed and ACKs every page.
`upgrade_firmware.py` in its default BROADCAST mode reports success for every page while verifying
nothing.

## Baseline at start (2026-08-04)

| | count |
|---|---|
| Arduino on-device modules (`tm_*.cpp`) | 63 |
| Arduino `TEST_RESULT` assertions (static) | 843 |
| Arduino assertions executed in a full run | 1042 |
| Python hardware tests (`test_*.py`) | 77 |
| Python host-only tests (`test_host_*.py`) | 9 |

**Target:** ~126 Arduino modules / ~1700 static assertions, and a meaningful increase on the
Python side. "Approximately double."

## Hardware

- **Rack:** `/dev/cu.usbserial-110`, 35 motors, ALL alias 88 → address by unique ID.
  Currently fw **0.15.12.0**. Shared resource: leave it reset, clean, alias 88, no fatal errors.
- **ESP32-S3:** `/dev/cu.usbmodem2101`, bench motor `99856389A2B46555` on its RS485 bus,
  currently fw **0.15.9.0**. Cannot be reflashed from the host (no serial path to that motor).
- **Host mode:** `Servomotor_TestSuite_ESP32S3/host_suite <port> <uid> <module>` runs the SAME
  Arduino modules from the Mac, so the rack can exercise the library against 0.15.12.0.

### Hard-won hardware lessons (do not relearn these)

- `run_all_tests.py` is a SINGLE-MOTOR suite. On the 35-motor rack it produces collision
  garbage, and `test_set_device_alias` **broadcasts**, which re-aliased all 35 motors from 88
  to 10. Individual tests mostly accept a 16-hex unique ID via `-a`; the runner does not.
- Arduino modules that broadcast or touch aliases are NOT rack-safe in host mode:
  `broadcast`, `multi_device`, `two_motors`, `addressing`, `wrong_addressing`, `alias_edges`,
  `uid_edges`. Also skip `calibration*` (needs a free shaft) and `bootloader_window`.
- Keep MOSFETs OFF where possible: the COMMANDED position advances regardless, so most
  planner/queue behaviour can be checked without anything turning. Rack-safe and load-safe.
- `cp` is aliased to `cp -i` here; a scripted restore can hang. Use python `shutil.copyfile`.
- Any new `.cpp` in the sketch folder is compiled into the DEVICE build — host-only files need
  `#if !defined(ARDUINO)`. And `Communication::openSerialPort()` is a no-op off-device, so host
  binaries must call `serialPort.begin(230400)` themselves or every test "passes" for the
  wrong reason.

## Cross-motor verification (2026-08-04)

The new modules were developed against one rack motor (`5E2E8161CF1D2624`). To rule out having
tuned them to that unit, five were re-run against a different one (`9474B90381CC1317`):

| module | motor 1 | motor 2 |
|---|---|---|
| `queue_accounting` | 20/20 | 20/20 |
| `determinism` | 21/21 | 21/21 |
| `error_taxonomy` | 29/29 | 29/29 |
| `stop_semantics` | 36/36 | 36/36 |
| `deviation_tracking` | 31/31 | 31/31 |

Identical throughout. Combined with `test_fleet_consistency.py` agreeing across all 35 motors,
the new tests are motor-independent.

## Command coverage: where the thin spots actually are

`Arduino_library/Servomotor_TestSuite_ESP32S3/coverage_report.py` maps every command in
`motor_commands.json` to its Arduino method and counts call sites across the `tm_*.cpp` modules.
Run `./coverage_report.py --gaps 8` for the thin tail.

Result as of 2026-08-04 (84 modules): **only ONE of 48 commands has zero uses** — `Firmware
upgrade` (cmd 23), which is destructive and slow and is owned by the Python suite instead. So
there is no blind spot at the command level.

The useful signal is the thin TAIL, which is where the remaining work should aim:

| uses | id | command |
|---|---|---|
| 2 | 7 | Capture hall sensor data — parameter-rich, has its own error codes 20/21/49 |
| 2 | 24 | Get product description |
| 3 | 9 | Get current time |
| 3 | 10 | Time sync |
| 3 | 37 | Get comprehensive position |
| 4 | 18 | Get product specs |
| 5 | 33 | Get hall sensor statistics |
| 5 | 35 | Read multipurpose buffer |
| 7 | 32 | Control hall sensor statistics |
| 7 | 39 | Get max PID error |

Caveat the script states itself: a HIGH count means "used often", not "well tested" —
`getPositionRaw` leads at 234 because it is how nearly every other assertion is measured. Only the
low end is meaningful, and counts understate commands that have both a converted and a `...Raw`
form.

## Ideas: areas believed UNCOVERED (working list)

1. Planner arithmetic: negative displacements, sign symmetry, values at int32/int64 edges
2. Composition: chained moves, superposition onto nonzero velocity, mixed move types
3. Unit-conversion boundaries: round-to-zero, float precision, extremes per unit
4. State-machine matrix: every command in every state (homing / calibrating / closed loop / fatal)
5. Limit interactions: changing limits mid-queue, safety fence vs planner, deviation limit
6. Queue arithmetic: exact 32-slot capacity, slot cost per move type, fill-to-exactly-full
7. Error recovery completeness: after EACH distinct fatal error, does reset fully restore?
8. Persistence: what survives reset vs power cycle
9. Timing: command floods during motion, back-to-back framing
10. Regression guards for the three 0.15.12.0 bugs (partly done in test_move_edge_cases.py)

## Findings

*(bugs found, with root cause; NOT fixed — see Tom's instruction)*

### F1 — Lowering `max_velocity` mid-move faults an already-validated move (behavioural, fw 0.15.12.0)

**Severity: not a crash; a usability/safety-design wart worth Tom's judgement. Not fixed.**

**What happens.** Queue a move that is legal under the current limits, then call
`set_maximum_velocity` with a value *below* the speed the move is already travelling at. The
motor immediately raises fatal **error 16, ERROR_VEL_TOO_HIGH**, aborts the move, and requires a
reset. Verified on rack motor `5E2E8161CF1D2624`, fw 0.15.12.0
(`tm_setting_timing` section 2b: ceiling 5 → 0.2 rot/s mid-move → `fatal=16`).

**Root cause.** Two independent velocity checks with different scopes:

1. *Queue time* — `add_to_queue()` checks the derived acceleration and the **predicted** velocity
   against the limits (`motor_control.c:1795`, `:1900`, `:1944`). This is what validates a move.
2. *Every control tick* — `motor_control.c:2187`:
   ```c
   if((current_velocity_i64 > max_velocity) || (current_velocity_i64 < -max_velocity)) {
       fatal_error(ERROR_VEL_TOO_HIGH);
   }
   ```
   This runs 31250 times a second against the **live** velocity and the **current** value of
   `max_velocity`.

`set_max_velocity()` (`motor_control.c:3341`) writes `max_velocity` immediately, with no reference
to what is in flight. So check 2 is evaluated against a limit the in-flight move was never
validated against, and the move that was legal one tick ago becomes a fatal error the next.

**Why it matters.** The obvious way to slow a machine down is to lower its speed limit. Doing that
while it is moving faults it instead — an abrupt uncommanded stop plus a latched fatal error, which
is the opposite of the intended effect. The safe orderings (stop first, or only ever *raise* the
limit mid-flight) are not stated anywhere in the API documentation.

**Options, if Tom wants it changed** (all deliberately NOT implemented):
- Leave as is and document the asymmetry in the API docs + `knowhow.md`. Cheapest, no firmware risk.
- Make `set_max_velocity` reject a value below the current in-flight speed with a *non-fatal*
  error, so the caller learns without the machine faulting.
- Have the new ceiling apply only from the next queue item, matching how the trapezoid shape is
  already latched at queue time.

**Regression guard in place:** `tm_setting_timing` asserts the current behaviour explicitly
(error 16), so whichever way this is resolved, the test will flag the change rather than silently
tracking it.

### F2 — A faulted motor can report only its error number: no telemetry survives a fault

**Severity: not a crash; a diagnostic limitation for Tom's judgement. Not fixed.**

**What happens.** Once a fatal error is latched, **`get_status` (cmd 16) is the only command that
returns a real reply.** Every other command — including `get position`, `get supply voltage`,
`get temperature`, `get current time`, `get firmware version`, `get product info`, `ping`,
`get hall sensor position` — answers with the latched error code instead of its data.
`system reset` still works and is the only way out.

Measured on rack motor `5E2E8161CF1D2624`, fw 0.15.12.0, probing twelve commands in two orders.
The rival explanation ("the first request after a fault is consumed") was ruled out deliberately:
`ping` is refused even when sent first, and `get_status` succeeds on the fourth consecutive call.
So it is a genuine per-command distinction.

**Root cause.** `fatal_error()` (`common_source_files/error_handling.c:169`) calls `__disable_irq()`
and never returns — it spins in a `while(1)` that flashes the red LED and hand-pumps the UART via
`systick_half_cycle_delay_plus_handle_commands()`. Inside that loop, `if_fatal_error_then_respond_flag`
causes an error packet to be transmitted for the pending command. `get_status` is the one handler
that completes and transmits its own reply first.

**Why it matters.** The behaviour is a sound fail-safe — a faulted machine SHOULD refuse to act, and
answering with the reason beats going silent. But it also means an operator whose motor just stopped
can retrieve exactly one number and nothing else. Temperature, supply voltage, position and the
clock are precisely the readings needed to work out *why* it faulted (overheat? brownout? mechanical
jam?), and all of them are unavailable at the only moment they matter.

**Options, if Tom wants it changed** (all deliberately NOT implemented):
- Leave as is, and document it in the API docs + `knowhow.md` — the cheapest option and no firmware
  risk. At minimum users should be told to poll telemetry *before* a fault, not after.
- Whitelist the read-only telemetry getters in the fatal-error loop alongside `get_status`. They
  touch no motor state and cannot restart motion, so the fail-safe property is preserved.
- Snapshot the key telemetry into the status reply at the moment `fatal_error()` fires, so the one
  answerable command carries the context with it.

**Regression guard in place:** `tm_fatal_state_commands` (25 assertions) asserts the current
contract in full — including that motion commands and `enableMosfets` stay refused, that the
original error code is not overwritten by later attempts, and that reset always recovers.

## Progress log

| when | what | result |
|---|---|---|
| 2026-08-04 | campaign started; baseline recorded | — |
| 2026-08-04 | `tm_planner_arithmetic` (38 assertions) | 38/38 on rack |
| 2026-08-04 | `tm_move_composition` (21 assertions) | 21/21 on rack |
| 2026-08-04 | `tm_unit_equivalence` (26 assertions) | 26/26 on rack |
| 2026-08-04 | `tm_queue_accounting` (20 assertions) | 20/20 on rack |
| 2026-08-04 | `tm_setting_timing` (19 assertions) | 19/19 on rack; surfaced finding **F1** |
| 2026-08-04 | `tm_determinism` (21 assertions) | 21/21 on rack |
| 2026-08-04 | `test_fleet_consistency.py` (15 checks) | 15/15 on 4 motors; 35-motor run in progress |
| 2026-08-04 | `tm_error_taxonomy` (29 assertions) | 29/29 on rack |
| 2026-08-04 | `tm_position_accumulator` (53 assertions) | 53/53 on rack |
| 2026-08-04 | `test_fleet_consistency.py` full run | **15/15 across all 35 motors**, 5m56s |

| 2026-08-04 | `tm_crc32_matrix` (25 assertions) | 25/25 on rack |
| 2026-08-04 | all 8 new modules on ESP32-S3 (fw **0.15.9.0**) | see "negative control" below |

| 2026-08-04 | `tm_multimove_bitmask` (14 assertions, 30 masks) | 14/14 on rack |
| 2026-08-04 | `tm_stop_semantics` (36 assertions) | 36/36 on rack |
| 2026-08-04 | `tm_time_domain` (32 assertions) | 32/32 on rack; surfaced finding **F2** |
| 2026-08-04 | `tm_fatal_state_commands` (25 assertions) | 25/25 on rack |
| 2026-08-04 | added `tf_motion_helpers.h` | shared setup so new modules avoid the known traps |

| 2026-08-04 | `tm_queue_underrun` (29 assertions) | 29/29 on rack, first try |
| 2026-08-04 | `tm_readonly_purity` (48 assertions) | 48/48 on rack, first try |
| 2026-08-04 | `tm_move_type_equivalence` (27 assertions) | 27/27 on rack, first try |
| 2026-08-04 | `test_host_data_file_integrity.py` (19 checks) | 19/19; host suite now 10/10 |

| 2026-08-04 | `tm_soak` (17 assertions, ~5 min) | 17/17 on rack |
| 2026-08-04 | `tm_command_state_matrix` (13 assertions, 36 cells) | 13/13 on rack |
| 2026-08-04 | `test_host_unit_parity.py` (45 checks) | 45/45 |

| 2026-08-04 | `tm_duration_extremes` (34 assertions) | 34/34 on rack |

| 2026-08-04 | `tm_reset_completeness` (19 assertions) | 19/19 on rack |
| 2026-08-04 | `tm_deviation_tracking` (31 assertions) | 31/31 on rack |

| 2026-08-04 | **11-module batch** (8 agent-authored + 3 hand-written) | **471/471 on a clean bus** |
| 2026-08-04 | `test_host_test_suite_hygiene.py` (10 checks) | 10/10; caught 2 wrong cmd numbers |

| 2026-08-04 | **batch 2** (8 agent-authored) | **323/323 on a clean bus** |

| 2026-08-05 | **batch 3** (8 agent-authored) | **312/312 on rack** (after 2 fixes) |
| 2026-08-05 | **batch 4** (8 agent-authored) | **307/307 on rack, all clean first try** |

**FINAL TOTALS (2026-08-05)**

| | baseline | final | change |
|---|---|---|---|
| Arduino on-device modules | 63 | **119** | +56 (1.89x) |
| Arduino static assertions | 843 | **2348** | +1505 (**2.79x**) |
| Python host-only tests | 9 | **12** | +3 |
| Python hardware tests | 77 | **78** | +1 |

The brief was to approximately DOUBLE the test cases. On assertions — the closest thing to a count
of test cases — the suite is at **2.79x** the baseline. On module count it is at 1.89x.
All 32 new modules pass on the rack; the host suite is 12/12.

### Batch 3 (312 assertions) and batch 4 (307 assertions)

| module | assertions | | module | assertions |
|---|---|---|---|---|
| `tm_repeated_command_idempotence` | 44 | | `tm_position_far_field` | 40 |
| `tm_power_on_defaults` | 40 | | `tm_unit_extreme_values` | 40 |
| `tm_homing_arithmetic` | 40 | | `tm_recovery_sequences` | 39 |
| `tm_concurrent_command_pressure` | 40 | | `tm_realistic_motion_patterns` | 39 |
| `tm_settings_interaction_matrix` | 39 | | `tm_streaming_motion` | 38 |
| `tm_response_field_integrity` | 39 | | `tm_firmware_compat_gating` | 38 |
| `tm_boundary_value_sweep` | 38 | | `tm_motion_timing_precision` | 37 |
| `tm_error_code_reachability` | 34 | | `tm_command_latency_profile` | 36 |

Batch 4 needed no fixes at all. Batch 3 needed two, and both turned into documented behaviour
rather than weakened tests: the `goToPosition` convergence property (observation O2) and the
safety-fence reply race (finding F3).

Notable results established by these two batches:
- **The 3-slot/2-slot planner frontier is exactly `max_acceleration_raw == 8 * max_velocity_raw`**
  (`delta_t1 = 16v/a`). One raw count either side flips the plan. The float setters cannot reach
  the boundary — `109951162.7776f` rounds to the wrong side — so the module uses the raw setters.
- **Homing has no arithmetic of its own**: its closed-loop precondition is evaluated first and
  unconditionally, so no parameter value (zero, negative, INT32_MIN, UINT32_MAX) changes the answer
  from error 13 — not even a zero duration, which would otherwise be 34.
- **The position accumulator is genuinely 96-bit**: identical behaviour 34 billion counts from the
  origin as at zero, across the 2^31 and 2^32 carry boundaries in both signs. The 32-bit absolute
  go-to layered on top fails loudly with `ERROR_MOVE_TOO_FAR` when its reach runs out, rather than
  wrapping.
- **Motion timing is deterministic**, not merely correct on average: a move runs for exactly the
  commanded number of 32 us ticks and stops on an exact tick, with bounded jitter.
- **300 moves streamed** through the 32-slot ring, topped up as it drains, never stall and land
  exactly on the origin.

### Batch 2, measured on a clean bus

| module | assertions |
|---|---|
| `tm_communication_statistics` | 45 |
| `tm_position_source_consistency` | 41 |
| `tm_time_sync_protocol` | 41 |
| `tm_status_flag_semantics` | 40 |
| `tm_multipurpose_buffer` | 40 |
| `tm_queue_ordering` | 39 |
| `tm_packet_size_encoding` | 39 |
| `tm_indicator_commands` | 38 |
| **total** | **323, zero failures** |

Two things the authoring agents got right that are worth keeping:
- One rejected a WRONG PREMISE in my brief. I asked it to prove the communication counters
  increment by sending pings; it read `common_source_files/RS485.c` and found there is no packet
  counter at all — all six are ERROR counters, so pings leave them at zero. It asserted the
  opposite (30 clean commands leave every counter at zero) and drove increments with malformed
  frames instead.
- The same module's rack safety rests on filter ORDERING: its injected frame is extended-addressed
  and too short to hold a command plus CRC32, so it dies at `RS485.c:255-259` on the target while
  every other motor discards it at the unique-ID comparison (`RS485.c:236-243`) *before* any
  counter is touched. The module asserts that ordering explicitly, which is what makes injecting
  malformed frames safe on a 35-motor bus.

### Batch 2 on the ESP32-S3 (fw 0.15.9.0) — 7 of 8 identical

Run against the pre-fix bench motor: `communication_statistics` 45/45, `indicator_commands` 38/38,
`packet_size_encoding` 39/39, `position_source_consistency` 41/41, `queue_ordering` 39/39,
`status_flag_semantics` 40/40, `time_sync_protocol` 41/41 — all matching the rack exactly.

`multipurpose_buffer` was 38/2. Both failures are the SAME pre-fix divide-by-zero already
documented: the module induces a fault with an over-speed move, and on 0.15.9.0 that move is
silently ACCEPTED (delta_t1 = 0 -> acceleration 0), so no fault latches and the "faulted motor
refuses the read" check has nothing to test. Not a new finding, and not a difference in the
multipurpose buffer itself.

### A pre-existing mislabel the hygiene guard was blind to

A batch-3 agent noticed that `tm_homing_edges.cpp:6` had called Homing "cmd 30" since it was
written. Homing is **cmd 14**; 30 is `Set safety limits`. Fixed.

`test_host_test_suite_hygiene.py` should have caught this and did not. Its regex captured the
literal text before the parenthesis — "HOMING command" — which is not a command name, so the check
found nothing to compare and silently passed. It now strips a leading "the " and a trailing
" command" before matching. **A guard that fails open is worse than no guard**, because it is
credited with coverage it does not have; this one was, for two days.

### F3 — `Set safety limits` loses its own reply about a third of the time when the new fence excludes the current position

**Severity: not a crash, not a safety failure — the fence works. It is a diagnosability defect and
a documented-race gap. Not fixed.**

**What happens.** Set a well-formed fence (lower < upper) that does not contain the current
position. The fence works correctly: `ERROR_SAFETY_LIMIT_EXCEEDED` (25) is raised within one
control tick. But the command's own reply frequently never arrives, so the host sees a bare
timeout and cannot tell "the motor faulted" from "the motor is gone" without a follow-up
`get_status`.

Measured on rack motor `5E2E8161CF1D2624`, fw 0.15.12.0, 30 trials each with a control:

| case | replies received | replies LOST | fatal code |
|---|---|---|---|
| fence EXCLUDES the current position | 20/30 | **10/30 (33%)** | 25 in 29/30 |
| fence INCLUDES it (control) | 30/30 | **0/30** | 0 in 30/30 |

The control rules out bus contention or a flaky link: the loss is caused by the fault, not by the
transport.

**Root cause, and why it is interesting.** The handler (`firmware/Src/main.c:698-716`) does:

```c
if (limits_input.lower_limit > limits_input.upper_limit) {
    fatal_error(ERROR_PARAMETER_OUT_OF_RANGE);   // line 710
}
set_movement_limits(...);                        // line 712  <-- fence is now live
...
rs485_transmit_no_error_packet(is_broadcast);    // line 715  <-- reply sent here
```

Between :712 and :715 the fence is in force but the reply has not been sent, and the control ISR
re-checks the fence every tick (`motor_control.c:2640`) and calls `fatal_error()`, which disables
interrupts and never returns.

**The author already knew about this race.** The comment on line 710 says inverted limits
"would put every position out of bounds and trip the per-tick safety check within one control
tick, **racing this command's own reply**" — and inverted limits are therefore rejected
deterministically *before* the limits are applied. That mitigation is correct and this campaign
confirms it works (inverted fences reliably return code 34).

But it only covers the inverted case. A perfectly well-formed fence that simply excludes the
current position reaches :712, arms the same race, and is not protected. The mitigation solved one
instance of a general problem.

`fatal_error()` does try to rescue an in-flight reply
(`common_source_files/error_handling.c`: `if (rs485_transmit_not_done()) { rs485_drain_transmit(); ... }`),
but that only helps once transmission has *started*; in the window between :712 and :715 there is
nothing to drain.

**Options, if Tom wants it changed** (deliberately NOT implemented):
- Send the reply BEFORE applying the limits, so the caller always learns the command was accepted
  and then discovers the fault via `get_status` — the ordering the rest of the API already implies.
- Extend the existing :710 guard to reject a fence that excludes the current position, the same way
  inverted limits are rejected. This changes the API contract (such a fence is currently legal and
  arguably useful as a deliberate trip).
- Leave the behaviour and document that this specific command may not reply, so callers must treat
  a timeout here as "check status" rather than "device lost".

**Regression guard:** `tm_boundary_value_sweep` exercises this rung; its assertion now accepts
either outcome (reply or no reply) while still requiring the fence to fault with code 25, so the
test records the behaviour instead of failing intermittently on it.

## Observations (not bugs, but worth knowing)

### O2 — a single move lands up to one count short; absolute moves converge, relative ones do not

Measured repeatedly across the whole campaign: a trapezoid move delivers up to **one count less**
than commanded — 409599 of 409600, 1638399 of 1638400, 2147483640 of 2147483647. The planner's
segment arithmetic is integer division (`compute_trapezoid_move`), so slightly under one count is
lost per move.

The consequence differs by command, and it is worth stating plainly because it decides which one a
caller should build a machine on:

- **`Go to position` (absolute) CONVERGES.** The second application sees the one-count error and
  corrects it, landing exactly on the target; a third changes nothing. So it is not idempotent on
  first application — it is convergent, and idempotent from the second onwards.
- **`Trapezoid move` (relative) does not self-correct**, so the residue accumulates: measured at
  worst 8 counts over 20 moves of 20 rotations each (`tm_position_accumulator`), i.e. under half a
  count per move.

Neither is a defect — 1 count in 3,276,800 is 0.3 ppm, far below anything mechanical — but a caller
doing thousands of relative moves without ever issuing an absolute one will drift, slowly. Out-and-back
patterns cancel exactly (the soak measured **zero** residue over 200 cycles) because the rounding
goes the same way in both directions.

`tm_repeated_command_idempotence` asserts the convergence property explicitly, including that a
third application is a no-op.

### O1 — `RCC_ICSCR`'s calibration byte reports the EFFECTIVE value, not the factory constant

`Time sync` (cmd 10) returns `rccIcscr`. RM0444 describes the register as `HSICAL[7:0]`
"initialized automatically at startup" plus `HSITRIM[6:0]`, "an additional user-programmable
trimming value that is **added to** the HSICAL bits" — which reads as "the low byte is a read-only
factory constant". `tm_time_sync_protocol` originally asserted that and failed.

Measured on rack motor `5E2E8161CF1D2624` (fw 0.15.12.0), driving the PI controller to both rails:

```
trim 62  ->  icscr 0x3E88   (low byte 136)
trim 66  ->  icscr 0x428C   (low byte 140)
```

The low byte moves by the **same delta as the trim**, so the field reports the effective
calibration, not the raw factory constant.

**Not a firmware defect.** `common_source_files/clock_calibration.c:49` only ever writes
`new_clock_cal_value << RCC_ICSCR_HSITRIM_Pos` and reads the register straight back at `:55`;
nothing in the firmware can move the low byte. It matters only if someone tries to recover the
factory calibration from this field — they will get the trimmed value instead. The test now
asserts the relationship that is actually true (both bytes move together by the same amount),
which still catches a corrupted or half-written register.

### The 11-module batch, measured on a verified-clean bus

| module | assertions |
|---|---|
| `tm_hall_capture` | 64 |
| `tm_telemetry_conversions` | 51 |
| `tm_diagnostics_readouts` | 44 |
| `tm_safety_fence_arithmetic` | 41 |
| `tm_goto_absolute_semantics` | 40 |
| `tm_velocity_move_semantics` | 40 |
| `tm_acceleration_move_semantics` | 40 |
| `tm_multimove_capacity` | 40 |
| `tm_current_and_pid_settings` | 40 |
| `tm_zero_position_semantics` | 40 |
| `tm_device_identity` | 31 |
| **total** | **471, zero failures** |

An earlier run of the same batch showed 37 failures across five modules. **Every one was either bus
contention from subagents running `host_suite` concurrently, or one of two test bugs of mine**
(`hall_capture`'s sample rate outrunning the link, and `diagnostics_readouts` asserting an ordering
that the empty-accumulator sentinel deliberately violates). Both are fixed and both became extra
coverage rather than being weakened. No firmware defect was involved.

**A reset really does restore everything.** Each setting was probed behaviourally (most have no
getter): speed limit, acceleration limit, deviation window, safety fence, position, queue, latched
fault, clock and CRC32 framing all return to their defaults, verified by three cycles of crippling
every setting at once and recovering. This is the assumption the whole suite rests on and it is now
asserted rather than assumed.

### The deviation watchdog, characterised (`tm_deviation_tracking`)

With the MOSFETs off the rotor never follows, so the hall reading is constant (measured spread:
**84 counts while the commanded position advanced 1,638,399**) and the deviation is therefore
exactly the commanded displacement. That turns error 45 into a calibrated instrument:

| configured window | counts | observed trip point |
|---|---|---|
| 0.25 rot | 819,200 | ~732,583 |
| 0.5 rot | 1,638,400 | ~1,518,986 |
| 1 rot | 3,276,800 | ~3,128,090 |
| 2 rot (**factory default**) | 6,553,600 | ~6,374,624 |
| 4 rot | 13,107,200 | ~13,031,769 |

Trip points are monotonic in the window and track it proportionally; the shortfall is the sampling
lag of polling position over the bus, not an early trip. Moves to half of each window complete
untouched, so the watchdog is invisible in normal operation.

**The duration axis is clean to its ceiling.** A one-rotation move spread over `uint32` max ticks
(4294967295 = just over 38 hours) is accepted and creeps at 22.9 counts/s against a predicted 23.8;
the rate scales inversely with duration across two decades (ratio 1.99 for a 2x duration); and the
38-hour move is cancellable by emergency stop in 314 ms. No wrap in the unsigned
`delta_t2 = total_time - 2*delta_t1` subtraction: a `total_time` below the ramp length is refused
(error 15) rather than wrapping into an astronomically long coast. Large displacements in tiny
times, including INT32_MAX in 1 tick, are all refused with error 15 — never truncated.

**The soak is the strongest single result so far.** 200 out-and-back cycles with the MOSFETs off,
no reset between them: **residue exactly 0 counts at every checkpoint**, 0 faults, resting queue
depth 0, supply voltage stable at 24.1-24.2 V. Then 40 more fill-and-drain rounds (360 moves) with
no leak. Five post-soak moves all measured 409599 of a commanded 409600 — the same accuracy as a
cold machine. There is no accumulation, no leak and no drift in the planner.

### The command/state table (recorded by `tm_command_state_matrix`, fw 0.15.12.0)

| command | IDLE | MOVING | QUEUE FULL |
|---|---|---|---|
| all six reads (status, position, queue depth, voltage, temperature, ping) | accepted | accepted | accepted |
| set maximum velocity / acceleration / PID constants | accepted | accepted | accepted |
| zero position | accepted | **REFUSED** | **REFUSED** |
| trapezoid move | accepted | accepted | **REFUSED** |
| emergency stop | accepted | accepted | accepted |

Every one of the 36 cells is definite (no timeouts) and repeatable across three trials, and the
machine recovers from every refusal. Combined with `tm_fatal_state_commands`, the fourth column
(FAULTED) is: only `get_status` answers.

Notable positive results from this batch:
- The velocity and acceleration fixed-point scales agree to **1 count in 1.6 million**
  (`VELOCITY_SHIFT_LEFT` 12 vs `ACCELERATION_SHIFT_LEFT` 8): a velocity chain covering 0.5 rot
  measured 1638399 against a predicted 1638400, and a trapezoid move measured 4095999 of 4096000.
- **Read-only commands are genuinely pure.** Polling any of ten getters continuously through a
  2-second move leaves the endpoint identical to the undisturbed reference, and 40 mixed reads
  consume zero queue slots. Supervisory polling is safe.
- **Zero underruns** (`ERROR_RUN_OUT_OF_QUEUE_ITEMS`) across 73 trapezoid moves spanning six ramp
  lengths, including 25 in the former one-tick-ramp race regime.

### The ESP32-S3 is a NEGATIVE CONTROL, and it worked

The bench motor behind the ESP32-S3 (`99856389A2B46555`) is still on **fw 0.15.9.0** — i.e. BEFORE
the three planner fixes. There is no serial path from the Mac to it (`/dev/cu.usbserial-1420` and
`-1430` were probed and have no motors), so it cannot be upgraded from here.

That turns out to be valuable rather than a problem: it lets the same test binary run against
buggy firmware and fixed firmware, which is the only way to show the new tests are not vacuous.

`tm_planner_arithmetic` on 0.15.9.0: **34 passed, 4 failed** — and the four failures are precisely
the divide-by-zero bug:

```
maxVel=0.10 -> accepted=1 fatal=0 moved=0      <- silent no-op: accepted, no error, no motion
maxVel=0.20 -> accepted=1 fatal=0 moved=0
maxVel=0.30 -> accepted=1 fatal=0 moved=0
1000 counts over 1 ticks: accepted but moved 0, asked 1000
```

The identical module on the rack's 0.15.12.0 passes all 44. So the tests genuinely detect the
class of bug they were written for, and the fix genuinely closes it — verified on two different
firmware versions with one binary.

**Consequence for reading ESP32-S3 results:** failures there are EXPECTED for anything that
asserts 0.15.12.0 behaviour (the 2-slot fast path, the one-tick-ramp regime, `total_time == 1`).
Do not log them as new findings. Rack results are the authority for current firmware.

Full cross-firmware comparison of the first eight new modules:

| module | rack (0.15.12.0) | ESP32-S3 bench (0.15.9.0) | the difference |
|---|---|---|---|
| `planner_arithmetic` | 44/44 | 34/38 | divide-by-zero: silent no-op |
| `move_composition` | 21/21 | 21/21 | — |
| `unit_equivalence` | 26/26 | 26/26 | — |
| `queue_accounting` | 20/20 | 9/20 | 2-slot fast path does not exist pre-fix |
| `setting_timing` | 19/19 | 19/19 | — |
| `determinism` | 21/21 | 19/21 | the one-tick-ramp race |
| `error_taxonomy` | 29/29 | 26/29 | over-speed move silently accepted, never faults |
| `position_accumulator` | 53/53 | 53/53 | — |

**Every single failure on the old firmware traces to a bug that 0.15.12.0 fixed, and every module
that does not touch those bugs passes identically on both.** That is the strongest available
evidence that these tests detect real defects rather than merely passing.

Notable positive result: `trapezoid move` at **INT32_MAX (+/-2147483647 counts, ~655 rotations)**
is performed correctly and in the commanded direction — 2147483640 of 2147483647 delivered over
55 s. No int32 wrap at the edge. Accumulation over 400 rotations drifts at most 8 counts, and six
30-rotation out-and-back legs leave **zero** residue.

### The new Python fleet test

`python_programs/test_fleet_consistency.py` is a genuinely new KIND of test: it probes every
motor on the bus individually (by unique ID, never a broadcast) and asserts they all AGREE —
same firmware, same distance for the same move, same slot cost, same error code for the same
illegal move. A single motor passing says the firmware works somewhere; 35 agreeing says it works
because of the code and not because of that board. Run it with
`python3 test_fleet_consistency.py -p /dev/cu.usbserial-110` (add `--limit N` for a quick pass).

### Subagents WILL use the hardware unless every prompt forbids it

On 2026-08-04 a workflow authored eight modules in parallel. The authoring prompt forbade running
anything on hardware ("there is a single shared serial bus and other work is using it"). The
follow-up CRITIQUE prompt did not repeat that rule — and the critics ran `./host_suite` against the
rack **concurrently with my own verification batch**.

Symptoms, in the order they appeared and were misread:
- `tm_telemetry_conversions` reported the first raw supply-voltage reads failing, then 24.2 V from
  trial 2 onwards — which looks exactly like a post-reset settling bug.
- A Python probe then threw `SerialException: device reports readiness to read but returned no data
  (device disconnected or **multiple access on port**?)`.
- `pgrep` showed two foreign `host_suite` processes on `/dev/cu.usbserial-110`.

**RS485 is a shared bus with no arbitration.** Two hosts talking at once corrupt each other's
frames, and the failures look like device faults, not contention. Any result gathered while a
second process had the port is worthless.

**How to apply:**
- Repeat the no-hardware rule in EVERY prompt of a multi-phase workflow, not just the first.
- Before trusting any hardware run, check `pgrep -f host_suite` — and after, too.
- If results look like intermittent device misbehaviour, check for bus contention BEFORE
  investigating the firmware.

### Counting discipline

**Assertion counts must be read from the `Assertions: N passed, M failed` line of an actual run,
never inferred.** `tm_planner_arithmetic` was recorded here as 44 assertions; the real figure is
38 (3+8+10 calls to `expectMoveOrError`, one assertion each, plus 4+9+3+1 elsewhere). The 44 came
from my reading of the first, pre-fix run and was never re-checked against a clean run. It was
caught only because a cross-motor run printed 38. Totals below are being re-measured from run
output rather than carried forward from notes.

### False alarms already burned (do not repeat)

- `tm_planner_arithmetic` first reported `1000 counts over 200000 ticks: moved 626, asked 1000`.
  That was MY test bug: a fixed wait capped at 4000 ms against a 6400 ms move; 4000/6400 = 0.625.
  **Never wait a computed duration for a move — poll `getNQueuedItems()` to zero with a bounded
  deadline.** Every module here now uses a `drainQueue()` helper.
- `multimoveList_t` is `{int32_t value; uint32_t timeSteps;}` — acceleration (type bit 0) or
  velocity (type bit 1). It is **not** displacement/duration; multimove cannot express a trapezoid
  move. Confirmed in `MULTIMOVE_COMMAND` in firmware `main.c`.
- A velocity segment **persists after its duration expires**. Any velocity sequence must end with
  an explicit zero-velocity segment or the motor keeps going.
- **Limits are pass/fail checks, never clamps.** The firmware does NOT stretch an over-ambitious
  move to fit `max_velocity`/`max_acceleration` — it refuses it with a fatal error (15, 16 or 28).
  Two of my assertions assumed clamping and failed for that reason. Read `add_to_queue()` before
  predicting an outcome.
- A **fatal error aborts the whole queue**, including legal moves queued before the offending one.
- Slot cost of one trapezoid move: **3** normally, **2** on the 0.15.12.0 one-tick-ramp fast path
  (`delta_t1 <= 1 && total_time >= 3`). So 10 moves fit in the queue normally, 16 on the fast path.
  Both verified.
- **MOSFETs-off testing is capped at ~2 rotations** unless you raise the deviation limit.
  `DEFAULT_MAX_ALLOWABLE_POSITION_DEVIATION` is `ONE_REVOLUTION_MICROSTEPS * 2`
  (`motor_control.c:97`). With the output stage off the shaft never follows, so deviation grows
  with every commanded count and anything past two turns trips **error 45**. Raise it with
  `setMaxAllowablePositionDeviation` for any large-displacement test — this cost a whole module
  run to learn. A reset restores the default.
- The **experimental velocity ceiling** is `2 x MAX_VELOCITY` (`motor_control.h:61`), about
  18.7 rot/s. A displacement sweep that does not scale its durations just ends up measuring the
  speed limit and refusing everything.
- **Acceleration segments ACCUMULATE velocity across a queued sequence.** Four consecutive
  acceleration segments of `20 rot/s^2 x 0.15 s` reach 12 rot/s, so a ceiling picked for the typical
  pattern refuses the worst one. This cost two debug cycles in `tm_multimove_bitmask`.
- **`--only N` on the ESP32-S3 can silently skip the requested test.** The device keeps its
  progress in NVS; if the previous run's completion record did not land before the next invocation
  reset the board, the boot reports `TEST_CRASH <previous idx>` and exits WITHOUT running the test
  you asked for. Run each `--only` twice and take the second result. Two modules were falsely
  recorded as "crashed" this way.
- **Strip C comments before scanning firmware source.** `test_host_data_file_integrity.py`'s first
  run reported `ERROR_POSITION_OUT_OF_RANGE` and `ERROR_HALL_POSITION_OUT_OF_RANGE` as "raised but
  undefined". Both appear ONLY in commented-out `fatal_error()` calls (`motor_control.c:3177`,
  `:3182`), so neither is reachable. Dead code reads as live code to a naive regex.
- **`ADD_TO_QUEUE_TEST_COMMAND` (cmd 120) is deliberately not in `motor_commands.json`** — it is a
  firmware-internal harness (`main.c:718`), not public API. The host test carries an explicit
  allowlist for it, plus a staleness check so the allowlist cannot silently excuse nothing.
- **Use `tf_motion_helpers.h` for new modules** (`tfhReset`, `tfhFreshOpen`, `tfhDrain`,
  `tfhTicksFor`, `tfhMove`). Every helper in it exists because getting it wrong produced a false
  bug report during this campaign.
