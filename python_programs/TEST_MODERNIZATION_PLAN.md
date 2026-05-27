# Test Suite Modernization & Coverage Plan

Status: **Decisions from the 2026-05-19 review are incorporated below.**
Ready to implement as `WORK_CHECKLIST.md` TODO #5. Companion to
`TEST_INVENTORY.md` (the landscape snapshot).

## Goal

Every motor command has its own dedicated, automatically-run test that makes a
real pass/fail assertion. On top of that, a set of **functional tests**
exercises specific behaviour across multiple commands. Useful tests that exist
today but are not run (the "obsolete" pile) are modernized onto the
`servomotor` library — or explicitly retired.

End state: a developer changes firmware or the Python library, runs one
command, and knows whether anything regressed across the whole command set.

## Starting point for a fresh session

To begin TODO #5 with no extra context:
- All test programs live in `python_programs/`. Run them with the hardware
  runner `run_all_tests.py` (needs a motor) or the host-only `run_host_tests.py`.
- Use an existing passing test — `test_ping.py` is the simplest — as the
  structural template for new per-command tests; it shows the standard
  `servomotor` open/reset/run/close pattern (also documented in `CLAUDE.md`).
- New *host* tests (migrated from the obsolete non-hardware tests) use the
  `host_test_framework.py` mini-framework and are named `test_host_*.py`.
- `TEST_SUMMARY.md` is a Markdown table with columns
  `Filename | Description | Module Used | Obsolete | Prints PASS/FAIL`.
  `run_all_tests.py` runs every row where `Module Used = servomotor` and
  `Obsolete = No`, **in table order**. A test missing from the table never runs.
- The 48 command definitions are in `servomotor/motor_commands.json`; the
  generated method name is the command string lowercased with spaces → `_`.
- Hardware/port/alias details (test motor is an M17 at alias `X`) are in
  `WORK_CHECKLIST.md` → "Environment / hardware notes".
- Then follow "Suggested order" at the bottom of this doc, starting at step 1.

## Decisions (2026-05-19 review)

1. **Importance tiers** accepted as proposed, with one change: **Start
   calibration (command 6) moves from Tier 1 to Tier 2.**
2. **Test structure: a test for every command, plus functional tests.** Each
   of the 48 commands gets its own `test_<command>.py`. Additionally, a set of
   functional tests exercises specific functionality spanning several commands.
3. **Calibration is tested and ordered first.** `test_start_calibration.py`
   must run *before* any test that relies on closed-loop mode or on accurate
   position sensing.
4. **`test_homing_micropython.py` stays a separate MicroPython test.** It is
   not expected to pass right now — the MicroPython controller is not yet
   connected — so it is kept out of the standard PC auto-run suite.
5. **Single motor for now.** All multi-device tests (multi-device time sync,
   multi-motor stress) are deferred: done later, after every single-motor test
   is written and passing. Two motors can be connected at that point.
6. **Human-confirmation prompts auto-detect a TTY.** No flag needed — if stdin
   is a TTY the test prompts the human; if not (e.g. under `run_all_tests.py`)
   it skips the prompt and passes on the automated layer.
7. **Firmware upgrade gets a real test** (command 23). `test_firmware_upgrade.py`
   — using **only the new firmware/bootloader protocol**, never the legacy one —
   upgrades to the latest firmware and checks the version, downgrades to a kept
   older firmware and checks again, then upgrades back to the latest and checks
   the expected version. Command 23 moves from T3 to T2 and is no longer "out of
   scope". See "Firmware upgrade test (command 23)" below.

## Why this is needed

- The motor exposes **48 commands**. A scan of all 41 `test_*.py` files shows
  **24 commands are exercised** in some way and **24 are never called at all**.
- Several "covered" commands are only used incidentally as test *setup* (e.g.
  `enable_mosfets`, `system_reset`) — no test actually *verifies* them.
- **17 test files are catalogued as obsolete**, so `run_all_tests.py` skips
  them. 16 are motor-related — they target the deprecated `communication`
  module, raw `serial`, or are otherwise superseded — losing genuinely useful
  scenarios (closed-loop entry, enable/disable reliability, time sync,
  position-vs-hall cross-check). The disposition table below modernizes,
  merges, migrates, or retires each one.
- **5 test files are not catalogued in `TEST_SUMMARY.md`**, so the runner
  silently ignores them even though some are already modern and good.

## Current state (2026-05-19)

41 `test_*.py` files exist in `python_programs/`:

| Group | Count | Runs automatically? |
| :-- | :-- | :-- |
| Modern `servomotor` hardware tests (catalogued, non-obsolete) | 14 | Yes — via `run_all_tests.py` |
| Host-only tests (`test_host_*.py`) | 4 | Yes — `run_all_tests.py` + `run_host_tests.py` |
| Catalogued but obsolete (`Obsolete = Yes`) | 17 | No — runner skips them |
| Uncatalogued (absent from `TEST_SUMMARY.md`) | 5 | No — runner cannot see them |
| Application test (`test_glue_machine_rotation_matrix.py`) | 1 | No |

(14 + 4 + 17 + 5 + 1 = 41.) Of the 17 obsolete files, 16 are motor-related and
are handled in the disposition table below; the 17th,
`test_continuous_graphing.py`, is an unrelated Matplotlib demo — left alone.

### The 5 uncatalogued files — triage (this is step 1)

| File | Module | Prints `PASSED`? | Action |
| :-- | :-- | :-- | :-- |
| `test_correct_and_incorrect_addressing.py` | `servomotor` | yes | Catalogue as a functional test — ready as-is. |
| `test_get_product_info.py` | `servomotor` | yes | Catalogue as the per-command test for command 22 — ready as-is. |
| `test_set_device_alias.py` | `servomotor` | yes | Catalogue as the per-command test for command 21 — ready as-is. |
| `test_detect_devices.py` | `servomotor` | **no** | Add a `PASSED` last line + non-zero exit on failure, **then** catalogue as the per-command test for command 20. |
| `test_homing_micropython.py` | `servomotor` | no | Catalogue with module label `servomotor (MicroPython)` (anything other than exactly `servomotor`) so the PC runner skips it (decision #4); not expected to pass yet. |

## Hardware availability & test observability

### No external encoder
An external encoder was available previously but is **not available now**.
Consequences:
- The external-encoder field of `Get comprehensive position` (command 37) and
  the position-vs-hall-vs-encoder cross-check cannot be verified at present.
- The functional test `test_position_telemetry.py` covers motor position vs
  hall position. The external-encoder comparison is a **conditional check**:
  skipped with a clear `SKIPPED: no external encoder` message — a skip is not
  a failure — and re-enabled automatically when encoder hardware returns. The
  check stays in the code so nothing is lost.

### Human-confirmation tests (LEDs, vibrate, identify)
Some commands produce an effect only a human can verify — LEDs lighting up,
the motor vibrating, the identify blink. These tests are **kept**, split into
two layers:
- **Communication layer — automated, non-blocking.** The test sends the
  command and asserts it round-trips with no fatal error or timeout. This
  proves the firmware/communication path that drives the indicator still
  works — the part most likely to break silently. This layer runs inside
  `run_all_tests.py`.
- **Visual-confirmation layer — manual, opt-in.** When stdin is a TTY the test
  additionally prompts the human ("Are all LEDs on? [y/n]"). Under
  `run_all_tests.py` stdin is not a TTY, so the prompt is skipped automatically
  and the test passes on the communication layer alone — it **never blocks**.
- The test prints `PASSED` / `FAILED` from the automated layer (and from the
  human's answer when one was given), so the runner can still grade it.

## Test structure: per-command tests + functional tests

Two categories:

- **Per-command tests** — one `test_<command>.py` per command, named after the
  command's generated method (e.g. `Go to position` → `test_go_to_position.py`).
  Focused: minimal setup, exercises that one command, asserts its result.
- **Functional tests** — exercise a specific behaviour across several commands
  (e.g. comms during high-speed motion, queued multi-segment moves, framing-
  error recovery, position cross-checks, closed-loop motion). Most existing
  modern scenario tests already are functional tests.

A command is "done" when it has a passing per-command test; functional tests
add interaction coverage on top.

## Test ordering & dependencies

`run_all_tests.py` runs tests in the order their rows appear in
`TEST_SUMMARY.md` (the table is parsed and executed top-to-bottom). Ordering
is therefore controlled by **row order in `TEST_SUMMARY.md`**.

Required constraints:
- `test_firmware_upgrade.py` runs **first** — it ends by installing the latest
  firmware, so every later test runs against that firmware (decision #7).
- `test_start_calibration.py` runs **after** the firmware-upgrade test but
  **before** `test_go_to_closed_loop.py`, the closed-loop functional test, and
  any test that depends on accurate position sensing (decision #3).
- Row order in `TEST_SUMMARY.md` is what enforces this.

Caveat to be aware of: the firmware-upgrade test reflashes the device three
times and calibration recalibrates it — together they add several minutes to a
suite run and write device flash. Acceptable per the review; noted here so the
cost is not a surprise.

## Per-command coverage matrix (48 commands)

Tiers: **T1** core · **T2** important · **T3** nice-to-have / special.
Status: `exists` · `exists*` (exists but uncatalogued — needs a `TEST_SUMMARY.md`
row) · `new`.

| # | Command | Tier | Per-command test file | Status |
| :-- | :-- | :-- | :-- | :-- |
| 0 | Disable MOSFETs | T1 | `test_disable_mosfets.py` | exists |
| 1 | Enable MOSFETs | T1 | `test_enable_mosfets.py` | exists |
| 2 | Trapezoid move | T1 | `test_trapezoid_move.py` | exists |
| 3 | Set maximum velocity | T1 | `test_set_maximum_velocity.py` | exists |
| 4 | Go to position | T1 | `test_go_to_position.py` | exists |
| 5 | Set maximum acceleration | T1 | `test_set_maximum_acceleration.py` | exists |
| 6 | Start calibration | T2 | `test_start_calibration.py` | exists — runs after firmware upgrade |
| 7 | Capture hall sensor data | T3 | `test_capture_hall_sensor_data.py` | exists |
| 8 | Reset time | T2 | `test_reset_time.py` | exists |
| 9 | Get current time | T2 | `test_get_current_time.py` | exists |
| 10 | Time sync | T2 | `test_time_sync.py` | exists (modernized from obsolete) |
| 11 | Get n queued items | T1 | `test_get_n_queued_items.py` | exists |
| 12 | Emergency stop | T1 | `test_emergency_stop.py` | exists |
| 13 | Zero position | T1 | `test_zero_position.py` | exists |
| 14 | Homing | T1 | `test_homing.py` | exists |
| 15 | Get hall sensor position | T2 | `test_get_hall_sensor_position.py` | exists |
| 16 | Get status | T1 | `test_get_status.py` | exists |
| 17 | Go to closed loop | T1 | `test_go_to_closed_loop.py` | exists |
| 18 | Get product specs | T2 | `test_get_product_specs.py` | exists |
| 19 | Move with acceleration | T1 | `test_move_with_acceleration.py` | exists |
| 20 | Detect devices | T1 | `test_detect_devices.py` | exists* — needs a `PASSED` line |
| 21 | Set device alias | T1 | `test_set_device_alias.py` | exists* |
| 22 | Get product info | T1 | `test_get_product_info.py` | exists* |
| 23 | Firmware upgrade | T2 | `test_firmware_upgrade.py` | exists — heavy; see "Firmware upgrade test" below |
| 24 | Get product description | T2 | `test_get_product_description.py` | exists |
| 25 | Get firmware version | T1 | `test_get_firmware_version.py` | exists |
| 26 | Move with velocity | T1 | `test_move_with_velocity.py` | exists |
| 27 | System reset | T1 | `test_system_reset.py` | exists |
| 28 | Set maximum motor current | T1 | `test_set_maximum_motor_current.py` | exists |
| 29 | Multimove | T1 | `test_multimove.py` | exists |
| 30 | Set safety limits | T1 | `test_set_safety_limits.py` | exists (modernized from obsolete) |
| 31 | Ping | T1 | `test_ping.py` | exists |
| 32 | Control hall sensor statistics | T3 | `test_control_hall_sensor_statistics.py` | exists |
| 33 | Get hall sensor statistics | T3 | `test_get_hall_sensor_statistics.py` | exists |
| 34 | Get position | T1 | `test_get_position.py` | exists |
| 35 | Read multipurpose buffer | T3 | `test_read_multipurpose_buffer.py` | exists |
| 36 | Test mode | T1 | `test_test_mode.py` | exists |
| 37 | Get comprehensive position | T1 | `test_get_comprehensive_position.py` | exists (encoder field not verifiable now) |
| 38 | Get supply voltage | T2 | `test_get_supply_voltage.py` | exists |
| 39 | Get max PID error | T2 | `test_get_max_pid_error.py` | exists |
| 40 | Vibrate | T3 | `test_vibrate.py` | exists — human-confirm |
| 41 | Identify | T3 | `test_identify.py` | exists — human-confirm |
| 42 | Get temperature | T1 | `test_get_temperature.py` | exists |
| 43 | Set PID constants | T2 | `test_set_pid_constants.py` | exists |
| 44 | Set max allowable position deviation | T2 | `test_set_max_allowable_position_deviation.py` | exists |
| 45 | Get debug values | T3 | `test_get_debug_values.py` | exists — shape/smoke check |
| 46 | CRC32 control | T1 | `test_crc32_control.py` | exists |
| 47 | Get communication statistics | T2 | `test_get_communication_statistics.py` | exists |

Summary: **all 48 per-command tests now exist and pass individually** —
the original 12, the 2 modernized in step 2 (cmds 10 Time sync, 30 Set
safety limits), the 15 T1 tests written 2026-05-21/22 (cmds 0, 1, 2, 3,
5, 11, 12, 13, 16, 17, 27, 28, 29, 34, 46), the 10 T2 tests written
2026-05-25 (cmds 8, 9, 15, 18, 24, 38, 39, 43, 44, 47), the 7 T3 tests
written 2026-05-25 (cmds 7 Capture hall sensor data, 32 Control hall
sensor statistics, 33 Get hall sensor statistics, 35 Read multipurpose
buffer, 40 Vibrate, 41 Identify, 45 Get debug values), and the 2
specials written 2026-05-26 (cmd 23 Firmware upgrade — ordered first;
cmd 6 Start calibration — ordered second). (12 + 2 + 15 + 10 + 7 + 2 =
48.)

Specials notes worth remembering for follow-ups:
  * **Cmd 23 (Firmware upgrade)** — implements the page-by-page transfer
    inline via `servomotor.execute_command(FIRMWARE_UPGRADE_COMMAND,
    [payload], alias_or_unique_id=..., crc32_enabled=True)`, alias-
    addressed so every page is ACK'd. New protocol only (proper size
    encoding LSB=1 + CRC32 on every packet); the legacy path is never
    exercised. Wall-clock ~17 s on the bench M17 for three full reflashes
    (~7.5 KB/s actual throughput). "Latest" is auto-picked by
    version-sorting the matching
    `servomotor_<model>_fw*_scc<scc>_hw<hw>.firmware` files in
    `firmware/firmware_releases/`. The kept-older firmware is hardcoded
    to `servomotor_M17_fw0.14.0.0_scc3_hw1.5.firmware` — that file MUST
    remain in the repo for this test. End state: device is on the latest
    firmware (the rationale for ordering this test first).
  * **Cmd 6 (Start calibration)** — polls `get_status` until the
    calibrating bit (3) clears; the M17 takes ~21 s on the bench.
    When calibration finishes the firmware auto-reboots the MCU, so
    the poll that catches the bit-clear lands inside the bootloader's
    post-reset window and `get_status` returns `0x0001`. The test
    treats both "calibrating bit clear" and "bootloader bit set" as
    "calibration done", then issues a `system_reset` + 2 s delay to
    push the device back into the application before continuing. End-
    to-end usability is verified by going into closed loop and running
    a small move; if calibration data is bad, that move trips a
    deviation check or misses target. Wall-clock ~29 s on the bench
    M17 against firmware 0.15.0.0. See the
    [[calibration-auto-reboots]] memory note for the auto-reboot
    behavior.

T3 notes worth remembering for follow-ups:
  * **Cmd 7 (Capture hall sensor data)** — the JSON description warns
    "work in progress; don't send this command", so the streaming
    success path is intentionally not exercised. The test only drives
    the parameter-validation surface (five invalid inputs each raise
    `FatalError(49)`); a future test should add the streaming case
    once the wrapper supports variable-length / multi-segment replies.
  * **Cmd 35 (Read multipurpose buffer)** — populator chosen is
    `test_mode(3)` (PID debug data inside closed loop). Two
    alternatives were considered and rejected: `test_mode(4)` (GC6609
    register dump) auto-clears `test_mode`, which would have let us
    cleanly assert "second read times out", but the bit-bang stalls
    waiting for the chip's UART on the bench M17; `start_calibration`
    populates reliably but is far too slow. Because `test_mode(3)`
    does not auto-clear and PID re-fills the buffer on every clear,
    we drop the "second read times out" assertion and rely on
    `system_reset()` for cleanup. The test docstring spells this out.
  * **Cmd 40 (Vibrate)** — the firmware's vibrate function is
    `#ifdef PRODUCT_NAME_M1`; on M17 the function body is empty but
    cmd-40 dispatch still returns success. So the comms layer always
    runs and the visual layer is gated on TTY + product == M1.
  * **`test_mode(0)` is a firmware lockup path discovered while
    writing cmd 35's test** — see WORK_CHECKLIST.md TODO #8 and the
    `led-test-mode-is-stuck-forever` memory note. None of the T3
    tests use it; cleanup is via `system_reset()` everywhere.

## Functional / multi-command tests

| Test file | Exercises | Status |
| :-- | :-- | :-- |
| `test_communication_while_high_speed.py` | Ping stress during a fast motion profile | exists — keep |
| `test_fast_short_move_with_velocity.py` | Queuing many short velocity moves | exists — keep |
| `test_gradual_speed_up.py` | Ramping velocity to the speed limit | exists — keep |
| `test_trigger_framing_error.py` | RS485 framing-error detect + recovery + comm stats | exists — keep |
| `test_correct_and_incorrect_addressing.py` | Alias / unique-ID / broadcast addressing paths | exists* — catalogue |
| `test_random_speed_stress.py` | Random-speed multi-command stress | exists — keep (single-motor mode now; multi later) |
| `test_position_telemetry.py` | Motor position vs hall position cross-check (encoder skipped) | new — modernized |
| `test_closed_loop_motion.py` | Enter closed loop, move, verify position holding | new — modernized |
| `test_enable_disable_reliability.py` | Rapid enable/disable + reset cycling | new — modernized |
| `test_leds.py` | LED control via test mode, human-confirmed | new — modernized |
| `test_time_sync_multiple_devices.py` | Time sync across >1 motor | deferred — opt-in, after single-motor suite is green (decision #5) |

## Firmware upgrade test (command 23)

`test_firmware_upgrade.py` is the per-command test for the firmware-upgrade
command. It is heavy — it reflashes the device three times — but the owner has
specifically requested it (decision #7).

**Protocol — new only.** The test must use **only the new firmware/bootloader
protocol**, never the legacy protocol. `upgrade_firmware.py` exposes both via
`--firmware-protocol` (resetting the device into the bootloader) and
`--bootloader-protocol` (transferring the firmware pages). The new protocol
uses proper size encoding (LSB = 1) and CRC32 checksums; the old one does not.
The test passes `--firmware-protocol new --bootloader-protocol new` (or the
equivalent library calls) and never exercises the old protocol.

**Sequence:**
1. Upgrade the device to the **latest** firmware. Read it back with the
   `Get firmware version` command (command 25) and assert it equals the latest
   version.
2. Downgrade to a **kept older firmware** — a known-good earlier release
   deliberately retained in the repo for this test. Read the version back and
   assert it matches that older firmware.
3. Upgrade back to the **latest** firmware. Read the version back and assert it
   is the expected latest version.

This leaves the device on the latest firmware — the desired end state, and the
reason this test is ordered first (see "Test ordering & dependencies").

**Firmware files.** Both firmware files must match the test motor's model,
hardware version, and software-compatibility code — for the current test M17
that is the `servomotor_M17_fw<ver>_scc3_hw1.5.firmware` pattern in
`firmware/firmware_releases/` (that directory already holds several M17
releases). The test resolves "latest" by version-sorting the matching files.
The "kept older firmware" is a specific designated file that must remain in the
repo for this test — choose one and name it explicitly in the test.

**Caveat:** three full reflashes plus version checks add several minutes to a
suite run and write the device flash repeatedly. Acceptable per the review;
noted so the cost is not a surprise.

## Disposition of the obsolete test files

| Obsolete file | Action |
| :-- | :-- |
| `test_enable_disable.py` | **Modernize** → functional `test_enable_disable_reliability.py`; per-command basics go in `test_enable_mosfets.py` / `test_disable_mosfets.py`. |
| `test_iterate_reset_and_enable.py` | **Merge** into `test_enable_disable_reliability.py` as the reset-cycle variant. |
| `test_go_to_closed_loop_mode_and_spin_motor.py` | **Modernize** → functional `test_closed_loop_motion.py`. |
| `test_go_to_closed_loop_mode.py` | The Goertzel analysis is a research tool — **keep as a tool**, not a pass/fail test. |
| `test_go_to_closed_loop_mode_and_then_get_data_many_times.py` | Data-collection tool — **keep as a tool**. |
| `test_go_to_closed_loop_mode_plot_data.py` | Plotting script — leave as-is (`N/A`). |
| `test_motor_position_vs_hall_sensor_position_vs_external_encoder_position.py` | **Modernize** → functional `test_position_telemetry.py`; external-encoder comparison is conditional, skipped while no encoder is available. |
| `test_time_sync.py` | **Modernize** → per-command `test_time_sync.py` (single device). |
| `test_time_sync_multiple_devices.py` | **Modernize but defer** → multi-device functional test, opt-in, after the single-motor suite is green. |
| `test_safety_limit.py` | **Modernize** → per-command `test_set_safety_limits.py` (drop the raw-`serial` usage). |
| `test_all_leds_on.py` | **Modernize** → functional `test_leds.py`: LED-control command/comms verified automatically, LEDs human-confirmed when interactive. Not retired — the comms path matters. |
| `test_servomotor_module.py` | No verification, superseded — **retire**. |
| `test_json_read.py` | Utility, not a test — **retire**. |
| `test_servomotor_module_get_command_id.py` | Non-hardware unit test — **migrate** into the host-only suite (`test_host_*`). |
| `test_terminal_formatting.py` | Non-hardware — **migrate** into the host-only suite. |
| `test_getch.py` | Non-hardware input utility — **migrate** into the host-only suite. |

## Out of scope for the auto-run suite

- **`test_homing_micropython.py`** — kept as a separate MicroPython reference
  test (decision #4). Catalogued with a non-`servomotor` module label so the
  PC runner does not pick it up; not expected to pass until the MicroPython
  controller is connected.

(Firmware upgrade, previously listed here, is now an in-suite test — see
"Firmware upgrade test (command 23)" above.)

## Conventions every modernized / new test must follow

1. Use the `servomotor` library only — never the deprecated `communication`
   module or raw `serial`.
2. Follow the standard pattern in `CLAUDE.md`: `set_serial_port_from_args`,
   `open_serial_port`, construct `M3`, `system_reset`, run, `close_serial_port`.
3. Accept `-p`, `-P`, `-a`, `-v` arguments (matching the existing tests).
4. Print `PASSED` as the **last stdout line** on success; exit non-zero on
   failure — this is how `run_all_tests.py` grades the test.
5. Be **OS-agnostic**: no hardcoded `/dev/tty*` paths, no POSIX-only calls.
   This directly supports the cross-OS work (WORK_CHECKLIST.md TODO #6).
6. Add a row to `TEST_SUMMARY.md` (`Module = servomotor`, `Obsolete = No`) so
   the runner picks it up — an uncatalogued test is an invisible test.
7. Keep each test reasonably fast; the heavy tests (firmware upgrade,
   calibration) are explicitly ordered and acknowledged, not a surprise.
8. A test that needs human confirmation must default to **non-blocking**: skip
   the prompt when stdin is not a TTY and pass on the automated communication
   layer alone. It must never hang the automated suite waiting for input.
9. A check that depends on hardware that may be absent (e.g. the external
   encoder) must **skip cleanly with a stated reason**, not fail.

## Acceptance criteria

- Every one of the 48 commands has a dedicated `test_<command>.py` that makes a
  real assertion (firmware upgrade included — `test_firmware_upgrade.py`).
- The functional tests above cover the key multi-command interactions.
- No useful scenario from the obsolete pile is lost (each is modernized,
  merged, migrated, or explicitly retired with a reason — see the table).
- `TEST_SUMMARY.md` lists every `test_*.py` file (no silent omissions), with
  rows ordered so calibration precedes closed-loop / position-dependent tests.
- `run_all_tests.py` passes end-to-end on a real M17.

## Suggested order

1. **Catalogue & triage** — add the 5 uncatalogued files to `TEST_SUMMARY.md`;
   confirm which already pass. Cheap, immediate coverage gain.
2. **Modernize the obsolete pile** into the functional tests and per-command
   tests they map to (closed loop, enable/disable, time sync, position
   telemetry, safety limits, LEDs).
3. **Create the remaining per-command tests, tier by tier** — all T1 first,
   then T2, then T3.
4. **Fix `TEST_SUMMARY.md` row order**: `test_firmware_upgrade.py` first, then
   `test_start_calibration.py`, then the closed-loop and position-dependent
   tests.
5. **Defer multi-device tests** until two motors are connected and the whole
   single-motor suite is green (decision #5).
