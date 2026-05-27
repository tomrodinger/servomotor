# Servomotor — Work Checklist & Resumption Guide

**Purpose:** a durable checklist of everything in flight, so work can resume even
after the chat context is cleared. Update the checkboxes as items are finished.

Last updated: 2026-05-28 (TODOs #1, #2, #3, #5 all committed and pushed-ready.
Working tree is clean except for items deliberately left untracked. Firmware
0.15.0.0 for M17 has been released and committed.)

> **▶ NEXT TASK: tom is choosing among the still-open TODOs:**
> - **TODO #6** cross-OS verification (Windows / Linux) — uses the now-committed
>   single-version library and the host-only suite as the cheap first probe
> - **TODO #7** make the high-level M3 wrapper able to drive multimove + crc32
>   (removes the `execute_command` workaround in `test_multimove.py` /
>   `test_crc32_control.py`)
> - **TODO #8** clear the T2-test gotchas (M3 time-unit conversion wrong,
>   `test_mode(0)` firmware lockup, MOSFET-enable commutation-alignment
>   transient — see [[m3-wrapper-time-unit-conversion-broken]] /
>   [[led-test-mode-is-stuck-forever]] / [[enable-mosfets-commutation-transient]])
> - **TODO #9** MicroPython quality-of-life (M3 int-precision sentinel bug,
>   `test_get_temperature` heap-busting refactor, optional perf-gap test tolerances)
> - **TODO #4** minor cleanups (low priority)
>
> No hard blocker on any of them; all are tom's choice.
>
> **Session of 2026-05-27/28 summary.** TODO #1 fully committed across 7
> commits (`63d4de1`–`d6759da`). TODO #2 / #3 / #5 work that had been
> uncommitted-but-done was committed across 8 more commits
> (`10eaded`–`c0b2451`), including the firmware 0.15.0.0 release
> (`fd8b600`) and an updated `.gitignore` (`c0b2451`). 15 new commits
> total on `main`; nothing pushed yet.

---

## How to resume after a context clear

1. Read this file top to bottom.
2. Run `git status` and `git log -1` to see the working-tree state.
3. Read the companion doc for the task being picked up (table below).
4. Continue from the unchecked items under "To do", following "Suggested order".

**Git state as of 2026-05-28:** branch `main`, **15 commits ahead** of
`origin/main`. Working tree is clean except for items deliberately left
untracked (see "What is still untracked" below). Nothing has been pushed.

Commits in this run (oldest → newest), all on `main`:

| Hash | Subject (truncated) |
| :-- | :-- |
| `63d4de1` | Makefile G031 fix + ADC.c G031 shim + M23 ADC sequence |
| `a6023cd` | M23 current_streaming/control + PWM + m23 PC tools; fw bump to 0.15.0.0 |
| `60f2887` | `time_sync` uint64→uint32 bug fix (firmware + bootloader) |
| `ef63e31` | motor_control.c M17 calibration tuning + per-product baseline tolerance |
| `a8705b9` | Python bugfixes (detect_and_set_alias, magnetic_disk_machine_V2) |
| `030d4d4` | Datasheets: M17-34 model + dimension drawings + datasheet v1.8 |
| `d6759da` | Marketing typo fixes |
| `10eaded` | TODO #3 — MicroPython library refactor (single-version library) |
| `b5aec36` | TODO #2 — host-only test suite |
| `3a44cee` | TODO #5 — test modernization (48 per-command tests + 5 modified) |
| `c453786` | MicroPython hardware deployment (shims, mp_runner, deploy scripts) |
| `4ffaab1` | Planning + design docs (this file, CHANGE_ANALYSIS, CLAUDE, MP designs) |
| `9e7d353` | New tools (m23 capture, magnetic_disk_machine_V2, binary_diff, pwm_adc visualizer) |
| `fd8b600` | Release firmware 0.15.0.0 for Servomotor M17 |
| `c0b2451` | `.gitignore` for scratch / AI working notes (`.firmware` NOT ignored) |

**What is still untracked** (intentional, per "leave for now" decisions):

- `Drivers/CMSIS/Device/ST/STM32G4xx/`, `Drivers/STM32G4xx_HAL_Driver/`
- `bootloader_STM32G431/`
- `Mechanical/`, `Pictures/applications/`, `assembly_steps/`, `datasheets/`,
  `pricing_calculations/`
- Root-level `micropython/` (separate from the committed
  `python_programs/micropython/`)
- Older un-released `.firmware` binaries in `firmware/firmware_releases/`
  (M17 0.14.x, M23 0.14.x, M3 0.14.2, M4 0.14.2). The current release is
  `servomotor_M17_fw0.15.0.0_scc3_hw1.5.firmware`, which IS committed.

**What is in `.gitignore`** (group A from the disposition pass, committed
in `c0b2451`):

- `bootloader_STM32G031/m23_capture_*.bin`, `binary_diff_report.txt`
- `firmware/waveforms`, `python_programs/hall_sensor_data_no_avg/`,
  `python_programs/suite_run_*.log`
- `*.py.damaged`
- `autogeneration/plans/`, `bootloader_STM32G031/plans/`, `firmware/plans/`
  (AI working notes)
- `firmware/.claude/` (per-tool local settings)
- `marketting_page/AI_*.md`, `NEXTJS_*.md`, `AUTOGENERATOR_*.md` (AI handoff
  scratch in marketting_page)

Note: `firmware/firmware_releases/*.firmware` is **not** in `.gitignore` —
released firmware binaries are tracked. See
[[firmware-releases-tracked-in-git]] memory note.

---

## Reference documents (all in the repo)

| Doc | Contents |
| :-- | :-- |
| `WORK_CHECKLIST.md` (this file) | Master checklist / resumption guide |
| `CHANGE_ANALYSIS.md` (repo root) | Full inventory of every uncommitted change, M17-risk flagged |
| `python_programs/MICROPYTHON_COMPATIBILITY_DESIGN.md` | Proposed architecture for the cross-platform refactor |
| `python_programs/TEST_INVENTORY.md` | Test-suite landscape + gap analysis |
| `python_programs/TEST_MODERNIZATION_PLAN.md` | Plan to test every important command + modernize the obsolete tests (TODO #5) |

> All five reference docs above are **committed** as of 2026-05-28 (commit
> `4ffaab1`). The host-only test suite, MicroPython platform-layer files,
> and `firmware/Src/current_control.c/.h` are also committed.

---

## Environment / hardware notes

- Test motor: an **M17** (a QC reject — stiff shaft), now running firmware
  **0.15.0.0** (the just-released version — `test_firmware_upgrade.py` leaves
  the bench on the latest), **calibrated** (an uncalibrated motor makes
  closed-loop control malfunction; that caused earlier false homing crashes).
- Assigned alias: **`X`** (byte value 88). It had no alias (255) originally.
  Original test motor unique ID: `99856389A2B46555`.
- **No external encoder available** (one was available previously). Tests that
  cross-check against an external encoder must skip that check cleanly until
  encoder hardware is back — see `TEST_MODERNIZATION_PLAN.md`.
- Serial port observed: `/dev/cu.usbserial-120` (CPython direct path; may
  change between sessions, use `-P` to pick interactively). For the
  MicroPython bridge, the ESP32-S3 enumerates as `/dev/cu.usbmodem1201`.
- If the motor does not respond at alias `X`: run
  `python3 test_detect_devices.py -p <port>` to find it, then `set_device_alias`.
- Run the full Python suite (~13:33 wall-clock on the bench M17, 63/63 PASSED
  most recently as of 2026-05-27):
  `cd python_programs && python3 run_all_tests.py -p <port> -a X`
- Hardware-free pre-commit check (~seconds):
  `cd python_programs && python3 run_host_tests.py`
- Firmware build: `cd firmware && make PRODUCT_NAME=M17` (M1/M2 builds fail for
  a pre-existing unrelated reason — missing autogenerated hall-sensor table).

---

## ✅ Completed

- [x] Analyzed all uncommitted changes → `CHANGE_ANALYSIS.md`.
- [x] Fixed `firmware/Makefile` (M1/M2 → STM32G030, others → STM32G031),
      verified the M17 binary is byte-identical both ways. **Still uncommitted**
      (it is part of the change pile in TODO #1 below).
- [x] Wrote `MICROPYTHON_COMPATIBILITY_DESIGN.md` and `TEST_INVENTORY.md`.
- [x] Ran the full hardware test suite — baseline established (14/14).
- [x] Fixed `test_homing.py` — added the simulated hard-stop case (drops motor
      current mid-homing to trigger the firmware crash detection), and corrected
      the `motor_commands.json` "Set maximum motor current" description.
      **Committed & pushed to main as `66414cf`.**
- [x] Built the host-only Python test suite (TODO #2). New files in
      `python_programs/`: `host_test_framework.py` (shared mini-framework +
      `FakeSerial` + `build_response_packet`), `test_host_crc32.py`,
      `test_host_command_encoding.py`, `test_host_response_decoding.py`,
      `test_host_import_smoke.py`, and `run_host_tests.py` (hardware-free
      runner). All 4 tests pass (111 checks). Catalogued in `TEST_SUMMARY.md`
      so `run_all_tests.py` also picks them up (18 servomotor tests now).
      **Uncommitted** — part of the change pile.
- [x] Found & fixed a real bug while building the suite: `M3()` constructed
      with no alias crashed with `TypeError` (`AllMotors.__init__` range-checked
      `alias_or_unique_id` before testing for `None`, even though `None` is a
      supported deferred-alias state). One-line guard added in
      `servomotor/M3.py`. **Uncommitted** — this fix is independent of the
      MicroPython refactor and can be committed on its own.

---

## 📋 To do

### 1. Review & commit/revert the pending change pile — ✅ DONE (2026-05-27)
*See `CHANGE_ANALYSIS.md` for the full inventory. All 7 commit groups landed.*

- [x] **Makefile fix + ADC.c G031 register-define shim** — `63d4de1`
- [x] **M23 firmware feature set** — `a6023cd` (error_text.h, main.c, PWM.c/h,
      current_streaming.c/h, new current_control.c/h, m23 tools)
- [x] **clock_calibration bug fix** — `60f2887` (time_sync uint64→uint32,
      common_source_files + bootloader kept in sync)
- [x] **`motor_control.c` M17-affecting changes** — `ef63e31` (hardware-tested,
      calibration time 3.5×, hall threshold 2000, 1.4-turn data collection)
- [x] **Python bugfixes** — `a8705b9` (detect_and_set_alias, magnetic_disk_machine)
- [x] **Datasheets** — `030d4d4` (M17-34 model, dimension drawings, datasheet v1.8)
- [x] **Marketing** — `d6759da` (features.txt typo fixes)
- [x] **Untracked large dirs** — left untracked by design (STM32G4 drivers,
      Mechanical/, assembly_steps/, firmware_releases/, etc.)
- NOTE: Python library files (M3.py, __init__.py, command_loader.py,
  device_detection.py, communication.py) remain uncommitted — handled by TODO #3.

### 2. Build a host-only Python test suite (the safety net) — ✅ DONE
*Prerequisite for #3. See `TEST_INVENTORY.md`. Done 2026-05-19.*
Every existing test needs a physical motor; nothing caught a pure-software
regression (like the old `to_bytes` bug) without hardware — now it does.

- [x] Command-encoding tests — `test_host_command_encoding.py`: packs every
      parameter data type + full packet framing, asserts exact bytes.
- [x] Response-decoding tests — `test_host_response_decoding.py`: feeds known
      response byte streams, asserts parsed values + error/CRC paths.
- [x] CRC32 test — `test_host_crc32.py`: asserts `calculate_crc32` and the
      pure-Python MicroPython CRC32 match published check vectors.
- [x] Import / smoke test — `test_host_import_smoke.py`: imports the package,
      builds `M3`, checks generated methods + loaded tables.
- [x] Runnable on standard Python (verified) and MicroPython (no argparse, no
      external deps, ASCII-only output). MicroPython run not yet executed —
      do that when an ESP32-S3 / Pico 2 is on the bench.
- [x] `run_host_tests.py` — hardware-free runner for fast pre-commit / CI.

### 3. Implement the MicroPython compatibility refactor (the main goal)
*Do AFTER #2 so the PC build is provably not broken. See
`MICROPYTHON_COMPATIBILITY_DESIGN.md`. PC-side refactor done 2026-05-26;
MicroPython hardware run still pending.*

- [x] Add `int_to_bytes` / `int_from_bytes` / `wrap_text` / `module_path` helpers
      to `platform_utils.py`. **Done 2026-05-26.**
- [x] Add a unified `open_serial_port()` entry point to `serial_abstraction.py`.
      **Done 2026-05-26.** Wraps the existing `serial_functions.open_serial_port`
      on CPython (preserves the saved-device-file + interactive menu logic),
      returns a `StandardSerial` adapter so the rest of the library only
      sees the abstract `SerialPort` interface.
- [x] Create `platform_compat.py` (Enum shim, typing stubs). **Done 2026-05-26.**
- [x] Rewrite `communication.py` as a single version with zero `is_micropython()`
      branches. **Done 2026-05-26.** `communication_micropython.py` is **renamed
      to `communication_micropython.py.bak`** (kept on disk per
      [[dont-delete-prefer-bak]]); no module imports it any longer.
- [x] Clean up `M3.py`, `__init__.py`, `command_loader.py`, `device_detection.py`
      to single-version (move inline branches into the platform layer).
      **Done 2026-05-26.** All four modules now contain zero `is_micropython()`
      checks; `__init__.py` only re-exports the platform-detection symbols.
- [x] Verify: `run_host_tests.py` stays green on standard Python and the
      hardware path still works. **Done 2026-05-26.**
      `run_host_tests.py` 7/7 PASSED (suite has grown since the original
      "4/4"). Hardware spot-checks on bench M17 (alias `X`,
      `/dev/cu.usbserial-120`): `test_ping.py` 723/723 pings PASSED in 3 s;
      `test_get_status.py` PASSED. **Full hardware suite re-run done
      2026-05-26: 63/63 PASSED in 13:34.00 wall-clock (812.91 s of test
      runtime)**, matching the pre-refactor baseline of 13:33. The
      MicroPython refactor introduces zero regressions on the PC path.
- [x] Deploy to MicroPython (ESP32-S3 / Pico 2) and run the MicroPython
      smoke test. **Done 2026-05-27 on ESP32-S3 (MicroPython
      1.28.0 stock build `ESP32_GENERIC_S3-20260406-v1.28.0.bin`,
      8 MB flash). 45 / 53 hardware tests PASSED on bench M17 (alias
      `X`, calibrated, firmware 0.15.0.0).** The ESP32-S3 became the
      RS485 bridge to the motor (UART1, TX=Pin(4), RX=Pin(5), through
      a transceiver), replacing the old USB-RS485 dongle path. See
      [[micropython-deploy-state]] for the on-device layout and host
      tooling.

      **PASSED (45 of 53)** — the entire library (single-version,
      no `is_micropython()` branches outside the platform layer) plus
      `argparse` + `string` + `os.path` + `random` + `collections`
      compat shims drives the real motor end-to-end. Motion tests
      (move_with_velocity, trapezoid_move, homing, go_to_position,
      go_to_closed_loop, start_calibration, set_pid_constants, etc.),
      telemetry tests (get_status, get_position,
      get_comprehensive_position, get_hall_sensor_position, etc.),
      protocol tests (crc32_control, multimove, ping,
      communication_while_high_speed, correct_and_incorrect_addressing,
      detect_devices), and parameter-setter tests all pass.

      **8 remaining failures by root cause:**
      **CPython suite re-run on 2026-05-27 after the MP-compat
      patches landed:** 62 / 63 PASSED in **13:03.92 wall-clock**
      (782.83 s of test runtime). One transient flake:
      `test_fast_short_move_with_velocity.py` FAILED with
      `FatalError(18)` = ERROR_QUEUE_IS_FULL on move 2 — and the same
      test PASSED standalone immediately afterwards (389 moves in
      30.94 s). Not a regression; the test has a known race where the
      "wait for queue space" guard only kicks in after counting 32
      queued moves, but a tiny early-bench latency can fill the
      device-side queue faster. Tracked as a future test-hardening
      item, not a blocker for the refactor.

      * **MicroPython perf gaps (4):** MP runs the pure-Python unit
        conversion and packet encoding ~9× slower than CPython
        (~27 pps vs ~240 pps for ping). Tests with CPython-tuned
        timing tolerances fail not because of bugs but because the
        client is the bottleneck.
        — `test_fast_short_move_with_velocity` ERROR `FatalError(18)`
          (ERROR_QUEUE_IS_FULL — MP can't push commands into the move
          queue fast enough).
        — `test_gradual_speed_up` ERROR same `FatalError(18)` (also
          a move-queue test).
        — `test_time_sync` FAILED — sync drift of 8–11 ms, well above
          the ±5 ms tolerance; the bench M17 is fine, MP is just slow
          at the host-side sync loop.
        — `test_get_current_time` FAILED — 1.3 s gap between
          `get_current_time(time_unit="timesteps")` and
          `get_current_time(time_unit="microseconds")` is mistaken
          for a unit-conversion error; the actual cause is the
          wrapper's JSON-driven conversion code taking that long on
          MP between the two reads.
      * **Library precision bug (1):** `test_get_max_pid_error` FAILED
        because `M3.convert_from_internal()` does `value / 1.0` on the
        INT32_MAX sentinel, returning `2147483600.0` on MP's float
        precision instead of `2147483647`. Fix in `M3.py`:
        short-circuit when `factors[target_unit] == 1.0` to skip the
        float division and preserve the int. Tracked as TODO #9 below.
      * **Test design (1):** `test_get_temperature` ERROR MemoryError.
        154 s thermal soak that accumulates samples in a list; MP's
        ~230 KB heap can't hold a full run. Refactor the test to
        stream / aggregate on the fly. Tracked as TODO #9 below.
      * **Fundamentally non-portable (2):** `test_trigger_framing_error`
        imports `pyserial` directly to inject malformed bytes — the
        platform abstraction doesn't expose that surface, and reaching
        around it on MP would need a separate device-side raw-write
        helper. `test_random_speed_stress` depends on
        `paho.mqtt.client`, `uuid`, and multi-device fleet operation;
        not a single-device-on-MP target.

      **Compat shims required (kept under `python_programs/micropython/`
      once committed):**
      * `argparse.py` — micropython-lib's minimal port + an
        `ArgumentParser.error(msg)` method.
      * `string.py` — `digits`, `ascii_letters`, `printable`, etc.
      * `ospath_shim.py` — `join`, `dirname`, `basename`, `abspath`
        (cannot be installed as `os.path` because MicroPython's `os`
        module is frozen; the tests that needed `os.path` were
        instead edited to use portable string manipulation —
        `test_get_product_specs.py` and `test_multimove.py`).
      * `mp_random_shim.py` — re-exports `random`, adds `choices(…)`
        and a chunked `getrandbits(n)` that handles n > 32.
      * `mp_collections_shim.py` — re-exports `collections`, adds
        `defaultdict`.
      * `mp_runner.py` — on-device runner with `sys.argv` injection
        per test, `builtins.exit = sys.exit` shim, `gc.collect()`
        between tests, `/results.txt` persistence so chunked runs
        resume.
      * `mp_sweep.sh` — host-side driver that machine-resets between
        chunks to dodge heap-fragmentation `MemoryError`s, and
        re-runs the on-device runner until `/results.txt` covers the
        whole test list.

      **Source patches that survived to disk (uncommitted):**
      * `test_get_product_specs.py` — replaced
        `os.path.dirname/abspath/join` with portable string
        manipulation; works on CPython on every OS and MP.
      * `test_multimove.py` — same `os.path` fix.
      * `test_capture_hall_sensor_data.py` — replaced
        `{**BASE_VALID, "x": y}` dict-unpack literals (MP 1.28 doesn't
        accept that syntax) with a `_with(**overrides)` helper.
      * `test_get_current_time.py` and `test_reset_time.py` — replaced
        `time.monotonic()` with a `_monotonic_us()` helper that uses
        `time.ticks_us()` on MP and `time.monotonic()` on CPython.
      * `test_gradual_speed_up.py` — removed unused `import zlib`
        (MP 1.28 has no `zlib`; `deflate` is the replacement, but it
        wasn't actually used here).

### 4. Minor cleanups
- [x] Add `test_get_product_info.py` to `TEST_SUMMARY.md` — done as part of
      TODO #5 catalogue-and-triage (commit `3a44cee`).
- [ ] ~~Remove `fw0.14.3.0` verification rebuilds from `firmware/firmware_releases/`.~~
      Obsolete — per [[firmware-releases-tracked-in-git]], `.firmware` files are
      release artifacts; we keep them rather than deleting them.
- [ ] `Arduino_library/` has heavy `.bak` / stray-binary clutter (low priority;
      Arduino library work is not planned soon).

### 5. Modernize tests & cover every motor command
*Not started. Review decisions incorporated 2026-05-19. See
`python_programs/TEST_MODERNIZATION_PLAN.md` for the full per-command coverage
matrix, functional-test list, and obsolete-test disposition.*
Decision: **one `test_<command>.py` per command (48 total) plus functional
tests** spanning multiple commands. 12 per-command tests already exist (9
catalogued + 3 uncatalogued); 36 are new. (12 + 36 = 48.)

- [x] Catalogue & triage the 5 uncatalogued `test_*.py` files into
      `TEST_SUMMARY.md`; confirm which already pass. **Done 2026-05-20 —
      verified 22/22 green via `python3 run_all_tests.py -p /dev/cu.usbserial-120
      -a X` against the M17 test motor.** Added 5 rows: 4 modern hardware
      tests (`test_correct_and_incorrect_addressing.py`, `test_detect_devices.py`,
      `test_get_product_info.py`, `test_set_device_alias.py`) as `servomotor`/
      `No`/`Yes`, plus `test_homing_micropython.py` with module label
      `servomotor (MicroPython)` so the PC runner skips it (decision #4).
      The suite went from 14 → 22 catalogued passing tests (14 hardware + 4
      host before; 18 hardware + 4 host now).

      What the plan's "ready as-is" entries had missed and what I had to fix:

      1. **Runner compatibility (-a):** the 4 hardware tests above did not
         accept `-a/--alias`, but `run_all_tests.py` always passes `-a X`,
         so argparse would have rejected them. Added an *honored* (not
         ignored) `-a` to each, per the alias-meaning rule
         [[tests-use-supplied-alias-meaningfully]]:
         • `test_detect_devices.py` — accepts `-a`; if supplied and != 255,
           asserts a device with that alias is in the detected list. Also
           prints `PASSED` and exits non-zero when no devices found.
         • `test_get_product_info.py` — uses `-a` (default `X`) as the
           expected alias; exercises **both** alias-addressing and
           unique-ID-addressing paths of cmd 22 and asserts identical
           results. Bootstraps 255→target if needed; fails loudly on any
           other unexpected alias.
         • `test_set_device_alias.py` — uses `-a` as the "home alias";
           random valid alias is chosen != target; **adds a step (4) at the
           end of every repeat that restores the alias to `-a`**, so the
           test is suite-safe (without this it would leave the device at
           255 and break every later test).
         • `test_correct_and_incorrect_addressing.py` — uses `-a` as the
           "correct" alias instead of a random pick; new `ensure_alias_is()`
           helper bootstraps 255→target; leaves the device at target alias.

      2. **Two pre-existing bugs in `test_correct_and_incorrect_addressing.py`**
         (surfaced when the test was first run against the M17 motor — they
         had presumably been "passing" only on an M3):
         • Hardcoded `assert product_code.strip() == "M3"` — replaced with
           a model-agnostic "starts with `M`" check.
         • Timeout detection used `"timeout" in str(e).lower()`, but
           `servomotor.communication.TimeoutError` is sometimes raised with
           no message (`raise TimeoutError` at communication.py:365). Now
           also matches the exception class name.

      3. **`PASSED`-last-line ordering in `test_detect_devices.py` and
         `test_get_product_info.py`.** Both had their `print("PASSED")` inside
         the `try` block whose `finally` calls `servomotor.close_serial_port()`
         — which itself prints "Closed the serial port", making *that* the
         actual last line of stdout. `run_all_tests.py` grades on the last
         line, so it scored both as "FAILED (Missing PASSED message)" even
         though they exited 0. Restructured to print the PASSED/FAILED
         verdict *after* the `finally` runs. (Caught only because I actually
         ran the suite — surfaced lesson: [[verify-before-declaring-done]].)
- [x] Modernize the obsolete pile into the functional + per-command tests it
      maps to (closed loop, enable/disable, time sync, position telemetry,
      safety limits, LEDs — see the disposition table in the plan). **Done** —
      `test_closed_loop_motion.py`, `test_enable_disable_reliability.py`,
      `test_time_sync.py`, `test_position_telemetry.py`, `test_set_safety_limits.py`,
      `test_leds.py` (manual layer) all exist, catalogued, and pass.
- [x] Create the remaining per-command tests, tier by tier — T1, then T2,
      then T3, then the two specials. **T1 done (2026-05-21/22), T2 done
      (2026-05-25), T3 done (2026-05-25), specials done (2026-05-26).**
      15 new T1 tests written and each verified individually against the
      bench M17 (alias `X`): cmds 0, 1, 2, 3, 5, 11, 12, 13, 16, 17, 27, 28,
      29, 34, 46. **10 new T2 tests written 2026-05-25**, each verified
      individually against the same bench M17: cmds 8 Reset time, 9 Get
      current time, 15 Get hall sensor position, 18 Get product specs, 24 Get
      product description, 38 Get supply voltage, 39 Get max PID error,
      43 Set PID constants, 44 Set max allowable position deviation,
      47 Get communication statistics. **7 new T3 tests written 2026-05-25**,
      each verified individually against the same bench M17: cmds 7 Capture
      hall sensor data, 32 Control hall sensor statistics, 33 Get hall sensor
      statistics, 35 Read multipurpose buffer, 40 Vibrate (human-confirm),
      41 Identify (human-confirm), 45 Get debug values. **46 of 48
      per-command tests now exist and pass; 2 remain (firmware upgrade +
      calibration).**
      Notes (T3-specific):
      • `test_capture_hall_sensor_data.py` (cmd 7) drives ONLY the
        parameter-validation surface — the JSON description warns "work
        in progress; don't send this command" and the streaming success
        path would need wrapper support for variable-length /
        multi-segment replies. Five invalid inputs each raise
        `FatalError(49)` (ERROR_CAPTURE_BAD_PARAMETERS); each is
        cleared with `system_reset`.
      • `test_read_multipurpose_buffer.py` (cmd 35) populates with
        `test_mode(3)` (PID debug, type 4) inside closed loop and
        cleans up via `system_reset`. Two alternative populators were
        considered and rejected: `test_mode(4)` (GC6609 register dump
        — auto-clears `test_mode` cleanly but the bit-bang stalls on
        the bench M17), and `start_calibration` (reliable but far too
        slow). We deliberately drop the "second read times out"
        assertion because PID rewrites the buffer on its very next
        ~32 µs iteration after each clear.
      • `test_vibrate.py` (cmd 40) — the firmware vibrate function is
        `#ifdef PRODUCT_NAME_M1`; on M17 the function body is empty,
        but cmd-40 dispatch still returns success, so the comms layer
        works on every variant. The visual layer is gated on TTY +
        product == M1.
      • `test_identify.py` (cmd 41) — pings mid-blink and post-blink
        to prove RS485 stays responsive while SysTick drives the
        ~2.4 s LED animation.
      • `test_get_debug_values.py` (cmd 45) — only asserts ranges
        and quiescent values; deliberately does NOT assert
        monotonicity on the profiler "max" fields or hall-position
        deltas because those are *read-with-reset* in firmware
        (profiler.c:44-47, motor_control.c:1572-1584).
      • The `test_mode(0)` firmware lockup was discovered here while
        writing the cmd-35 test (first try cleared test_mode via
        `test_mode(0)` and locked the firmware — recovered with a
        power-cycle). All T3 tests now use `system_reset()` to clean
        up test mode. See the
        [[led-test-mode-is-stuck-forever]] memory note and the new
        bullet in TODO #8 below.

      Notes (T1/T2 carried forward):
      `test_multimove.py` and `test_crc32_control.py` drive the bus
      via `servomotor.communication.execute_command` (the high-level M3
      wrapper cannot do multimove's mixed unit conversion, and CRC control
      needs per-call CRC flags — both true on the committed library too,
      not a regression). Setter tests assert real rejection paths:
      vel/accel/current over-limits return ERROR_VEL_TOO_HIGH (16) /
      ERROR_ACCEL_TOO_HIGH (15) / ERROR_MAX_PWM_VOLTAGE_TOO_HIGH (23). T2
      additions also assert real behavior: cmd 9 verified against wall
      clock in raw microseconds (the wrapper's `seconds` unit applies
      the timesteps-based factor 31250 — see the long comment in
      `test_get_current_time.py` — so the test uses
      `time_unit="timesteps"` as an identity passthrough); cmd 39 verified
      via the [INT32_MAX, INT32_MIN] sentinel pre-closed-loop and a real
      bounded range post-move with read-then-reset; cmd 43 verified
      differentially (good defaults land on target, weak gains produce
      much larger PID error); cmd 44 verified via
      ERROR_POSITION_DEVIATION_TOO_LARGE (45) tripping on a tight limit
      and again with a negative input (firmware's `llabs()` fold).
      **Specials done 2026-05-26:**
      `test_firmware_upgrade.py` (cmd 23) drives the page-by-page
      transfer inline via `servomotor.execute_command(...,
      crc32_enabled=True)`, alias-addressed so each page is ACK'd. New
      protocol only. Three reflashes (latest → kept-older → latest)
      with `Get firmware version` (cmd 25) check after each. Wall-clock
      ~17 s on the bench M17. `test_start_calibration.py` (cmd 6)
      polls `get_status` for the calibrating bit to clear, then
      kicks the device out of the post-reboot bootloader window with
      `system_reset` + ~2 s wait (calibration auto-reboots — see
      [[calibration-auto-reboots]]), then verifies usability via
      `go_to_closed_loop` and a small closed-loop move on target.
      Wall-clock ~29 s on the bench M17 against firmware 0.15.0.0
      (the earlier "0.15.0.0 calibration regression" sighting was a
      bench false alarm caused by polling inside the bootloader's
      post-reset window).
- [x] Build `test_firmware_upgrade.py` (command 23) — done 2026-05-26.
      See the bullet immediately above for what it does.
- [x] Order `TEST_SUMMARY.md` rows so `test_firmware_upgrade.py` runs first,
      then `test_start_calibration.py`, then the rest. Done 2026-05-26:
      the two specials are now rows 1 and 2 of the table body; everything
      else stays alphabetical so all closed-loop / position-dependent
      tests (test_closed_loop_motion, test_go_to_closed_loop,
      test_position_telemetry, test_homing, etc.) run after them.
- [ ] Defer multi-device tests (multi-device time sync, multi-motor stress)
      until two motors are connected and the single-motor suite is green.

### 6. Verify the Python library + tests run on Windows and Linux
*Not started. Development has been on macOS only.*
The `servomotor` library and the test programs must work on the three major
desktop OSes, not just macOS.

- [ ] Run `run_host_tests.py` (no hardware needed) on Windows and on Linux —
      catch path/encoding/line-ending issues cheaply first.
- [ ] Verify serial-port enumeration and `open_serial_port()` on Windows
      (`COMx` ports) and Linux (`/dev/ttyUSB*`, `/dev/ttyACM*`).
- [ ] Run the full hardware suite (`run_all_tests.py`) against a real motor
      from a Windows host and a Linux host.
- [ ] Fix any OS-specific assumptions found (hardcoded `/dev/tty*` paths,
      POSIX-only calls, terminal/ANSI handling, file-path separators).
- [ ] Note: all new/modernized tests from TODO #5 must be OS-agnostic by
      construction (see the conventions in `TEST_MODERNIZATION_PLAN.md`).

### 7. Make the high-level M3 wrapper able to drive multimove + crc32_control
*DEFERRED — do this LATER, only after the MicroPython refactor (TODO #3) is
working. Not on the critical path; do NOT pick this up while finishing TODO #5.*

Two commands cannot currently be issued through the high-level `servomotor.M3`
methods and must be driven via `servomotor.communication.execute_command`
instead (working examples are in `test_multimove.py` / `test_crc32_control.py`):

- **Multimove (cmd 29):** `moveList` uses unit type
  `mixed_acceleration_velocity_time`; the M3 generic conversion path has no
  `_mixed_acceleration_velocity_time_unit` attribute, so `M3.multimove(...)`
  raises `AttributeError`. Needs a real handler that converts each
  `[value, duration]` pair per the move-type bit (acceleration vs velocity).
- **CRC32 control (cmd 46):** the M3 methods always append a CRC, so they can't
  express "send this packet without a CRC". The wrapper would need a per-call
  (or stateful) CRC mode that tracks the firmware's current CRC state.

⚠️ Both limitations exist on the **committed** library too (verified against
HEAD) — they are pre-existing, NOT a regression from the MicroPython refactor.
Confirm the fix keeps the host suite green (`run_host_tests.py`) and re-run the
two affected tests; ideally also simplify those tests to use the high-level
methods once the wrapper supports them.

### 9. Address MicroPython-specific issues surfaced by the 2026-05-27 sweep
*Discovered during the 53-test MicroPython sweep. None blocks
day-to-day CPython work; these are quality-of-life fixes that lift
the MP pass rate.*

- [ ] **`M3.convert_from_internal()` loses int precision when the
      conversion factor is 1.0.** `value / 1.0` produces a float, and
      MicroPython's float type isn't wide enough to represent
      INT32_MAX (2147483647) exactly — it comes back as
      2147483600.0. `test_get_max_pid_error` FAILS on MP because of
      this. Fix in `python_programs/servomotor/M3.py`: short-circuit
      when the conversion factor is exactly 1.0, return the original
      int unchanged. Also affects any other read of a sentinel-shaped
      32-bit value through a passthrough unit. Re-run
      `test_get_max_pid_error.py` on the ESP32-S3 after the fix.

- [ ] **`test_get_temperature.py` accumulates samples in a list across
      a 154 s thermal soak — too much for the ESP32-S3's ~230 KB
      heap, even on a fresh boot.** Refactor the test to compute
      running statistics (min/max/mean) on the fly instead of
      retaining every sample. CPython will still pass; MP will then
      also pass.

- [ ] **Optional: improve `test_time_sync`, `test_get_current_time`,
      `test_fast_short_move_with_velocity`, `test_gradual_speed_up`
      to tolerate the MicroPython speed gap (~9× slower client).** Two
      options: (a) widen the timing tolerances on MP only via a
      `servomotor.is_micropython()` branch in the test, or (b) accept
      these as known-failing on MP since they're really stress tests
      of host-side throughput. (b) is fine for now — they all pass on
      CPython and the bench has a CPython baseline.

- [ ] **Optional: install the compat shims under
      `python_programs/micropython/` and commit them.** Today they
      live in `/tmp` (preserved on this host but not in the repo).
      They are: `argparse.py`, `string.py`, `ospath_shim.py`,
      `mp_random_shim.py`, `mp_collections_shim.py`, `mp_runner.py`,
      `mp_sweep.sh`. Without them committed, the next person to
      deploy to a fresh ESP32-S3 has to rebuild them.

### 8. Address gotchas surfaced while writing the T2 tests
*Discovered 2026-05-25 while writing the T2 per-command tests. Not blocking
— both have clean, isolated work-arounds in the T2 test files — but the
owner does not want gotchas to linger, so fix them after the more pressing
items (#1, #3, #6) are done.*

- [ ] **`servomotor.M3` time-unit conversion is wrong for the
      microsecond-internal commands.** The JSON spec declares cmd 9
      (`Get current time`) and cmd 10 (`Time sync`) with
      `InternalUnit: microseconds`, but the conversion-factor table in
      `servomotor/unit_conversions_*.json` is sized for
      `internal = timesteps` (seconds=31250, microseconds=0.03125). The
      wrapper does `value / factors[target_unit]`, which gives correct
      numbers when the raw value is in timesteps (e.g. cmd 19 time
      inputs) but is wrong by a factor of ~32 (1 timestep = 32 µs on
      M17) when the raw value is already in microseconds. So
      `set_time_unit("seconds"); get_current_time()` reads ~32× too
      large, and `set_time_unit("microseconds")` reads ~32× too large
      as well. `test_get_current_time.py` and `test_reset_time.py`
      work around this with `time_unit="timesteps"` (factor 1.0 →
      identity passthrough) plus a long comment block; see those
      files. Fix options:
      (a) split the conversion table by `InternalUnit` so
      `time_timesteps` and `time_microseconds` have separate factor
      sets, **or** (b) pick a single canonical internal unit for time
      and align both the JSON spec and the wrapper. Whichever path is
      chosen, drop the `time_unit="timesteps"` workaround in the two
      tests above and use the natural unit. Re-run the host suite
      (`run_host_tests.py`) — `test_host_command_encoding.py` exercises
      time inputs and must stay green.

- [ ] **`test_mode(0)` locks up the motor — same `set_led_test_mode`
      freeze as 10..13, but worse because it happens before the reply
      is sent.** Found 2026-05-25 while writing
      `test_read_multipurpose_buffer.py`: the cmd-36 handler in
      `firmware/Src/main.c:880-882` routes `test_mode == 0` *inline*
      into `set_led_test_mode(0)` (defined at main.c:178-189), which
      `__disable_irq();` + `while(1);`s **before** the no-error reply
      is transmitted. So `test_mode(0)` times out AND every subsequent
      RS485 command also times out; only a hardware power-cycle
      recovers. (10..13 do the same `while(1);`, but at least *after*
      replying.) Bench reproduction: from a freshly-reset M17, a
      single `motor.test_mode(0)` was enough — see
      `python_programs/test_read_multipurpose_buffer.py` for the
      avoidance pattern and the [[led-test-mode-is-stuck-forever]]
      memory note for the full repro. Fix: route `test_mode == 0` to a real
      "disable test mode" path (just `set_motor_test_mode(0)`, no
      `set_led_test_mode` call), and make `set_led_test_mode` itself
      periodically poll for an end-test-mode command instead of
      `__disable_irq();` + `while(1);`. Until that fix lands, no
      auto-suite test may use `test_mode(0)` to clear a test mode —
      use `system_reset()` instead. `test_read_multipurpose_buffer.py`
      now does that, and the avoidance is documented in its module
      docstring.

- [ ] **MOSFET-enable commutation-alignment transient is observable
      and trips two different commands if you act too soon after
      `enable_mosfets`.** When MOSFETs energise the firmware steps the
      rotor into a known commutation angle; the rotor may need tens to
      hundreds of milliseconds to physically settle, and during that
      window the hall accumulator advances by up to ~13 k counts
      (~0.4 % of a rotation). Two symptoms surfaced while writing the
      T2 tests:
      • Cmd 44 — if a tight `max_allowable_position_deviation` is set
        *before* the enable, `ERROR_POSITION_DEVIATION_TOO_LARGE` (45)
        fires on the very next control-loop tick.
        `test_set_max_allowable_position_deviation.py` works around
        this by ordering the calls as enable → zero → tighten → move
        (instead of the more natural tighten → enable → zero → move).
      • Cmd 13 + cmd 15 — if `zero_position` is called *before* the
        rotor has finished settling, the hall accumulator (which
        `zero_position` snaps to 0 in software) drifts back to a
        non-zero value over the settle window, so a subsequent
        `get_hall_sensor_position` read at "zero" returns a few
        thousand counts.  `test_get_hall_sensor_position.py` works
        around this by ordering the calls as enable → settle → zero
        → read (instead of enable → zero → settle).
      Fix options:
      (a) suppress the deviation check **and** defer the
      hall-position accumulator settle for a short window after
      MOSFETs enable, **or** (b) document the required call order
      ("enable, then wait for the rotor to settle, then do anything
      else") in the firmware Description for the affected commands in
      `python_programs/servomotor/motor_commands.json` so users find
      the gotcha before tripping it. After the firmware fix, drop the
      reorder workarounds in the two tests above.

---

## Suggested order

1. ~~**#2 — host-only test suite**~~ ✅ done 2026-05-19.
2. ~~**#5 — modernize tests & full command coverage**~~ ✅ done 2026-05-26
   (48/48 per-command tests; full suite 63/63 PASSED in 13:33 on the bench M17).
3. **#3 — MicroPython refactor** — PC-side complete 2026-05-26 (host 7/7,
   hardware ping/get_status green on bench M17). Still owed: full hardware
   suite re-run (~13:33) to certify, and MicroPython hardware deploy when a
   board is available.
4. **#6 — cross-OS verification** — defer until after #3's MicroPython
   hardware run so the single-version library is what gets verified across
   Windows / Linux.
5. ~~**#1 — change-pile review**~~ ✅ done 2026-05-27 (7 commits, `63d4de1`–`d6759da`).
6. **#7 — M3 wrapper multimove/crc32 support** — after #3. Removes the
   `execute_command` workaround in `test_multimove.py` / `test_crc32_control.py`.
7. **#8 — clear the T2-test gotchas** — alongside or right after #7 (both are
   wrapper / firmware-UX polish). Removes the only two work-arounds in the T2
   tests.
