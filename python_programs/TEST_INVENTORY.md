# Test Inventory & Assessment

Snapshot of the test programs that exist today for exercising the motor, plus a
gap analysis. Purpose: decide what test coverage is needed before refactoring
the Python library for MicroPython compatibility.

---

## A. Python library tests (`python_programs/`)

### Runner
`run_all_tests.py` is the harness:
1. Parses `TEST_SUMMARY.md`, selects rows where **Module = `servomotor`** and
   **Obsolete = `No`**.
2. Explicitly skips `test_servomotor_module.py` and
   `test_servomotor_module_get_command_id.py`.
3. Runs each remaining test as a subprocess: `python3 <test> -p PORT -a ALIAS`.
4. Pass criteria: exit code 0 **and** the last stdout line contains `PASSED`.
5. Per-test timeout: 240 s. Exits non-zero if any test fails.

Invocation: `python3 run_all_tests.py -p /dev/ttyUSB0 -a X`
(or `-P` to pick the port interactively).

### Catalogue (`TEST_SUMMARY.md`): 32 test files total
- 16 use the modern `servomotor` library — 15 marked non-obsolete, 1 obsolete.
- 9 use the deprecated `communication` module — all obsolete.
- 1 uses raw `serial`.
- 6 are `N/A` (utilities / plotting / no motor).

### Auto-run suite (14 tests — ALL require connected hardware)
1. `test_ping.py` — ping round-trip, payload echo check.
2. `test_get_firmware_version.py` — firmware-version command, main + bootloader.
3. `test_get_comprehensive_position.py` — motor/hall/encoder position.
4. `test_get_temperature.py` — temperature command under load + thermal limit.
5. `test_go_to_position.py` — position moves in multiple units.
6. `test_move_with_velocity.py` — velocity moves in multiple units.
7. `test_move_with_acceleration.py` — acceleration moves in multiple units.
8. `test_homing.py` — homing in multiple units/directions.
9. `test_fast_short_move_with_velocity.py` — many short queued moves.
10. `test_gradual_speed_up.py` — ramp velocity to the speed limit.
11. `test_communication_while_high_speed.py` — ping stress during a fast profile.
12. `test_random_speed_stress.py` — multi-motor random-speed stress test.
13. `test_test_mode.py` — test-mode command, invalid/fatal-error scenarios.
14. `test_trigger_framing_error.py` — RS485 framing-error detect/recover.

NOTE: `test_get_product_info.py` exists in the folder but is **not catalogued in
`TEST_SUMMARY.md`**, so the runner does not pick it up. Add a row for it (and
audit for other uncatalogued tests) so coverage is not silently lost.

### Obsolete tests (use the old `communication` module — 9)
`test_all_leds_on`, `test_enable_disable`, `test_iterate_reset_and_enable`,
`test_go_to_closed_loop_mode` (+ 3 closed-loop variants),
`test_time_sync`, `test_time_sync_multiple_devices`,
`test_motor_position_vs_hall_sensor_position_vs_external_encoder_position`.
These still contain useful scenarios (LEDs, enable/disable, closed-loop entry,
time sync) but are not run and target the deprecated module.

### Non-hardware tests (all currently marked obsolete, none auto-run)
`test_servomotor_module_get_command_id.py` (unit test of `get_command_id`),
`test_terminal_formatting.py`, `test_getch.py`, `test_json_read.py`.

### KEY GAP — relevant to the MicroPython refactor
**There is no host-only test in the auto-run suite.** Every auto-run test needs
a physical motor on a serial port. Consequences:
- The `to_bytes` regression that broke the PC build would NOT be caught without
  hardware — there is no test that packs a command and checks the bytes.
- CI / quick pre-commit checks are impossible without a motor connected.

Recommended additions before refactoring (host-only, no motor needed):
- **Command-encoding tests**: for representative commands (and every parameter
  data type — u8/u16/u32/i32/u64, alias, unique-id, lists), build the packet and
  assert the exact bytes. Catches `to_bytes`/`from_bytes` and unit-conversion
  regressions directly.
- **Response-decoding tests**: feed known response byte streams, assert the
  parsed values (signed/unsigned across all widths).
- **CRC32 test**: assert `calculate_crc32` matches known vectors (also validates
  the pure-Python MicroPython implementation against `zlib`).
- **Import/smoke test**: `import servomotor; from servomotor import M3` plus
  building an `M3` instance — catches import-graph breakage.
These would run on both standard Python and MicroPython and form the safety net
the refactor verification plan depends on.

### Coverage assessment
Good hardware coverage of motion (position/velocity/acceleration/homing),
queuing, telemetry (position/temperature/firmware/product info), and error paths
(test mode, framing errors). Thin or missing: closed-loop mode (only in obsolete
tests), enable/disable and reset cycling (obsolete), time sync (obsolete),
device detection / alias assignment, and — critically — anything that runs
without hardware.

---

## B. Arduino library tests (`Arduino_library/`) — for awareness, not now

### How it works
- `build_tests.sh` compiles each `test_*.cpp` natively with `g++ -std=c++17`,
  linking `ArduinoEmulator.cpp` (emulates the Arduino API on the host) plus
  `Servomotor.cpp`, `Communication.cpp`, `DataTypes.cpp`,
  `AutoGeneratedUnitConversions.cpp`, `test_framework.cpp`.
- `run_all_tests.sh` builds, then runs each compiled `test_*` executable, and
  repeats the run in **two addressing modes** (alias and 16-hex-digit unique ID).
- Custom mini-framework in `test_framework.h/.cpp` (no external dependency).

### Test programs (8)
`test_emergency_stop`, `test_enable_disable_mosfets`,
`test_get_comprehensive_position`, `test_get_temperature`,
`test_move_with_acceleration`, `test_move_with_velocity`, `test_multi_move`,
`test_unit_conversions`.

Plus examples: `example_one_move`, `example_one_move_two_motors`,
`example_multi_move`, `example_trapezoid_move`.

### Notable
- `test_unit_conversions` builds **without** `-DREQUIRE_SERIAL_PORT` and runs
  with no hardware — this is exactly the kind of host-only test the Python suite
  lacks. Worth mirroring on the Python side.
- All other Arduino tests need a serial port + device ID.
- The directory is cluttered with many timestamped `.bak` files
  (`Servomotor.cpp.*.bak`, `Commands.h.*.bak`, etc.) and stray build artifacts
  (`*.o`, compiled `test_*` binaries) — a cleanup candidate, separate from
  testing work.

### Status
Per the project owner: the Arduino library is **not** being worked on soon. No
action proposed here — this section is informational only.

---

## Summary recommendation

Before implementing the MicroPython refactor:
1. Confirm the 14–15 hardware tests currently pass on standard Python against a
   real motor — this is the behavioural baseline.
2. Add a host-only test group (encoding / decoding / CRC32 / import smoke test)
   so regressions are caught without hardware and the same tests can later run
   under MicroPython.
3. Optionally modernise the still-useful obsolete tests (closed-loop,
   enable/disable, time sync) onto the `servomotor` module so they re-enter the
   auto-run suite.

Only once the suite is green should the four-module refactor in
`MICROPYTHON_COMPATIBILITY_DESIGN.md` proceed.
