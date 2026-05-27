# Repository Change Analysis

Analysis of all uncommitted changes, produced so each change can be reviewed
and selectively committed or reverted. M17 is in production — changes touching
M17 are flagged. M1/M2/M3/M4/M23 are not in production.

---

## 1. Makefile fix (DONE — authorized change)

**What was wrong:** The uncommitted Makefile only assigned `STM32G031xx` to M23;
everything else (including M17) got `STM32G030xx`.

**Fixed to:** M1 and M2 -> `STM32G030xx`; all other products (M3, M4, M17, M23)
-> `STM32G031xx`, using `$(filter ...)`.

**M17 safety — verified:** Built M17 from the current working tree twice — once
forced to `STM32G030xx`, once to `STM32G031xx` — resulting `.bin` files are
byte-for-byte identical. The chip define only selects the CMSIS header; the G031
header is a strict superset (same peripheral register layouts, same IRQ numbers
for shared peripherals — confirmed by diffing the `IRQn_Type` enums). Safe for M17.

**Coupling:** M17/M3/M4 will NOT compile under `STM32G031xx` unless the `ADC.c`
register-define compatibility shim (item 2c) is also committed. The G031 header
renamed `ADC_TR2_LT2_Pos` -> `ADC_AWD2TR_LT2_Pos`. The Makefile fix cannot be
committed alone — it needs at minimum that ADC.c shim.

---

## 2. Firmware changes

### 2a. main.c — touches M17
- Version bump `BUGFIX 2 -> 3` (firmware 0.14.2 -> 0.14.3). Affects all products.
- `#include "current_control.h"` and `current_control_init()` — both inside
  `#ifdef PRODUCT_NAME_M23`, so no effect on M17.

### 2b. error_text.h — shared, low risk
- Adds `ERROR_STREAMING_OVERFLOW = 54` + text string. Purely additive; doesn't
  alter existing error codes. Needed by the M23 streaming change (2f). Safe for M17.

### 2c. ADC.c / ADC.h — touches M17 (analyzed inert)
- G031 register-define compat shim (`#ifndef ADC_TR2_LT2_Pos ...`). Required for
  the Makefile fix to compile M17/M3/M4. Compiles to identical code under G030.
- M17 analog-watchdog setup removed: `AWD1EN`/`TR2`/`TR3`/`AWD2CR`/`AWD3CR` writes
  now wrapped in `#if M1||M2`. M17 never enabled `ADC1_IRQn` and the polled
  `check_if_ADC_watchdog*` calls in main.c are already `#if M1`/`M1||M2` guarded —
  so this was dead code on M17. Removing it is behaviorally inert for M17 but does
  change the M17 binary.
- `ADC1_IRQHandler` rewritten (EOS LED-toggle + M1/M2 watchdog). `ADC1_IRQn` is
  never enabled in NVIC for any product — dead code everywhere. Inert.
- M23-only: new 8-channel ADC sequence, TIM1_TRGO2 triggering,
  `get_phase_a/b_current()`. All `#ifdef PRODUCT_NAME_M23`.

### 2d. PWM.c / PWM.h — M23-only functionally
- All functional changes inside `#ifdef PRODUCT_NAME_M23` (OC3REF ADC trigger at
  CCR3=512, center-aligned mode, MMS2 config, 128 MHz TIM1 clock define). The one
  non-guarded edit converts an already-commented-out line into a commented-out
  line inside an `#ifdef` — no effect on M17.

### 2e. motor_control.c — REAL M17 behavior changes — needs hardware testing
- `CALIBRATION_TIME` for M17: `get_update_frequency()*4` -> `*896>>8` (x3.5).
  Comment says experimentally tuned to reduce resonance/vibration. Intentional.
- `HALL_PEAK_FIND_THRESHOLD`: `4000 -> 2000` in the `#else` branch — applies to
  M17 and M23. Changes hall calibration peak detection on M17.
- `CALIBRATION_DATA_COLLECTION_N_TURNS_TIMES_256`: `384 -> 358` (1.5 -> 1.4 turns)
  and `CALIBRATION_DATA_N_ITEMS` formula gains `+ 3`. Unconditional — affects M17
  calibration data buffer/collection.
- `MOTOR_CURRENT_BASELINE_TOLERANCE`: refactored per-product; M17 value stays
  `200` (no behavior change); M23 -> `1970`.
- `#ifdef M23_CURRENT_CONTROL` PI-control blocks: `M23_CURRENT_CONTROL` is
  commented out in current_control.h, so this code is not compiled anywhere.
  Dormant.

### 2f. current_streaming.c / .h — M23-only
- Telemetry frame redesigned: 10-byte sync-word frame -> 9-byte frame
  (`i_a_ref`, `i_a_actual`, `i_b_actual`, `pwm_a`, `uint8 checksum`). Buffer
  overflow now raises `ERROR_STREAMING_OVERFLOW` instead of silently dropping
  frames. All `#ifdef PRODUCT_NAME_M23`.

### 2g. New files current_control.c / current_control.h (untracked) — M23-only
- PI current-control module (all `#ifdef PRODUCT_NAME_M23`). The PI loop is gated
  behind the commented-out `M23_CURRENT_CONTROL`; only `current_control_init()`
  and telemetry-ref getters/setters are live. Compiles & links into M23, dormant.

**Firmware summary:** Makefile + ADC.c shim are safe for M17 (verified). The
`motor_control.c` changes (2e) are the only items that genuinely alter M17
runtime behavior. Everything else is M23-only or dead/inert code.

---

## 3. Python library — MicroPython cross-platform support (one cohesive feature)

Modified: `servomotor/M3.py`, `__init__.py`, `command_loader.py`,
`device_detection.py` — add MicroPython fallbacks (Enum shim, `ujson`, path
handling, typing stubs), version `0.9.1 -> 0.10.0`.

New untracked files this depends on: `servomotor/platform_detect.py`,
`platform_utils.py`, `serial_abstraction.py`, `communication_micropython.py`,
`terminal_format_wrapper.py`, plus `MICROPYTHON_DEPLOYMENT_GUIDE.md`,
`micropython.md`, `cross_platform_architecture.md`, `deploy_to_esp32.sh`,
`deploy_to_raspberry_pi_pico_2.sh`, `example_trapezoid_move_micropython.py`,
`test_homing_micropython.py`, and the `micropython/` dir.

These are interdependent — committing the 4 modified files without the new
`platform_detect.py`/`platform_utils.py`/`serial_abstraction.py` would break
standard-Python imports too (they `import` those at top level).

KNOWN ISSUE (per project owner): the MicroPython version broke the PC version.
`communication.py` was restored to a working PC version; the MicroPython variant
was kept as `communication_micropython.py`. See section 9 for the detailed
analysis and recommendation.

---

## 4. Other Python changes (independent, low risk)
- `detect_and_set_alias_all_devices.py`: bugfix — `use_this_alias_or_device_id`
  -> `use_this_alias_or_unique_id` (correct method name).
- `magnetic_disk_machine/magnetic_disk_machine_V2.py`: uses
  `M3(255).system_reset()` instead of raw `execute_command`; sleep 2.5s -> 1.0s.
- `TEST_SUMMARY.md`: doc update for new test-mode numbering (LED modes 10-13,
  fatal-error trigger offset +14 instead of +12). Matches firmware test-mode
  renumbering already committed in b22512d.
- `tools/m23_telemetry_protocol.py`, `m23_current_viewer.py`, `m23_stream_test.py`:
  switch the PC-side parser from sync-word framing to checksum framing — must be
  committed together with firmware change 2f to stay in sync. Minor stale-naming
  note: the `M23TelemetryFrame` dataclass still uses field names
  `current_a/current_b` rather than `i_a_ref/i_a_actual`.

## 5. Datasheet changes
- `specs.py`, `introduction.txt`, `generate_datasheet.py`, `versions.txt`: add the
  M17-34 model (4th model), corrected power/dimension figures, switch overview
  image `m3_series_overview.jpg -> m17_series_overview.jpg`, add datasheet v1.8.
- `servomotor_datasheet_latest.pdf` (binary): regenerated output of the above.
- Depends on new untracked images: `M17-34/40/48/60_dimensions.{pdf,png}`,
  `m17_series_overview.jpg`, `servomotor_datasheet_v1.8_Apr_13_2026.pdf`.

## 6. Marketing
- `marketting_page/features.txt`: pure typo fixes. Safe.
- Untracked `marketting_page/*.md` (6 files): Next.js integration handoff notes —
  appear to be AI working docs, probably not meant for the repo.

## 7. Untracked — likely should NOT be committed (build artifacts / scratch)
- `firmware/firmware_releases/servomotor_*.firmware` (M17, M23, M3, M4) — outputs.
- `bootloader_STM32G031/m23_capture_*.bin` (7 files), `binary_diff_report.txt`.
- `autogeneration/.../*.py.damaged` (3 files) — corrupted file backups.
- `*/plans/` dirs, `firmware/tools/`, `firmware/waveforms`,
  `python_programs/hall_sensor_data_no_avg/` — dev scratch.
- `CLAUDE.md` — project guidance file (committing it is your call).

## 8. Untracked — large additions needing a deliberate decision
- `Drivers/CMSIS/Device/ST/STM32G4xx/`, `Drivers/STM32G4xx_HAL_Driver/`,
  `bootloader_STM32G431/` — STM32G4 support (a different MCU family).
- `Mechanical/`, `Pictures/applications/`, `assembly_steps/`, `datasheets/`,
  `pricing_calculations/` — new asset directories.

---

## Notes

### M1/M2 builds
M1 and M2 currently fail to build (`SEGMENT_CHANGE_VALUES_TABLE_INITIALIZER
undeclared` in `hall_sensor_calculations.c`) — pre-existing issue (missing
autogenerated table, file unmodified), unrelated to the Makefile fix. The
Makefile correctly assigns them `STM32G030xx`. M17 and M23 both build.

### Build artifacts created during verification
Verification builds wrote into `firmware/firmware_releases/`: overwrote
`servomotor_M17_fw0.14.3.0_scc3_hw1.5.firmware` and created
`servomotor_M23_fw0.14.3.0_scc1_hw0.10.firmware`.

### Caution flags for committing
- (a) The Makefile fix must go with the ADC.c G031 shim.
- (b) `motor_control.c` item 2e changes real M17 calibration behavior — should be
  hardware-tested before shipping.
- (c) The MicroPython modified files and the telemetry-protocol files each must be
  committed as a group with their dependencies.

---

## 9. Python cross-platform: communication.py — detailed analysis

### Current on-disk state
- `communication.py` — committed; the restored OLD version that works on PC.
- `communication_micropython.py` — untracked; the AI's rewrite that runs on
  MicroPython but BREAKS on PC. Created by renaming the AI's `communication.py`.
- `__init__.py`, `M3.py`, `command_loader.py`, `device_detection.py` — still the
  AI's MicroPython-era versions (modified), with inline `if is_micropython()`
  branches.
- `platform_detect.py`, `platform_utils.py`, `serial_abstraction.py`,
  `terminal_format_wrapper.py` — new helper files; reviewed, PC-safe.

### Why communication_micropython.py breaks on PC — ROOT CAUSE
The file calls `int.to_bytes(length, "little", signed)` with the `signed`
argument passed POSITIONALLY (e.g. `value.to_bytes(4, "little", is_signed)`).

- CPython signature: `int.to_bytes(length=1, byteorder='big', *, signed=False)`
  — `signed` is keyword-only. Positional call raises:
  `TypeError: to_bytes() takes at most 2 positional arguments (3 given)`
  (confirmed by running it on this machine).
- Every command that encodes a typed/signed parameter hits this at runtime, so
  the motor cannot be commanded from a PC.

Everything else in `communication_micropython.py` is already correctly guarded
with `is_standard_python()` (conditional imports of `os`, `serial_functions`,
`textwrap`, `to_bytes`), so those would not break PC by themselves — they are
just verbose. The `to_bytes` signature is the only true PC breakage.

### The two files are ~95% identical
`diff communication.py communication_micropython.py` shows the differences are
confined to: (1) the import block, (2) `calculate_crc32` (3 lines),
(3) serial-port opening (~7 lines), (4) `get_terminal_columns` (1 line),
(5) ~10 scattered `to_bytes`/`from_bytes` calls. The remaining ~840 lines of
protocol logic are identical.

### Recommendation: ONE communication.py, no platform branches inside it
Keeping two ~900-line near-duplicate files means every protocol fix must be made
twice and they WILL drift. Instead, unify into a single `communication.py` that
contains ZERO `if is_micropython()` branches and only calls small helpers:

1. `to_bytes`/`from_bytes` -> add `int_to_bytes(value, length, signed=False)` and
   `int_from_bytes(data, signed=False)` to `platform_utils.py`. Implement with a
   form valid on BOTH platforms (mask negatives, then `.to_bytes(length,"little")`
   with positional byteorder — works on CPython and MicroPython). No branching.
2. `calculate_crc32` -> already in `platform_utils.py`; just import it.
3. serial-port open -> single entry point in `serial_abstraction.py` that does
   the platform branch + PC port-selection internally.
4. `get_terminal_columns` -> already in `platform_utils.py`; just import it.
5. `textwrap` -> small `wrap_text()` helper in `platform_utils.py`.
6. terminal formatting -> already abstracted by `terminal_format_wrapper.py`.

Then DELETE `communication_micropython.py`.

The only files that legitimately contain platform branches: `platform_detect.py`,
`platform_utils.py`, `serial_abstraction.py`, `terminal_format_wrapper.py` —
small and single-purpose. This matches the "platform-specific stuff in separate
files, no scattered #ifdef-style code" requirement.

The same cleanup should be applied to `M3.py`, `__init__.py`, `command_loader.py`:
move their inline `if is_micropython()` blocks (Enum shim, JSON path handling)
into helper files so those modules also become single-version.
