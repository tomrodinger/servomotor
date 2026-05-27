# MicroPython Compatibility — Proposed Architecture

Status: **IMPLEMENTED 2026-05-26 / 27.** All "Proposed structure" items below
landed. The refactor passes CPython 62/63 on bench M17 (the one fail is a
confirmed pre-existing flake in `test_fast_short_move_with_velocity.py`) and
MicroPython 45/53 on the ESP32-S3 bridge (the 8 remaining failures all have
non-refactor root causes — see TODO #3 in `WORK_CHECKLIST.md`). One
library bug surfaced during the MP sweep is tracked as TODO #9:
`M3.convert_from_internal()` loses int precision when the conversion factor
is 1.0 (the `value / 1.0` returns a float that MicroPython can't represent
INT32_MAX exactly).

## Goal

The `servomotor` Python library must run unmodified on:
- Standard Python (Mac / PC / Raspberry Pi) — the production path.
- MicroPython (ESP32-S3, Raspberry Pi Pico 2).

Constraint from the project owner: **one version of each module.** No
`#ifdef`-style platform branches scattered through the main code. Anything that
genuinely differs per platform lives in a small, single-purpose "platform layer"
module. The main modules contain zero `is_micropython()` checks.

## Background — why the previous attempt failed

The previous AI produced a working MicroPython build but broke the PC build, then
the owner reverted `communication.py` and kept the MicroPython copy as
`communication_micropython.py`.

Root cause of the PC breakage: `communication_micropython.py` calls
`int.to_bytes(length, "little", signed)` with `signed` passed **positionally**.
CPython requires `signed` to be keyword-only and raises
`TypeError: to_bytes() takes at most 2 positional arguments (3 given)`.

Secondary problem: the fix was done by duplicating the whole ~900-line module
instead of isolating the ~5 small platform-specific spots. Two near-identical
copies will inevitably drift.

## Target structure

```
servomotor/
  # ---- platform layer (the ONLY place is_micropython() appears) ----
  platform_detect.py        platform detection only
  platform_utils.py         CRC32, terminal width, JSON, byte<->int, text wrap,
                            file/time/random helpers, module path resolution
  serial_abstraction.py     SerialPort classes + create/open entry points
  terminal_format_wrapper.py  message formatting (color on PC, plain on MCU)
  platform_compat.py        NEW: language-level shims (Enum, typing stubs)

  # ---- main modules: single-version, no platform branches ----
  communication.py          imports from the platform layer only
  M3.py
  command_loader.py
  device_detection.py
  __init__.py
```

`communication_micropython.py` is **deleted** once `communication.py` is unified.

## Platform-layer responsibilities

### platform_detect.py  (already exists, keep as-is)
`is_micropython()`, `is_standard_python()`, `is_esp32()`, `get_platform_info()`.

### platform_utils.py  (exists; ADD the items below)
Already has: `calculate_crc32`, `get_terminal_columns`, `load_json_file`,
`save_json_file`, file/time/random helpers.

Add:
- `int_to_bytes(value, length, signed=False)` — works on both platforms by
  masking negatives, then calling `.to_bytes(length, "little")` (positional
  byteorder, no `signed` kwarg — valid on CPython and MicroPython):
  ```python
  def int_to_bytes(value, length, signed=False):
      if signed and value < 0:
          value += 1 << (8 * length)
      return value.to_bytes(length, "little")
  ```
- `int_from_bytes(data, signed=False)` — `int.from_bytes(data, "little")` plus
  manual two's-complement, since MicroPython's `from_bytes` lacks `signed`:
  ```python
  def int_from_bytes(data, signed=False):
      result = int.from_bytes(data, "little")
      if signed and result >= (1 << (8 * len(data) - 1)):
          result -= 1 << (8 * len(data))
      return result
  ```
- `wrap_text(text, initial_indent='', subsequent_indent='', width=80)` — uses
  `textwrap` on standard Python, simple fallback on MicroPython.
- `module_path(filename)` — resolves a path to a file inside the `servomotor`
  package. Uses `os.path.dirname(__file__)` on standard Python, a fixed
  `'servomotor/'` prefix on MicroPython. Centralizes the JSON-path logic that is
  currently duplicated in `M3.py` and `__init__.py`.

### serial_abstraction.py  (exists; ADD a unified open entry point)
Already has `SerialPort`, `StandardSerial`, `MicroPythonSerial`,
`create_serial_port()`.

Add `open_serial_port(port, baudrate=230400, timeout=...)` that:
- on standard Python, performs the interactive/automatic port selection that
  `serial_functions.open_serial_port()` does today, then returns a `SerialPort`;
- on MicroPython, returns a `MicroPythonSerial`.

`communication.py` then calls one function regardless of platform.

### terminal_format_wrapper.py  (already exists, keep)
`format_error/info/warning/success/debug`, `STYLE`. Color on PC, plain on MCU.

### platform_compat.py  (NEW)
Language-level shims so the main modules can use normal syntax:
- `Enum` — real `enum.Enum` on standard Python; the lightweight shim (currently
  inline in `M3.py`) on MicroPython.
- `List`, `Tuple` — real `typing` symbols on standard Python; `list`/`tuple`
  aliases on MicroPython (currently an inline try/except in `device_detection.py`).

## Per-module changes (all four modules)

### communication.py
- Delete the inline import branches; import `int_to_bytes`, `int_from_bytes`,
  `calculate_crc32`, `get_terminal_columns`, `wrap_text` from `platform_utils`,
  `open_serial_port` from `serial_abstraction`, formatters from
  `terminal_format_wrapper`.
- Replace every `int.to_bytes(...)` / `int.from_bytes(...)` call with
  `int_to_bytes(...)` / `int_from_bytes(...)`.
- Replace `serial_functions.open_serial_port(...)` with `open_serial_port(...)`.
- Replace `shutil.get_terminal_size().columns` with `get_terminal_columns()`.
- Result: no `is_micropython()` anywhere in the file.
- **Delete `communication_micropython.py`.**

### M3.py
- Import `Enum` from `platform_compat` (remove the ~35-line inline shim).
- Build JSON paths with `platform_utils.module_path(...)` and load them with
  `platform_utils.load_json_file(...)` (remove the inline path branches).

### __init__.py
- Build `data_types.json` / `motor_commands.json` paths with
  `platform_utils.module_path(...)`; single code path.

### command_loader.py
- Use `platform_utils.load_json_file(...)`; remove the try/except import dance.

### device_detection.py
- Import `List`, `Tuple` from `platform_compat`; remove the inline try/except.

## Verification plan (after the test suite exists)

1. Get the comprehensive test suite green on standard Python against real
   hardware (baseline).
2. Apply the refactor above.
3. Re-run the full test suite on standard Python — must stay green. This proves
   the PC path is not broken.
4. Add host-only encoding tests (see TEST_INVENTORY.md) that pack representative
   commands and assert exact bytes — these catch regressions like the `to_bytes`
   bug without hardware.
5. Deploy to MicroPython (ESP32-S3) and run the MicroPython smoke test.

## Design principles to keep

- `is_micropython()` appears **only** inside the platform layer.
- Main modules read top-to-bottom as ordinary Python.
- Each platform-layer module has one concern; no giant catch-all.
- Prefer code that is genuinely identical on both platforms (e.g. the
  `int_to_bytes` form) over branching — branch only where the platforms truly
  cannot share an implementation (serial I/O, color codes, `Enum`).
