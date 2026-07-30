# Servomotor Arduino-library ON-DEVICE test suite (ESP32-S3)

Tests the Servomotor Arduino library **natively on Arduino-class hardware**: all
test modules compile into one firmware that runs on the custom Gearotons
ESP32-S3 RS485 controller, driving a real motor over Serial1 (RX=GPIO5,
TX=GPIO4) at 230400 baud.

## Architecture

- **`tm_*.cpp`** — one test module per command group (15 modules, ~250
  assertions total, mirroring the Python per-command test suite). Entry point:
  `void tm_<name>(void)`. Modules only use the `tf_framework.h` API.
- **`tf_framework.h/.cpp`** — portable mini framework: `TEST_RESULT`,
  `tfGetMotor()` (unique-ID addressed motor), `checkMotorError` (records, never
  exits), `printf` redirected to USB CDC.
- **`test_registry.cpp`** — the ordered table of tests, each with a known-bug
  annotation (expected failures until fixed) and a hang timeout.
- **`Servomotor_TestSuite_ESP32S3.ino`** — the harness:
  - runs **one test per boot** and **resets the ESP32-S3 between tests**
  - run progress + per-test pass/fail stats persist in **NVS flash** (survive
    reset AND power cycle)
  - a per-test **esp_timer watchdog** force-reboots on a hang; the next boot
    records the test as CRASH and continues the sequence
  - a stale run from a different firmware build is refused (build-id check)
  - serial **CLI** to trigger tests over the debug link (`help`, `list`,
    `run`, `run <n>`, `results`, `detect`, `uid <hex>`, `stop`, `status`)
- **`truncated_frame_test.cpp`** + **`build_truncated_frame_test.sh`** +
  **`run_truncated_frame_test.py`** — a standalone transport-layer regression test for
  `Communication::getResponse()`, needing **no motor and no hardware**. It creates a
  pseudo-terminal and plays a scripted fake device on the far end, so a reply that starts
  and never finishes can be produced on demand (a real motor cannot be asked to truncate a
  reply on cue). Four cases: `silent`, `truncated`, `resync`, `partial`. It guards the
  0.10.1 fix for the receive hang — before that fix, `truncated` hung forever and `partial`
  left a stale byte that surfaced as a spurious `-7` on the next command. Suitable for CI.

- **`host_main.cpp`** + **`build_host_suite.sh`** — the SAME modules also build
  on the Mac against `../ArduinoEmulator.h` for quick iteration without
  flashing: `./host_suite <port> <16-hex-uid> [all|<index>|<name>]`.
- **`run_suite_mac.py`** — Mac-side orchestrator: resets the board, sends
  `run`, collects the streamed log across the between-test reboots, echoes the
  final summary, exit code 0 iff no *unexpected* failures.

## Usage

```bash
./flash_suite.sh                      # refresh installed lib + compile + flash
python3 run_suite_mac.py              # full run (all tests, reset between each)
python3 run_suite_mac.py --only 5     # single test by index
python3 run_suite_mac.py --attach     # just watch an already-active run
```

The suite streams `TEST_START` / `TEST_DONE` / `TEST_CRASH` markers plus every
assertion; the final table (`==== SUITE RESULTS ====` / `SUITE_SUMMARY ...
unexpected=K`) comes from NVS, so it is complete even if the Mac attached late.

## Known-bug annotations

Tests marked with registry bugs assert the CORRECT behavior on purpose and are
EXPECTED to fail until the bugs are fixed (then they should flip to PASS):

- `product_info` — BUG-25 (Get product description: fixed-size struct vs
  null-terminated string)
- `varlen_bugs` — BUG-26/27 (Capture hall sensor data / Read multipurpose
  buffer: fixed-size struct vs variable-length response)
- `time` — BUG-28 (Get current time / Time sync: 32× error, codegen not
  InternalUnit-aware)

`SUITE_SUMMARY unexpected=` counts only failures NOT covered by these
annotations (crashes always count as unexpected).

## Notes

- The device build uses the INSTALLED library (`~/Documents/Arduino/libraries/
  Servomotor`); `flash_suite.sh` refreshes it from the repo first so you never
  test stale code.
- Between-test state: `nvs` namespace `ardsuite`. A power cycle mid-run resumes
  where it left off.
- Do not use `Serial.printf` in modules (object-like `printf` macro).
