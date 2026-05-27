#!/usr/bin/env python3
"""
Per-command test for "Capture hall sensor data" (cmd 7).

The JSON description in `servomotor/motor_commands.json` is explicit:

  "Start sending hall sensor data (work in progress; don't send this
   command)"

…and indeed, on a successful call the firmware enters a multi-segment
RS485 streaming mode (firmware/Src/main.c:301-360) that the standard
Python wrapper has no path for: it would `rs485_start_the_packet()`
with a custom payload size and then dribble out samples one at a time
in the main loop while ignoring further commands. Driving that path
from this test would mean writing a one-off receiver, and on a bad
day could lock the device into a "still capturing" state until a
power-cycle — exactly the kind of side effect a T3 per-command test
should avoid.

What the test DOES exercise is the parameter-validation surface, which
is the part of cmd 7 most likely to silently regress. The firmware
performs six checks in sequence (main.c:314-339) and raises
`fatal_error(ERROR_CAPTURE_BAD_PARAMETERS)` (code 49) on any
violation. That error is recoverable via `system_reset` (the RS485
ISR is polled manually inside the fatal_error loop, see
common_source_files/error_handling.c:115-149).

Importantly, `process_packet()` calls `if_fatal_error_then_respond()`
**before** the command dispatch (main.c:232-234), so when the cmd-7
handler calls fatal_error, the fatal_error loop transmits an
error-packet response and the Python wrapper raises
`servomotor.communication.FatalError(49)` — NOT a TimeoutError. For
each invalid-input case we assert:

  * `motor.capture_hall_sensor_data(...)` raises `FatalError` whose
    error code is 49 (ERROR_CAPTURE_BAD_PARAMETERS).
  * `get_status()` then also reports fatal_error_code == 49 (the
    error is latched in the device's global state too).
  * `system_reset()` clears the latched error.

The cases covered:
  1. capture_type = 0 (out of range 1..3).
  2. capture_type = 4 (out of range 1..3).
  3. division_factor = 0 (must be non-zero).
  4. channels_to_capture_bitmask = 0 (no bits set).
  5. channels_to_capture_bitmask = 0b1000 (bit beyond [0,2]).

(We do NOT issue a valid capture. That is intentional — see above.)

If this command ever graduates out of "work in progress", a follow-up
test should add the streaming-response side, but it will need wrapper
support for variable-length / multi-segment replies first.
"""

import argparse
import sys
import time
import servomotor
from servomotor.communication import FatalError as ServoFatalError

RESET_DELAY_S = 1.5
ERROR_CAPTURE_BAD_PARAMETERS = 49   # firmware error_text.h:56

# A "would-be-valid-if-not-for-the-one-bad-field" baseline. We mutate one field
# per case to make ONLY that field the cause of the rejection.
BASE_VALID = dict(
    capture_type=1,
    n_points_to_capture=10,
    channels_to_capture_bitmask=1,
    time_steps_per_sample=1000,
    n_samples_to_sum=1,
    division_factor=1,
)

def _with(**overrides):
    """Return BASE_VALID with the given overrides applied (MicroPython-friendly,
    avoids dict-unpacking syntax which MicroPython 1.28 does not support)."""
    d = dict(BASE_VALID)
    d.update(overrides)
    return d


INVALID_CASES = [
    ("capture_type=0 (out of range 1..3)",
        _with(capture_type=0)),
    ("capture_type=4 (out of range 1..3)",
        _with(capture_type=4)),
    ("division_factor=0 (must be non-zero)",
        _with(division_factor=0)),
    ("channels_to_capture_bitmask=0 (no channels selected)",
        _with(channels_to_capture_bitmask=0)),
    ("channels_to_capture_bitmask=0b1000 (bit outside [0,2])",
        _with(channels_to_capture_bitmask=0b1000)),
]


def send_cmd7_expect_fatal_error(motor, params, expected_code, label):
    """Send cmd 7 with the given parameters; expect FatalError with the given code in the response."""
    try:
        motor.capture_hall_sensor_data(
            params["capture_type"],
            params["n_points_to_capture"],
            params["channels_to_capture_bitmask"],
            params["time_steps_per_sample"],
            params["n_samples_to_sum"],
            params["division_factor"],
        )
    except ServoFatalError as e:
        # FatalError(error_code) — int(str(e)) on the raised object recovers the code; the
        # argument is the raw byte from the error packet payload.
        got = e.args[0] if e.args else None
        if got != expected_code:
            raise AssertionError(f"{label}: FatalError code = {got}, expected {expected_code}")
        return
    except Exception as e:
        raise AssertionError(f"{label}: expected FatalError({expected_code}), got "
                             f"{type(e).__name__}: {e}")
    raise AssertionError(f"{label}: cmd 7 returned success on an invalid-parameter call "
                         f"(firmware should fatal_error instead of replying success)")


def assert_fatal_error(motor, expected_code, label):
    status = motor.get_status()
    if not status or len(status) < 2:
        raise AssertionError(f"{label}: get_status returned unusable value: {status!r}")
    flags, err = status[0], status[1]
    if err != expected_code:
        raise AssertionError(f"{label}: fatal_error_code={err}, expected {expected_code} "
                             f"(ERROR_CAPTURE_BAD_PARAMETERS) (flags={flags:#x})")


def assert_no_fatal_error(motor, label):
    status = motor.get_status()
    if not status or len(status) < 2:
        raise AssertionError(f"{label}: get_status returned unusable value: {status!r}")
    flags, err = status[0], status[1]
    if err != 0:
        raise AssertionError(f"{label}: fatal_error_code={err}, expected 0 (flags={flags:#x})")


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Capture hall sensor data' (cmd 7).")
    parser.add_argument('-p', '--port', help='Serial port device')
    parser.add_argument('-P', '--PORT', action='store_true', help='Show available ports and prompt for selection')
    parser.add_argument('-a', '--alias', default='X', help='Alias of the device to control (default: X)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose output')
    parser.add_argument('--repeat', type=int, default=1, help='Number of times to repeat the test (default: 1)')
    args = parser.parse_args()

    verbose_level = 2 if args.verbose else 0
    servomotor.set_serial_port_from_args(args)

    success = False
    failure_message = ""
    try:
        servomotor.open_serial_port()
        motor = servomotor.M3(args.alias, verbose=verbose_level)

        for repeat_idx in range(args.repeat):
            if args.repeat > 1:
                print(f"\n========== REPEAT {repeat_idx + 1} / {args.repeat} ==========")

            for label, params in INVALID_CASES:
                print(f"\n[{label}] resetting; expect a clean post-reset state...")
                motor.system_reset()
                time.sleep(RESET_DELAY_S)
                assert_no_fatal_error(motor, f"before [{label}]")

                print(f"[{label}] sending cmd 7 — expect FatalError({ERROR_CAPTURE_BAD_PARAMETERS}) in the response...")
                send_cmd7_expect_fatal_error(motor, params, ERROR_CAPTURE_BAD_PARAMETERS, label)

                print(f"[{label}] reading status — expect fatal_error_code = {ERROR_CAPTURE_BAD_PARAMETERS}...")
                assert_fatal_error(motor, ERROR_CAPTURE_BAD_PARAMETERS, label)

                print(f"[{label}] system_reset — error must clear...")
                motor.system_reset()
                time.sleep(RESET_DELAY_S)
                assert_no_fatal_error(motor, f"after recovery from [{label}]")

        success = True

    except Exception as e:
        failure_message = str(e) if str(e) else type(e).__name__
    finally:
        servomotor.close_serial_port()

    if success:
        print("\nPASSED")
        return 0
    print(f"\nFAILED: {failure_message}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
