#!/usr/bin/env python3
"""
Per-command test for "Read multipurpose buffer" (cmd 35).

The multipurpose buffer is a single in-RAM region the firmware aliases
to the calibration scratchpad. Several different commands can write
into it (firmware/Src/motor_control.h:131-135):

  type 1 = calibration data
  type 2 = go-to-closed-loop data (M1 only on the current firmware)
  type 3 = vibrate data (M1 only on the current firmware)
  type 4 = PID debug data (sizeof = 20 bytes: 5 × int32)
  type 5 = GC6609 register dump (10 regs × 8 bytes = 80 bytes)

The cmd 35 handler (firmware/Src/main.c:779-836) is special:

  * If the buffer is empty (`multipurpose_data_type == 0`) the firmware
    sends NO response at all. The Python wrapper times out — by design.
  * If the buffer holds data, the firmware sends a custom packet
    whose payload is [data_type (1 byte)][data_size bytes]. After a
    successful transmit it calls clear_multipurpose_data() — so
    follow-up reads of the same data return nothing UNLESS something
    is actively repopulating the buffer.

How we populate the buffer for this test
========================================

We use `test_mode(3)` (`READ_PID_DEBUG_DATA_TEST_MODE`) inside closed
loop. The PID controller in `motor_control.c:2270-2278` writes 20
bytes (struct pid_debug_data_struct) into the buffer **only when**
`multipurpose_data_type == 0` — so the populate is effectively
one-shot per (clear → next-PID-iteration) cycle, even though
`test_mode` itself stays set at 3 (it does NOT auto-clear).

Two alternative populators were considered and rejected for this
test:

  * `test_mode(4)` (GC6609 register dump, type 5) — would have been
    great because it auto-clears `test_mode` on completion
    (motor_control.c:2853), so a follow-up cmd 35 would genuinely
    time out. But the bit-bang stalls in `waiting_for_start`
    (GC6609.c:359) if the chip's UART does not respond, which on the
    bench M17 it didn't reliably do — the buffer never populated
    even after 5 s of waiting. Not robust enough for an auto-suite
    test.
  * `start_calibration` (type 1) — populates the buffer reliably but
    a full calibration is far too slow for a T3 test.

⚠️  IMPORTANT GOTCHA — DO NOT USE `test_mode(0)` TO CLEAR THE TEST MODE.
The cmd-36 handler routes `test_mode == 0` inline into
`set_led_test_mode(0)` (firmware/Src/main.c:880-882), which is
`__disable_irq();` + `while(1);` — the firmware locks up until a
hardware power-cycle. This is the same freeze as the documented LED
test modes 10..13 but happens BEFORE the no-error reply is sent. See
the `led-test-mode-is-stuck-forever` memory note and
WORK_CHECKLIST.md TODO #8 for the firmware-side fix. To clean up after
this test, we use `system_reset()` — that resets the static
`test_mode` flag with no `set_led_test_mode` call on the way out.

What this test verifies
=======================

  1. Empty buffer (post-reset, before any populator runs) → cmd 35
     raises `servomotor.communication.TimeoutError`. Proves the
     "no response when empty" branch is wired correctly.
  2. After entering closed loop and asserting `test_mode(3)`, cmd 35
     returns a non-empty bytes payload whose first byte is the
     data-type tag (== 4, `MULTIPURPOSE_DATA_TYPE_PID_DEBUG_DATA`)
     and whose remaining length is exactly
     sizeof(struct pid_debug_data_struct) = 20.
  3. The 20-byte payload decodes as five int32 fields with values
     inside the i32 range — proves the bytes survived round-trip and
     the firmware's reported size matched the wrapper's received
     length.
  4. After `system_reset()` (which clears both `test_mode` and the
     buffer), cmd 35 times out again — closing the loop on the
     empty-buffer behavior.
  5. No fatal error is left on the device throughout.

(We deliberately do NOT assert "an immediate second cmd 35 times out"
after the populated read. That would be wrong: PID iterations run
every ~32 µs in closed loop and the gating predicate
`multipurpose_data_type == 0` becomes true again the moment our read
clears it, so the buffer is silently repopulated long before any
follow-up RS485 round-trip can complete. The empty-buffer behavior is
already covered by steps 1 and 4 above, where no PID activity is
touching the buffer.)

This test needs a calibrated motor (closed loop requires it). The
bench M17 is calibrated per WORK_CHECKLIST.md.
"""

import argparse
import struct
import sys
import time
import servomotor
from servomotor.communication import TimeoutError as ServoTimeoutError

RESET_DELAY_S = 1.5
ENABLE_SETTLE_S = 0.3
CLOSED_LOOP_TIMEOUT_S = 6.0
PID_DEBUG_CAPTURE_WINDOW_S = 0.05   # one PID iteration is ~32 µs; this is plenty

CLOSED_LOOP_BIT = 1 << 2

TEST_MODE_PID_DEBUG = 3
# NOTE: we DELIBERATELY never call test_mode(0). See the module docstring.

MULTIPURPOSE_DATA_TYPE_PID_DEBUG_DATA = 4
PID_DEBUG_PAYLOAD_BYTES = 5 * 4  # 5 × int32

I32_MIN, I32_MAX = -(1 << 31), (1 << 31) - 1


def attempt_read_buffer(motor):
    """Call cmd 35; return bytes payload, or None on TimeoutError."""
    try:
        result = motor.read_multipurpose_buffer()
    except ServoTimeoutError:
        return None
    except Exception as e:
        # Bare-TimeoutError fallback (see test_correct_and_incorrect_addressing.py).
        if type(e).__name__ == "TimeoutError":
            return None
        raise
    if not isinstance(result, (bytes, bytearray)):
        raise AssertionError(f"cmd 35 returned non-bytes: {type(result).__name__}: {result!r}")
    return bytes(result)


def assert_empty_buffer_times_out(motor, label):
    result = attempt_read_buffer(motor)
    if result is not None:
        raise AssertionError(f"{label}: cmd 35 returned a response on an empty buffer: {result!r}")


def wait_for_closed_loop(motor, timeout_s):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        status = motor.get_status()
        if status and len(status) >= 2 and (status[0] & CLOSED_LOOP_BIT):
            return
        time.sleep(0.1)
    raise AssertionError(f"closed-loop flag never set within {timeout_s} s; calibrated?")


def assert_no_fatal_error(motor, label):
    status = motor.get_status()
    if not status or len(status) < 2:
        raise AssertionError(f"{label}: get_status returned unusable value: {status!r}")
    flags, err = status[0], status[1]
    if err != 0:
        raise AssertionError(f"{label}: fatal_error_code={err} (flags={flags:#x})")


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Read multipurpose buffer' (cmd 35).")
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
    motor = None
    try:
        servomotor.open_serial_port()
        motor = servomotor.M3(args.alias, verbose=verbose_level)

        for repeat_idx in range(args.repeat):
            if args.repeat > 1:
                print(f"\n========== REPEAT {repeat_idx + 1} / {args.repeat} ==========")

            print("\nResetting; the empty multipurpose buffer must produce a TimeoutError on cmd 35...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            assert_empty_buffer_times_out(motor, "post-reset empty buffer")
            assert_no_fatal_error(motor, "after post-reset empty read")

            print("Entering closed loop and asserting test_mode 3 so PID writes its debug struct...")
            motor.enable_mosfets()
            time.sleep(ENABLE_SETTLE_S)
            motor.go_to_closed_loop()
            wait_for_closed_loop(motor, CLOSED_LOOP_TIMEOUT_S)
            motor.test_mode(TEST_MODE_PID_DEBUG)
            time.sleep(PID_DEBUG_CAPTURE_WINDOW_S)

            print("Reading cmd 35; expect [data_type=4][20 bytes of pid_debug_data]...")
            raw = attempt_read_buffer(motor)
            if raw is None:
                raise AssertionError("cmd 35 returned no data even though test_mode 3 was asserted "
                                     "inside closed loop — PID never populated the buffer?")
            if len(raw) != 1 + PID_DEBUG_PAYLOAD_BYTES:
                raise AssertionError(f"cmd 35 payload length {len(raw)} != "
                                     f"{1 + PID_DEBUG_PAYLOAD_BYTES} (1 type byte + 20 PID bytes)")
            data_type = raw[0]
            if data_type != MULTIPURPOSE_DATA_TYPE_PID_DEBUG_DATA:
                raise AssertionError(f"cmd 35 first byte = {data_type} (expected "
                                     f"{MULTIPURPOSE_DATA_TYPE_PID_DEBUG_DATA} for PID debug)")
            fields = struct.unpack("<iiiii", bytes(raw[1:]))
            print(f"  data_type={data_type}, pid_debug="
                  f"{dict(zip(['error','P_term','I_term','D_term','output'], fields))}")
            for name, value in zip(["error", "P_term", "I_term", "D_term", "output"], fields):
                if not (I32_MIN <= value <= I32_MAX):
                    raise AssertionError(f"PID debug field '{name}'={value} outside i32 range")

            print("system_reset to clear test_mode + exit closed loop + clear the buffer "
                  "(NOT test_mode(0) — see docstring)...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)

            print("Final empty-buffer check after system_reset...")
            assert_empty_buffer_times_out(motor, "post-cleanup empty buffer")
            assert_no_fatal_error(motor, "after post-cleanup empty read")

        success = True

    except Exception as e:
        failure_message = str(e) if str(e) else type(e).__name__
    finally:
        if motor is not None:
            try:
                motor.disable_mosfets()
            except Exception:
                pass
        servomotor.close_serial_port()

    if success:
        print("\nPASSED")
        return 0
    print(f"\nFAILED: {failure_message}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
