#!/usr/bin/env python3
"""
Per-command test for "Get communication statistics" (cmd 47).

Cmd 47 takes one u8 input (resetCounter; 1 = also zero the counters,
0 = read only) and returns six u32 counters that the firmware
accumulates in the RS485 interrupt path:

    [crc32_errors,
     packet_decode_errors,
     first_bit_errors,
     framing_errors,
     overrun_errors,
     noise_errors]

What this test verifies:

  1. The response is a 6-element list of non-negative integers — proves
     the framing of the multi-field reply round-trips end-to-end.
  2. Counters are non-decreasing under correct traffic: send a handful
     of well-formed commands between two read-only (resetCounter=0)
     reads; no counter may go down. A counter going down with no reset
     would mean the firmware is mis-handling the read.
  3. Resetting works: call with resetCounter=1, then immediately read
     with resetCounter=0; all six counters must be zero in that second
     read. (We use the result of step 2 as evidence the bus is currently
     quiet — only well-formed traffic in flight.)
  4. After the reset, sending more correct traffic still leaves the
     counters at zero — proves valid traffic does not spuriously bump
     any counter on a quiet bus.

(Inducing an actual error to prove the counters *do* increment is the
job of test_trigger_framing_error.py, a functional test that owns the
low-level injection. This per-command test sticks to the documented
read/reset behavior of cmd 47 itself.)
"""

import argparse
import sys
import time
import servomotor

RESET_DELAY_S = 1.5
N_FIELDS = 6
FIELD_NAMES = ["crc32_errors", "packet_decode_errors", "first_bit_errors",
               "framing_errors", "overrun_errors", "noise_errors"]
N_TRAFFIC = 30   # ping bursts to stress correct traffic between reads


def assert_six_nonneg_ints(stats, label):
    if not isinstance(stats, list) or len(stats) != N_FIELDS:
        raise AssertionError(f"{label}: expected {N_FIELDS}-element list, got: {stats!r}")
    for name, v in zip(FIELD_NAMES, stats):
        if not isinstance(v, int):
            raise AssertionError(f"{label}: {name} is not int: {v!r} ({type(v).__name__})")
        if v < 0:
            raise AssertionError(f"{label}: {name} is negative: {v}")


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Get communication statistics' (cmd 47).")
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
        motor = servomotor.M3(args.alias, time_unit="seconds", position_unit="encoder_counts",
                              verbose=verbose_level)

        for repeat_idx in range(args.repeat):
            if args.repeat > 1:
                print(f"\n========== REPEAT {repeat_idx + 1} / {args.repeat} ==========")

            print("\nResetting device (also zeroes the firmware's RS485 error counters at boot)...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)

            print("Read once (reset=0); response shape and types...")
            s0 = motor.get_communication_statistics(0)
            print(f"  initial -> {dict(zip(FIELD_NAMES, s0))}")
            assert_six_nonneg_ints(s0, "initial read")

            print(f"Driving {N_TRAFFIC} well-formed pings; counters must not decrease...")
            for _ in range(N_TRAFFIC):
                motor.ping(bytes(10))  # any well-formed traffic
            s1 = motor.get_communication_statistics(0)
            print(f"  after {N_TRAFFIC} pings -> {dict(zip(FIELD_NAMES, s1))}")
            assert_six_nonneg_ints(s1, "post-traffic read")
            for name, a, b in zip(FIELD_NAMES, s0, s1):
                if b < a:
                    raise AssertionError(f"counter '{name}' decreased without a reset: "
                                         f"{a} -> {b}")

            print("Reset (reset=1) and read again (reset=0); all counters must be zero...")
            motor.get_communication_statistics(1)
            s2 = motor.get_communication_statistics(0)
            print(f"  post-reset -> {dict(zip(FIELD_NAMES, s2))}")
            assert_six_nonneg_ints(s2, "post-reset read")
            for name, v in zip(FIELD_NAMES, s2):
                if v != 0:
                    raise AssertionError(f"after reset counter '{name}' is {v}, expected 0")

            print(f"Driving {N_TRAFFIC} more well-formed pings; counters must STAY zero (valid "
                  f"traffic must not bump any counter on a quiet bus)...")
            for _ in range(N_TRAFFIC):
                motor.ping(bytes(10))
            s3 = motor.get_communication_statistics(0)
            print(f"  post-reset+traffic -> {dict(zip(FIELD_NAMES, s3))}")
            assert_six_nonneg_ints(s3, "post-reset post-traffic read")
            for name, v in zip(FIELD_NAMES, s3):
                if v != 0:
                    raise AssertionError(f"counter '{name}' = {v} after valid-only traffic; "
                                         f"a well-formed ping bumped a counter")

            motor.system_reset()
            time.sleep(RESET_DELAY_S)

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
