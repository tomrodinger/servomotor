#!/usr/bin/env python3
"""
Per-command test for "Control hall sensor statistics" (cmd 32).

Cmd 32 takes one u8 (subcommand):
  0 → turn statistics gathering OFF (no reset of accumulators)
  1 → reset accumulators AND turn statistics gathering ON
  any other value → no-op (firmware ignores)
The reply is the standard no-error packet.

Statistics gathering is the *side effect* this command controls: while
ON, every ADC scan of the hall sensors (inside the TIM1-driven
get_sensor_position() ISR) advances min[3], max[3], sum[3], and n —
see firmware/Src/hall_sensor_calculations.c:72-87. Statistics start
OFF at boot (the active flag is a static uint8_t).

What this test verifies:

  1. The command returns success in all three input modes (0, 1, no-op).
  2. After a reset, statistics are OFF by default: a brief wait shows
     `measurementCount` does NOT advance.
  3. Sending control=1 (reset & enable) makes measurementCount start
     growing; over a 0.3-second window it must reach a non-trivial
     value (the M17 motor control loop runs at ~32 kHz so we expect
     thousands of samples).
  4. Sending control=0 (disable) freezes measurementCount: after a
     read, a second read taken 0.2 s later must be IDENTICAL.
  5. Sending control=1 again resets the accumulators: the first read
     after the second enable must be SMALLER than the read taken
     before the disable (proving min[]=65535, max[]=0, sum[]=0, n=0
     were re-applied — firmware
     hall_sensor_calculations.c:248-259), and then grows again.
  6. A no-op value (e.g. 5) leaves the current state alone — sending
     it while disabled keeps stats disabled (count stays frozen).
  7. No fatal error is set at any point.

(The shape/contents of the stats reply itself are verified by
`test_get_hall_sensor_statistics.py`. This test owns the *control*
semantics of cmd 32.)
"""

import argparse
import sys
import time
import servomotor

RESET_DELAY_S = 1.5
SAMPLE_WINDOW_S = 0.3
FREEZE_WINDOW_S = 0.2
MIN_EXPECTED_SAMPLES = 100  # M17 ADC ISR is ~32 kHz; 0.3 s yields ~9600 samples,
                            # so 100 is a deliberately loose lower bound that still
                            # proves "growing > 0" without being flaky on host jitter.


def get_stats(motor):
    return motor.get_hall_sensor_statistics()


def measurement_count(stats):
    # Layout: [maxHall1, maxHall2, maxHall3, minHall1, minHall2, minHall3,
    #          sumHall1, sumHall2, sumHall3, measurementCount]
    if not isinstance(stats, list) or len(stats) != 10:
        raise AssertionError(f"get_hall_sensor_statistics returned unexpected shape: {stats!r}")
    return stats[9]


def assert_no_fatal_error(motor, label):
    status = motor.get_status()
    if not status or len(status) < 2:
        raise AssertionError(f"{label}: get_status returned unusable value: {status!r}")
    flags, err = status[0], status[1]
    if err != 0:
        raise AssertionError(f"{label}: fatal_error_code={err}, expected 0 (flags={flags:#x})")


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Control hall sensor statistics' (cmd 32).")
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

            print("\nResetting device; statistics gathering must be OFF by default...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)

            n_before = measurement_count(get_stats(motor))
            time.sleep(FREEZE_WINDOW_S)
            n_after = measurement_count(get_stats(motor))
            print(f"  off-by-default count: {n_before} -> {n_after} (delta={n_after - n_before})")
            if n_after != n_before:
                raise AssertionError(f"statistics already gathering after reset: "
                                     f"count {n_before} -> {n_after}, expected no change")
            assert_no_fatal_error(motor, "after reset")

            print(f"control=1 (reset & enable); measurementCount must grow within {SAMPLE_WINDOW_S} s...")
            motor.control_hall_sensor_statistics(1)
            time.sleep(SAMPLE_WINDOW_S)
            n_running = measurement_count(get_stats(motor))
            print(f"  count after enable window: {n_running}")
            if n_running < MIN_EXPECTED_SAMPLES:
                raise AssertionError(f"after control=1, count={n_running} < {MIN_EXPECTED_SAMPLES} "
                                     f"in {SAMPLE_WINDOW_S} s; stats did not start")
            assert_no_fatal_error(motor, "after control=1")

            print(f"control=0 (disable); count must freeze for the next {FREEZE_WINDOW_S} s...")
            motor.control_hall_sensor_statistics(0)
            n_frozen_a = measurement_count(get_stats(motor))
            time.sleep(FREEZE_WINDOW_S)
            n_frozen_b = measurement_count(get_stats(motor))
            print(f"  count while disabled: {n_frozen_a} -> {n_frozen_b}")
            if n_frozen_b != n_frozen_a:
                raise AssertionError(f"after control=0, count still moving: "
                                     f"{n_frozen_a} -> {n_frozen_b}")
            assert_no_fatal_error(motor, "after control=0")

            print("Sending a no-op value (5) while disabled; count must stay frozen...")
            motor.control_hall_sensor_statistics(5)
            time.sleep(FREEZE_WINDOW_S)
            n_after_noop = measurement_count(get_stats(motor))
            print(f"  count after no-op: {n_after_noop}")
            if n_after_noop != n_frozen_b:
                raise AssertionError(f"no-op value (5) changed the state: count "
                                     f"{n_frozen_b} -> {n_after_noop}")
            assert_no_fatal_error(motor, "after no-op")

            print("control=1 again; the reset must drop count BELOW the previous frozen value, then grow...")
            motor.control_hall_sensor_statistics(1)
            n_just_after_reset = measurement_count(get_stats(motor))
            time.sleep(SAMPLE_WINDOW_S)
            n_grown = measurement_count(get_stats(motor))
            print(f"  count after re-enable: {n_just_after_reset} -> {n_grown} "
                  f"(previous frozen value was {n_frozen_b})")
            if n_just_after_reset >= n_frozen_b:
                raise AssertionError(f"control=1 did not reset: count {n_just_after_reset} "
                                     f">= previous frozen {n_frozen_b}")
            if n_grown <= n_just_after_reset:
                raise AssertionError(f"after re-enable, count did not grow: "
                                     f"{n_just_after_reset} -> {n_grown}")
            assert_no_fatal_error(motor, "after re-enable")

            # Leave stats disabled so we don't burden a later test.
            motor.control_hall_sensor_statistics(0)
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
