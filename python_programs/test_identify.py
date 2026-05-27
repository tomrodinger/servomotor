#!/usr/bin/env python3
"""
Per-command test for "Identify" (cmd 41), two-layer pattern (decision
#6 of TEST_MODERNIZATION_PLAN.md).

The identify command tells the motor to make its green LED flash
rapidly so a human can pick the right device out of a bus full of
otherwise-identical motors. The firmware handler in main.c:925-934
disables the SysTick interrupt briefly, sets
`n_identify_flashes = 30`, re-enables SysTick, and acknowledges. The
SysTick handler (main.c:147-159) then runs the blink cycle on its
own: each cycle is 8 ticks of 10 ms (3 LED-on + 5 LED-off) and 30
cycles takes ~2.4 s (the JSON description says "3 seconds" — close).
After the 30 cycles, the LED returns to its normal once-per-second
heartbeat.

This works on every product variant — no `#ifdef PRODUCT_NAME_*`
gating on the identify path — so unlike vibrate (cmd 40) the visual
layer is meaningful on the bench M17 too.

Layers:
  * **Communication layer (always runs, never blocks).** Sends
    identify and asserts it round-trips with no timeout or fatal
    error. The motor must remain responsive during AND after the
    blink window — we ping mid-blink and post-blink to prove the
    RS485 ISR keeps running while the SysTick blink animation plays
    (those two interrupts share priority space, so a regression
    that starves RS485 during identify would show up here).
  * **Visual-confirmation layer (TTY only, opt-in).** When stdin is
    a TTY, prompts the human "Did the green LED flash rapidly?".
    Skipped non-interactively (`run_all_tests.py` redirects stdin),
    where the test passes on the comms layer alone.

What this test verifies:
  1. cmd 41 returns success with no fatal error.
  2. A ping issued ~1 s into the blink window also round-trips, and
     the post-blink ping does too — proves the device stays
     responsive during the blink animation.
  3. (TTY only) The human confirms they saw the green LED flash
     rapidly. Failure to confirm is a test failure.
"""

import argparse
import sys
import time
import servomotor

IDENTIFY_BLINK_DURATION_S = 2.5     # firmware: ~2.4 s (30 cycles × 80 ms); 2.5 is the comfortable upper bound
MID_BLINK_PING_DELAY_S = 1.0
POST_BLINK_SETTLE_S = 0.3
RESET_DELAY_S = 1.5


def assert_no_fatal_error(motor, label):
    status = motor.get_status()
    if not status or len(status) < 2:
        raise AssertionError(f"{label}: get_status returned unusable value: {status!r}")
    flags, err = status[0], status[1]
    if err != 0:
        raise AssertionError(f"{label}: fatal_error_code={err} (flags={flags:#x})")


def assert_ping_ok(motor, label):
    payload = bytes(range(10))
    r = motor.ping(payload)
    if r != payload:
        raise AssertionError(f"{label}: ping payload mismatch: sent={payload!r}, got={r!r}")


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Identify' (cmd 41), with optional human-confirm.")
    parser.add_argument('-p', '--port', help='Serial port device')
    parser.add_argument('-P', '--PORT', action='store_true', help='Show available ports and prompt for selection')
    parser.add_argument('-a', '--alias', default='X', help='Alias of the device to control (default: X)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose output')
    parser.add_argument('--repeat', type=int, default=1, help='Number of times to repeat the test (default: 1)')
    args = parser.parse_args()

    verbose_level = 2 if args.verbose else 0
    servomotor.set_serial_port_from_args(args)

    interactive = sys.stdin.isatty()

    success = False
    failure_message = ""
    motor = None
    try:
        servomotor.open_serial_port()
        motor = servomotor.M3(args.alias, verbose=verbose_level)

        for repeat_idx in range(args.repeat):
            if args.repeat > 1:
                print(f"\n========== REPEAT {repeat_idx + 1} / {args.repeat} ==========")

            print("\nResetting device...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            assert_no_fatal_error(motor, "after reset")

            print(f"Comms layer: sending identify (visual confirmation: "
                  f"{'on (TTY)' if interactive else 'skipped — not interactive'})...")
            t_identify = time.time()
            motor.identify()
            assert_no_fatal_error(motor, "after identify")

            print(f"  pinging mid-blink (t+{MID_BLINK_PING_DELAY_S:.1f}s) — must still respond...")
            time.sleep(max(0.0, MID_BLINK_PING_DELAY_S - (time.time() - t_identify)))
            assert_ping_ok(motor, "mid-blink ping")

            # Wait out the rest of the blink window so the visual confirmation question
            # asks about a completed animation.
            elapsed = time.time() - t_identify
            remaining = IDENTIFY_BLINK_DURATION_S - elapsed
            if remaining > 0:
                time.sleep(remaining)
            time.sleep(POST_BLINK_SETTLE_S)

            print("  post-blink ping must round-trip too...")
            assert_ping_ok(motor, "post-blink ping")
            assert_no_fatal_error(motor, "after blink window")

            if interactive:
                try:
                    ans = input("Did the green LED flash rapidly? [y/n]: ").strip().lower()
                except (EOFError, KeyboardInterrupt):
                    ans = ""
                if not ans.startswith("y"):
                    raise AssertionError("visual confirmation failed: human reported no rapid LED blink")
                print("  visual confirmation: OK")

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
