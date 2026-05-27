#!/usr/bin/env python3
"""
Per-command test for "Vibrate" (cmd 40), two-layer pattern (decision #6
of TEST_MODERNIZATION_PLAN.md).

The vibrate command is *only* implemented on M1 in the current firmware
— `void vibrate(uint8_t)` in firmware/Src/motor_control.c:1162 is
wrapped in `#ifdef PRODUCT_NAME_M1`, so on M2/M17/M23 the function body
is empty. The cmd-40 dispatch in main.c:908-923 still calls it and
still sends back the no-error reply, so the communication path is
exercised end-to-end on every product variant, even when there is
nothing to visually confirm.

Layers:
  * **Communication layer (always runs, never blocks).** Sends
    vibrate(1) and vibrate(0); both must round-trip with no timeout
    and leave no fatal error. This is the part most likely to break
    silently.
  * **Visual-confirmation layer (TTY + M1 only, opt-in).** When stdin
    is a TTY AND the device reports product model M1, prompts the
    human "Did the motor vibrate?". Skipped non-interactively
    (`run_all_tests.py` redirects stdin) and skipped on non-M1
    products (no physical effect to confirm).

What this test verifies:
  1. cmd 40 with level=1 returns success and leaves no fatal error.
  2. cmd 40 with level=0 returns success and leaves no fatal error.
  3. After vibrate(0), the device is responsive (ping round-trips) —
     proves the on→off cycle ends in a clean state.
  4. (TTY+M1 only) The human confirms physical vibration during
     level=1. Failure to confirm is a test failure.

The vibrate-level-0 cleanup is run in the `finally` block as well,
so an interrupted M1 run cannot leave the motor stuck vibrating.
"""

import argparse
import sys
import time
import servomotor

VIBRATE_DURATION_S = 1.5    # how long to vibrate for the visual layer
RESET_DELAY_S = 1.5


def get_product_model_letter(motor):
    """Return the first letter of the product code (e.g. 'M' from 'M17 ') or '?' on any error."""
    try:
        info = motor.get_product_info()
        # get_product_info returns a list; first field is product_code (8-byte string).
        if info and len(info) >= 1:
            code = info[0]
            if isinstance(code, str) and code:
                return code.strip()[:3]   # 'M17', 'M1', 'M2', 'M23'
    except Exception:
        pass
    return "?"


def assert_no_fatal_error(motor, label):
    status = motor.get_status()
    if not status or len(status) < 2:
        raise AssertionError(f"{label}: get_status returned unusable value: {status!r}")
    flags, err = status[0], status[1]
    if err != 0:
        raise AssertionError(f"{label}: fatal_error_code={err} (flags={flags:#x})")


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Vibrate' (cmd 40), with optional human-confirm.")
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

            product = get_product_model_letter(motor)
            print(f"  product model: {product!r}")
            visual_layer_applicable = (product == "M1")
            do_visual = interactive and visual_layer_applicable

            print(f"Comms layer: sending vibrate(1)... (visual confirmation: "
                  f"{'on (M1 + TTY)' if do_visual else 'skipped — '+ ('not interactive' if not interactive else 'no vibrate on '+product)})")
            motor.vibrate(1)
            assert_no_fatal_error(motor, "after vibrate(1)")

            if do_visual:
                print(f"  motor should be vibrating for ~{VIBRATE_DURATION_S} s...")
                time.sleep(VIBRATE_DURATION_S)
                try:
                    ans = input("Is the motor vibrating? [y/n]: ").strip().lower()
                except (EOFError, KeyboardInterrupt):
                    ans = ""
                if not ans.startswith("y"):
                    raise AssertionError("visual confirmation failed: human reported no vibration on M1")
                print("  visual confirmation: OK")
            else:
                # Brief settle just to give the firmware a moment, even on the no-op path.
                time.sleep(0.1)

            print("Comms layer: sending vibrate(0)...")
            motor.vibrate(0)
            assert_no_fatal_error(motor, "after vibrate(0)")

            # The device must remain responsive after the on/off cycle.
            print("Post-cycle: ping must round-trip cleanly...")
            payload = bytes(range(10))
            r = motor.ping(payload)
            if r != payload:
                raise AssertionError(f"post-vibrate ping payload mismatch: sent={payload!r}, got={r!r}")

            motor.system_reset()
            time.sleep(RESET_DELAY_S)

        success = True

    except Exception as e:
        failure_message = str(e) if str(e) else type(e).__name__
    finally:
        # Defensive: make sure the motor is not left vibrating on an error path
        # (only matters on M1, but the call is harmless on M2/M17/M23).
        if motor is not None:
            try:
                motor.vibrate(0)
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
