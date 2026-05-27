#!/usr/bin/env python3
"""
Functional test: Enable / Disable MOSFETs reliability.

Modernized from the obsolete tests that this replaces:
  * test_enable_disable.py — rapid enable/disable cycling
  * test_iterate_reset_and_enable.py — reset-and-enable cycling

The per-command basics for "Enable MOSFETs" (cmd 1) and "Disable MOSFETs"
(cmd 0) live in their own dedicated tests; this functional test focuses on
reliability — rapid toggling and reset cycling — and verifies the device
keeps responding and reports no fatal error throughout.

Phases:
  A. Rapid enable/disable cycling — `--n-toggle-cycles` iterations
     (default 50). After the loop, `get_status` must return no fatal error
     and the MOSFETs-enabled flag must be clear.
  B. Reset/enable cycling — `--n-reset-cycles` iterations (default 10).
     Each iteration: enable → disable → system_reset → wait → get_status.
     The device must come back in application mode with no fatal error
     after every reset.
  C. Final sanity: a ping round-trips, confirming communication is still
     working at the end.
"""

import argparse
import sys
import time
import servomotor

# Bit positions in the get_status flags word — must match common_source_files/device_status.h.
STATUS_IN_THE_BOOTLOADER_FLAG_BIT = 0
STATUS_MOSFETS_ENABLED_FLAG_BIT = 1

DEFAULT_TOGGLE_CYCLES = 50
DEFAULT_RESET_CYCLES = 10
RESET_DELAY_S = 1.5  # time to let the device come back up in app mode after system_reset


def assert_app_mode_no_error(motor, label):
    """Read get_status and require: no fatal error, not in the bootloader.

    Returns (flags, fatal_error_code) on success; raises AssertionError otherwise.
    """
    status = motor.get_status()
    if not status or not isinstance(status, list) or len(status) < 2:
        raise AssertionError(f"{label}: get_status returned unusable value: {status!r}")
    flags, err = status[0], status[1]
    if err != 0:
        raise AssertionError(f"{label}: fatal_error_code={err} (expected 0). flags={flags:#x}")
    if flags & (1 << STATUS_IN_THE_BOOTLOADER_FLAG_BIT):
        raise AssertionError(f"{label}: device is in the bootloader (flags={flags:#x}, expected application mode)")
    return flags, err


def main():
    parser = argparse.ArgumentParser(description="Functional test: rapid enable/disable + reset/enable cycling reliability.")
    parser.add_argument('-p', '--port', help='Serial port device')
    parser.add_argument('-P', '--PORT', action='store_true', help='Show available ports and prompt for selection')
    parser.add_argument('-a', '--alias', default='X', help='Alias of the device to control (default: X)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose output')
    parser.add_argument('--n-toggle-cycles', type=int, default=DEFAULT_TOGGLE_CYCLES,
                        help=f'Number of enable/disable cycles in phase A (default: {DEFAULT_TOGGLE_CYCLES})')
    parser.add_argument('--n-reset-cycles', type=int, default=DEFAULT_RESET_CYCLES,
                        help=f'Number of reset/enable cycles in phase B (default: {DEFAULT_RESET_CYCLES})')
    parser.add_argument('--repeat', type=int, default=1, help='Number of times to repeat the whole test (default: 1)')
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

            # Start from a clean state.
            print("Resetting device to a known state...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            assert_app_mode_no_error(motor, "after initial reset")

            # ---- Phase A: rapid enable/disable cycling ----
            n_a = args.n_toggle_cycles
            print(f"\nPhase A: {n_a} enable/disable cycles...")
            phase_a_start = time.time()
            for i in range(n_a):
                motor.enable_mosfets()
                motor.disable_mosfets()
            phase_a_duration = time.time() - phase_a_start
            per_cycle_ms = (phase_a_duration * 1000.0 / n_a) if n_a > 0 else 0.0
            print(f"  done in {phase_a_duration:.2f}s ({per_cycle_ms:.1f} ms per cycle)")

            flags, _ = assert_app_mode_no_error(motor, "after phase A")
            if flags & (1 << STATUS_MOSFETS_ENABLED_FLAG_BIT):
                raise AssertionError(
                    f"After phase A the MOSFETs-enabled flag is still set (flags={flags:#x}); "
                    f"the final disable did not stick."
                )

            # ---- Phase B: reset/enable cycling ----
            n_b = args.n_reset_cycles
            print(f"\nPhase B: {n_b} reset/enable cycles...")
            for i in range(n_b):
                motor.enable_mosfets()
                motor.disable_mosfets()
                motor.system_reset()
                time.sleep(RESET_DELAY_S)
                assert_app_mode_no_error(motor, f"phase B iteration {i+1}")
                if args.verbose:
                    print(f"  iteration {i+1}: OK")
            print("  done")

            # ---- Phase C: final ping round-trip ----
            print("\nPhase C: final ping check...")
            payload = bytes(range(10))
            response = motor.ping(payload)
            if response != payload:
                raise AssertionError(f"Final ping mismatch: sent {payload!r}, got {response!r}")
            print("  ping round-trip OK")

        success = True

    except Exception as e:
        failure_message = str(e) if str(e) else type(e).__name__
    finally:
        # Leave the device disabled in a known state.
        if motor is not None:
            try:
                motor.disable_mosfets()
            except Exception:
                pass
        servomotor.close_serial_port()

    # Print verdict AFTER close_serial_port() runs so PASSED is the literal
    # last line of stdout (run_all_tests.py grades on the last line).
    if success:
        print("\nPASSED")
        return 0
    print(f"\nFAILED: {failure_message}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
