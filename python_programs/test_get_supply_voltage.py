#!/usr/bin/env python3
"""
Per-command test for "Get supply voltage" (cmd 38).

Cmd 38 has no input and returns u16 — supply voltage in decivolts
(value / 10 = volts) internally. The M3 wrapper applies the unit
conversion declared in unit_conversions_*.json so user-facing reads come
back in the configured voltage unit.

What this test verifies:
  1. The response is a numeric scalar (not None / not a list / etc.).
  2. Reading in volts returns a value in a plausible range for any
     reasonable PSU driving these motors (6 V – 40 V) — wide enough that
     any sane lab supply passes, narrow enough that 0 or 200 fails.
  3. Unit conversion is self-consistent: the same instantaneous voltage
     read in volts and in millivolts converts (millivolts / 1000 ≈
     volts) within a tight tolerance — proves the voltage unit layer in
     the M3 wrapper is wired up correctly for this command.
  4. The reading is stable: a short stream of consecutive reads stays
     within a small relative band — proves the value is a live ADC read,
     not a jumpy garbage number.
"""

import argparse
import sys
import time
import servomotor

RESET_DELAY_S = 1.5
MIN_VOLTS = 6.0
MAX_VOLTS = 40.0
UNIT_TOLERANCE_VOLTS = 0.2     # |volts vs (millivolts/1000)| (back-to-back reads)
STABILITY_BAND_VOLTS = 1.0     # range across N stability samples
N_STABILITY_SAMPLES = 10


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Get supply voltage' (cmd 38).")
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
                              voltage_unit="volts", verbose=verbose_level)

        for repeat_idx in range(args.repeat):
            if args.repeat > 1:
                print(f"\n========== REPEAT {repeat_idx + 1} / {args.repeat} ==========")

            print("\nResetting device...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)
            time.sleep(0.2)  # let the supply ADC settle after reset

            print("Reading supply voltage in volts; must be a scalar in a plausible range...")
            motor.set_voltage_unit("volts")
            v = motor.get_supply_voltage()
            print(f"  supply voltage = {v} V")
            if not isinstance(v, (int, float)):
                raise AssertionError(f"get_supply_voltage returned non-numeric: {v!r}")
            if not (MIN_VOLTS <= v <= MAX_VOLTS):
                raise AssertionError(f"supply voltage {v} V outside plausible range "
                                     f"[{MIN_VOLTS}, {MAX_VOLTS}]")

            print("Reading the same supply in millivolts; conversion must agree...")
            motor.set_voltage_unit("millivolts")
            v_mv = motor.get_supply_voltage()
            motor.set_voltage_unit("volts")  # restore
            v_from_mv = v_mv / 1000.0
            diff = abs(v_from_mv - v)
            print(f"  in millivolts = {v_mv} mV -> {v_from_mv} V (diff vs volts read = {diff:.4f} V)")
            if diff > UNIT_TOLERANCE_VOLTS:
                raise AssertionError(f"volts ({v}) and millivolts ({v_mv}) readings disagree by "
                                     f"{diff:.4f} V (> {UNIT_TOLERANCE_VOLTS})")

            print(f"Reading {N_STABILITY_SAMPLES} consecutive samples; spread must be within "
                  f"{STABILITY_BAND_VOLTS} V...")
            samples = [motor.get_supply_voltage() for _ in range(N_STABILITY_SAMPLES)]
            lo, hi = min(samples), max(samples)
            spread = hi - lo
            print(f"  samples: min={lo} V, max={hi} V, spread={spread:.4f} V")
            if spread > STABILITY_BAND_VOLTS:
                raise AssertionError(f"supply voltage unstable: spread {spread:.4f} V > "
                                     f"{STABILITY_BAND_VOLTS} V")

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
