#!/usr/bin/env python3
"""
Per-command test for "Set maximum motor current" (cmd 28).

Cmd 28 input: (u16 motorCurrent, u16 regenerationCurrent) in arbitrary units
(the JSON notes 150–200 is a suitable working value; regenerationCurrent is
currently unused by the firmware). It sets the current limit the driver
applies to the motor windings. There is no read-back command and no
command-time rejection for a valid in-range current. This test asserts the
observable, non-flaky properties:

  1. A sequence of valid current values is each accepted with a success
     response and leaves the device with no fatal error.
  2. An excessively high current is rejected — the firmware converts the
     current limit into a PWM voltage and rejects values that map above the
     safe maximum with ERROR_MAX_PWM_VOLTAGE_TOO_HIGH (23). This proves the
     value is validated, not blindly stored.
  3. After setting a normal working current, an open-loop move still executes
     and lands on target — i.e. the setting does not break motion.

NOTE: the *physical* effect of the current limit (delivered torque) cannot be
verified without a calibrated load or a current sensor on the bench, so this
test deliberately does not assert torque. The current-streaming tooling
(tools/m23_current_*) is the place that measures actual phase current.
"""

import argparse
import sys
import time
import servomotor
from servomotor.communication import FatalError

COUNTS_PER_ROTATION = 3276800
RESET_DELAY_S = 1.5
TOLERANCE_COUNTS = 80
WORKING_CURRENT = 200
CURRENT_VALUES = [50, 150, 200, 0]   # valid working values (JSON: 150-200 suitable)
EXCESSIVE_CURRENT = 65535            # max u16 — maps above the safe PWM voltage, must be rejected
ERROR_MAX_PWM_VOLTAGE_TOO_HIGH = 23


def get_err(motor, label):
    status = motor.get_status()
    if not status or len(status) < 2:
        raise AssertionError(f"{label}: get_status returned unusable value: {status!r}")
    err = status[1]
    if err != 0:
        raise AssertionError(f"{label}: fatal_error_code={err}, expected 0")
    return err


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Set maximum motor current' (cmd 28).")
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
        # current_unit "arbitrary_units" → pass raw integer values straight through
        motor = servomotor.M3(args.alias, time_unit="seconds", position_unit="encoder_counts",
                              current_unit="internal_current_units", verbose=verbose_level)

        for repeat_idx in range(args.repeat):
            if args.repeat > 1:
                print(f"\n========== REPEAT {repeat_idx + 1} / {args.repeat} ==========")

            print("\nResetting device...")
            motor.system_reset()
            time.sleep(RESET_DELAY_S)

            print("Setting a spread of valid current values; each must be accepted, no fatal error...")
            for c in CURRENT_VALUES:
                motor.set_maximum_motor_current(c, c)
                get_err(motor, f"after set_maximum_motor_current({c}, {c})")
                print(f"  current={c}: accepted, no fatal error.")

            print(f"Setting an excessive current ({EXCESSIVE_CURRENT}); must be rejected with code {ERROR_MAX_PWM_VOLTAGE_TOO_HIGH}...")
            rejected = False
            try:
                motor.set_maximum_motor_current(EXCESSIVE_CURRENT, EXCESSIVE_CURRENT)
            except FatalError as e:
                code = e.args[0] if e.args else None
                if code != ERROR_MAX_PWM_VOLTAGE_TOO_HIGH:
                    raise AssertionError(f"excessive current rejected with FatalError({code}); expected {ERROR_MAX_PWM_VOLTAGE_TOO_HIGH}")
                print(f"  correctly rejected with FatalError({code}) = ERROR_MAX_PWM_VOLTAGE_TOO_HIGH.")
                rejected = True
            if not rejected:
                status = motor.get_status()
                err = status[1] if status and len(status) >= 2 else None
                if err == ERROR_MAX_PWM_VOLTAGE_TOO_HIGH:
                    print(f"  correctly entered fatal_error({err}) = ERROR_MAX_PWM_VOLTAGE_TOO_HIGH.")
                else:
                    raise AssertionError(f"excessive current was NOT rejected (fatal_error={err}); current is not validated.")
            # Clear the latched fatal error before continuing.
            motor.system_reset()
            time.sleep(RESET_DELAY_S)

            print(f"Setting a normal working current ({WORKING_CURRENT}) and running an open-loop move...")
            motor.set_maximum_motor_current(WORKING_CURRENT, WORKING_CURRENT)
            motor.enable_mosfets()
            motor.zero_position()
            target = COUNTS_PER_ROTATION // 4
            motor.trapezoid_move(target, 1.0)
            while motor.get_n_queued_items() > 0:
                time.sleep(0.01)
            time.sleep(0.1)
            pos = motor.get_position()
            print(f"  move with working current ended at {pos} (target ~{target}).")
            if abs(pos - target) > TOLERANCE_COUNTS:
                raise AssertionError(f"move with working current ended at {pos}, expected ~{target}")
            get_err(motor, "after move with working current")

            motor.trapezoid_move(0, 1.0)
            while motor.get_n_queued_items() > 0:
                time.sleep(0.01)
            motor.disable_mosfets()
            motor.system_reset()
            time.sleep(RESET_DELAY_S)

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
