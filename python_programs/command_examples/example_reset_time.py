#!/usr/bin/env python3
"""
Example: Reset time -- zero the device's absolute microsecond clock.
Reads the clock, resets it, then reads again: the second reading is near zero.
WARNING: 'Reset time' also clears the entire motion queue and stops any motion
instantly (no deceleration; MOSFETs stay enabled) -- only send it at rest.
"""
import time
import servomotor

ALIAS = 'X'                             # Device alias; change if needed
SERIAL_PORT = "/dev/tty.usbserial-110"  # Change to your serial port (e.g. "COM3" on Windows).
                                        #  Set SERIAL_PORT = "MENU" to be prompted interactively.
servomotor.set_serial_port(SERIAL_PORT)
servomotor.open_serial_port()
motor = servomotor.M3(ALIAS, time_unit="seconds", position_unit="shaft_rotations",
                      velocity_unit="rotations_per_second",
                      acceleration_unit="rotations_per_second_squared", verbose=0)
try:
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset

    time.sleep(1.0)                     # let the clock accumulate a clearly nonzero count
    before_s = motor.get_current_time()
    print(f"Clock before reset: {before_s:.3f} s since system reset")

    motor.reset_time()                  # clock -> 0 (also empties the motion queue!)
    after_s = motor.get_current_time()
    print(f"Clock after reset:  {after_s:.6f} s")
    print("Second reading is near zero -- just the command round-trip time remains.")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
