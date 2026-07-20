#!/usr/bin/env python3
"""
Example: Get current time -- read the device's absolute microsecond clock.
Reads the clock twice, one second apart, and shows it advancing in step with
the host. The clock counts up from power-on or the last 'Reset time'.
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

    t1_s = motor.get_current_time()     # in the configured time unit (seconds here)
    print(f"Device clock: {t1_s:.6f} s")

    time.sleep(1.0)                     # let both clocks advance by one host second

    t2_s = motor.get_current_time()
    print(f"Device clock: {t2_s:.6f} s")
    print(f"Elapsed on device: {t2_s - t1_s:.3f} s (expect about 1.0 s)")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
