#!/usr/bin/env python3
"""
Example: Get product specs -- read the two constants that define the motor's motion math.
Prints the control-loop update frequency (Hz) and the position counts per one shaft
rotation. Expect e.g. 31250 Hz and 3276800 counts/rotation on an M17.
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

    # Two outputs -> the wrapper returns a flat list [updateFrequency, countsPerRotation].
    update_frequency, counts_per_rotation = motor.get_product_specs()

    # Always query these rather than hardcoding: countsPerRotation is product-specific
    # (3276800 on M3/M17/M23 but 639744 on M1, 4569600 on M2), and one motion "timestep"
    # is exactly 1/updateFrequency seconds.
    print(f"Update frequency:    {update_frequency} Hz")
    print(f"One timestep:        {1.0 / update_frequency * 1e6:.1f} microseconds")
    print(f"Counts per rotation: {counts_per_rotation} counts per shaft rotation")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
