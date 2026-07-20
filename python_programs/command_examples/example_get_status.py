#!/usr/bin/env python3
"""
Example: Get status -- read the motor's status flags and fatal error code.
Enables the MOSFETs so one live flag is set, then prints every flag bit by
name plus the fatal error code. Expect bit 1 SET, all others clear, error 0.
"""
import time
import servomotor

ALIAS = 'X'                             # Device alias; change if needed
SERIAL_PORT = "/dev/tty.usbserial-110"  # Change to your serial port (e.g. "COM3" on Windows).
                                        #  Set SERIAL_PORT = "MENU" to be prompted interactively.
FLAG_NAMES = [
    (0, "In bootloader (if set, all other bits read 0)"),
    (1, "MOSFETs enabled"),
    (2, "Closed loop mode"),
    (3, "Calibration in progress"),
    (4, "Homing in progress"),
    (5, "Going to closed loop (only ever set on M1 products)"),
    (6, "Busy with a long-running task"),
]

servomotor.set_serial_port(SERIAL_PORT)
servomotor.open_serial_port()
motor = servomotor.M3(ALIAS, time_unit="seconds", position_unit="shaft_rotations",
                      velocity_unit="rotations_per_second",
                      acceleration_unit="rotations_per_second_squared", verbose=0)
try:
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset

    motor.enable_mosfets()              # sets status bit 1 so the report shows a live flag
    time.sleep(0.3)                     # let the rotor settle after energizing

    flags, fatal_error_code = motor.get_status()   # returns [statusFlags, fatalErrorCode]
    print(f"Raw status flags: 0b{flags:07b}")
    for bit, name in FLAG_NAMES:
        state = "SET" if (flags >> bit) & 1 else "clear"
        print(f"  Bit {bit}  {name:<48} {state}")
    print(f"Fatal error code: {fatal_error_code} (0 = no fatal error)")
    # 'Get status' is one of only two commands still answered after a fatal error
    # (the other is 'System reset'). In the fatal-error state the flags are
    # cleared to 0 (only the in-bootloader bit is kept), so use the fatal error code for diagnosis. On any
    # nonzero code, recover with system_reset() followed by a 1.5 s bus-silent wait.
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
