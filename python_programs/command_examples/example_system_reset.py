#!/usr/bin/env python3
"""
Example: System reset -- return the motor to a known-clean power-on state.
Resets the device, honors the mandatory 1.5 s bus-silent window, then reads
'Get status' to confirm a clean baseline (application running, MOSFETs off,
no latched fatal error). Start every session this way.
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
    # 'System reset' restores the power-on state: MOSFETs disabled, default
    # limits and gains, position and clock zeroed. It is also the ONLY software
    # way to clear a latched fatal error, so it is both the first command of
    # every session and the recovery step whenever anything goes wrong.
    motor.system_reset()

    # After reset the device runs its bootloader for a short window (~250 ms)
    # before launching the application. ANY packet it hears in that window pins
    # it in the bootloader, where normal commands stop working. Keep the bus
    # completely silent for a full 1.5 s -- do not talk to ANY device on it.
    time.sleep(1.5)

    # Verify the clean baseline. get_status() returns [statusFlags, fatalErrorCode].
    status_flags, fatal_error_code = motor.get_status()
    in_bootloader = bool(status_flags & (1 << 0))    # bit 0: stuck in bootloader
    mosfets_enabled = bool(status_flags & (1 << 1))  # bit 1: MOSFETs enabled
    print(f"Status flags: 0b{status_flags:08b}")
    print(f"  In bootloader:    {in_bootloader} (expected: False)")
    print(f"  MOSFETs enabled:  {mosfets_enabled} (expected: False)")
    print(f"Fatal error code:   {fatal_error_code} (expected: 0)")
    if not in_bootloader and not mosfets_enabled and fatal_error_code == 0:
        print("Clean post-reset baseline confirmed.")
    else:
        # A packet leaked onto the bus during the silent window; reset again
        # and wait the full delay.
        print("Unexpected state -- send system_reset() again and wait 1.5 s.")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
