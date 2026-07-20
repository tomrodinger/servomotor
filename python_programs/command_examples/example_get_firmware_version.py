#!/usr/bin/env python3
"""
Example: Get firmware version -- read the running firmware (or bootloader) version.
Prints the version as major.minor.patch.development plus the in-bootloader flag.
On a healthy device after reset the flag is 0 (main firmware answering).
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

    # Two outputs -> [versionBytes, inBootloader]. Version bytes arrive
    # least-significant-first: [development, patch, minor, major] -- reverse
    # them for the conventional major.minor.patch.dev display.
    version, in_bootloader = motor.get_firmware_version()
    dev, patch, minor, major = version
    print(f"Firmware version: {major}.{minor}.{patch}.{dev}")
    print(f"In bootloader:    {in_bootloader}")
    if in_bootloader:
        # The version above is then the BOOTLOADER's own version, not the application's.
        # Recover with another system_reset() followed by the 1.5 s silent wait.
        print("WARNING: device is in the bootloader; reported version is the bootloader's.")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
