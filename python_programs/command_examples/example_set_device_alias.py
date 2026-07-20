#!/usr/bin/env python3
"""
Example: Set device alias -- change a device's one-byte alias from 'X' to 'Y' and back.
The device saves the new alias to flash and then reboots itself; each change is
verified by pinging the device at its new alias.
"""
import os
import time
import servomotor

ALIAS = 'X'                             # Current device alias; change if needed
NEW_ALIAS = 'Y'                         # Temporary alias for this demo. Valid aliases: 0-251
                                        #  (252-254 are reserved; 255 removes the alias)
REBOOT_WAIT = 1.0                       # s: flash write + automatic reboot after the command.
                                        #  Bench measurement showed the device answers at the
                                        #  new alias in application mode after ~0.3 s;
                                        #  0.5-1 s gives margin.
SERIAL_PORT = "/dev/tty.usbserial-110"  # Change to your serial port (e.g. "COM3" on Windows).
                                        #  Set SERIAL_PORT = "MENU" to be prompted interactively.
servomotor.set_serial_port(SERIAL_PORT)
servomotor.open_serial_port()
motor = servomotor.M3(ALIAS, time_unit="seconds", position_unit="shaft_rotations",
                      velocity_unit="rotations_per_second",
                      acceleration_unit="rotations_per_second_squared", verbose=0)

def verify_alias(alias):
    # A ping that echoes our exact bytes proves the device now answers at this alias.
    # use_this_alias_or_unique_id() takes the raw one-byte alias VALUE (it does not
    # convert strings the way the M3 constructor does), hence ord().
    motor.use_this_alias_or_unique_id(ord(alias))
    payload = os.urandom(10)            # 'Ping' requires exactly 10 bytes
    echoed = motor.ping(payload)
    assert bytes(echoed) == payload, "ping echo mismatch"
    print(f"Device responds correctly at alias '{alias}'")

try:
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset

    print(f"Changing alias '{ALIAS}' -> '{NEW_ALIAS}' ...")
    motor.set_device_alias(NEW_ALIAS)
    # The device sends the success response first, then writes the alias to flash and
    # REBOOTS itself. Keep the bus silent until it is back: bench measurement showed
    # it answering at the new alias in application mode after ~0.3 s, so 0.5-1 s of
    # waiting gives margin.
    time.sleep(REBOOT_WAIT)
    verify_alias(NEW_ALIAS)

    print(f"Changing alias back '{NEW_ALIAS}' -> '{ALIAS}' ...")
    motor.set_device_alias(ALIAS)
    time.sleep(REBOOT_WAIT)             # same flash-write + reboot wait as above
    verify_alias(ALIAS)
    print("Done: alias restored to its original value.")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
