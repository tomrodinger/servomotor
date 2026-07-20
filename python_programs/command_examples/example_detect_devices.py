#!/usr/bin/env python3
"""
Example: Detect devices -- discover every servomotor on the RS485 bus.
Broadcast discovery needs no alias. Prints each device's 64-bit unique ID
(hex) and current alias -- expect one summary line per connected motor
(the detection helper also prints its own progress log).
"""
import servomotor

N_DETECTIONS = 3                        # Detection rounds; results are merged across rounds
                                        #  because a single round can miss a device
SERIAL_PORT = "/dev/tty.usbserial-110"  # Change to your serial port (e.g. "COM3" on Windows).
                                        #  Set SERIAL_PORT = "MENU" to be prompted interactively.
servomotor.set_serial_port(SERIAL_PORT)
servomotor.open_serial_port()
try:
    # detect_devices_iteratively() runs the full discovery protocol for us. Each round it
    # broadcasts 'System reset' and keeps the bus silent for 1.5 s (the mandatory
    # post-reset wait), flushes stray bytes, then broadcasts 'Detect devices': every
    # device answers once with its unique ID and alias, each at a random 0-950 ms offset
    # to avoid collisions -- which is why one round can miss a device and several are run.
    devices = servomotor.detect_devices_iteratively(n_detections=N_DETECTIONS)

    # GOTCHA (handled by the helper): for ~1 s after answering 'Detect devices' a device
    # silently IGNORES all bus traffic (its collision-avoidance window). The helper waits
    # out that lockout and finishes with a reset, so the bus is clean when it returns.
    print(f"\nFound {len(devices)} device(s):")
    for dev in devices:
        if dev.alias == 255:
            alias_str = "255 (no alias assigned; address it by unique ID)"
        elif 33 <= dev.alias <= 126:
            alias_str = f"{dev.alias} (ASCII '{chr(dev.alias)}')"
        else:
            alias_str = str(dev.alias)
        print(f"  Unique ID: {dev.unique_id:016X}   Alias: {alias_str}")
finally:
    servomotor.close_serial_port()
