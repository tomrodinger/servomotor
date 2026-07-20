#!/usr/bin/env python3
"""
Example: Test mode -- trigger the one generally useful and safe test mode, value 3.
Test mode 3 captures a single PID debug snapshot (error, P, I, D, output) into the
multipurpose buffer, which we read back and print. Read the DANGER notes below before
experimenting with any other value.
"""
import time
import servomotor

ALIAS = 'X'                             # Device alias; change if needed
SERIAL_PORT = "/dev/tty.usbserial-110"  # Change to your serial port (e.g. "COM3" on Windows).
                                        #  Set SERIAL_PORT = "MENU" to be prompted interactively.
TEST_MODE_PID_SNAPSHOT = 3              # the only test mode demonstrated here
# DANGER -- never send these test mode values:
#   10-13       : the firmware hangs FOREVER; only a power cycle recovers the device
#   0           : safe on firmware >= 0.15.4.0 (clears any motor/overtemperature test mode); hangs forever on OLDER firmware
#   14-73       : each one deliberately latches a fatal error (error code = value - 14)
# To clear an active test mode, system_reset() works on every firmware version;
# test_mode(0) also clears it safely on firmware >= 0.15.4.0 (on older firmware it hangs the device).

servomotor.set_serial_port(SERIAL_PORT)
servomotor.open_serial_port()
motor = servomotor.M3(ALIAS, time_unit="seconds", position_unit="shaft_rotations",
                      velocity_unit="rotations_per_second",
                      acceleration_unit="rotations_per_second_squared", verbose=0)
try:
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset

    # The PID snapshot is only meaningful in closed-loop mode, so enter it first.
    # go_to_closed_loop() enables the MOSFETs by itself; skipping a separate
    # 'Enable MOSFETs' may even give a gentler engagement.
    motor.go_to_closed_loop()
    deadline = time.time() + 6.0
    while True:                         # poll until the closed-loop bit (bit 2) sets
        status_flags, fatal_error_code = motor.get_status()
        if status_flags & 0b100:
            break
        if time.time() > deadline:
            raise SystemExit("Never entered closed loop -- has this motor been calibrated?")
        time.sleep(0.1)
    time.sleep(0.3)

    motor.test_mode(TEST_MODE_PID_SNAPSHOT)
    time.sleep(0.1)                     # give the control loop a moment to store the snapshot

    data = motor.read_multipurpose_buffer()
    if data[0] == 0:                    # a single 0 byte = buffer empty (fw 0.15.4.0+;
        print("Multipurpose buffer was empty (no snapshot stored)")  # older firmware timed out instead)
    else:
        print(f"Data type tag: {data[0]} (4 = PID debug snapshot)")
        for n, name in enumerate(["error", "P term", "I term", "D term", "output"]):
            value = int.from_bytes(data[1 + n * 4:5 + n * 4], "little", signed=True)
            print(f"  {name}: {value} (internal units)")

    # Clear the test mode with a reset. (As of firmware 0.15.4.0, test_mode(0) also
    # clears it safely; on OLDER firmware value 0 hangs the device until power cycle.)
    motor.system_reset()
    time.sleep(1.5)                     # bus must stay silent after reset
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
