#!/usr/bin/env python3
"""
Example: Read multipurpose buffer -- read out and clear the device's one-shot data buffer.
Enters closed loop, stores one PID debug snapshot in the buffer with test mode 3, then reads
the buffer back and prints the five PID values. Each dataset can be read exactly ONCE: a
successful read clears the buffer, and reading an empty buffer returns a single 0 byte (firmware 0.15.4.0 and later; older firmware sent nothing, so the read timed out).
"""
import time
import servomotor
from servomotor import communication

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

    # PID debug data is only meaningful in closed-loop mode, so enter it first.
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

    motor.test_mode(3)                  # store ONE PID debug snapshot into the buffer
    time.sleep(0.1)                     # give the control loop a moment to write it

    data = motor.read_multipurpose_buffer()
    # An empty buffer returns a single byte of 0 (the "nothing stored" tag) as of
    # firmware 0.15.4.0. (Older firmware sent NO response at all -- there, a
    # communication.TimeoutError meant "buffer empty".)
    if data[0] == 0:
        raise SystemExit("Multipurpose buffer was empty (no dataset stored)")

    print(f"Data type tag: {data[0]} (4 = PID debug snapshot)")
    for n, name in enumerate(["error", "P term", "I term", "D term", "output"]):
        value = int.from_bytes(data[1 + n * 4:5 + n * 4], "little", signed=True)
        print(f"  {name}: {value} (internal units)")

    # Clear the test mode with a reset. (As of firmware 0.15.4.0, test_mode(0)
    # also clears it safely; on OLDER firmware value 0 hangs the device, so a
    # reset is the safe way on any firmware version.)
    motor.system_reset()
    time.sleep(1.5)                     # bus must stay silent after reset
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
