#!/usr/bin/env python3
"""
Example: Set PID constants -- tune the closed-loop controller gains.
Enters closed loop, applies the known-good M17 gains, then performs a small move
so you can observe the motor tracking under the freshly set constants.
"""
import time
import servomotor

ALIAS = 'X'                             # Device alias; change if needed
SERIAL_PORT = "/dev/tty.usbserial-110"  # Change to your serial port (e.g. "COM3" on Windows).
                                        #  Set SERIAL_PORT = "MENU" to be prompted interactively.
PID_P = 2000                            # known-good M17 proportional gain (raw fixed-point)
PID_I = 5                               # known-good M17 integral gain
PID_D = 175000                          # known-good M17 derivative gain; values below 32 are
                                        #  quantized to ZERO derivative action
servomotor.set_serial_port(SERIAL_PORT)
servomotor.open_serial_port()
motor = servomotor.M3(ALIAS, time_unit="seconds", position_unit="shaft_rotations",
                      velocity_unit="rotations_per_second",
                      acceleration_unit="rotations_per_second_squared", verbose=0)
try:
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset
    motor.go_to_closed_loop()           # PID gains only act in closed loop. This enables the
                                        #  MOSFETs by itself; skipping a separate
                                        #  'Enable MOSFETs' may even give a gentler engagement.
    deadline = time.time() + 6.0
    while True:
        status_flags, fatal_error_code = motor.get_status()
        if status_flags & (1 << 2):     # bit 2 = closed-loop mode active
            break
        if time.time() > deadline:
            raise TimeoutError("Never entered closed loop -- is the motor calibrated?")
        time.sleep(0.1)
    time.sleep(0.3)                     # settle after the mode change so the zero is clean
    motor.zero_position()
    # The gains apply immediately (even mid-move) and are NOT validated; there is no
    # read-back command, so keep your own record of what you set. Nothing persists:
    # any reset or power cycle restores the firmware defaults.
    motor.set_pid_constants(PID_P, PID_I, PID_D)
    print(f"PID constants set: P={PID_P} I={PID_I} D={PID_D}")
    motor.trapezoid_move(1.0, 2.0)      # small safe move to watch it track: 1 rotation in 2 s
    while motor.get_n_queued_items() > 0:
        time.sleep(0.05)                # poll the queue; never busy-poll without a sleep
    time.sleep(0.2)                     # brief mechanical settling before reading position
    print(f"Move complete under new gains; position = {motor.get_position():.3f} shaft rotations")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
