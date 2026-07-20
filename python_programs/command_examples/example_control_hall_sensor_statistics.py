#!/usr/bin/env python3
"""
Example: Control hall sensor statistics -- start and freeze hall sensor data gathering.
Sends 1 to reset-and-start statistics gathering, samples for 1 second, then sends 0
to freeze the snapshot. A read-back of the measurement count proves data was captured.
"""
import time
import servomotor

ALIAS = 'X'                             # Device alias; change if needed
GATHER_SECONDS = 1.0                    # How long to let statistics accumulate
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

    # Value 1 always RESETS the statistics (max=0, min=65535, sums=0, count=0)
    # AND starts gathering -- there is no way to start without resetting.
    # Note: broadcast (alias 255) does nothing for this command; use a real alias.
    motor.control_hall_sensor_statistics(1)
    print(f"Statistics reset and gathering started; sampling for {GATHER_SECONDS} s ...")
    time.sleep(GATHER_SECONDS)

    # Value 0 freezes the accumulated statistics so later reads see a stable
    # snapshot instead of a moving target. The data stays readable until the
    # next reset-and-start (or a system reset).
    motor.control_hall_sensor_statistics(0)
    print("Gathering frozen.")

    stats = motor.get_hall_sensor_statistics()   # last field is the measurement count
    print(f"Measurements captured in the {GATHER_SECONDS} s window: {stats[9]}")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
