#!/usr/bin/env python3
"""
Example: Get hall sensor statistics -- read per-sensor max/min/sum/count and compute means.
Starts statistics gathering, samples for 1 second, then prints the statistics for each
of the three analog hall sensors. Steady values with modest max-min spread mean a
healthy, low-noise sensor system.
"""
import time
import servomotor

ALIAS = 'X'                             # Device alias; change if needed
SERIAL_PORT = "/dev/tty.usbserial-110"  # Change to your serial port (e.g. "COM3" on Windows).
                                        #  Set SERIAL_PORT = "MENU" to be prompted interactively.
GATHER_SECONDS = 1.0                    # How long to let statistics accumulate

servomotor.set_serial_port(SERIAL_PORT)
servomotor.open_serial_port()
motor = servomotor.M3(ALIAS, time_unit="seconds", position_unit="shaft_rotations",
                      velocity_unit="rotations_per_second",
                      acceleration_unit="rotations_per_second_squared", verbose=0)
try:
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset

    # Gathering is OFF after reset. Without starting it first, every field
    # (even the minimums) reads 0, which can be misread as dead sensors.
    motor.control_hall_sensor_statistics(1)      # 1 = reset AND start gathering
    time.sleep(GATHER_SECONDS)

    stats = motor.get_hall_sensor_statistics()   # reading neither stops nor resets
    max1, max2, max3, min1, min2, min3, sum1, sum2, sum3, count = stats
    print(f"Measurement count: {count}")
    # Each recorded value is the sum of 4 oversampled 12-bit ADC readings,
    # so the valid range is 0..16380 (not 0..4095). No averages are returned;
    # compute mean = sum / count yourself.
    for name, mx, mn, sm in (("Hall 1", max1, min1, sum1),
                             ("Hall 2", max2, min2, sum2),
                             ("Hall 3", max3, min3, sum3)):
        mean = sm / count if count else 0.0
        print(f"{name}: max={mx}  min={mn}  sum={sm}  mean={mean:.1f}  (ADC counts, 0..16380)")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
