#!/usr/bin/env python3
"""
Example: Get debug values -- read the motor's 30-field diagnostic snapshot.
Prints every value with its label: velocity/acceleration state, control-loop
profiler times, raw hall sensor voltages, commutation info, and PWM voltage.
"""
import time
import servomotor

ALIAS = 'X'                             # Device alias; change if needed
SERIAL_PORT = "/dev/tty.usbserial-110"  # Change to your serial port (e.g. "COM3" on Windows).
                                        #  Set SERIAL_PORT = "MENU" to be prompted interactively.

# The 30 output fields, in wire order. All values are raw internal units.
LABELS = [
    "maxAcceleration", "maxVelocity", "currentVelocity", "measuredVelocity",
    "nTimeSteps", "debugValue1", "debugValue2", "debugValue3", "debugValue4",
    "allMotorControlCalculationsProfilerTime", "allMotorControlCalculationsProfilerMaxTime",
    "getSensorPositionProfilerTime", "getSensorPositionProfilerMaxTime",
    "computeVelocityProfilerTime", "computeVelocityProfilerMaxTime",
    "motorMovementCalculationsProfilerTime", "motorMovementCalculationsProfilerMaxTime",
    "motorPhaseCalculationsProfilerTime", "motorPhaseCalculationsProfilerMaxTime",
    "motorControlLoopPeriodProfilerTime", "motorControlLoopPeriodProfilerMaxTime",
    "hallSensor1Voltage", "hallSensor2Voltage", "hallSensor3Voltage",
    "commutationPositionOffset", "motorPhasesReversed",
    "maxHallPositionDelta", "minHallPositionDelta", "averageHallPositionDelta",
    "motorPwmVoltage",
]

servomotor.set_serial_port(SERIAL_PORT)
servomotor.open_serial_port()
motor = servomotor.M3(ALIAS, time_unit="seconds", position_unit="shaft_rotations",
                      velocity_unit="rotations_per_second",
                      acceleration_unit="rotations_per_second_squared", verbose=0)
try:
    motor.system_reset()
    time.sleep(1.5)                     # mandatory: keep the bus silent after reset

    # CAUTION: reading resets some fields (the profiler max-times and the hall
    # position delta min/max/average), so those cover only the window since the
    # previous read -- they are NOT cumulative or monotonic. Hall-delta sentinels
    # -2000000000 / +2000000000 mean "no samples accumulated since the last read".
    values = motor.get_debug_values()   # multi-output command -> flat list of 30 values
    print(f"Received {len(values)} debug values (raw internal units):")
    for label, value in zip(LABELS, values):
        print(f"  {label:44s} = {value}")
finally:
    try:
        motor.disable_mosfets()
    except Exception:
        pass
    servomotor.close_serial_port()
