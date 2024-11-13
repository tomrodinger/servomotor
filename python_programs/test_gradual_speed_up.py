#!/usr/bin/env python3
import servomotor
import time


SPEED_STEP = 0.5
STEP_TIME = 0.5
MAX_VELOCITY = 200.0

MOVE_DISPLACEMENT_MOTOR_UNITS_PER_ROTATION = 4320000
MOVE_TIME_MOTOR_UNITS_PER_SECOND = 31250

motorX = servomotor.M3("X", motor_type="M3", time_unit="seconds", position_unit="degrees", velocity_unit="degrees/s", acceleration_unit="degree/s^2", current_unit="mA", voltage_unit="V", temperature_unit="C", verbose=True)

servomotor.open_serial_port()
motorX.system_reset()
time.sleep(1.2)
motorX.enable_mosfets()
communication_velocity = 10000000
move_time_motor_units = 16000

current_speed = 0.0
while 1:
    print(f"=== speed = {current_speed} =============================================================================")
    current_speed += SPEED_STEP

    temperature = motorX.get_temperature()
    print(f"Temperature: {temperature}")

    while 1:
        n_queued_items = motorX.get_n_queued_items()
        print(f"Queue items: {n_queued_items}")
        if n_queued_items >= 32:
            print("The queue is full, so we will wait for a moment")
            time.sleep(0.5)
        else:
            break

    status = motorX.get_status()
    print(f"Status: {status}")
    if len(status) == 2 and status[1] != 0:
        print(f"A fatal error occured with the motor. The error code is: {status[1]}")
        break

    #move with velocity, convert to motor units and add it to the list
    internal_velocity = int(current_speed / 60 * (float(MOVE_DISPLACEMENT_MOTOR_UNITS_PER_ROTATION) / float(MOVE_TIME_MOTOR_UNITS_PER_SECOND)) * float(1 << 32))
    print(f"Internal velocity: {internal_velocity}")
    communication_velocity = int(internal_velocity) >> 12
    print(f"Communication velocity for move command: {communication_velocity}")

    #after the velocity or acceleretion was added, add the time
    move_time_motor_units = int(STEP_TIME * MOVE_TIME_MOTOR_UNITS_PER_SECOND)
    print(f"Timesteps: {move_time_motor_units}")

    motorX.move_with_velocity(velocity=communication_velocity, time=move_time_motor_units)
    if current_speed > MAX_VELOCITY:
        print("Reached maximum velocity")
        break

servomotor.close_serial_port()
del motorX

