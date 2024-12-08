#!/usr/bin/env python3

import random
import struct
import argparse
import time
import zlib
import math
import numpy as np
import servomotor

X_ALIAS = ord('X')
Y_ALIAS = ord('Y')
Z_ALIAS = ord('Z')
GLUE_DISPENSE_ALIAS = ord('G')
ALL_MOTORS_ALIAS = 255

PLUNGER_ENABLED = False
TIME_SYNC_PERIOD = 0.1
TIME_SYNC_PRINT_STATISTICS_PERIOD = 1.0
REQUIRED_SUCCESSFUL_PINGS = 30
X_Y_MOTOR_CURRENT_SETTING = 50
Z_MOTOR_CURRENT_SETTING = 100
X_Y_REDUCTION_RATIO = 28 / 14
Z_REDUCTION_RATIO = 18.0 / 14
PULLEY_N_TEETH = 14
BELT_TOOTH_PITCH = 2.0
MM_PER_PULLEY_ROTATION = BELT_TOOTH_PITCH * PULLEY_N_TEETH / X_Y_REDUCTION_RATIO
X_Y_SPAN_MM = 227.0
X_Y_SPAN_ROTATIONS = X_Y_SPAN_MM / MM_PER_PULLEY_ROTATION * X_Y_REDUCTION_RATIO
Z_SPAN_MM = 120.0
Z_LEAD_SCREW_MM_PER_ROTATION = 4.0 / Z_REDUCTION_RATIO
Z_SPAN_ROTATIONS = Z_SPAN_MM / Z_LEAD_SCREW_MM_PER_ROTATION
INTERNAL_TIME_UNIT_HZ = 64000000 / 2048
HOMING_MAX_TIME_S = 10.0
GLUE_CIRCLE1_DIAMETER = 24.0
GLUE_CIRCLE_TIME = 20.0
HOPPING = True
N_HOPS = 40
HOP_TIME_SECONDS = GLUE_CIRCLE_TIME / N_HOPS
GLUE_DISPENSE_VOLUME_MM_CUBED_PER_STATOR = 800.0
N_CIRCLES = 3
N_CIRCLES_DISPENSING_GLUE = N_CIRCLES - 1
GLUE_DISPENSE_VOLUME_MM_CUBED_PER_CIRCLE = GLUE_DISPENSE_VOLUME_MM_CUBED_PER_STATOR / N_CIRCLES_DISPENSING_GLUE
X_START_POSITION_MM = 6.0 + GLUE_CIRCLE1_DIAMETER
X_GO_TO_START_TIME = 2.0
Y_START_POSITION_MM = 6.0 + GLUE_CIRCLE1_DIAMETER / 2
Y_GO_TO_START_TIME = 2.0
Z_DISPENSE_POSITION_MM = 90.0 # the higher is this number, the further up will be the machine base with the stators, hence the closer will be the glue dispensing to the stators
Z_GO_TO_DISPENSE_POSITION_TIME = 1.0
Z_START_POSITION_MM = Z_DISPENSE_POSITION_MM - 20.0 # we will move the stators down 20mm when moving the glue dispenser to the next unit
Z_GO_TO_START_TIME = 4.0
QUEUE_SIZE = 32
DISTANCE_BETWEEN_UNITS = 61.0
GLUE_DOT_HOP_HEIGHT_MM = 4.0
PLUNGER_SPAN_MM = 94.0
PLUNGER_PULLEY_N_TEETH = 14
PLUNGER_MM_PER_PULLEY_ROTATION = BELT_TOOTH_PITCH * PLUNGER_PULLEY_N_TEETH
PLUNGER_SPAN_ROTATIONS = PLUNGER_SPAN_MM / PLUNGER_MM_PER_PULLEY_ROTATION
DETECT_PLUNGER_POSITION_MOTOR_CURRENT_SETTING = 40
DETECT_PLUNGER_POSITION_HOMING_MAX_TIME_S = 5
PLUNGER_MOTOR_CURRENT_SETTING = 390
PLUNGER_CROSS_SECTION_AREA = math.pi * (19.0 / 2.0) ** 2
GLUE_N_PLUNGERS = 2 # we are mixing two parts of glue to make the finished glue, which will be dispensed
GLUE_DISPENSE_VOLUME_PER_PLUNGER_MM = PLUNGER_CROSS_SECTION_AREA * 1.0
PLUNGER_MOVE_DISTANCE_MM_PER_CIRCLE = GLUE_DISPENSE_VOLUME_MM_CUBED_PER_CIRCLE / GLUE_DISPENSE_VOLUME_PER_PLUNGER_MM
PLUNGER_SHAFT_ROTATIONS_PER_CIRCLE = PLUNGER_MOVE_DISTANCE_MM_PER_CIRCLE / PLUNGER_MM_PER_PULLEY_ROTATION
PLUNGER_VELOCITY_ROTATIONS_PER_SECOND = PLUNGER_SHAFT_ROTATIONS_PER_CIRCLE / GLUE_CIRCLE_TIME
PLUNGER_MOVE_DISTANCE_PER_GLUE_DOT_SHAFT_ROTATIONS = PLUNGER_SHAFT_ROTATIONS_PER_CIRCLE / N_HOPS
PLUNGER_PULL_BACK_MM_TO_STOP_DISPENSING = -10.0
PLUNGER_PULL_BACK_TIME = 0.2 * 10


POSITIONS = [
    [0,                          0],
    [DISTANCE_BETWEEN_UNITS,     0],
    [DISTANCE_BETWEEN_UNITS * 2, 0],
    [DISTANCE_BETWEEN_UNITS * 3, 0],
    [DISTANCE_BETWEEN_UNITS * 3, DISTANCE_BETWEEN_UNITS],
    [DISTANCE_BETWEEN_UNITS * 2, DISTANCE_BETWEEN_UNITS],
    [DISTANCE_BETWEEN_UNITS,     DISTANCE_BETWEEN_UNITS],
    [0,                          DISTANCE_BETWEEN_UNITS],
    [0,                          DISTANCE_BETWEEN_UNITS * 2],
    [DISTANCE_BETWEEN_UNITS,     DISTANCE_BETWEEN_UNITS * 2],
    [DISTANCE_BETWEEN_UNITS * 2, DISTANCE_BETWEEN_UNITS * 2],
    [DISTANCE_BETWEEN_UNITS * 3, DISTANCE_BETWEEN_UNITS * 2],
    [DISTANCE_BETWEEN_UNITS * 3, DISTANCE_BETWEEN_UNITS * 3],
    [DISTANCE_BETWEEN_UNITS * 2, DISTANCE_BETWEEN_UNITS * 3],
    [DISTANCE_BETWEEN_UNITS,     DISTANCE_BETWEEN_UNITS * 3],
    [0,                          DISTANCE_BETWEEN_UNITS * 3],
]


def compute_crc32(data_64bit, data_8bit):
    # Convert to bytes in little-endian format
    data_bytes = data_64bit.to_bytes(8, 'little') + data_8bit.to_bytes(1, 'little')
    
    # Compute the CRC32
    crc32_value = zlib.crc32(data_bytes)
    return crc32_value


def check_device_with_ping(motor, required_successful_pings):
    for ping_number in range(required_successful_pings):
        try:
            # Let's generate a random byte array with 10 bytes and send it to the device
            random_10_bytes = bytes([random.randint(0, 255) for _ in range(10)])
            response = motor.ping(random_10_bytes)
            if response == random_10_bytes:
                print(f"Ping {ping_number + 1} successful")
            else:
                print(f"Error: Ping {ping_number + 1} failed. Expected response: {random_10_bytes}, but got: {response}")
                exit(1)
        except Exception as e:
            print(f"Communication error: {e}")
            print("Could not ping the device. Please check it. Exiting.")
            exit(1)
    print(f"Successfully pinged the device with alias {motor.alias} {required_successful_pings} times")


def do_homing(motor, motor_current_setting, max_displacement, max_time):
    human_readable_alias = servomotor.get_human_readable_alias(motor.alias)
    print(f"Setting the current of the motor with alias {human_readable_alias} to {motor_current_setting} (which is some arbitrary unit)")
    response = motor.set_maximum_motor_current(motor_current_setting, motor_current_setting)
    print("Response:", response)
    print("Going into closed loop mode now")
    response = motor.go_to_closed_loop()
    print("Response:", response)
#    motor.set_position_unit("shaft_rotations")
#    motor.set_time_unit("seconds")
    print(f"Homing the motor with alias {human_readable_alias}. The maximum displacement is {max_displacement} and the maximum time is {max_time} seconds")
    response = motor.homing(max_displacement, max_time)
    print("Response:", response)
    # wait for the motor to finish homing
    while True:
        response = motor.get_status()
        print("The motor status is", response[0], "and the fatal error code is", response[1])
        # extract bit number 3 from the motor status byte, which is the homing bit
        homing = (response[0] & 0b00010000) != 0
        if not homing:
            break
        time.sleep(0.2)
    print("The motor has finished homing")


def compute_half_ellipse_trajectory(start_point, end_point, height, move_vector, n_points, t_total):
    """
    Compute velocities and times for a half-ellipse trajectory between two points in 3D space.
    """
    start_point = np.array(start_point, dtype=float)
    end_point = np.array(end_point, dtype=float)
    move_vector = np.array(move_vector, dtype=float)
    
    # Vector from start to end
    D = end_point - start_point
    L = np.linalg.norm(D)
    if L == 0:
        raise ValueError("Start point and end point cannot be the same.")
    U = D / L  # Unit vector from start to end
    
    # Compute normal vector to the plane
    N = np.cross(D, move_vector)
    N_norm = np.linalg.norm(N)
    if N_norm == 0:
        raise ValueError("Move vector cannot be parallel to the direction vector from start to end.")
    W = N / N_norm  # Unit normal vector to the plane
    
    # V_axis lies in the plane and is perpendicular to U
    V_axis = np.cross(W, U)
    
    # Semi-major and semi-minor axes
    a = L / 2
    b = height
    
    # Parameter theta from 0 to pi
    theta = np.linspace(0, np.pi, n_points)
    
    # Compute ellipse points in plane coordinates
    u = a * (1 - np.cos(theta))  # From 0 to 2a
    v = b * np.sin(theta)
    
    # Compute points along the trajectory
    trajectory_points = []
    for ui, vi in zip(u, v):
        point = start_point + U * ui + V_axis * vi
        trajectory_points.append(point)
    
    # Compute distances between points
    distances = [np.linalg.norm(trajectory_points[i+1] - trajectory_points[i]) 
                 for i in range(len(trajectory_points)-1)]
    total_distance = sum(distances)
    
    # Compute times for each segment
    times = [(dist / total_distance) * t_total for dist in distances]
    
    # Compute velocities for each segment
    velocities = [(trajectory_points[i+1] - trajectory_points[i]) / times[i] 
                  for i in range(len(times))]
    
    # Append zero velocity and minimal time for the last point
    velocities.append(np.array([0.0, 0.0, 0.0]))
    times.append(0.00001)
    
    return velocities, times, trajectory_points


class TimeSync:
    def __init__(self, motors):
        self.motors = motors
        self.master_start_time = time.time()
        # Reset the time of all motors
        for motor in motors:
            response = motor.reset_time()
        self.next_time_sync = self.master_start_time + TIME_SYNC_PERIOD
        self.max_time_error = 0.0
        self.next_time_sync_print_statistics_time = self.master_start_time + TIME_SYNC_PRINT_STATISTICS_PERIOD
        

    def logic(self):
        if time.time() >= self.next_time_sync:
            current_time = time.time() - self.master_start_time
            current_time_us = int(current_time * 1000000)
            for motor in self.motors:
                response = motor.time_sync(current_time_us)
                time_error_us = abs(response[0])
                print(f"The time error for alias {motor.alias} is {time_error_us} microseconds")
                if time_error_us > self.max_time_error:
                    self.max_time_error = time_error_us
            self.next_time_sync = time.time() + TIME_SYNC_PERIOD
        if time.time() >= self.next_time_sync_print_statistics_time:
            print("=============================================================================================================================")
            print("Time sync statistics:")
            for motor in self.motors:
                print(f"   max error = {self.max_time_error} microseconds")
            print("=============================================================================================================================")
            self.next_time_sync_print_statistics_time = time.time() + TIME_SYNC_PRINT_STATISTICS_PERIOD


def wait_for_queue_space(motors):
    while True:
        queue_sizes = []
        for motor in motors:
            n_queued_items = motor.get_n_queued_items()
            queue_sizes.append(n_queued_items)
        if all(queue_size < QUEUE_SIZE for queue_size in queue_sizes):
            break
        time.sleep(0.1)


def wait_queue_empty(motors):
    while True:
        queue_sizes = []
        for motor in motors:
            n_queud_items = motor.get_n_queued_items()
            queue_sizes.append(n_queud_items)
        if all(queue_size == 0 for queue_size in queue_sizes):
            break
        time.sleep(0.1)


def enable_or_disable_glue_dispenser(motor, enable):
    # check if the enabled flag exists in that motor class
    if not hasattr(motor, "enabled"):
        motor.enabled = True

    if motor.enabled == False and enable:
        print("Enabling glue dispenser")
        response = motor.enable_mosfets()
        motor.enabled = True
    elif motor.enabled == True and not enable:
        print("Disabling glue dispenser")
        response = motor.disable_mosfets()
        motor.enabled = False


# Define the arguments for this program. This program takes in an optional -p option to specify the serial port device
parser = argparse.ArgumentParser(description='Add some random moves to the queue to test the calculations of the safety limits')
parser.add_argument('-p', '--port', help='serial port device', default=None)
parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
parser.add_argument('-v', '--verbose', help='print verbose messages', action='store_true')
args = parser.parse_args()

servomotor.set_serial_port_from_args(args)

motor255 = servomotor.M3(ALL_MOTORS_ALIAS, verbose=args.verbose)
motorX = servomotor.M3(X_ALIAS, verbose=args.verbose)
motorY = servomotor.M3(Y_ALIAS, verbose=args.verbose)
motorZ = servomotor.M3(Z_ALIAS, verbose=args.verbose)
motorG = servomotor.M3(GLUE_DISPENSE_ALIAS, verbose=args.verbose)
servomotor.open_serial_port()

# Reset all devices
motor255.system_reset()
time.sleep(1.5)

# Let's ping all devices the required number of times to ensure communication reliability
check_device_with_ping(motorX, REQUIRED_SUCCESSFUL_PINGS)
check_device_with_ping(motorY, REQUIRED_SUCCESSFUL_PINGS)
check_device_with_ping(motorZ, REQUIRED_SUCCESSFUL_PINGS)
check_device_with_ping(motorG, REQUIRED_SUCCESSFUL_PINGS)

# Let's now set the motor current and do homing on all the axes
do_homing(motorZ, Z_MOTOR_CURRENT_SETTING,   -Z_SPAN_ROTATIONS + 1,      HOMING_MAX_TIME_S)
do_homing(motorX, X_Y_MOTOR_CURRENT_SETTING, -X_Y_SPAN_ROTATIONS + 0.25, HOMING_MAX_TIME_S)
do_homing(motorY, X_Y_MOTOR_CURRENT_SETTING, -X_Y_SPAN_ROTATIONS + 0.25, HOMING_MAX_TIME_S)

# Let's zero all the positions
response = motorX.zero_position()
response = motorY.zero_position()
response = motorZ.zero_position()

response = motorZ.go_to_position(Z_START_POSITION_MM / Z_LEAD_SCREW_MM_PER_ROTATION, Z_GO_TO_START_TIME)
wait_queue_empty([motorZ])


# Last is to move the plunger to the place where the glue starts, which is where there is resistance
if PLUNGER_ENABLED:
    do_homing(motorG, DETECT_PLUNGER_POSITION_MOTOR_CURRENT_SETTING, PLUNGER_SPAN_ROTATIONS, DETECT_PLUNGER_POSITION_HOMING_MAX_TIME_S)
    response = motorG.zero_position()
    print(f"Setting the current of the motor with alias {motorG.alias} to {PLUNGER_MOTOR_CURRENT_SETTING} (which is some arbitrary unit)")
    response = motorG.set_maximum_motor_current(PLUNGER_MOTOR_CURRENT_SETTING, PLUNGER_MOTOR_CURRENT_SETTING)
    print("Response:", response)

time.sleep(5)

time_sync = TimeSync([motorX, motorY, motorZ, motorG])

log_fh = open("glue_dispenser_comprehensive_positions.log", "w")


for position_x, position_y in POSITIONS:
    # Let's move the axes to the start position
    position_x = position_x + X_START_POSITION_MM
    position_y = position_y + Y_START_POSITION_MM
    response = motorX.go_to_position(position_x / MM_PER_PULLEY_ROTATION, X_GO_TO_START_TIME)
    response = motorY.go_to_position(position_y / MM_PER_PULLEY_ROTATION, Y_GO_TO_START_TIME)
    # Wait until the queues for X and Y and Z are all empty
    while 1:
        n_queud_items_X = motorX.get_n_queued_items()
        n_queud_items_Y = motorY.get_n_queued_items()
        n_queud_items_Z = motorZ.get_n_queued_items()
        if n_queud_items_X == 0 and n_queud_items_Y == 0 and n_queud_items_Z == 0:
            break
        time.sleep(0.1)

    # Move Z such that we are ready to dispense glue
    response = motorZ.go_to_position(Z_DISPENSE_POSITION_MM / Z_LEAD_SCREW_MM_PER_ROTATION, Z_GO_TO_DISPENSE_POSITION_TIME)

    # Now we need to move the dispenser in a circular fashion. We will queue up a bunch of "Move with velocity commnads" to do this
    n_circle_points = N_HOPS
    circle_diameter = 24.0
    time_per_circle = GLUE_CIRCLE_TIME
    time_per_circle_step = time_per_circle / n_circle_points
    angle_step_radians = 2 * math.pi / n_circle_points
    for circle_counter in range(N_CIRCLES):
        x_velocities_converted = []
        y_velocities_converted = []
        z_velocities_converted = []
        glue_velocities = []
        move_times = []
        for i in range(n_circle_points):
            angle = i * angle_step_radians
            next_angle = angle + angle_step_radians
            x = circle_diameter * 0.5 * (math.cos(angle) + 1)
            y = circle_diameter * 0.5 * (math.sin(angle) + 1)
            next_x = circle_diameter * 0.5 * (math.cos(next_angle) + 1)
            next_y = circle_diameter * 0.5 * (math.sin(next_angle) + 1)
            if HOPPING:
                trajectory_velocities, _move_times, trajectory_points = compute_half_ellipse_trajectory([x, y, 0], [next_x, next_y, 0], GLUE_DOT_HOP_HEIGHT_MM, [0, 0, -1], 10, HOP_TIME_SECONDS)
                move_times.extend(_move_times)
                glue_velocity = PLUNGER_VELOCITY_ROTATIONS_PER_SECOND
                # Let's convert all the velocities into the right unit for the motors
                for velocity in trajectory_velocities:
                    x_velocity_mm_per_s = velocity[0]
                    y_velocity_mm_per_s = velocity[1]
                    z_velocity_mm_per_s = velocity[2]
                    # and convert the velocity from mm/s to microsteps per motor time unit
                    x_velocities_converted.append(x_velocity_mm_per_s / MM_PER_PULLEY_ROTATION)
                    y_velocities_converted.append(y_velocity_mm_per_s / MM_PER_PULLEY_ROTATION)
                    z_velocities_converted.append(z_velocity_mm_per_s / Z_LEAD_SCREW_MM_PER_ROTATION)
                    if circle_counter < N_CIRCLES_DISPENSING_GLUE:
                        glue_velocities.append(glue_velocity)
                    else:
                        glue_velocities.append(0.0)
            else:
                delta_x = next_x - x
                delta_y = next_y - y
                x_velocity_mm_per_s = delta_x / time_per_circle_step
                y_velocity_mm_per_s = delta_y / time_per_circle_step
                # and convert the velocity from mm/s to microsteps per motor time unit
                x_velocity_converted = x_velocity_mm_per_s / MM_PER_PULLEY_ROTATION
                y_velocity_converted = y_velocity_mm_per_s / MM_PER_PULLEY_ROTATION
                glue_velocity = PLUNGER_MOVE_DISTANCE_PER_GLUE_DOT_SHAFT_ROTATIONS
                x_velocities_converted.append(x_velocity_converted)
                y_velocities_converted.append(y_velocity_converted)
                z_velocities_converted.append(0.0)
                if circle_counter < N_CIRCLES_DISPENSING_GLUE:
                    glue_velocities.append(glue_velocity)
                else:
                    glue_velocities.append(0.0)
                move_times.append(time_per_circle_step)
        if circle_counter >= N_CIRCLES - 1:  # stop the motion at the end of doing all circle
            x_velocities_converted.append(0.0)
            y_velocities_converted.append(0.0)
            z_velocities_converted.append(0.0)
            glue_velocities.append(0.0)
            move_times.append(0.001)
        # Let's iterate through all the moves and execute them
        for x_velocity_converted, y_velocity_converted, z_velocity_converted, glue_velocity, move_time in zip(x_velocities_converted, y_velocities_converted, z_velocities_converted, glue_velocities, move_times):
            # Let's check how many queued items there are and if the queue is full then we should wait for it to get space
            wait_for_queue_space([motorX, motorY, motorZ, motorG])
            response = motorX.move_with_velocity(x_velocity_converted, move_time)
            response = motorY.move_with_velocity(y_velocity_converted, move_time)
            response = motorZ.move_with_velocity(z_velocity_converted, move_time)
            if PLUNGER_ENABLED:
                response = motorG.move_with_velocity(glue_velocity, move_time)
                if glue_velocity != 0.0:
                    enable_or_disable_glue_dispenser(motorG, True)
                else:
                    enable_or_disable_glue_dispenser(motorG, False)
                # Get the comprehensive positions of the glue dispenser motor, to see if we dispensed the expected amount of glue. Log it to a file
                response = motorG.get_comprehensive_position()
                # Log all the data that is returned in the response to the log file, one item per column
                log_fh.write(" ".join([str(item) for item in response]) + "\n")
                time_sync.logic()

    wait_queue_empty([motorX, motorY, motorZ])

    response = motorZ.go_to_position(Z_START_POSITION_MM / Z_LEAD_SCREW_MM_PER_ROTATION, Z_GO_TO_DISPENSE_POSITION_TIME)

    time.sleep(Z_GO_TO_DISPENSE_POSITION_TIME * 1.1)

log_fh.close()

servomotor.close_serial_port()
del motorX
del motorY
del motorZ
del motorG
