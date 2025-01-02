#!/usr/bin/env python3

import time
from getch import getch

QUEUE_SIZE = 32
MAXIMUM_GLUE_PURGE_TIME = 60 * 5  # 5 minutes
MIN_MOVE_TIME_S = 1.0/(64000000/2048)*10 # DEBUG adding a * 10 factor to make the move time longer to help debug a problem
PLUNGER_VELOCITY_ROTATIONS_PER_SECOND = 1000

def wait_for_queue_space(motors):
    min_n_queued_items = QUEUE_SIZE
    while True:
        max_n_queued_items = 0
        for motor in motors:
            n_queued_items = motor.get_n_queued_items()
            if n_queued_items < min_n_queued_items:
                min_n_queued_items = n_queued_items
            if n_queued_items > max_n_queued_items:
                max_n_queued_items = n_queued_items
        if max_n_queued_items < QUEUE_SIZE:
            break
        time.sleep(0.01)
    return min_n_queued_items, max_n_queued_items


def wait_queue_empty(motors):
    print("Waiting for the queue to empty")
    while True:
        queue_sizes = []
        for motor in motors:
            n_queud_items = motor.get_n_queued_items()
            queue_sizes.append(n_queud_items)
        print(f"Queue sizes: {queue_sizes}")
        if all(queue_size == 0 for queue_size in queue_sizes):
            break
        time.sleep(0.1)
    print("The queue is empty now")


class TestMotor:
    def move_with_velocity(self, velocity, time):
        print(f"Moving with velocity {velocity} for {time} seconds")
        pass
    def get_n_queued_items(self):
        return 4


def enable_or_disable_glue_dispenser(motor, enable):
    print(f"{'Enabling' if enable else 'Disabling'} glue dispenser")
    pass

motorG = TestMotor()

purge_start_time = time.time()
try:
    while True:
        if time.time() - purge_start_time > MAXIMUM_GLUE_PURGE_TIME:
            print("Error: Maximum glue purge time exceeded. Exiting.")
            exit(1)
        
        min_n_queued_items, max_n_queued_items = wait_for_queue_space([motorG])
        print(f"The minimum and maximum number of queued items are {min_n_queued_items} and {max_n_queued_items}")
        if min_n_queued_items <= 3:
            motorG.move_with_velocity(PLUNGER_VELOCITY_ROTATIONS_PER_SECOND, 0.1)
        
        ch = getch()
        if ch:
            print(f"Key pressed: {ch}")
            break
except KeyboardInterrupt:
    print("\nProgram interrupted by user")

# Cleanup code
print("Running cleanup...")
motorG.move_with_velocity(0, MIN_MOVE_TIME_S)
wait_queue_empty([motorG])
enable_or_disable_glue_dispenser(motorG, False)
time.sleep(5)
print("Cleanup complete, exiting.")
