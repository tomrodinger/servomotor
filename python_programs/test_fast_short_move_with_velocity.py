#!/usr/bin/env python3

import servomotor
import time
import random

TOTAL_TEST_TIME_SECONDS = 60 * 5  # 5 minutes
ITERATION_TEST_TIME_SECONDS = 2  # 5 minutes
MIN_MOVE_TIME_S = 1.0/(64000000/2048)*10 # DEBUG adding a * 10 factor to make the move time longer to help debug a problem
MIN_DURATION = MIN_MOVE_TIME_S
MAX_DURATION = 0.010
QUEUE_SIZE = 32


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


def test_fast_short_moves():
    # Initialize motor with rotations_per_second as velocity unit
    motorX = servomotor.M3("X",
                          time_unit="seconds",
                          position_unit="encoder_counts",
                          velocity_unit="rotations_per_second",
                          acceleration_unit="degrees_per_second_squared",
                          current_unit="amps",
                          voltage_unit="volts",
                          temperature_unit="celsius",
                          verbose=0)

    try:
        servomotor.open_serial_port()
        print("\nResetting motor...")
        motorX.system_reset()
        time.sleep(1.5)  # Required delay after reset
        
        print("\nEnabling MOSFETs...")
        motorX.enable_mosfets()
        time.sleep(0.5)

        start_time = time.time()
        iteration_number = 0
        while time.time() - start_time < TOTAL_TEST_TIME_SECONDS:
            print("\nStarting fast short moves test...")
            iteration_start_time = time.time()
            moves_count = 0

            min_n_queued_items = QUEUE_SIZE # we will start this with an unrealistic high value so that we are forced to get the real number later
            max_n_queued_items = 0

            # Run test for 10 seconds
            while 1:
                # Flip a coin to see if we will set a zero velocity or non-zero velocity
                if random.randint(0, 1):
                    # Generate random velocity
                    velocity = random.uniform(-1.0, 1.0)
                else:
                    velocity = 0.0
                if moves_count >= QUEUE_SIZE:
                    # Generate random duration between 0.001 and 0.060 seconds
                    duration = random.uniform(MIN_DURATION, MAX_DURATION)
                else:
                    duration = MAX_DURATION # put lin long moves to make sure fills up and does not empty too fast
                
                if max_n_queued_items > QUEUE_SIZE - 1:
                    # Wait if queue is getting too full
                    min_n_queued_items, max_n_queued_items = wait_for_queue_space([motorX])
                
                if time.time() - iteration_start_time < ITERATION_TEST_TIME_SECONDS:
                    # Queue the move
                    motorX.move_with_velocity(velocity=velocity, duration=duration)
                else:
                    # Add a final stop command
                    motorX.move_with_velocity(velocity=0, duration=0.001)
                    print("\nWaiting for moves to complete...")
                    break
                max_n_queued_items += 1
                moves_count += 1
                
                print(f"Queued move {moves_count}: velocity={velocity:.3f} rot/s, duration={duration:.3f}s")

            
            # Wait for all moves to complete
            wait_queue_empty([motorX])

            iteration_number += 1
            print(f"\nIteration {iteration_number} complete! Executed {moves_count} moves")

    except Exception as e:
        print(f"\nTest failed: {e}")
        raise
    finally:
        servomotor.close_serial_port()
        del motorX

if __name__ == "__main__":
    test_fast_short_moves()
    print("\nPASSED")
