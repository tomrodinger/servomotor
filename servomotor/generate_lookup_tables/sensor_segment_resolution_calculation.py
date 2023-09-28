#!/usr/bin/env python3


def calculate_n_steps(rough_n_steps, n_sub_steps, magnetic_ring_pole_pairs, n_hall_sensors):
    magnetic_ring_poles = magnetic_ring_pole_pairs * 2

    print("First, here are the original settings:")

    ONE_REVOLUTION_STEPS = 7
    sensor_segment_resolution = rough_n_steps * n_sub_steps * ONE_REVOLUTION_STEPS / magnetic_ring_pole_pairs / n_hall_sensors

    print("   PRODUCT_NAME_M1:")
    print("      sensor_segment_resolution:", sensor_segment_resolution)

    ONE_REVOLUTION_STEPS = 50
    sensor_segment_resolution = rough_n_steps * n_sub_steps * ONE_REVOLUTION_STEPS / (magnetic_ring_pole_pairs * n_hall_sensors)

    print("   PRODUCT_NAME_M2:")
    print("      sensor_segment_resolution:", sensor_segment_resolution, "  <--- this is not an integer, this is a problem")

    print("================================================================================================================")

    print("Now, lets try to find a solution by changing the value of n_steps:")

    # we want the result to have no fractional part
    # what we can easily control is the rough_n_steps
    # lets first isolate the rough_n_steps
    # then change sensor_segment_resolution until n_steps becomes an integer
    sensor_segment_resolution_int = int(sensor_segment_resolution)
    while True:
        n_steps = sensor_segment_resolution_int * magnetic_ring_pole_pairs * n_hall_sensors / n_sub_steps / ONE_REVOLUTION_STEPS
        if n_steps == int(n_steps):
            break
        sensor_segment_resolution_int -= 1
    chosen_n_steps = int(n_steps)
    print("chosen_n_steps:", chosen_n_steps)
    sensor_segment_resolution_int = chosen_n_steps * n_sub_steps * ONE_REVOLUTION_STEPS / (magnetic_ring_pole_pairs * n_hall_sensors)
    print("   sensor_segment_resolution_int:", sensor_segment_resolution_int, "  <--- this is now an integer, we found a solution")

    sensor_segment_resolution_int = int(sensor_segment_resolution)
    while True:
        n_steps = sensor_segment_resolution_int * magnetic_ring_pole_pairs * n_hall_sensors / n_sub_steps / ONE_REVOLUTION_STEPS
        if n_steps == int(n_steps):
            break
        sensor_segment_resolution_int += 1
    print("n_steps:", n_steps)
    sensor_segment_resolution_int = n_steps * n_sub_steps * ONE_REVOLUTION_STEPS / (magnetic_ring_pole_pairs * n_hall_sensors)
    print("   sensor_segment_resolution_int:", sensor_segment_resolution_int, "  <--- this is also an integer, we found another solution")

    print("================================================================================================================")
    print("Now, let's recalculate the sensor_segment_resolution_int for the new chosen_n_steps for both motors:")

    print("Using chosen_n_steps:", chosen_n_steps)

    ONE_REVOLUTION_STEPS = 7
    sensor_segment_resolution_int_M1 = chosen_n_steps * n_sub_steps * ONE_REVOLUTION_STEPS / magnetic_ring_pole_pairs / n_hall_sensors
    # do a sanity check here to make sure that the result is an integer
    if sensor_segment_resolution_int_M1 != int(sensor_segment_resolution_int_M1):
        print("ERROR: sensor_segment_resolution_int_M1 is not an integer")
        exit(1)
    sensor_segment_resolution_int_M1 = int(sensor_segment_resolution_int_M1)

    print("PRODUCT_NAME_M1:")
    print("   sensor_segment_resolution_int_M1:", sensor_segment_resolution_int_M1)

    ONE_REVOLUTION_STEPS = 50
    sensor_segment_resolution_int_M2 = int(chosen_n_steps * n_sub_steps * ONE_REVOLUTION_STEPS / (magnetic_ring_pole_pairs * n_hall_sensors))
    # do a sanity check here to make sure that the result is an integer
    if sensor_segment_resolution_int_M2 != int(sensor_segment_resolution_int_M2):
        print("ERROR: sensor_segment_resolution_int_M2 is not an integer")
        exit(1)
    sensor_segment_resolution_int_M2 = int(sensor_segment_resolution_int_M2)

    print("PRODUCT_NAME_M2:")
    print("   sensor_segment_resolution_int_M2:", sensor_segment_resolution_int_M2)

    return chosen_n_steps, sensor_segment_resolution_int_M1, sensor_segment_resolution_int_M2

