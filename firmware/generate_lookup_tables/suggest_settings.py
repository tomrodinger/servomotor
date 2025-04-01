#!/usr/bin/env python3
import math
import sys
import time
import sensor_segment_resolution_calculation

# This program computes a value for the N_COMMUTATION_STEPS setting such that the same commutation table could be used for both
# Servomotor#1 (M1) and Servomotor#2 (M2) and have all the integer math work out.

MAGNETIC_RING_POLE_PAIRS = 28
N_HALL_SENSORS = 3
ROUGH_COMMUTATION_N_STEPS = 360 # we want n_steps to be roughly this, but the algorithm will adjust it so that the required numbers are integers
N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT = 8
N_COMMUTATION_SUB_STEPS = (1 << N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT)
n_steps, sensor_segment_resolution_M1, sensor_segment_resolution_M2 = sensor_segment_resolution_calculation.calculate_n_steps(ROUGH_COMMUTATION_N_STEPS, N_COMMUTATION_SUB_STEPS, MAGNETIC_RING_POLE_PAIRS, N_HALL_SENSORS)

print("This is what this program suggests to use for the values to set in the SETTINGS_product-name.py file:")
print("   N_COMMUTATION_STEPS = %d\n" % (n_steps))
print("   N_COMMUTATION_SUB_STEPS = %d\n" % (N_COMMUTATION_SUB_STEPS))
print("   N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT = %d\n" % (N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT))

