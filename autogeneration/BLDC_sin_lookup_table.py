#!/usr/bin/env python3
import math
import sys
import time
import sensor_segment_resolution_calculation

import os

# Get the absolute path of the current script
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# Get the firmware directory
FIRMWARE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../firmware/Src/"))

# Add parent directory to Python path to find settings files
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # Add parent directory to Python path

OUTPUT_HEADER_FILENAME_PREFIX = os.path.join(FIRMWARE_DIR, "commutation_table_")

PURE_SINE_MODE = 0
SIX_STEP_MODE = 1
MODIFIED_SINE_MODE = 2
MODE = PURE_SINE_MODE
SIMPLE_TABLE = 0
N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT = None
N_PHASES = None
SINE_EXPONENT = 1.0

# Get the product name from the command line. The product name is something like M1, M2, M3, M23, etc.
# If the product name is not specified, then print out an error message and give the user a list of
# the available product names.
product_name = None
if len(sys.argv) == 2:
    product_name = sys.argv[1]
else:
    print("Error: please specify a product name on the command line")
    print("Usage: %s product-name" % (sys.argv[0]))
    print("Product names are M1, M2, M3, M23, etc.")
    exit(1)
if product_name == "M1":
    import SETTINGS_M1
    ONE_REVOLUTION_ELECTRICAL_CYCLES = SETTINGS_M1.ONE_REVOLUTION_ELECTRICAL_CYCLES
    ONE_REVOLUTION_HALL_SENSOR_CYCLES = SETTINGS_M1.ONE_REVOLUTION_HALL_SENSOR_CYCLES
    N_COMMUTATION_STEPS = SETTINGS_M1.N_COMMUTATION_STEPS
    N_COMMUTATION_SUB_STEPS = SETTINGS_M1.N_COMMUTATION_SUB_STEPS
    N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT = SETTINGS_M1.N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT
    N_PHASES = 3
    ANGLE_BETWEEN_PHASES = 120.0
    START_ANGLE = 30.0
elif product_name == "M2":
    import SETTINGS_M2
    ONE_REVOLUTION_ELECTRICAL_CYCLES = SETTINGS_M2.ONE_REVOLUTION_ELECTRICAL_CYCLES
    ONE_REVOLUTION_HALL_SENSOR_CYCLES = SETTINGS_M2.ONE_REVOLUTION_HALL_SENSOR_CYCLES
    N_COMMUTATION_STEPS = SETTINGS_M2.N_COMMUTATION_STEPS
    N_COMMUTATION_SUB_STEPS = SETTINGS_M2.N_COMMUTATION_SUB_STEPS
    N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT = SETTINGS_M2.N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT
    START_ANGLE = 30.0
elif product_name == "M3":
    import SETTINGS_M3
    ONE_REVOLUTION_ELECTRICAL_CYCLES = SETTINGS_M3.ONE_REVOLUTION_ELECTRICAL_CYCLES
    ONE_REVOLUTION_HALL_SENSOR_CYCLES = SETTINGS_M3.ONE_REVOLUTION_HALL_SENSOR_CYCLES
    N_COMMUTATION_STEPS = SETTINGS_M3.N_COMMUTATION_STEPS
    N_COMMUTATION_SUB_STEPS = SETTINGS_M3.N_COMMUTATION_SUB_STEPS
    START_ANGLE = 30.0
elif product_name == "M23":
    import SETTINGS_M23
    ONE_REVOLUTION_ELECTRICAL_CYCLES = SETTINGS_M23.ONE_REVOLUTION_ELECTRICAL_CYCLES
    ONE_REVOLUTION_HALL_SENSOR_CYCLES = SETTINGS_M23.ONE_REVOLUTION_HALL_SENSOR_CYCLES
    N_COMMUTATION_STEPS = SETTINGS_M23.N_COMMUTATION_STEPS
    N_COMMUTATION_SUB_STEPS = SETTINGS_M23.N_COMMUTATION_SUB_STEPS
    N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT = SETTINGS_M23.N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT
    N_PHASES = 2
    ANGLE_BETWEEN_PHASES = 90.0
    START_ANGLE = 0.0
    SINE_EXPONENT = 1.0
else:
    print("Error: unsupported product name:", product_name)
    print("You should update this executable and create a SETTINGS_product-name.py file")
    exit(1)

ONE_REVOLUTION_MICROSTEPS = N_COMMUTATION_STEPS * N_COMMUTATION_SUB_STEPS * ONE_REVOLUTION_ELECTRICAL_CYCLES
N_HALL_SENSORS = 3
ANGLE_STEP = 360.0 / N_COMMUTATION_STEPS
AMPLITUDE = (1 << 24) - 1
output_header_filename = f"{OUTPUT_HEADER_FILENAME_PREFIX}{product_name}.h"


def six_step_function(angle):
    angle = angle * 180.0 / math.pi
    if angle >= 90.0 - 60.0 and angle < 90.0 + 60.0:
        return 1.0
    elif angle >= 90.0 - 60.0 + 180 and angle < 90.0 + 60.0 + 180.0:
        return -1.0
    else:
        return 0.0


def exponent_keep_sign(x, exponent):
    if x > 0.0:
        return math.pow(x, exponent)
    elif x < 0.0:
        return -math.pow(-x, exponent)
    else:
        return 0.0
    

if N_PHASES:
    ANGLE_BETWEEN_PHASES_RADIANS = ANGLE_BETWEEN_PHASES / 180.0 * math.pi
    all_phase_data = []
    min_min_phase = 1.0e10
    max_max_phase = -1.0e10

    angle = START_ANGLE
    for i in range(N_COMMUTATION_STEPS):
        if MODE == SIX_STEP_MODE:
            phase1 = (AMPLITUDE / 2.0) * six_step_function(angle / 180.0 * math.pi - ANGLE_BETWEEN_PHASES_RADIANS * 0.0)
            phase2 = (AMPLITUDE / 2.0) * six_step_function(angle / 180.0 * math.pi - ANGLE_BETWEEN_PHASES_RADIANS * 1.0)
            phase3 = (AMPLITUDE / 2.0) * six_step_function(angle / 180.0 * math.pi - ANGLE_BETWEEN_PHASES_RADIANS * 2.0)
        else:
            phase1 = (AMPLITUDE / 2.0) * exponent_keep_sign(math.sin(angle / 180.0 * math.pi - ANGLE_BETWEEN_PHASES_RADIANS * 0.0), SINE_EXPONENT)
            phase2 = (AMPLITUDE / 2.0) * exponent_keep_sign(math.sin(angle / 180.0 * math.pi - ANGLE_BETWEEN_PHASES_RADIANS * 1.0), SINE_EXPONENT)
            phase3 = (AMPLITUDE / 2.0) * exponent_keep_sign(math.sin(angle / 180.0 * math.pi - ANGLE_BETWEEN_PHASES_RADIANS * 2.0), SINE_EXPONENT)
        min_phase = min(phase1, phase2, phase3)
        if MODE != PURE_SINE_MODE:
            adjustment = -min_phase
            phase1 = phase1 + adjustment
            phase2 = phase2 + adjustment
            phase3 = phase3 + adjustment
        min_phase = min(phase1, phase2, phase3)
        max_phase = max(phase1, phase2, phase3)
        if min_phase < min_min_phase:
            min_min_phase = min_phase
        if max_phase > max_max_phase:
            max_max_phase = max_phase
        all_phase_data.append([angle, phase1, phase2, phase3])
        angle = angle + ANGLE_STEP

    shift = -min_min_phase
    scale_factor = AMPLITUDE / (max_max_phase - min_min_phase)

    new_list = []
    for a in all_phase_data:
        angle = a[0]
        phase1 = (a[1] + shift) * scale_factor
        phase2 = (a[2] + shift) * scale_factor
        phase3 = (a[3] + shift) * scale_factor
        new_list.append([angle, phase1, phase2, phase3])
    all_phase_data = new_list

fh = open("waveforms", "w")
fh2 = open(output_header_filename, "w")
print("Writing out the file: %s" % (fh2.name))

fh2.write("#ifndef __COMMUTATION_TABLE__\n")
fh2.write("#define __COMMUTATION_TABLE__\n")
fh2.write("\n")
fh2.write("// * * * DO NOT EDIT * * * Instead, edit the program that autogenerates this\n")
script_name = os.path.basename(sys.argv[0])
fh2.write("// This file was autogenerated by executing: %s %s\n" % (script_name, product_name))
fh2.write("// (possibly via autogenerate.py)\n")
fh2.write("// Generated on: %s\n" % (time.strftime("%b %-d %Y %H:%M:%S")))
fh2.write("\n")
if N_PHASES == 2 or N_PHASES == 3:
    fh2.write("struct three_phase_data_struct {\n")
    fh2.write("   uint32_t phase1;\n")
    fh2.write("   uint32_t phase2;\n")
    if N_PHASES == 3:
        fh2.write("   uint32_t phase3;\n")
    fh2.write("   int32_t phase1_slope;\n")
    fh2.write("   int32_t phase2_slope;\n")
    if N_PHASES == 3:
        fh2.write("   int32_t phase3_slope;\n")
    fh2.write("};\n")
    fh2.write("\n")
fh2.write("#define N_COMMUTATION_STEPS %d\n" % (N_COMMUTATION_STEPS))
fh2.write("#define N_COMMUTATION_SUB_STEPS %d\n" % (N_COMMUTATION_SUB_STEPS))
if N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT:
    fh2.write("#define N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT %d\n" % (N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT))
fh2.write("#define ONE_REVOLUTION_ELECTRICAL_CYCLES %d\n" % (ONE_REVOLUTION_ELECTRICAL_CYCLES))
fh2.write("#define ONE_REVOLUTION_MICROSTEPS %d\n" % (ONE_REVOLUTION_MICROSTEPS))

if N_PHASES == 2 or N_PHASES == 3:
    fh2.write("#define MAX_PHASE_VALUE %d\n" % (AMPLITUDE))
    fh2.write("\n")
    fh2.write("#define COMMUTATION_LOOKUP_TABLE_INITIALIZER { \\\n")

    if SIMPLE_TABLE:
        for phase_data in all_phase_data:
            angle = phase_data[0]
            phase1 = phase_data[1]
            phase2 = phase_data[2]
            if N_PHASES == 3:
                phase3 = phase_data[3]
            phase1_rounded = int(phase1 + 0.5)
            phase2_rounded = int(phase2 + 0.5)
            if N_PHASES == 3:
                phase3_rounded = int(phase3 + 0.5)
            if N_PHASES == 2:
                print("{%d, %d}," % (phase1_rounded, phase2_rounded))
            else:
                print("{%d, %d, %d}," % (phase1_rounded, phase2_rounded, phase3_rounded))
    else:
        lookup_table = []
        for i in range(len(all_phase_data)):
            this_phase_data = all_phase_data[i]
            angle = this_phase_data[0]
            if i + 1 == len(all_phase_data):
                next_phase_data = all_phase_data[0]
            else:
                next_phase_data = all_phase_data[i + 1]
            this_phase1 = this_phase_data[1]
            this_phase2 = this_phase_data[2]
            if N_PHASES == 3:
                this_phase3 = this_phase_data[3]
            next_phase1 = next_phase_data[1]
            next_phase2 = next_phase_data[2]
            if N_PHASES == 3:
                next_phase3 = next_phase_data[3]
            phase1_slope = next_phase1 - this_phase1
            phase2_slope = next_phase2 - this_phase2
            if N_PHASES == 3:
                phase3_slope = next_phase3 - this_phase3
            phase1_rounded = int(this_phase1 + 0.5)
            phase2_rounded = int(this_phase2 + 0.5)
            if N_PHASES == 3:
                phase3_rounded = int(this_phase3 + 0.5)
            phase1_slope_rounded = int(phase1_slope + 0.5)
            phase2_slope_rounded = int(phase2_slope + 0.5)
            if N_PHASES == 3:
                phase3_slope_rounded = int(phase3_slope + 0.5)

            if N_PHASES == 2:
                print("{%10d, %10d, %10d, %10d}," % (phase1_rounded, phase2_rounded, phase1_slope, phase2_slope))
                fh2.write("   {%10d, %10d, %10d, %10d}, \\\n" % (phase1_rounded, phase2_rounded, phase1_slope, phase2_slope))
            else:
                print("{%10d, %10d, %10d, %10d, %10d, %10d}," % (phase1_rounded, phase2_rounded, phase3_rounded,
                                                    phase1_slope, phase2_slope, phase3_slope))
                fh2.write("   {%10d, %10d, %10d, %10d, %10d, %10d}, \\\n" % (phase1_rounded, phase2_rounded, phase3_rounded,
                                                            phase1_slope, phase2_slope, phase3_slope))

            angle_increment = ANGLE_STEP / N_COMMUTATION_SUB_STEPS
            phase1_increment = int(phase1_slope_rounded / N_COMMUTATION_SUB_STEPS + 0.5)
            phase2_increment = int(phase2_slope_rounded / N_COMMUTATION_SUB_STEPS + 0.5)
            if N_PHASES == 3:
                phase3_increment = int(phase3_slope_rounded / N_COMMUTATION_SUB_STEPS + 0.5)
            for j in range(N_COMMUTATION_SUB_STEPS):
                if N_PHASES == 2:
                    fh.write("%f %d %d\n" % (angle, phase1_rounded, phase2_rounded))
                else:
                    fh.write("%f %d %d %d\n" % (angle, phase1_rounded, phase2_rounded, phase3_rounded))
                angle = angle + angle_increment
                phase1_rounded = phase1_rounded + phase1_increment
                phase2_rounded = phase2_rounded + phase2_increment
                if N_PHASES == 3:
                    phase3_rounded = phase3_rounded + phase3_increment

    fh2.write("}\n")

fh2.write("\n#endif\n")

fh.close()
fh2.close()
