#!/usr/bin/env python3
import math
import sys
import time
import sensor_segment_resolution_calculation

OUTPUT_HEADER_FILENAME_PREFIX = "../Src/commutation_table_"

PURE_SINE_MODE = 0
SIX_STEP_MODE = 1
MODIFIED_SINE_MODE = 2
MODE = PURE_SINE_MODE
SIMPLE_TABLE = 0

# Get the product name from the command line. The product name is something like M1, M2, M3, M4, etc.
# If the product name is not specified, then print out an error message and give the user a list of
# the available product names.
product_name = None
if len(sys.argv) == 2:
    product_name = sys.argv[1]
else:
    print("Error: please specify a product name on the command line")
    print("Usage: %s product-name" % (sys.argv[0]))
    print("Product names are M1, M2, M3, M4, etc.")
    exit(1)
if product_name == "M1":
    import SETTINGS_M1
    ONE_REVOLUTION_ELECTRICAL_CYCLES = SETTINGS_M1.ONE_REVOLUTION_ELECTRICAL_CYCLES
    ONE_REVOLUTION_HALL_SENSOR_CYCLES = SETTINGS_M1.ONE_REVOLUTION_HALL_SENSOR_CYCLES
    N_COMMUTATION_STEPS = SETTINGS_M1.N_COMMUTATION_STEPS
    N_COMMUTATION_SUB_STEPS = SETTINGS_M1.N_COMMUTATION_SUB_STEPS
    N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT = SETTINGS_M1.N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT
    include_N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT = 1
    generate_commutation_table = 1
elif product_name == "M2":
    import SETTINGS_M2
    ONE_REVOLUTION_ELECTRICAL_CYCLES = SETTINGS_M2.ONE_REVOLUTION_ELECTRICAL_CYCLES
    ONE_REVOLUTION_HALL_SENSOR_CYCLES = SETTINGS_M2.ONE_REVOLUTION_HALL_SENSOR_CYCLES
    N_COMMUTATION_STEPS = SETTINGS_M2.N_COMMUTATION_STEPS
    N_COMMUTATION_SUB_STEPS = SETTINGS_M2.N_COMMUTATION_SUB_STEPS
    N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT = SETTINGS_M2.N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT
    include_N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT = 1
    generate_commutation_table = 1
elif product_name == "M3":
    import SETTINGS_M3
    ONE_REVOLUTION_ELECTRICAL_CYCLES = SETTINGS_M3.ONE_REVOLUTION_ELECTRICAL_CYCLES
    ONE_REVOLUTION_HALL_SENSOR_CYCLES = SETTINGS_M3.ONE_REVOLUTION_HALL_SENSOR_CYCLES
    N_COMMUTATION_STEPS = SETTINGS_M3.N_COMMUTATION_STEPS
    N_COMMUTATION_SUB_STEPS = SETTINGS_M3.N_COMMUTATION_SUB_STEPS
#    N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT = SETTINGS_M3.N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT
    include_N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT = 0
    generate_commutation_table = 0
elif product_name == "M4":
    import SETTINGS_M4
    ONE_REVOLUTION_ELECTRICAL_CYCLES = SETTINGS_M4.ONE_REVOLUTION_ELECTRICAL_CYCLES
    ONE_REVOLUTION_HALL_SENSOR_CYCLES = SETTINGS_M4.ONE_REVOLUTION_HALL_SENSOR_CYCLES
    N_COMMUTATION_STEPS = SETTINGS_M4.N_COMMUTATION_STEPS
    N_COMMUTATION_SUB_STEPS = SETTINGS_M4.N_COMMUTATION_SUB_STEPS
    include_N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT = 0
    generate_commutation_table = 0
else:
    print("Error: unsupported product name:", product_name)
    print("You should update this executable and create a SETTINGS_product-name.py file")
    exit(1)

ONE_REVOLUTION_MICROSTEPS = N_COMMUTATION_STEPS * N_COMMUTATION_SUB_STEPS * ONE_REVOLUTION_ELECTRICAL_CYCLES
N_HALL_SENSORS = 3
START_ANGLE = 30.0
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
    

all_phase_data = []
min_min_phase = 1.0e10
max_max_phase = -1.0e10

angle = START_ANGLE
for i in range(N_COMMUTATION_STEPS):
    if MODE == SIX_STEP_MODE:
        phase1 = (AMPLITUDE / 2.0) * six_step_function(angle / 180.0 * math.pi - 2.0 * math.pi / 3.0 * 0.0)
        phase2 = (AMPLITUDE / 2.0) * six_step_function(angle / 180.0 * math.pi - 2.0 * math.pi / 3.0 * 1.0)
        phase3 = (AMPLITUDE / 2.0) * six_step_function(angle / 180.0 * math.pi - 2.0 * math.pi / 3.0 * 2.0)
    else:
        phase1 = (AMPLITUDE / 2.0) * math.sin(angle / 180.0 * math.pi - 2.0 * math.pi / 3.0 * 0.0)
        phase2 = (AMPLITUDE / 2.0) * math.sin(angle / 180.0 * math.pi - 2.0 * math.pi / 3.0 * 1.0)
        phase3 = (AMPLITUDE / 2.0) * math.sin(angle / 180.0 * math.pi - 2.0 * math.pi / 3.0 * 2.0)
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
fh2.write("// This file was autogenerated by executing: %s %s\n" % (sys.argv[0], product_name))
fh2.write("// Today's date is: %s\n" % (time.strftime("%c")))
fh2.write("\n")
if generate_commutation_table:
    fh2.write("struct three_phase_data_struct {\n")
    fh2.write("   uint32_t phase1;\n")
    fh2.write("   uint32_t phase2;\n")
    fh2.write("   uint32_t phase3;\n")
    fh2.write("   int32_t phase1_slope;\n")
    fh2.write("   int32_t phase2_slope;\n")
    fh2.write("   int32_t phase3_slope;\n")
    fh2.write("};\n")
    fh2.write("\n")
fh2.write("#define N_COMMUTATION_STEPS %d\n" % (N_COMMUTATION_STEPS))
fh2.write("#define N_COMMUTATION_SUB_STEPS %d\n" % (N_COMMUTATION_SUB_STEPS))
if include_N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT:
    fh2.write("#define N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT %d\n" % (N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT))
fh2.write("#define ONE_REVOLUTION_ELECTRICAL_CYCLES %d\n" % (ONE_REVOLUTION_ELECTRICAL_CYCLES))
fh2.write("#define ONE_REVOLUTION_MICROSTEPS %d\n" % (ONE_REVOLUTION_MICROSTEPS))

if generate_commutation_table:
    fh2.write("#define MAX_PHASE_VALUE %d\n" % (AMPLITUDE))
    fh2.write("\n")
    fh2.write("#define COMMUTATION_LOOKUP_TABLE_INITIALIZER { \\\n")

    if SIMPLE_TABLE:
        for phase_data in all_phase_data:
            angle = phase_data[0]
            phase1 = phase_data[1]
            phase2 = phase_data[2]
            phase3 = phase_data[3]
            phase1_rounded = int(phase1 + 0.5)
            phase2_rounded = int(phase2 + 0.5)
            phase3_rounded = int(phase3 + 0.5)
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
            this_phase3 = this_phase_data[3]
            next_phase1 = next_phase_data[1]
            next_phase2 = next_phase_data[2]
            next_phase3 = next_phase_data[3]
            phase1_slope = next_phase1 - this_phase1
            phase2_slope = next_phase2 - this_phase2
            phase3_slope = next_phase3 - this_phase3
            phase1_rounded = int(this_phase1 + 0.5)
            phase2_rounded = int(this_phase2 + 0.5)
            phase3_rounded = int(this_phase3 + 0.5)
            phase1_slope_rounded = int(phase1_slope + 0.5)
            phase2_slope_rounded = int(phase2_slope + 0.5)
            phase3_slope_rounded = int(phase3_slope + 0.5)
    #       lookup_table.append([phase1_rounded, phase2_rounded, phase3_rounded,
    #                            phase1_slope, phase2_slope, phase1_slope])

            print("{%10d, %10d, %10d, %10d, %10d, %10d}," % (phase1_rounded, phase2_rounded, phase3_rounded,
                                                phase1_slope, phase2_slope, phase3_slope))
            fh2.write("   {%10d, %10d, %10d, %10d, %10d, %10d}, \\\n" % (phase1_rounded, phase2_rounded, phase3_rounded,
                                                        phase1_slope, phase2_slope, phase3_slope))

            angle_increment = ANGLE_STEP / N_COMMUTATION_SUB_STEPS
            phase1_increment = int(phase1_slope_rounded / N_COMMUTATION_SUB_STEPS + 0.5)
            phase2_increment = int(phase2_slope_rounded / N_COMMUTATION_SUB_STEPS + 0.5)
            phase3_increment = int(phase3_slope_rounded / N_COMMUTATION_SUB_STEPS + 0.5)
            for j in range(N_COMMUTATION_SUB_STEPS):
                fh.write("%f %d %d %d\n" % (angle, phase1_rounded, phase2_rounded, phase3_rounded))
                angle = angle + angle_increment
                phase1_rounded = phase1_rounded + phase1_increment
                phase2_rounded = phase2_rounded + phase2_increment
                phase3_rounded = phase3_rounded + phase3_increment

    fh2.write("}\n")

fh2.write("\n#endif\n")

fh.close()
fh2.close()

