#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
from tabulate import tabulate
import glob
import sys
sys.path.append("../../")  # Add parent directory to Python path to find settings files

OUTPUT_FOLDER = "analysis_output_2"
OUTPUT_HEADER_FILENAME_PREFIX = "../../servomotor/Src/hall_sensor_constants_"
WEIGHTS_FILE_PREFIX = "best_weights_from_all_trials_item*"

def write_out_header_file(filename, weights, product_name):
    if product_name == "M1":
        from SETTINGS_M1 import N_HALL_SENSORS, N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT, N_COMMUTATION_STEPS, N_COMMUTATION_SUB_STEPS, ONE_REVOLUTION_ELECTRICAL_CYCLES, \
                                ONE_REVOLUTION_HALL_SENSOR_CYCLES, HALL_SENSOR_SHIFT, SLOPE_SHIFT_RIGHT, OFFSET_SHIFT_RIGHT, HALL_SAMPLES_PER_PRINT, CALIBRATION_CAPTURE_STEP_SIZE
    elif product_name == "M2":
        from SETTINGS_M2 import N_HALL_SENSORS, N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT, N_COMMUTATION_STEPS, N_COMMUTATION_SUB_STEPS, ONE_REVOLUTION_ELECTRICAL_CYCLES, \
                                ONE_REVOLUTION_HALL_SENSOR_CYCLES, HALL_SENSOR_SHIFT, SLOPE_SHIFT_RIGHT, OFFSET_SHIFT_RIGHT, HALL_SAMPLES_PER_PRINT, CALIBRATION_CAPTURE_STEP_SIZE
    elif product_name == "M3":
        from SETTINGS_M3 import N_HALL_SENSORS, N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT, N_COMMUTATION_STEPS, N_COMMUTATION_SUB_STEPS, ONE_REVOLUTION_ELECTRICAL_CYCLES, \
                                ONE_REVOLUTION_HALL_SENSOR_CYCLES, HALL_SENSOR_SHIFT, SLOPE_SHIFT_RIGHT, OFFSET_SHIFT_RIGHT, HALL_SAMPLES_PER_PRINT, CALIBRATION_CAPTURE_STEP_SIZE
    else:
        print("Error: unsupported product name:", product_name)
        print("You should update this executable and create a SETTINGS_product-name.py file")
        exit(1)
    MAGNETIC_RING_POLES = ONE_REVOLUTION_HALL_SENSOR_CYCLES * 2
    TOTAL_NUMBER_OF_SEGMENTS = MAGNETIC_RING_POLES * 3
    SENSOR_SEGMENT_RESOLUTION = N_COMMUTATION_STEPS * N_COMMUTATION_SUB_STEPS * ONE_REVOLUTION_ELECTRICAL_CYCLES / ONE_REVOLUTION_HALL_SENSOR_CYCLES / N_HALL_SENSORS
    # do a sanity check here to make sure that SENSOR_SEGMENT_RESOLUTION is an integer
    if SENSOR_SEGMENT_RESOLUTION != int(SENSOR_SEGMENT_RESOLUTION):
        print("Error: SENSOR_SEGMENT_RESOLUTION is not an integer:", SENSOR_SEGMENT_RESOLUTION)
        exit(1)
    SENSOR_SEGMENT_RESOLUTION = int(SENSOR_SEGMENT_RESOLUTION)

    with open(filename, "w") as fh:
        fh.write("#ifndef __HALL_SENSOR_CONSTANTS__\n")
        fh.write("#define __HALL_SENSOR_CONSTANTS__\n")
        fh.write("\n")
        fh.write("#define N_HALL_SENSORS %d\n" % (N_HALL_SENSORS))
        fh.write("#define TOTAL_NUMBER_OF_SEGMENTS %d\n" % (TOTAL_NUMBER_OF_SEGMENTS))
        fh.write("#define HALL_SENSOR_SHIFT %d\n" % (HALL_SENSOR_SHIFT))
        fh.write("#define SLOPE_SHIFT_RIGHT %d\n" % (SLOPE_SHIFT_RIGHT))
        fh.write("#define OFFSET_SHIFT_RIGHT %d\n" % (OFFSET_SHIFT_RIGHT))
        fh.write("#define HALL_SAMPLES_PER_PRINT %d\n" % (HALL_SAMPLES_PER_PRINT))
        weights_per_hall_sensor = int(len(weights) / 3)
        fh.write("#define WEIGHTS_PER_HALL_SENSOR %d\n" % (weights_per_hall_sensor))
        fh.write("#define SENSOR_SEGMENT_RESOLUTION %d\n" % (SENSOR_SEGMENT_RESOLUTION))
        fh.write("#define SENSOR_SEGMENT_RESOLUTION_DIV_2 %d\n" % (SENSOR_SEGMENT_RESOLUTION >> 1))
        fh.write("#define CALIBRATION_CAPTURE_STEP_SIZE %d // this should be slow so that data can be captured and collected accurately\n" % (CALIBRATION_CAPTURE_STEP_SIZE))
        fh.write("\n")
        fh.write("\n")
        fh.write("struct hall_weights_struct {\n")
        fh.write("   int16_t h1[WEIGHTS_PER_HALL_SENSOR];\n")
        fh.write("   int16_t h2[WEIGHTS_PER_HALL_SENSOR];\n")
        fh.write("   int16_t h3[WEIGHTS_PER_HALL_SENSOR];\n")
        fh.write("};\n")
        fh.write("\n")
        fh.write("#define HALL_WEIGHTS_INITIALIZER { \\\n")
        for i in range(weights_per_hall_sensor):
            w0 = int(weights[i * 3 + 0] * (SENSOR_SEGMENT_RESOLUTION >> 1) + 0.5)
            w1 = int(weights[i * 3 + 1] * (SENSOR_SEGMENT_RESOLUTION >> 1) + 0.5)
            w2 = int(weights[i * 3 + 2] * (SENSOR_SEGMENT_RESOLUTION >> 1) + 0.5)
            if (w0 > 32767) or (w0 < -32768) or (w1 > 32767) or (w1 < -32768) or (w2 > 32767) or (w2 < -32768):
                print("Error: one or more of the weights are out of the uint16_t range: %d %d %d" % (w0, w1, w2))
                exit(1)
            fh.write("    {%6d, %6d, %6d}, \\\n" % (w0, w1, w2))
        fh.write("}\n\n")
        fh.write("#endif\n")
        fh.write("\n")
    print("Wrote out header to file:", filename)

# Read in all of the weight data. The data is located in the files that start with WEIGHTS_FILE_PREFIX. We need to find all the files
# and read the weights from the files. Each file will have just one row of weights and there will be 9 weights.
file_list = glob.glob(f"{OUTPUT_FOLDER}/{WEIGHTS_FILE_PREFIX}")
if len(file_list) == 0:
    print(f"Error: no files found in {OUTPUT_FOLDER}/{WEIGHTS_FILE_PREFIX}*")
    exit(1)
print(f"Found {len(file_list)} files in {OUTPUT_FOLDER}/{WEIGHTS_FILE_PREFIX}*")
data = []
for filename in file_list:
    print("  ", filename)
    # extract the item number from the filename. The filename ends with _item<number>
    parts = filename.split("_")
    if len(parts) < 2:
        print("Error: expected at least one underscore in filename:", filename)
        continue
    item_and_number = parts[-1]
    if item_and_number[:4] != "item":
        print("Error: expected filename to end with _item<number>:", filename)
        continue
    try:
        item_number = int(item_and_number[4:])
    except:
        print("Error: expected filename to end with _item<number>:", filename)
        continue
    print(f"    item number: {item_number}")
    # read in the file and make sure that there is just one line with 9 fields
    with open(filename, 'r') as file:
        full_file = file.read()
        if full_file.count("\n") != 1:
            print("Error: expected just one line in file:", filename)
            continue
        weights = full_file.split()
        if len(weights) != 9:
            print("Error: expected 9 fields in line:", full_file)
            continue
        # convert the weights to floats and append them to the data in the format good for pandas
        weights = [float(w) for w in weights]
        row = [item_number] + weights
        data.append(row)

# Convert data to a pandas dataframe
data = pd.DataFrame(data)
# Sort the data
data = data.sort_values(by=0).reset_index(drop=True)

# Rename columns for clarity
columns_names = ['Magnetic Ring Number'] + [f'Weight {i}' for i in range(1, 10)]
data.columns = columns_names

# Calculate averages for the weights
weight_averages = data.iloc[:, 1:].mean()

# Create table for console output
table_data = [(i, weight_averages[f'Weight {i}']) for i in range(1, 10)]
print(tabulate(table_data, headers=['Weight Index', 'Average Weight'], tablefmt="presto"))

# read in the product_name from a file called PRODUCT_NAME in the output folder
with open(OUTPUT_FOLDER + "/PRODUCT_NAME", "r") as fh:
    product_name = fh.read().strip()
output_header_filename = OUTPUT_HEADER_FILENAME_PREFIX + product_name + ".h"
write_out_header_file(output_header_filename, weight_averages, product_name)

# Plotting
fig, axes = plt.subplots(nrows=3, ncols=3, figsize=(15, 10))
fig.tight_layout(pad=5.0)

# Iterate over each weight to create subplot
for i, col in enumerate(columns_names[1:], 1):
    row, col_idx = (i - 1) // 3, (i - 1) % 3
    ax = axes[row, col_idx]
    
    ax.plot(data['Magnetic Ring Number'], data[f'Weight {i}'], marker='o')
    ax.set_title(f'Weight {i}')
    
    # Set consistent y-axis limits
    ax.set_ylim(-0.5, 1.5)

    # Add thicker black line at y=0 or y=1 and blue line at the average
    hline_y = 1.0 if i in [1, 4, 7] else 0.0
    ax.axhline(y=hline_y, color='black', linewidth=1.5)
    ax.axhline(y=weight_averages[f'Weight {i}'], color='blue', linewidth=1.0, linestyle='--')
    
    # Add gridlines
    ax.grid(color='gray', linestyle='--', linewidth=0.5, which='both')

    # Add labels
    if row == 2:  # Only label x-axis on bottom row
        ax.set_xlabel('Magnetic Ring Number')
    if col_idx == 0:  # Only label y-axis on first column
        ax.set_ylabel('Weight')

plt.show()
