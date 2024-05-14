#!/usr/bin/env python3
import glob

OUTPUT_FOLDER = "analysis_output_2" # files to analyse will be located in this directory
INPUT_FILE_PREFIX = "max_error_mm_from_all_trials_item*"

# Read in all of the data. The data is located in the files that start with INPUT_FILE_PREFIX. We need to find all the files
# and read the data from the files. Each file will have just one floating point number in it.
file_list = glob.glob(f"{OUTPUT_FOLDER}/{INPUT_FILE_PREFIX}")
if len(file_list) == 0:
    print(f"Error: no files found in {OUTPUT_FOLDER}/{INPUT_FILE_PREFIX}*")
    exit(1)
print(f"Found {len(file_list)} files in {OUTPUT_FOLDER}/{INPUT_FILE_PREFIX}*")
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
    # read in the file and make sure that there is just one floating point number in it
    with open(filename, 'r') as file:
        full_file = file.read().strip()
        try:
            error_value = float(full_file)
        except:
            print("Error: expected one floating point number in file:", filename)
            continue
        data.append((item_number, error_value))

# Sort the data by the second column (the error values)
sorted_data = sorted(data, key=lambda x: x[1])

# Calculate column widths
column_headers = ["Magnetic Ring Number", "Max Error (mm)"]

# Compute max length for each column including headers
max_trial_length = max(len(str(trial_number)) for trial_number, _ in sorted_data)
max_error_length = max(len(f"{error_value:.3f}") for _, error_value in sorted_data)

# Determine the width of each column
trial_column_width = max(max_trial_length, len(column_headers[0]))
error_column_width = max(max_error_length, len(column_headers[1]))

# Print the header
print(f"{column_headers[0]:<{trial_column_width}}  {column_headers[1]:<{error_column_width}}")

# Print a separator
print(f"{'-' * trial_column_width}  {'-' * error_column_width}")

# Print out the sorted data
for trial_number, error_value in sorted_data:
    print(f"{trial_number:<{trial_column_width}}  {error_value:<{error_column_width}.3f}")
