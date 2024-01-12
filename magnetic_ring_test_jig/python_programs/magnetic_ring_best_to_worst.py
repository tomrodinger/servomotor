#!/usr/bin/env python3

# Define the file path (use your own path if the file is in a different location)
file_path = 'analysis_output/max_error_mm_from_all_trials'

# Initialize a list to hold the data
data = []

# Read in the data from the file
with open(file_path, 'r') as file:
    for line in file:
        parts = line.split()
        if len(parts) == 2:  # Ensure the line has exactly two elements
            trial_number = int(parts[0])
            error_value = float(parts[1])
            data.append((trial_number, error_value))

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
