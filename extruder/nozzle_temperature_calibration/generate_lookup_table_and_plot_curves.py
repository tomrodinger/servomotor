#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np

# Constants
DATA_FILE_NAME = 'data.txt'
HEADER_FILENAME = '../Src/temperature_lookup_table.h'
ADC_MIN_VALUE = 1000
ADC_MAX_VALUE = 16000
LOOKUP_TABLE_ADC_VALUE_SHIFT = 9
lookup_table_adc_value_step = (1 << LOOKUP_TABLE_ADC_VALUE_SHIFT)


# Function to read data from file
def read_data_from_file(file_name):
    adc_values = []
    temperatures = []
    with open(file_name, 'r') as file:
        for line in file:
            parts = line.strip().split()
            if len(parts) == 2:
                adc, temp = parts
                adc_values.append(int(adc))
                temperatures.append(float(temp))
    return adc_values, temperatures


# Function to create a lookup table with slopes
def create_lookup_table(polynomial):
    adc_value = ADC_MIN_VALUE
    lookup_table = []
    while adc_value <= ADC_MAX_VALUE:
        temperature = polynomial(adc_value)
        next_temperature = polynomial(adc_value + lookup_table_adc_value_step)
        slope = (next_temperature - temperature) / lookup_table_adc_value_step
        lookup_table.append((temperature, slope))
        adc_value += lookup_table_adc_value_step
    return lookup_table


def calculate_integer_based_lookup_table(lookup_table):
    integer_lookup_table = []
    # Let's find a shift left value to make the temperatures as high as possible but without exceeding 65535 (16-bit)
    max_temperature = max([entry[0] for entry in lookup_table])
    temperature_shift_left = 1
    while 1:
        max_temperature_after_shifting = int(max_temperature * 2**temperature_shift_left + 0.5)
        if max_temperature_after_shifting > 65535:
            temperature_shift_left -= 1
            break
        temperature_shift_left += 1
    # lets also find a shift left value to make the slopes as high as possible but without exceeding 65535 (16-bit)
    max_slope = max([entry[1] for entry in lookup_table])
    slope_shift_left = 1
    while 1:
        max_slope_after_shifting = int(max_slope * 2**slope_shift_left + 0.5) # if you run into a "Error: the data in y_plot is decreasing" then you can try to remove the + 0.5
        if max_slope_after_shifting > 65535:
            slope_shift_left -= 1
            break
        slope_shift_left += 1
    # print the shift value in C notation
    print(f"TEMPERATURE_LOOKUP_TABLE_SHIFT_LEFT = {temperature_shift_left}")
    print(f"TEMPERATURE_LOOKUP_TABLE_SLOPE_SHIFT_LEFT = {slope_shift_left}")
    for entry in lookup_table:
        temperature_value = int((entry[0] * 2**temperature_shift_left) + 0.5)
        slope_value = int(entry[1] * 2**slope_shift_left + 0.5)
        integer_lookup_table.append((temperature_value, slope_value))
        print(f"{temperature_value}, {slope_value}")
    print()
    return integer_lookup_table, temperature_shift_left, slope_shift_left


def save_integer_based_lookup_table_in_C_header_format(integer_lookup_table, temperature_shift_left, slope_shift_left, filename):
    # print the shift value in C notation
    with open(filename, 'w') as fh:
        fh.write("#ifndef TEMPERATURE_LOOKUP_TABLE_H\n")
        fh.write("#define TEMPERATURE_LOOKUP_TABLE_H\n\n")
        fh.write(f"#define TEMPERATURE_ADC_MIN_VALUE {ADC_MIN_VALUE}\n")
        fh.write(f"#define LOOKUP_TABLE_ADC_VALUE_SHIFT {LOOKUP_TABLE_ADC_VALUE_SHIFT}\n")
        fh.write(f"#define TEMPERATURE_LOOKUP_TABLE_SHIFT_LEFT {temperature_shift_left}\n")
        fh.write(f"#define TEMPERATURE_LOOKUP_TABLE_SLOPE_SHIFT_LEFT {slope_shift_left}\n")
        fh.write(f"#define TEMPERATURE_LOOKUP_TABLE_SIZE {len(integer_lookup_table)}\n")
        fh.write("\n")
        fh.write("typedef struct __attribute__((__packed__)) {\n")
        fh.write("    uint16_t temperature;\n")
        fh.write("    uint16_t slope;\n")
        fh.write("} temperature_lookup_table_entry_t;\n")
        fh.write("\n")
        fh.write("// Integer based lookup table in C format:\n")
        fh.write("const temperature_lookup_table_entry_t temperature_lookup_table[] = {\n")
        for entry in integer_lookup_table:
            one_row = f"{entry[0]}, {entry[1]}"
            fh.write("    { " + one_row + " },\n")
        fh.write("};\n")
        fh.write("\n#endif\n")
    print("The header file was saved to: ", filename)


# Function to estimate temperature for given ADC value using lookup table
def estimate_temperature(adc_value, lookup_table):
    adjusted_adc_value = adc_value - ADC_MIN_VALUE
    if adjusted_adc_value < 0:
        temperature = -1
    lookup_table_index = int(adjusted_adc_value / lookup_table_adc_value_step)
    if lookup_table_index >= len(lookup_table):
        temperature = -1
    else:
        y0, slope = lookup_table[lookup_table_index]
        temperature = y0 + slope * (adjusted_adc_value - lookup_table_index * lookup_table_adc_value_step)
    return temperature


# Function to estimate temperature for given ADC value using lookup table
def estimate_temperature_from_integer_lookup_table(adc_value, integer_lookup_table, temperature_shift_left, slope_shift_left):
    adjusted_adc_value = adc_value - ADC_MIN_VALUE
    if adjusted_adc_value < 0:
        temperature = -1
    lookup_table_index = (adjusted_adc_value >> LOOKUP_TABLE_ADC_VALUE_SHIFT)
    if lookup_table_index >= len(integer_lookup_table):
        temperature = -1
    else:
        temperature, slope = integer_lookup_table[lookup_table_index]
        temperature = temperature >> temperature_shift_left
        temperature_slope_adjustment = slope * (adjusted_adc_value - lookup_table_index * lookup_table_adc_value_step)
        temperature_slope_adjustment = temperature_slope_adjustment >> slope_shift_left
    return temperature + temperature_slope_adjustment


# Read the data from file
adc_values, temperatures = read_data_from_file(DATA_FILE_NAME)

# Fit a polynomial
x = np.array(adc_values)
y = np.array(temperatures)
fit_degree = 7
coefficients = np.polyfit(x, y, fit_degree)
polynomial = np.poly1d(coefficients)

# Create a lookup table
lookup_table = create_lookup_table(polynomial)
# print out the lookup table
lookup_table_size = len(lookup_table)
print(f"Lookup table (the size is {lookup_table_size}):")
for entry in lookup_table:
    print(entry)

integer_lookup_table, temperature_shift_left, slope_shift_left = calculate_integer_based_lookup_table(lookup_table)
save_integer_based_lookup_table_in_C_header_format(integer_lookup_table, temperature_shift_left, slope_shift_left, HEADER_FILENAME)

# Generate points for plotting the fit curve and lookup table points
x_fit = np.linspace(min(x), max(x), 500)
y_fit = polynomial(x_fit)
x_plot = np.arange(min(x), max(x) + 1, 1)
y_plot = [estimate_temperature_from_integer_lookup_table(adc, integer_lookup_table, temperature_shift_left, slope_shift_left) for adc in x_plot]
print(y_plot)
# make sure that the data in y_plot is never decreasing (monothonic in positive direction). if it decreases, print an error and exit
for i in range(1, len(y_plot)):
    if y_plot[i] < y_plot[i-1]:
        print("Error: the data in y_plot is decreasing. Please check the lookup table.")
        print("You can try to remove the + 0.5 from this line: max_slope_after_shifting = int(max_slope * 2**slope_shift_left + 0.5)")
        exit(1)


# Plot
fig, ax = plt.subplots()
ax.plot(x, y, marker='o', linestyle='', label='Data')
ax.plot(x_fit, y_fit, label='Best fit curve')
ax.scatter(x_plot, y_plot, color='red', marker='.', label='Lookup Points', zorder=1)  # Changed to dots

# Title and labels
ax.set_title('Temperature vs. ADC Value of the Extruder')
ax.set_xlabel('ADC Value')
ax.set_ylabel('Temperature (degrees C)')

# Add legend to the plot
ax.legend()

# Show plot
plt.show()