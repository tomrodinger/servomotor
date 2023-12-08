#!/usr/bin/env python3

import glob
import matplotlib.pyplot as plt
import math
import cmath

# Get list of all relevant files
DIRECTORY = "logs"
GLOB_SEARCH_STRING = DIRECTORY + "/X_data_*"
GOERTZEL_ALGORITHM_RESULTS_FILENAME = DIRECTORY + "/goertzel_algorithm_results_floating_point_math.txt"

# This function reads a text file that has a list of data points. There are two columns.
# The first column is just the index and we don't need this. The second column contains
# the data we are interested in. Read it and return the list of values
def read_file_get_second_column(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()
    data = []
    for line in lines:
        data.append(int(line.split()[1]))
    return data

    
def goertzel_algorithm_floating_point_math(samples, sampling_rate, target_frequency):
    """
    Implements the Goertzel algorithm to find the single frequency component.
    
    :param samples: List of samples
    :param sampling_rate: Sampling rate in Hz
    :param target_frequency: Target frequency in Hz to detect in samples
    :return: tuple(real_component, imaginary_component)
    """
    
    # Initialize variables
    num_samples = len(samples)
    k = int(0.5 + (num_samples * target_frequency) / sampling_rate)  # The bin number
    omega = (2.0 * math.pi * k) / num_samples
    cosine = math.cos(omega)
    sine = math.sin(omega)
    coeff = 2.0 * cosine
    q0 = 0.0
    q1 = 0.0
    q2 = 0.0
    
    # Apply the algorithm
    for sample in samples:
        q0 = coeff * q1 - q2 + sample
        q2 = q1
        q1 = q0
    
    # Calculate the real and imaginary results
    # Taking into account leakage effect with cosine and sine adjustments
    real_part = (q1 - q2 * cosine)
    imaginary_part = (q2 * sine)
    
    return real_part, imaginary_part


files = sorted(glob.glob(GLOB_SEARCH_STRING), key=lambda x:int(x.split('_')[-1]))

if len(files) == 0:
    print("ERROR: No files found matching the search string: %s" % GLOB_SEARCH_STRING)
    exit(1)
# Loop through the files, read each one into a dataframe and plot it
goertzel_real_results = []
goertzel_imaginary_results = []
for filename in files:
    print("Processing file: %s" % filename)
    data = read_file_get_second_column(filename)
    print("Data:", data)
    goertzel_result = goertzel_algorithm_floating_point_math(data, len(data), 1.0)
    print("Goertzel:", goertzel_result)
    goertzel_real_results.append(goertzel_result[0])
    goertzel_imaginary_results.append(goertzel_result[1])

plt.scatter(goertzel_real_results, goertzel_imaginary_results, s=10) # s is the size of the points
# Adding a horizontal line at y=0 and a vertical line at x=0
plt.axhline(0, color='black', linewidth=1.0)
plt.axvline(0, color='black', linewidth=1.0)
# Adding a grid
plt.grid(True, which='both', linestyle='--', linewidth=0.5)
plt.xlabel('Real Part')
plt.ylabel('Imaginary Part')
plt.title('Goertzel Algorithm results for all data')
plt.legend() # To show a legend with the file names
plt.show()

    