#!/usr/bin/env python3

import sys
import math
import numpy as np
import os
import random
import copy
import glob
import re
import matplotlib.pyplot as plt

INPUT_FOLDER = "collected_data"
OUTPUT_FOLDER = "analysis_output"
GLOB_FILTER = "hall_calibration_one_rotation_*.txt"
OUTPUT_HEADER_FILENAME_PREFIX = OUTPUT_FOLDER + "/hall_sensor_constants_"
N_HALL_SENSORS = 3
SLOPE_CALCULATION_DELTA = 4
BEST_FIT_SEGMENT_MIN_LENGTH = 5
HALL_SENSOR_SHIFT = 30000
SLOPE_SHIFT_RIGHT = 22
OFFSET_SHIFT_RIGHT = 8
N_HALL_SENSORS = 3
PEAK_HALL_NUMBER = 2
MAX_OR_MIN = 1 # 1 means search for the global maximum, 0 means search for the global minimum
HALL_SAMPLES_PER_PRINT = 8
CALIBRATION_CAPTURE_STEP_SIZE = 128
#BLEND_EXTENT_SHIFT = 3
MIN_TO_MAX_THRESHOLD = 1000


# Get the product name from the command line. The product name is something like M1 or M2 or M3, etc.
# If the product name is not specified, then print out an error message and give the user a list of
# the available product names.
product_name = None
if len(sys.argv) == 2:
    product_name = sys.argv[1]
else:
    print("Error: please specify a product name on the command line")
    print("Usage: %s product-name" % (sys.argv[0]))
    print("Product names are M1, M2, M3, etc.")
    exit(1)
if product_name == "M1":
    import SETTINGS_M1
    ONE_REVOLUTION_ELECTRICAL_CYCLES = SETTINGS_M1.ONE_REVOLUTION_ELECTRICAL_CYCLES
    ONE_REVOLUTION_HALL_SENSOR_CYCLES = SETTINGS_M1.ONE_REVOLUTION_HALL_SENSOR_CYCLES
    N_COMMUTATION_STEPS = SETTINGS_M1.N_COMMUTATION_STEPS
    N_COMMUTATION_SUB_STEPS = SETTINGS_M1.N_COMMUTATION_SUB_STEPS
elif product_name == "M2":
    import SETTINGS_M2
    ONE_REVOLUTION_ELECTRICAL_CYCLES = SETTINGS_M2.ONE_REVOLUTION_ELECTRICAL_CYCLES
    ONE_REVOLUTION_HALL_SENSOR_CYCLES = SETTINGS_M2.ONE_REVOLUTION_HALL_SENSOR_CYCLES
    N_COMMUTATION_STEPS = SETTINGS_M2.N_COMMUTATION_STEPS
    N_COMMUTATION_SUB_STEPS = SETTINGS_M2.N_COMMUTATION_SUB_STEPS
elif product_name == "M3":
    import SETTINGS_M3
    ONE_REVOLUTION_ELECTRICAL_CYCLES = SETTINGS_M3.ONE_REVOLUTION_ELECTRICAL_CYCLES
    ONE_REVOLUTION_HALL_SENSOR_CYCLES = SETTINGS_M3.ONE_REVOLUTION_HALL_SENSOR_CYCLES
    N_COMMUTATION_STEPS = SETTINGS_M3.N_COMMUTATION_STEPS
    N_COMMUTATION_SUB_STEPS = SETTINGS_M3.N_COMMUTATION_SUB_STEPS
else:
    print("Error: unsupported product name:", product_name)
    print("You should update this executable and create a SETTINGS_product-name.py file")
    exit(1)

output_header_filename = f"{OUTPUT_HEADER_FILENAME_PREFIX}{product_name}.h"

# Calculate some constants
MAGNETIC_RING_POLES = ONE_REVOLUTION_HALL_SENSOR_CYCLES * 2
TOTAL_NUMBER_OF_SEGMENTS = MAGNETIC_RING_POLES * 3
ONE_CYCLE_DATA_POINTS = N_COMMUTATION_STEPS * N_COMMUTATION_SUB_STEPS * ONE_REVOLUTION_ELECTRICAL_CYCLES / CALIBRATION_CAPTURE_STEP_SIZE / HALL_SAMPLES_PER_PRINT
SENSOR_SEGMENT_RESOLUTION = N_COMMUTATION_STEPS * N_COMMUTATION_SUB_STEPS * ONE_REVOLUTION_ELECTRICAL_CYCLES / ONE_REVOLUTION_HALL_SENSOR_CYCLES / N_HALL_SENSORS
# do a sanity check here to make sure that SENSOR_SEGMENT_RESOLUTION is an integer
if SENSOR_SEGMENT_RESOLUTION != int(SENSOR_SEGMENT_RESOLUTION):
    print("Error: SENSOR_SEGMENT_RESOLUTION is not an integer:", SENSOR_SEGMENT_RESOLUTION)
    exit(1)
SENSOR_SEGMENT_RESOLUTION = int(SENSOR_SEGMENT_RESOLUTION)


class Plotting:
    def __init__(self):
        self.data = []

    def add_data(self, x_values, y_values):
        # If x_values is None, generate x_values starting from 0
        if x_values is None:
            x_values = list(range(len(y_values)))
        self.data.append((x_values, y_values))

    def save_to_file(self, filename, title='Scatter Plot', x_label='x-axis', y_label='y-axis', dpi=300):
        # Create the scatter plot with all the data lines
        plt.figure()
        for x_values, y_values in self.data:
            plt.plot(x_values, y_values, linestyle='-', marker='o', markersize=2) # Points joined by lines with smaller markers
        # Gridlines, labels, and title
        plt.grid(which='both', axis='both', linestyle='--', linewidth=0.5) # Horizontal and vertical gridlines
        plt.xlabel(x_label)
        plt.ylabel(y_label)
        plt.title(title)
        # Save the figure to a file
        plt.savefig(filename, format='png', dpi=dpi)
        plt.close() # Close the figure to free memory

    def clear_data(self):
        # Clear all the data lines
        self.data = []


def calculate_average_distance_between_lists(list1, list2):
    assert len(list1) == len(list2)
    sum = 0.0
    for l1, l2 in zip(list1, list2):
        distance = l2 - l1
        sum = sum + distance
    average = sum / len(list1)
    return average
        
class minima_and_maxima_finder:
    def __init__(self):
        self.current_minimum = 1000000
        self.current_minimum_index = 0
        self.current_maximum = -1000000
        self.current_maximum_index = 0
        self.minimum_or_maximum = None
        self.ignore_first = 1
        self.local_minima = []
        self.local_maxima = []
        self.local_minima_and_maxima = []
        self.local_minima_and_maxima_copy = []
        self.done = False

    def add_sensor_reading(self, d):
        if not self.done:
            if d < self.current_minimum:
                self.current_minimum = d
                self.current_minimum_index = i
            if d > self.current_maximum:
                self.current_maximum = d
                self.current_maximum_index = i
            if (d > self.current_minimum + MIN_TO_MAX_THRESHOLD) and (self.minimum_or_maximum != 1):
                print("Found a minimum at index", self.current_minimum_index)
                if self.ignore_first > 0:
                    print("Ignoring it")
                    self.ignore_first = self.ignore_first - 1
                else:
                    append_what = [self.current_minimum_index, self.current_minimum]
                    self.local_minima.append(append_what)
                    self.local_minima_and_maxima.append(append_what)
                    self.local_minima_and_maxima_copy.append(append_what)
                self.current_maximum = d
                self.minimum_or_maximum = 1
            if (d < self.current_maximum - MIN_TO_MAX_THRESHOLD) and (self.minimum_or_maximum != 0):
                print("Found a maximum at index", self.current_maximum_index)
                if self.ignore_first > 0:
                    print("Ignoring it")
                    self.ignore_first = self.ignore_first - 1
                else:
                    append_what = [self.current_maximum_index, self.current_maximum]
                    self.local_maxima.append(append_what)
                    self.local_minima_and_maxima.append(append_what)
                    self.local_minima_and_maxima_copy.append(append_what)
                self.current_minimum = d
                self.minimum_or_maximum = 0
            if (len(self.local_minima) == MAGNETIC_RING_POLES / 2) and (len(self.local_maxima) == MAGNETIC_RING_POLES / 2):
                self.done = True
            elif len(self.local_minima) > MAGNETIC_RING_POLES / 2:
                print("Error: collected too many local minima for some strange reason")
                exit(1)
            elif len(self.local_maxima) > MAGNETIC_RING_POLES / 2:
                print("Error: collected too many local maxima for some strange reason")
                exit(1)
        return self.done
        
    def get_local_minima():
        return self.local_minima

    def get_local_maxima():
        return self.local_maxima
    
    def save_minima_to_file(self, filename):
        with open(filename, "w") as fh:
            for m in self.local_minima:
                fh.write("%d %f\n" % (m[0], m[1]))

    def save_maxima_to_file(self, filename):
        with open(filename, "w") as fh:
            for m in self.local_maxima:
                fh.write("%d %f\n" % (m[0], m[1]))

    def save_minima_and_maxima_to_file(self, filename):
        with open(filename, "w") as fh:
            for m in self.local_minima_and_maxima:
                fh.write("%d %f\n" % (m[0], m[1]))
                
    def adjust_data_for_midline(self):
        midline = self.calculate_midline()
#        for d in self.local_minima:
#            d[1] = d[1] - midline
#        for d in self.local_maxima:
#            d[1] = d[1] - midline
        for d in self.local_minima_and_maxima:
            d[1] = d[1] - midline

    def adjust_data_by_offset(self, offset):
#        for d in self.local_minima:
#            d[1] = d[1] - midline
#        for d in self.local_maxima:
#            d[1] = d[1] - midline
        for d in self.local_minima_and_maxima:
            d[1] = d[1] + offset

    def get_minima_and_maxima(self):
        minima_and_maxima = []
        for m in self.local_minima_and_maxima:
            minima_and_maxima.append(m[1])
        return minima_and_maxima
                
    def rotate_minima_and_maxima(self):
        last_item = self.local_minima_and_maxima.pop()
        self.local_minima_and_maxima.insert(0, last_item)
        last_item = self.local_minima_and_maxima.pop()
        self.local_minima_and_maxima.insert(0, last_item)
        
    def calculate_difference(self):
        sum = 0.0
        for i in range(len(self.local_minima_and_maxima)):
            difference = self.local_minima_and_maxima[i][1] - self.local_minima_and_maxima_copy[i][1]
            squared_difference = float(difference) * float(difference)
            sum = sum + squared_difference
        mean_squared_difference = sum / len(self.local_minima_and_maxima)
        root_mean_squared_difference = math.sqrt(mean_squared_difference)
        return root_mean_squared_difference
        
    def calculate_midline(self):
        sum = 0.0
        for d in self.local_minima_and_maxima:
            minimum_or_maximum = d[1]
            sum = sum + float(minimum_or_maximum)
        average = sum / len(self.local_minima_and_maxima)
        return int(average)
    
    def calculate_magnitude(self):
        midline = self.calculate_midline()
        sum = 0.0
        for d in self.local_minima_and_maxima:
            minimum_or_maximum = d[1]
            sum = sum + abs(float(minimum_or_maximum - midline))
        average = sum / len(self.local_minima_and_maxima)
        return int(average)
        
    def calculate_average_cycle_distance(self):
        first_index = self.local_minima_and_maxima[0][0]
        last_index = self.local_minima_and_maxima[-1][0]
        return float(last_index - first_index) / (MAGNETIC_RING_POLES / 2.0 - 0.5)
        
    def get_minima_indeces(self):
        index_list = []
        for d in self.local_minima:
            i = d[0]
            index_list.append(i)
        return index_list

    def get_maxima_indeces(self):
        index_list = []
        for d in self.local_maxima:
            i = d[0]
            index_list.append(i)
        return index_list

        
def print_LUT_entry(fh, y1, y2, m, b, hall_n, next_hall_n):
    fh.write("   { %5d, %5d, %10d, %10d, %1d, %1d }, \\\n" % (min(y1, y2), max(y1, y2), m, b, hall_n, next_hall_n))

def get_correlation_coeff(x_data, y_data):
    m, b = np.polyfit(x_data, y_data, 1)
    correlation = np.corrcoef(x_data, y_data)[0, 1]
    if correlation > 0.5 or correlation < -0.5:
        return correlation
    else:
        return 0.0

def segment_calculations(fh, segment_number, data, x, previous_best_fit_hall_n, best_fit_hall_n):
#    print("Segment data:", data)
    n_data = len(data)
    x_data = range(x, x + n_data)
    y_data = data
    remove_margin = int(n_data / 8.2)
    x_data = x_data[remove_margin : len(data) - remove_margin]
    y_data = y_data[remove_margin : len(data) - remove_margin]
    n_data = len(x_data)

    min_x = x
    max_x = x + n_data - 1
    min_y = min(y_data)
    max_y = max(y_data)
    assert len(y_data) == n_data
    m, b = np.polyfit(x_data, y_data, 1)
    r = np.corrcoef(x_data, y_data)[0, 1]
    print("m, b, r:", m, b, r)

    m_reciprocal = 1.0 / m
    m_reciprocal_int = int(m_reciprocal * (1 << SLOPE_SHIFT_RIGHT) + 0.5)
    b_int = int((min_x - min_y * m_reciprocal) * (1 << OFFSET_SHIFT_RIGHT) + 0.5) + 1000
    b2 = (min_x - min_y * m_reciprocal)
#    print_LUT_entry(fh, max_y, min_y, m_reciprocal_int, b_int, previous_best_fit_hall_n, best_fit_hall_n)

    m, b = np.polyfit(y_data, x_data, 1)
    r = np.corrcoef(y_data, x_data)[0, 1]
    print("inverse: m, b, r:", m, b, r)

    m_int = int(m * (1 << SLOPE_SHIFT_RIGHT) + 0.5)
    b_int = int(b * (1 << OFFSET_SHIFT_RIGHT) + 0.5) + 1000
    print_LUT_entry(fh, max_y, min_y, m_int, b_int, previous_best_fit_hall_n, best_fit_hall_n)

    fh = open("segments/seg%d" % (segment_number), "w")
    for x, y in zip(x_data, y_data):
        best_fit_x = y * m + b
        output = "%f %f %f" % (x, y, best_fit_x)
        fh.write(output + "\n")
    fh.close()

def shift_data(data, shift_amounts):
    adjusted_data = []
    for d, i in zip(data, range(len(data))):
        d0 = d[0] + shift_amounts[0]
        d1 = d[1] + shift_amounts[1]
        d2 = d[2] + shift_amounts[2]
        adjusted_data.append([d0, d1, d2])
    return adjusted_data


def adjust_data(data, weights):
    adjusted_data = []
    for d, i in zip(data, range(len(data))):
        d0 = d[0]
        d1 = d[1]
        d2 = d[2]
        d0_adjusted = d0 * weights[0] + d1 * weights[1] + d2 * weights[2]
        d1_adjusted = d1 * weights[3] + d2 * weights[4] + d0 * weights[5]
        d2_adjusted = d2 * weights[6] + d0 * weights[7] + d1 * weights[8]
        adjusted_data.append([d0_adjusted, d1_adjusted, d2_adjusted])
    return adjusted_data

def get_change_list(data):
    d0_flag = 0
    d1_flag = 0
    d2_flag = 0
    last_change_sensor = 0
    last_change_direction = 0
    last_change_index = 0
    change_list = []
    for d, i in zip(data, range(len(data))):
        d0 = d[0]
        d1 = d[1]
        d2 = d[2]
        if d0 >= 0:
            if (d0_flag == 0):
                this_change_sensor = 0
                this_change_direction = 1
                this_change_index = i
                change_list.append([[this_change_sensor, this_change_direction, last_change_sensor, last_change_direction], this_change_index - last_change_index])
                last_change_sensor = 0
                last_change_direction = 1
                last_change_index = i
                d0_flag = 1
        else:
            if (d0_flag == 1):
                this_change_sensor = 0
                this_change_direction = 0
                this_change_index = i
                change_list.append([[this_change_sensor, this_change_direction, last_change_sensor, last_change_direction], this_change_index - last_change_index])
                last_change_sensor = 0
                last_change_direction = 0
                last_change_index = i
                d0_flag = 0
        if d1 >= 0:
            if (d1_flag == 0):
                this_change_sensor = 1
                this_change_direction = 1
                this_change_index = i
                change_list.append([[this_change_sensor, this_change_direction, last_change_sensor, last_change_direction], this_change_index - last_change_index])
                last_change_sensor = 1
                last_change_direction = 1
                last_change_index = i
                d1_flag = 1
        else:
            if (d1_flag == 1):
                this_change_sensor = 1
                this_change_direction = 0
                this_change_index = i
                change_list.append([[this_change_sensor, this_change_direction, last_change_sensor, last_change_direction], this_change_index - last_change_index])
                last_change_sensor = 1
                last_change_direction = 0
                last_change_index = i
                d1_flag = 0
        if d2 >= 0:
            if (d2_flag == 0):
                this_change_sensor = 2
                this_change_direction = 1
                this_change_index = i
                change_list.append([[this_change_sensor, this_change_direction, last_change_sensor, last_change_direction], this_change_index - last_change_index])
                last_change_sensor = 2
                last_change_direction = 1
                last_change_index = i
                d2_flag = 1
        else:
            if (d2_flag == 1):
                this_change_sensor = 2
                this_change_direction = 0
                this_change_index = i
                change_list.append([[this_change_sensor, this_change_direction, last_change_sensor, last_change_direction], this_change_index - last_change_index])
                last_change_sensor = 2
                last_change_direction = 0
                last_change_index = i
                d2_flag = 0
    return change_list


def get_change_list_v2(data):
    change_list = []
    last_d_sorted = 0
    last_change_direction = 0
    skip_these_first = 2
    DO_THIS_MANY = 28 * 6
    for d, i in zip(data, range(len(data))):
        d0 = d[0]
        d1 = d[1]
        d2 = d[2]
        d_sorted = sorted([(d0, 0), (d1, 1), (d2, 2)])
        if d_sorted[1][0] >= 0:
            this_change_direction = 0
        else:
            this_change_direction = 1
        d_sorted = (d_sorted[0][1], d_sorted[1][1], d_sorted[2][1])
        if d_sorted != last_d_sorted:
            this_change_sensor = d_sorted[1]
            this_change_index = i
            if skip_these_first == 0:
                change_list.append([[this_change_sensor, this_change_direction, last_change_sensor, last_change_direction], this_change_index - last_change_index])
            else:
                skip_these_first = skip_these_first - 1
            last_change_direction = this_change_direction
            last_change_sensor = this_change_sensor
            last_change_index = this_change_index
            last_d_sorted = d_sorted
        if len(change_list) >= DO_THIS_MANY:
            break
#    for cl in change_list:
#        print(cl)
#    exit()
    return change_list


def get_change_list_v3(data):
    change_list = []
    last_d_sorted = 0
    last_change_direction = 0
    skip_these_first = 2
    DO_THIS_MANY = 28 * 6
    for d, i in zip(data, range(len(data))):
        d0 = d[0]
        d1 = d[1]
        d2 = d[2]
        d_sorted = sorted([(d0, 0), (d1, 1), (d2, 2)])
        if d_sorted[1][0] >= 0:
            this_change_direction = 0
        else:
            this_change_direction = 1
        d_sorted = (d_sorted[0][1], d_sorted[1][1], d_sorted[2][1])
        if d_sorted != last_d_sorted:
            this_change_sensor = d_sorted[1]
            this_change_index = i
            if skip_these_first == 0:
                change_list.append([[this_change_sensor, this_change_direction, last_change_sensor, last_change_direction], this_change_index - last_change_index])
            else:
                skip_these_first = skip_these_first - 1
            last_change_direction = this_change_direction
            last_change_sensor = this_change_sensor
            last_change_index = this_change_index
            last_d_sorted = d_sorted
        if len(change_list) >= DO_THIS_MANY:
            break
#    for cl in change_list:
#        print(cl)
#    exit()
    return change_list


def save_data_to_file(data, filename):
    with open(filename, "w") as fh:
        for d, i in zip(data, range(len(data))):
            d0 = d[0]
            d1 = d[1]
            d2 = d[2]
            fh.write("%i %f %f %f\n" % (i, d0, d1, d2))

def calculate_score_based_on_deviation_from_ideal(change_list):
    total_run_length = 0
    for cl in change_list:
        run_length = cl[1]
        total_run_length = total_run_length + run_length
    if len(change_list) == 0:
        return 0.0
    ideal_run_length = float(total_run_length) / float(len(change_list))
    run_length_rmsd = 0.0
    for cl in change_list:
        run_length = cl[1]
        deviation = float(run_length) - ideal_run_length
        run_length_rmsd = run_length_rmsd + deviation * deviation * deviation * deviation
    run_length_rmsd = math.sqrt(run_length_rmsd / len(change_list))
    print("Total run length is %d   The ideal run length is: %f   Run length RMSD: %f" % (total_run_length, ideal_run_length, run_length_rmsd))
    return 1.0 / run_length_rmsd
    

def compute_score(change_list):
    change_list = sorted(change_list[2:])
    change_type_dict = {}
    for cl in change_list:
        key = "%d %d %d %d" % (cl[0][0], cl[0][1], cl[0][2], cl[0][3])
        run_length = cl[1]
#        print("key:", key, "run_length", run_length)
        if not key in change_type_dict.keys():
            change_type_dict[key] = run_length
        else:
            change_type_dict[key] = change_type_dict[key] + run_length
    sensor_positive_direction_good_score = 0
    sensor_negative_direction_good_score = 0
    sensor_positive_direction_bad_score = 0
    sensor_negative_direction_bad_score = 0
    for key in change_type_dict.keys():
        key_split = key.split()
        good_score = 0
        bad_score = 0
        if key_split[1] != key_split[3]: # detect if the direction is the opposite of the direction before
            good_score = change_type_dict[key]
        else:
            bad_score = change_type_dict[key]
        print("key:", key_split, "   good score:", good_score, "   bad score:", bad_score)
        if (key_split[0] == '0') and (key_split[2] == '1'):
            sensor_positive_direction_good_score = sensor_positive_direction_good_score + good_score
            sensor_positive_direction_bad_score = sensor_positive_direction_bad_score + bad_score
        elif (key_split[0] == '0') and (key_split[2] == '2'):
            sensor_negative_direction_good_score = sensor_negative_direction_good_score + good_score
            sensor_negative_direction_bad_score = sensor_negative_direction_bad_score + bad_score
        elif (key_split[0] == '1') and (key_split[2] == '2'):
            sensor_positive_direction_good_score = sensor_positive_direction_good_score + good_score
            sensor_positive_direction_bad_score = sensor_positive_direction_bad_score + bad_score
        elif (key_split[0] == '1') and (key_split[2] == '0'):
            sensor_negative_direction_good_score = sensor_negative_direction_good_score + good_score
            sensor_negative_direction_bad_score = sensor_negative_direction_bad_score + bad_score
        elif (key_split[0] == '2') and (key_split[2] == '0'):
            sensor_positive_direction_good_score = sensor_positive_direction_good_score + good_score
            sensor_positive_direction_bad_score = sensor_positive_direction_bad_score + bad_score
        elif (key_split[0] == '2') and (key_split[2] == '1'):
            sensor_negative_direction_good_score = sensor_negative_direction_good_score + good_score
            sensor_negative_direction_bad_score = sensor_negative_direction_bad_score + bad_score

    score = 0.0
    total_score = sensor_positive_direction_good_score + sensor_negative_direction_good_score + sensor_positive_direction_bad_score + sensor_negative_direction_bad_score
    sensor_positive_direction_score = sensor_positive_direction_good_score - sensor_positive_direction_bad_score
    sensor_negative_direction_score = sensor_negative_direction_good_score - sensor_negative_direction_bad_score
    if sensor_positive_direction_score > sensor_negative_direction_score:
        direction = 1
        confidence = (sensor_positive_direction_score - sensor_negative_direction_score) / total_score
        print("Sensors are triggering in the positive order, ie. 0 -> 1 -> 2 -> 0 ...")
        score = float(sensor_positive_direction_good_score - sensor_positive_direction_bad_score - sensor_negative_direction_good_score - sensor_negative_direction_bad_score)
        if sensor_positive_direction_bad_score + sensor_negative_direction_good_score + sensor_negative_direction_bad_score == 0:
            score = score + calculate_score_based_on_deviation_from_ideal(change_list)
    else:
        direction = 0
        if total_score == 0:
            confidence = 0.0
        else:
            confidence = (sensor_negative_direction_score - sensor_positive_direction_score) / total_score
        print("Sensors are triggering in the negative order, ie. 0 -> 2 -> 1 -> 0 ...")
        score = float(sensor_negative_direction_good_score - sensor_negative_direction_bad_score - sensor_positive_direction_good_score - sensor_positive_direction_bad_score)
        if sensor_negative_direction_bad_score + sensor_positive_direction_good_score + sensor_positive_direction_bad_score == 0:
            score = score + calculate_score_based_on_deviation_from_ideal(change_list)
    print("The level of confidence we have is:", confidence)
    print("The score is:", score)
    return score, direction, confidence

def random_change_to_weights(old_weights):
    change_index = random.randint(0, len(old_weights) - 1)
    change_amount = random.random() * 2.0 - 1.0 # generate a random number between -1 and 1
    change_amount = change_amount * 0.1
    new_weights = copy.deepcopy(old_weights)
    new_weights[change_index] = old_weights[change_index] + change_amount
    return new_weights


def random_change_to_weights_simplified(old_weights):
    while(1):
        change_index = random.randint(0, len(old_weights) - 1)
        if not change_index in [0, 3, 6]:
            break
    change_amount = random.random() * 2.0 - 1.0 # generate a random number between -1 and 1
    change_amount = change_amount * 0.1
    new_weights = copy.deepcopy(old_weights)
    new_weights[change_index] = old_weights[change_index] + change_amount
    return new_weights


def calculate_transition_function(data):
    result = []
    for d, i in zip(data, range(len(data))):
        d0 = d[0]
        d1 = d[1]
        d2 = d[2]
        d_sorted = sorted([(d0, 0), (d1, 1), (d2, 2)])
        top_d = d_sorted[2][0]
        middle_d = d_sorted[1][0]
        bottom_d = d_sorted[0][0]
        fraction = (top_d - middle_d) / (top_d - bottom_d)
        result.append([i, fraction])
    return result


def calculate_transition_function_v2(data):
    result = []
    for d, i in zip(data, range(len(data))):
        d0 = d[0]
        d1 = d[1]
        d2 = d[2]
        if ((d0 <= d1) and (d1 <= d2) or (d0 >= d1) and (d1 >= d2)):
            fraction = (d2 - d1) / abs(d2 - d0)
        if ((d1 <= d2) and (d2 <= d0) or (d1 >= d2) and (d2 >= d0)):
            fraction = (d0 - d2) / abs(d0 - d1)
        if ((d2 <= d0) and (d0 <= d1) or (d2 >= d0) and (d0 >= d1)):
            fraction = (d1 - d0) / abs(d1 - d2)
        else:
            print("Unexpected situation")
            exit(1)
        result.append([i, fraction])
    return result

def calculate_transition_function_v3(data):
    result = []
    max_numerator = -1000000000
    min_numerator = 1000000000
    best_fit_x_values = []
    best_fit_y_values = []
    previous_largest_sensor = None
    sensor_incremental_position = 0
    for d, i in zip(data, range(len(data))):
        d0 = int(d[0])
        d1 = int(d[1])
        d2 = int(d[2])
        if (d0 >= d1) and (d0 >= d2):
            largest_sensor = 0
            numerator = d1 - d2
            if(d2 > d1):
                denominator = d0 - d1
            else:
                denominator = d0 - d2
        elif (d1 >= d2) and (d1 >= d0):
            largest_sensor = 1
            numerator = d2 - d0
            if(d0 > d2):
                denominator = d1 - d2
            else:
                denominator = d1 - d0
        else:
            largest_sensor = 2
            numerator = d0 - d1
            if(d1 > d0):
                denominator = d2 - d0
            else:
                denominator = d2 - d1

        numerator >>= 3
        denominator >>= 3
        
        max_numerator = max(numerator, max_numerator)
        min_numerator = min(numerator, min_numerator)

        fraction = int(numerator * (SENSOR_SEGMENT_RESOLUTION >> 1) / denominator)
        fraction = fraction + (SENSOR_SEGMENT_RESOLUTION >> 1)

        if previous_largest_sensor == None:
            previous_largest_sensor = largest_sensor
        if largest_sensor != previous_largest_sensor:
            if largest_sensor - previous_largest_sensor == 1:
                sensor_incremental_position = sensor_incremental_position + SENSOR_SEGMENT_RESOLUTION
            elif largest_sensor - previous_largest_sensor == -1:
                sensor_incremental_position = sensor_incremental_position - SENSOR_SEGMENT_RESOLUTION
            elif largest_sensor - previous_largest_sensor == -2:
                sensor_incremental_position = sensor_incremental_position + SENSOR_SEGMENT_RESOLUTION
            else:
                sensor_incremental_position = sensor_incremental_position - SENSOR_SEGMENT_RESOLUTION
            previous_largest_sensor = largest_sensor
        
        result.append([i, fraction, sensor_incremental_position + fraction])
        
        best_fit_x_values.append(i)
        best_fit_y_values.append(sensor_incremental_position + fraction)

    print("Min to max numerator:", min_numerator, max_numerator)

    x_values = np.array(best_fit_x_values)
    y_values = np.array(best_fit_y_values)
    m, b = np.polyfit(x_values, y_values, 1)
    r = np.corrcoef(x_values, y_values)[0, 1]
    print("Final fit parameters: slope: %f intercept: %f" % (m, b))
    print("Final correlarion coefficient r:", r)
    print("Parts per million from 1:", (1.0 - r) * 1000000)
    
    max_x = -1000000000
    min_x = 1000000000
    max_x_error = -1.0e-10
    min_x_error = -1.0e-10
    rmsd = 0.0
    previous_x = -1.0e10
    monotonic = True
    i = 0
    for r in result:
        i = r[0]
        y = r[2]
        x = (y - b) / m
        x_error = float(i) - x
        max_x_error = max(max_x_error, x_error)
        min_x_error = max(min_x_error, x_error)
        rmsd = rmsd + x_error * x_error
        max_x = max(max_x, i)
        min_x = min(min_x, i)
        if x < previous_x:
            print(f"index: {i} x: {x} <--- NOT MONOTONIC")
            monotonic = False
        i = i + 1
        previous_x = x
    rmsd = math.sqrt(rmsd / len(result))
    print("Max x: %d   Min x: %d   ONE_CYCLE_DATA_POINTS: %d" % (max_x, min_x, ONE_CYCLE_DATA_POINTS))
    print("Max x error: %f   Min x error: %f   RMSD: %f" % (max_x_error, min_x_error, rmsd))
    max_error_rotations = max(abs(max_x_error), abs(min_x_error)) / ONE_CYCLE_DATA_POINTS
    print("Max error (rotations):", max_error_rotations)
    max_error_degrees = max_error_rotations * 360
    print("Max error (degrees):", max_error_degrees)
    max_error_mm = max_error_rotations * 20
    print("Max error (mm) assuming 20mm / rotation:", max_error_mm)
    if monotonic == False:
        print("Not monotonic")
        max_error_mm = 1000000

    return result, max_error_mm


def sort_sections(data):
    result = []
    for d, i in zip(data, range(len(data))):
        d0 = d[0]
        d1 = d[1]
        d2 = d[2]
        d_sorted = sorted([(d0, 0), (d1, 1), (d2, 2)])
        top_d = d_sorted[2][0]
        middle_d = d_sorted[1][0]
        bottom_d = d_sorted[0][0]
        result.append([bottom_d, middle_d, top_d])
    return result
    
    
    
def write_out_header_file(filename, weights):
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

# if the output folder does not exist, create it
if not os.path.exists(OUTPUT_FOLDER):
    os.makedirs(OUTPUT_FOLDER)

# if the file called f"{OUTPUT_FOLDER}/max_error_mm_from_all_trials" exists then erase it
max_error_mm_from_all_trials_filename = f"{OUTPUT_FOLDER}/max_error_mm_from_all_trials"
if os.path.exists(max_error_mm_from_all_trials_filename):
    os.remove(max_error_mm_from_all_trials_filename)

# if the file called f"{OUTPUT_FOLDER}/best_weights_from_all_trials" exists then erase it
best_weights_from_all_trials_filename = f"{OUTPUT_FOLDER}/best_weights_from_all_trials"
if os.path.exists(best_weights_from_all_trials_filename):
    os.remove(best_weights_from_all_trials_filename)



input_file_list = glob.glob(INPUT_FOLDER + "/" + GLOB_FILTER)
print("Found these input files:", input_file_list)

# lets iterate through all the files and process them
for input_filename in input_file_list:
    print("Processing:", input_filename)
    match = re.search(r'(\d+)\.txt$', input_filename)
    if match:
        item_number = match.group(1)
    else:
        print("Error: could not extract the item number from the filename:", input_filename)
        exit(1)
    print("The item number is:", item_number)
#    if item_number != "2":
#        continue

    data = []
    with open(input_filename) as fh:
        for line in fh:
            fields = line.strip().split()
            fields[0] = float(fields[0])
            fields[1] = float(fields[1])
            fields[2] = float(fields[2])
            data.append(fields)
    data = np.array(data)
    n_data_pp = len(data)
    # let's chop off 10% of the data on the frint and the back
    data = data[int(n_data_pp * 0.1) : int(n_data_pp * 0.9)]

    mamf0 = minima_and_maxima_finder()
    mamf1 = minima_and_maxima_finder()
    mamf2 = minima_and_maxima_finder()
    for d, i in zip(data, range(len(data))):
        mamf0.add_sensor_reading(d[0])
        mamf1.add_sensor_reading(d[1])
        mamf2.add_sensor_reading(d[2])

    mamf0.save_minima_and_maxima_to_file(f"{OUTPUT_FOLDER}/local_minima_and_maxima0_item{item_number}")
    mamf1.save_minima_and_maxima_to_file(f"{OUTPUT_FOLDER}/local_minima_and_maxima1_item{item_number}")
    mamf2.save_minima_and_maxima_to_file(f"{OUTPUT_FOLDER}/local_minima_and_maxima2_item{item_number}")

    midline0 = mamf0.calculate_midline()
    midline1 = mamf1.calculate_midline()
    midline2 = mamf2.calculate_midline()
    print("The midlines are: %d %d %d" % (midline0, midline1, midline2))

    data = shift_data(data, [-midline0, -midline1, -midline2])
    save_data_to_file(data, f"{OUTPUT_FOLDER}/data_minus_midline_item{item_number}")

    max_error_mm_from_all_trials = 1.0e20
    best_weights_from_all_trials = None
    plotting = Plotting()
    for trial in range(3):
        weights = [1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        max_error_mm = None
        new_max_error_mm_list = []
        iteration_number_list = []
        for i in range(100):
            if i == 0:
                new_weights = weights
            else:
                new_weights = random_change_to_weights(weights)
        #    new_weights = random_change_to_weights_simplified(weights)
            new_adjusted_data = adjust_data(data, new_weights)
            change_list = get_change_list_v3(new_adjusted_data)
            new_score, direction, confidence = compute_score(change_list)
            new_transition_function, new_max_error_mm = calculate_transition_function_v3(new_adjusted_data)
            print("The new_max_error_mm is:", new_max_error_mm)
            if (max_error_mm == None) or (new_max_error_mm < max_error_mm):
                adjusted_data = new_adjusted_data
                new_max_error_mm_list.append(new_max_error_mm)
                iteration_number_list.append(i)
                if new_max_error_mm < max_error_mm_from_all_trials:
                    best_weights_from_all_trials = new_weights
                    max_error_mm_from_all_trials = new_max_error_mm
                weights = new_weights
                score = new_score
                transition_function = new_transition_function
                max_error_mm = new_max_error_mm
                print(f"Found a better maximum error {new_max_error_mm} mm with these weights: {new_weights}")
            else:
                print(f"The previous maximum error {max_error_mm} mm with these weights: {new_weights} was better")

        plotting.add_data(iteration_number_list, new_max_error_mm_list)
        plotting.save_to_file(f"{OUTPUT_FOLDER}/max_error_mm_item{item_number}.png", "Max Error (mm) Vs. Iteration", "Iteration", "Max Error (mm)", dpi=600)

        with open(f"{OUTPUT_FOLDER}/new_max_error_mm_item{item_number}", "w") as fh:
            for new_max_error_mm in new_max_error_mm_list:
                fh.write("%f\n" % (new_max_error_mm))

        save_data_to_file(adjusted_data, f"{OUTPUT_FOLDER}/optimized_data_item{item_number}_trial{trial}")

        sorted_sections = sort_sections(adjusted_data)
        with open(f"{OUTPUT_FOLDER}/sorted_sections_item{item_number}", "w") as fh:
            for ss in sorted_sections:
                fh.write("%F %f %f\n" % (ss[0], ss[1], ss[2]))

        with open(f"{OUTPUT_FOLDER}/transition_function_item{item_number}", "w") as fh:
            for tf in transition_function:
                fh.write("%d %f %f\n" % (tf[0], tf[1], tf[2]))

        write_out_header_file(output_header_filename, weights)
        print("Wrote out the header file:", output_header_filename)

    # let's append the max_error_mm_from_all_trials value to a file along with the item_number
    with open(max_error_mm_from_all_trials_filename, "a") as fh:
        fh.write("%s %f\n" % (item_number, max_error_mm_from_all_trials))
    # let's append the best_weights_from_all_trials value to a file along with the item_number
    # but first we need to convert the weights, which is a list of floating point numbers inro a string with each weight separated by a space
    weights_string = ""
    for w in best_weights_from_all_trials:
        weights_string = weights_string + " " + str(w)
    with open(best_weights_from_all_trials_filename, "a") as fh:
        fh.write("%s %s\n" % (item_number, weights_string))
