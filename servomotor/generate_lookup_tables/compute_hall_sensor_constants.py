#!/usr/bin/env python3

import sys
import math
import numpy as np
import os
import random
import copy

OUTPUT_HEADER_FILENAME_PREFIX = "../Src/hall_sensor_constants_"
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


# Get the product name from the command line. The product name is something like M1, M2, M3, M4 etc.
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
elif product_name == "M4":
    import SETTINGS_M4
    ONE_REVOLUTION_ELECTRICAL_CYCLES = SETTINGS_M4.ONE_REVOLUTION_ELECTRICAL_CYCLES
    ONE_REVOLUTION_HALL_SENSOR_CYCLES = SETTINGS_M4.ONE_REVOLUTION_HALL_SENSOR_CYCLES
    N_COMMUTATION_STEPS = SETTINGS_M4.N_COMMUTATION_STEPS
    N_COMMUTATION_SUB_STEPS = SETTINGS_M4.N_COMMUTATION_SUB_STEPS
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
            monotonic = False
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

    
data = []

with open("hall_calibration_one_rotation.txt") as fh:
    for line in fh:
        fields = line.strip().split()
        fields[0] = float(fields[0])
        fields[1] = float(fields[1])
        fields[2] = float(fields[2])
        data.append(fields)
data = np.array(data)

mamf0 = minima_and_maxima_finder()
mamf1 = minima_and_maxima_finder()
mamf2 = minima_and_maxima_finder()
for d, i in zip(data, range(len(data))):
    mamf0.add_sensor_reading(d[0])
    mamf1.add_sensor_reading(d[1])
    mamf2.add_sensor_reading(d[2])

mamf0.save_minima_and_maxima_to_file("local_minima_and_maxima0")
mamf1.save_minima_and_maxima_to_file("local_minima_and_maxima1")
mamf2.save_minima_and_maxima_to_file("local_minima_and_maxima2")

midline0 = mamf0.calculate_midline()
midline1 = mamf1.calculate_midline()
midline2 = mamf2.calculate_midline()
print("The midlines are: %d %d %d" % (midline0, midline1, midline2))

data = shift_data(data, [-midline0, -midline1, -midline2])
save_data_to_file(data, "data_minus_midline")

#weights = [0.0, -0.2, -0.7, 0.0, 0.0, 0.2]
weights = [1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
adjusted_data = adjust_data(data, weights)
#save_data_to_file(adjusted_data, "adjusted_data")
change_list = get_change_list_v3(adjusted_data)
score, direction, confidence = compute_score(change_list)
dummy, max_error_mm = calculate_transition_function_v3(adjusted_data)

for i in range(1000):
    new_weights = random_change_to_weights(weights)
#    new_weights = random_change_to_weights_simplified(weights)
    adjusted_data = adjust_data(data, new_weights)
    change_list = get_change_list_v3(adjusted_data)
    new_score, direction, confidence = compute_score(change_list)
    dummy, new_max_error_mm = calculate_transition_function_v3(adjusted_data)
#    if new_score > score:
    if new_max_error_mm < max_error_mm:
        weights = new_weights
        score = new_score
        max_error_mm = new_max_error_mm
#        print("Found a better score %f with these weights:" % (new_score), new_weights)
        print("Found a better maximum error (mm) %f with these weights:" % (new_max_error_mm), new_weights)
        save_data_to_file(adjusted_data, "optimized_data")
    print("Best weight so far:", weights)

sorted_sections = sort_sections(adjusted_data)
with open("sorted_sections", "w") as fh:
    for ss in sorted_sections:
        fh.write("%F %f %f\n" % (ss[0], ss[1], ss[2]))

transition_function, max_error_mm = calculate_transition_function_v3(adjusted_data)
with open("transition_function", "w") as fh:
    for tf in transition_function:
        fh.write("%d %f %f\n" % (tf[0], tf[1], tf[2]))

write_out_header_file(output_header_filename, weights)
print("Wrote out the header file:", output_header_filename)

exit()

################################################################################################################################

for cl in change_list:
    if cl[0][1] == cl[0][3]:
        score = -cl[1]
    else:
        score = cl[1]
    print(cl, score)
    
exit()


#    d1_adjusted = d1 - d2 * 0.7
#    d2_adjusted = d2 + d1 * 0.2
#    d0_adjusted = d0 - d2 * 0.2
#    fh.write("%i %f %f %f %f %f %f\n" % (i, d0, d1, d2, d0_adjusted, d1_adjusted, d2_adjusted))



with open("adjusted_data", "w") as fh:
    for d, i in zip(data, range(len(data))):
        d0 = d[0] - midline0
        d1 = d[1] - midline1
        d2 = d[2] - midline2
        d1_adjusted = d1 - d2 * 0.7
        d2_adjusted = d2 + d1 * 0.2
        d0_adjusted = d0 - d2 * 0.2
        fh.write("%i %f %f %f %f %f %f\n" % (i, d0, d1, d2, d0_adjusted, d1_adjusted, d2_adjusted))
        
exit()

mamf0.adjust_data_for_midline()
mamf1.adjust_data_for_midline()
mamf2.adjust_data_for_midline()

mamf0.save_minima_and_maxima_to_file("local_minima_and_maxima_adjusted_for_midline0")
mamf1.save_minima_and_maxima_to_file("local_minima_and_maxima_adjusted_for_midline1")
mamf2.save_minima_and_maxima_to_file("local_minima_and_maxima_adjusted_for_midline2")

cycle_length0 = mamf0.calculate_average_cycle_distance()
cycle_length1 = mamf1.calculate_average_cycle_distance()
cycle_length2 = mamf2.calculate_average_cycle_distance()
cycle_length = (cycle_length0 + cycle_length1 + cycle_length2) / 3.0
print("Cycle length: %f   individual cycle lengths: %f %f %f" % (cycle_length, cycle_length0, cycle_length1, cycle_length2))

minima_indeces0 = mamf0.get_minima_indeces()
minima_indeces1 = mamf1.get_minima_indeces()
minima_indeces2 = mamf2.get_minima_indeces()
maxima_indeces0 = mamf0.get_maxima_indeces()
maxima_indeces1 = mamf1.get_maxima_indeces()
maxima_indeces2 = mamf2.get_maxima_indeces()
minima_distance01 = calculate_average_distance_between_lists(minima_indeces0, minima_indeces1) / cycle_length
minima_distance02 = calculate_average_distance_between_lists(minima_indeces0, minima_indeces2) / cycle_length
minima_distance12 = calculate_average_distance_between_lists(minima_indeces1, minima_indeces2) / cycle_length
maxima_distance01 = calculate_average_distance_between_lists(maxima_indeces0, maxima_indeces1) / cycle_length
maxima_distance02 = calculate_average_distance_between_lists(maxima_indeces0, maxima_indeces2) / cycle_length
maxima_distance12 = calculate_average_distance_between_lists(maxima_indeces1, maxima_indeces2) / cycle_length
print("minima distances: %f %f %f   maxima distances: %f %f %f" % (minima_distance01, minima_distance02, minima_distance12, maxima_distance01, maxima_distance02, maxima_distance12))

exit()

with open("rmsd_maxima_and_minima", "w") as fh:
    for i in range(int(MAGNETIC_RING_POLES / 2)):
        rmsd0 = mamf0.calculate_difference()
        rmsd1 = mamf1.calculate_difference()
        rmsd2 = mamf2.calculate_difference()
        rmsd = math.sqrt(rmsd0 * rmsd0 + rmsd1 * rmsd1 + rmsd2 * rmsd2)
        print("%d times rotated" % (i))
        print("RMSD0 =", rmsd0)
        print("RMSD1 =", rmsd1)
        print("RMSD2 =", rmsd2)
        print("RMSD =", rmsd)
        fh.write("%f %f %f %f\n" % (rmsd, rmsd0, rmsd1, rmsd2))
        mamf0.rotate_minima_and_maxima()
        mamf1.rotate_minima_and_maxima()
        mamf2.rotate_minima_and_maxima()

magnitude0 = mamf0.calculate_magnitude()
magnitude1 = mamf1.calculate_magnitude()
magnitude2 = mamf2.calculate_magnitude()
print("The magnitudes are: %d %d %d" % (magnitude0, magnitude1, magnitude2))

with open("six_step_flags", "w") as fh:
    for d, i in zip(data, range(len(data))):
        d0 = d[0]
        d1 = d[1]
        d2 = d[2]
        six_step_flags = 0
        if d0 > midline0:
            six_step_flags = six_step_flags | 1
        if d1 > midline1:
            six_step_flags = six_step_flags | 2
        if d2 > midline2:
            six_step_flags = six_step_flags | 4
        print("six step flags:", six_step_flags)
        fh.write("%d %d %d %d %d %d %d %d %d %d %d\n" % (i, d0, midline0, d0 > midline0, d1, midline1, d1 > midline1, d2, midline2, d2 > midline2, six_step_flags))

MIDLINE_SHIFT_FRACTION = 0.3
for shift_type in range(6):
    with open("six_step_flags_shift%d" % (shift_type), "w") as fh:
        for d, i in zip(data, range(len(data))):
            shifted_midline0 = midline0
            shifted_midline1 = midline1
            shifted_midline2 = midline2
            if i % 2 == 0:
                shift_direction = 1.0
            else:
                shift_direction = -1.0
            if (i >> 1) % 3 == 0:
                shifted_midline0 = shifted_midline0 + (magnitude0 * MIDLINE_SHIFT_FRACTION) * shift_direction
            elif (i >> 1) % 3 == 1:
                shifted_midline1 = shifted_midline1 + (magnitude1 * MIDLINE_SHIFT_FRACTION) * shift_direction
            else:
                shifted_midline2 = shifted_midline2 + (magnitude2 * MIDLINE_SHIFT_FRACTION) * shift_direction
            d0 = d[0]
            d1 = d[1]
            d2 = d[2]
            six_step_flags = 0
            if d0 > shifted_midline0:
                six_step_flags = six_step_flags | 1
            if d1 > shifted_midline1:
                six_step_flags = six_step_flags | 2
            if d2 > shifted_midline2:
                six_step_flags = six_step_flags | 4
            fh.write("%d %d %d %d %d %d %d %d %d %d %d\n" % (i, d0, shifted_midline0, d0 > shifted_midline0, d1, shifted_midline1, d1 > shifted_midline1, d2, shifted_midline2, d2 > shifted_midline2, six_step_flags))


exit()

fh = open("hall_sensor_slopes", "w")
fh2 = open("linearity_error", "w")
fh3 = open("best_fit_r", "w")
fh6 = open("shift_and_slope_reciprocal", "w")
fh7 = open("shift_and_slope_reciprocal_rounded", "w")
fh8 = open(output_header_filename, "w")
fh9 = open("peak_hall_number", "w")

fh8.write("#ifndef __LOOKUP_TABLE2__\n")
fh8.write("#define __LOOKUP_TABLE2__\n")
fh8.write("\n")
fh8.write("#define TOTAL_NUMBER_OF_SEGMENTS %d\n" % (TOTAL_NUMBER_OF_SEGMENTS))
fh8.write("#define HALL_SENSOR_SHIFT %d\n" % (HALL_SENSOR_SHIFT))
fh8.write("#define SLOPE_SHIFT_RIGHT %d\n" % (SLOPE_SHIFT_RIGHT))
fh8.write("#define HALL_SAMPLES_PER_PRINT %d\n" % (HALL_SAMPLES_PER_PRINT))
fh8.write("#define CALIBRATION_CAPTURE_STEP_SIZE %d // this should be slow so that data can be captured and collected accurately\n" % (CALIBRATION_CAPTURE_STEP_SIZE))
fh8.write("#define BLEND_EXTENT_SHIFT %d\n" % (BLEND_EXTENT_SHIFT))

fh8.write("\n")
fh8.write("struct hall_minima_and_maxima_struct {\n")
fh8.write("   uint16_t minimum_or_maximum;\n")
fh8.write("};\n")
fh8.write("\n")
fh8.write("#define HALL0_MINIMA_AND_MAXIMA_INITIALIZER { \\\n")
minima_and_maxima0 = mamf0.get_minima_and_maxima()
for m in minima_and_maxima0:
    fh8.write("%d, \\\n" % (m))
fh8.write("}\n\n")

fh8.write("\n")
fh8.write("#define HALL1_MINIMA_AND_MAXIMA_INITIALIZER { \\\n")
minima_and_maxima1 = mamf1.get_minima_and_maxima()
for m in minima_and_maxima1:
    fh8.write("%d, \\\n" % (m))
fh8.write("}\n\n")

fh8.write("\n")
fh8.write("#define HALL2_MINIMA_AND_MAXIMA_INITIALIZER { \\\n")
minima_and_maxima2 = mamf2.get_minima_and_maxima()
for m in minima_and_maxima2:
    fh8.write("%d, \\\n" % (m))
fh8.write("}\n\n")

fh8.write("\n")
fh8.write("struct hall_decode_data_struct {\n")
fh8.write("   uint16_t min;\n")
fh8.write("   uint16_t max;\n")
fh8.write("   int32_t slope;\n")
fh8.write("   int32_t offset;\n")
fh8.write("   uint8_t active_hall_sensor1;\n")
fh8.write("   uint8_t active_hall_sensor2;\n")
fh8.write("};\n")
fh8.write("\n")
fh8.write("#define HALL_DECODE_DATA_INITIALIZER { \\\n")

segment_start_index = 0
segment_end_index = 0
total_number_of_segments = 0
peak_hall_value = -1000000
max_hall_index = None
max_hall_segment = None
found_new_peak = 0

one_cycle_data = []
previous_max_slope_hall_n = -1
previous_max_best_fit_hall_n = -1
previous_min_best_fit_hall_n = -1
max_best_fit_segment_end_index = 0
min_best_fit_segment_end_index = 0
positive_or_negative_best_fit = None
ignore_first_qualifed_segments = 2
i0 = 0
im = i0 + SLOPE_CALCULATION_DELTA
i1 = im + SLOPE_CALCULATION_DELTA
hall_avg_value0 = 0
hall_avg_value1 = 0
hall_avg_value2 = 0
n_avg = 0
while total_number_of_segments < TOTAL_NUMBER_OF_SEGMENTS:
    print("im = %d" % (im))
    if i1 >= len(data):
        print("Error: ran out of data")
        exit(1)
    d0 = data[i0]
    dm = data[im]
    d1 = data[i1]
    if n_avg < ONE_CYCLE_DATA_POINTS:
        hall_avg_value0 = hall_avg_value0 + d0[0]
        hall_avg_value1 = hall_avg_value1 + d0[1]
        hall_avg_value2 = hall_avg_value2 + d0[2]
        n_avg = n_avg + 1
    else:
        print("Finished averaging over one cycle with %d points" % (ONE_CYCLE_DATA_POINTS))
    signed_slope0 = d1[0] - d0[0]
    signed_slope1 = d1[1] - d0[1]
    signed_slope2 = d1[2] - d0[2]
    slope0 = abs(d1[0] - d0[0])
    slope1 = abs(d1[1] - d0[1])
    slope2 = abs(d1[2] - d0[2])
    signed_linearity_error0 = (d0[0] + d1[0]) / 2.0 - dm[0]
    signed_linearity_error1 = (d0[1] + d1[1]) / 2.0 - dm[1]
    signed_linearity_error2 = (d0[2] + d1[2]) / 2.0 - dm[2]
    linearity_error0 = abs(signed_linearity_error0)
    linearity_error1 = abs(signed_linearity_error1)
    linearity_error2 = abs(signed_linearity_error2)
    output = "%f %f %f %f" % (im, linearity_error0, linearity_error1, linearity_error2)
    fh2.write(output + "\n")

    best_fit_x_values = range(i0, i1 + 1)
    best_fit_y_values0 = data[i0 : i1 + 1, 0]
    best_fit_y_values1 = data[i0 : i1 + 1, 1]
    best_fit_y_values2 = data[i0 : i1 + 1, 2]
    r0 = get_correlation_coeff(best_fit_x_values, best_fit_y_values0)
    r1 = get_correlation_coeff(best_fit_x_values, best_fit_y_values1)
    r2 = get_correlation_coeff(best_fit_x_values, best_fit_y_values2)
    best_fit_list = [(r0, 0), (r1, 1), (r2, 2)]
    best_fit_list.sort()
    max_best_fit_r = best_fit_list[2][0]
    min_best_fit_r = best_fit_list[0][0]
    max_best_fit_hall_n = best_fit_list[2][1]
    min_best_fit_hall_n = best_fit_list[0][1]
    output = "%f %f %f %f %f %f %f %f %f" % (im, r0, r1, r2, max_best_fit_r, min_best_fit_r, dm[0], dm[1], dm[2])
    fh3.write(output + "\n")

    if ignore_first_qualifed_segments == 0:
        one_cycle_data.append([im, dm[0], dm[1], dm[2]])
        if ((MAX_OR_MIN == 1) and (dm[PEAK_HALL_NUMBER] > peak_hall_value)) or ((MAX_OR_MIN == 0) and (dm[PEAK_HALL_NUMBER] < peak_hall_value)):
#            peak_hall_value = max(dm[0], dm[1], dm[2])
            peak_hall_value = dm[PEAK_HALL_NUMBER]
            max_hall_index = im
            found_new_peak = 1
            max_hall_segment = None
            print("******************** Found a maximum hall sensor reading: max_hall_index %d   max_best_fit_r %f   min_best_fit_r %f" % (max_hall_index, max_best_fit_r, min_best_fit_r))
            fh9.write("%d %d %d %d\n" % (im, peak_hall_value, max_hall_index, PEAK_HALL_NUMBER))


    m = None
    b = None
    if max_best_fit_hall_n != previous_max_best_fit_hall_n:
        max_best_fit_segment_start_index = max_best_fit_segment_end_index + 1
        max_best_fit_segment_end_index = im - 1
        max_best_fit_segment_len = max_best_fit_segment_end_index - max_best_fit_segment_start_index + 1
        if max_best_fit_segment_len >= BEST_FIT_SEGMENT_MIN_LENGTH:
            print("Found new MAX best fit segment from %d to %d with qualified length %d" % (max_best_fit_segment_start_index, max_best_fit_segment_end_index, max_best_fit_segment_len))
            if ignore_first_qualifed_segments == 0:
                if positive_or_negative_best_fit == None:
                    positive_or_negative_best_fit = 1
                elif positive_or_negative_best_fit == 1:
                    print("Error: a positive best fit does not follow a negative one. problem with the data. maybe noise")
                    exit(1)
                else:
                    positive_or_negative_best_fit = 1
                print("Doing calculations for segment %d" % (total_number_of_segments))
                segment_calculations(fh8, total_number_of_segments, data[max_best_fit_segment_start_index : max_best_fit_segment_end_index + 1, previous_max_best_fit_hall_n], max_best_fit_segment_start_index, previous_max_best_fit_hall_n, min_best_fit_hall_n)
                if found_new_peak and (MAX_OR_MIN == 0):
                    max_hall_segment = total_number_of_segments
                    found_new_peak = 0
                    print("*** The peak hall sensor reading is at segment %d" % (max_hall_segment))
                total_number_of_segments = total_number_of_segments + 1
            else:
                print("Ignoring this qualified segment near the start of the data")
                ignore_first_qualifed_segments = ignore_first_qualifed_segments - 1
        else:
            print("Found new MAX best fit segment with length %d" % (max_best_fit_segment_len))
        previous_max_best_fit_hall_n = max_best_fit_hall_n
    if min_best_fit_hall_n != previous_min_best_fit_hall_n:
        min_best_fit_segment_start_index = min_best_fit_segment_end_index + 1
        min_best_fit_segment_end_index = im - 1
        min_best_fit_segment_len = min_best_fit_segment_end_index - min_best_fit_segment_start_index + 1
        if min_best_fit_segment_len >= BEST_FIT_SEGMENT_MIN_LENGTH:
            print("Found new min best fit segment from %d to %d with qualified length %d" % (min_best_fit_segment_start_index, min_best_fit_segment_end_index, min_best_fit_segment_len))
            if ignore_first_qualifed_segments == 0:
                if positive_or_negative_best_fit == None:
                    positive_or_negative_best_fit = -1
                elif positive_or_negative_best_fit == -1:
                    print("Error: a negative best fit does not follow a positive one. problem with the data. maybe noise")
                    exit(1)
                else:
                    positive_or_negative_best_fit = -1
                print("Doing calculations for segnet %d" % (total_number_of_segments))
                segment_calculations(fh8, total_number_of_segments, data[min_best_fit_segment_start_index : min_best_fit_segment_end_index + 1, previous_min_best_fit_hall_n], min_best_fit_segment_start_index, previous_min_best_fit_hall_n, max_best_fit_hall_n)
                if found_new_peak and (MAX_OR_MIN == 1):
                    max_hall_segment = total_number_of_segments
                    found_new_peak = 0
                    print("*** The peak hall sensor reading is at segment %d" % (max_hall_segment))
                total_number_of_segments = total_number_of_segments + 1
            else:
                print("Ignoring this qualified segment near the start of the data")
                ignore_first_qualifed_segments = ignore_first_qualifed_segments - 1
        else:
            print("Found new min best fit segment with length %d" % (min_best_fit_segment_len))
        previous_min_best_fit_hall_n = min_best_fit_hall_n

    i0 = i0 + 1
    im = im + 1
    i1 = i1 + 1

if n_avg != ONE_CYCLE_DATA_POINTS:
    print("Error: did not find enough data to calculate the average hall values over one cycle")
    exit(1)

if max_hall_segment == None:
    print("Error: The hall sensor peak occured in an unknown segment")
    exit(1)

hall_avg_value0 = hall_avg_value0 / n_avg
hall_avg_value1 = hall_avg_value1 / n_avg
hall_avg_value2 = hall_avg_value2 / n_avg

fh8.write("}\n\n")
fh8.write("#define MAX_HALL_SEGMENT %d\n" % (max_hall_segment))
fh8.write("#define PEAK_HALL_NUMBER %d\n" % (PEAK_HALL_NUMBER))
fh8.write("#define HALL1_MIDPOINT %d\n" % (midline0))
fh8.write("#define HALL2_MIDPOINT %d\n" % (midline1))
fh8.write("#define HALL3_MIDPOINT %d\n" % (midline2))
fh8.write("\n")
fh8.write("#endif\n")
fh8.write("\n")

fh.close()
fh2.close()
fh3.close()
fh6.close()
fh7.close()
fh8.close()
fh9.close()

print("max_hall_index:", max_hall_index)

fh10 = open("hall_sensor_data_both_directions.txt", "w")
im = max_hall_index
dir = 1
for i in range(len(data) * 3):
    dm = data[im % len(data)]
    fh10.write("%d %d %d\n" % (dm[0], dm[1], dm[2]))
    if dir == 1:
        if im == len(data) - 1:
            dir = -1
    else:
        if im == 0:
            dir = 1
    im = im + dir

fh10.close()



fh = open("one_cycle_data.txt", "w")
im = max_hall_index
dir = 1
for d in one_cycle_data:
    six_step_flags = 0
    if d[1] > hall_avg_value0:
        six_step_flags = six_step_flags | 1
    if d[2] > hall_avg_value1:
        six_step_flags = six_step_flags | 2
    if d[3] > hall_avg_value2:
        six_step_flags = six_step_flags | 4
    fh.write("%d %d %d %d %d\n" % (d[0], d[1], d[2], d[3], six_step_flags))
fh.close()

ret = os.system("cp %s ./LookupTable2.h" % (output_header_filename))
if ret != 0:
    print("Error: Cannot copy file from %s to ./LookupTable.h")
    exit(1)
print("Compiling the C program called one_cycle_calculation.c")
ret = os.system("gcc one_cycle_calculations.c -o one_cycle_calculations")
if ret != 0:
    print("Error: compile of one_cycle_calculations.c failed. Check the program please.")
    exit(1)
ret = os.system("./one_cycle_calculations")
if ret != 0:
    print("Error: cannot execute one_cycle_calculations. Check to make sure that it was able to compile.")
    exit(1)

x_data = []
y_data = []
fh = open("one_cycle_data_vs_index.txt", "r")
for d in fh:
    fields = d.strip().split()
    x = fields[0]
    y = fields[1]
#    print(float(x), float(y))
    x_data.append(float(x))
    y_data.append(float(y))

fh.close()

x_data = np.array(x_data)
y_data = np.array(y_data)
m, b = np.polyfit(x_data, y_data, 1)
r = np.corrcoef(x_data, y_data)[0, 1]
print("Final fit parameters: slope: %f intercept: %f" % (m, b))
print("Final correlarion coefficient r:", r)
print("Parts per million from 1:", (1.0 - r) * 1000000)
