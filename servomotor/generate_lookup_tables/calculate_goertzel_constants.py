#!/usr/bin/env python3
import glob
import numpy as np
import math
import cmath
import matplotlib.pyplot as plt

GOERTZEL_ALGORITHM_N_SAMPLES = 80
HASH_DEFINES_SAVE_FILENAME = "../Src/goertzel_algorithm_constants.h"

MIN_INT32 = -(1 << 31)
MAX_INT32 = (1 << 31) - 1
MIN_INT64 = -(1 << 63)
MAX_INT64 = (1 << 63) - 1

min_int32 = MAX_INT32
max_int32 = MIN_INT32
min_int64 = MAX_INT64
max_int64 = MIN_INT64
max_percent_error = 0
normalized_phase_list = []
color_list = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'w']  # Add more colors if there are more than 8 datasets 
color_index = 0

MAX_INT_PERCENTAGE_ALLOWED = 30.0
MAX_PERCENT_ERROR_ALLOWED = 1.0

def record_min_and_max(x, n_bits):
    global min_int32
    global max_int32
    global min_int64
    global max_int64
    if n_bits == 32:
        if x < min_int32:
            min_int32 = x
        if x > max_int32:
            max_int32 = x
    elif n_bits == 64:
        if x < min_int64:
            min_int64 = x
        if x > max_int64:
            max_int64 = x
    else:
        print("This many bits is not supported by the record_min_and_max function:", n_bits)
        exit(1)


def record_min_and_max_error(x, x_ref):
    global max_percent_error
    if x_ref != 0:
        percent_error = abs((x - x_ref) / x_ref * 100)
        if percent_error > max_percent_error:
            max_percent_error = percent_error


def read_data_file(filename):
    data_list = []
    with open(filename, 'r') as file:
        data = file.readlines()
    for line in data:
        fields = line.split(":")
        if len(fields) == 2:
            value = int(line.split(":")[1])
            data_list.append(value)
    return data_list


class calculate_constants:
    def calculate(self, n_samples):
        print("Calculating the constants:")
        self.n_samples = n_samples
        sample_rate = n_samples
        frequency = 1.0
        f = frequency / sample_rate
        print("f: " + str(f))
        w_real = 2.0 * math.cos(2.0 * math.pi * f)
        w_imag = math.sin(2.0 * math.pi * f)
        print("   w_real: " + str(w_real))
        print("   w_imag: " + str(w_imag))
        self.w_real_shift = 20
        self.w_real_multiplier = int(w_real * (1 << self.w_real_shift) + 0.5)
        self.w_imag_shift = 20
        self.w_imag_multiplier = int(w_imag * (1 << self.w_imag_shift) + 0.5)
        print("w_real_multiplier:", self.w_real_multiplier)
        print("w_real_shift:", self.w_real_shift)
        print("w_imag_multiplier:", self.w_imag_multiplier)
        print("w_imag_shift:", self.w_imag_shift)

    def get_constants(self):
        return (self.w_real_multiplier, self.w_real_shift, self.w_imag_multiplier, self.w_imag_shift)

    def print_hash_defines(self):
        print("#define W_REAL_MULTIPLIER", self.w_real_multiplier)
        print("#define W_REAL_SHIFT", self.w_real_shift)
        print("#define W_IMAG_MULTIPLIER", self.w_imag_multiplier)
        print("#define W_IMAG_SHIFT", self.w_imag_shift)

    def save_hash_defines(self, filename):
        with open(filename, "w") as fh:
            fh.write("#ifndef __GOERTZEL_ALGORITHM_CONSTANTS__\n")
            fh.write("#define __GOERTZEL_ALGORITHM_CONSTANTS__\n")
            fh.write("\n")
            fh.write("#define W_REAL_MULTIPLIER " + str(self.w_real_multiplier) + "\n")
            fh.write("#define W_REAL_SHIFT " + str(self.w_real_shift) + "\n")
            fh.write("#define W_IMAG_MULTIPLIER " + str(self.w_imag_multiplier) + "\n")
            fh.write("#define W_IMAG_SHIFT " + str(self.w_imag_shift) + "\n")
            fh.write("\n")
            fh.write("#endif\n")


def goertzel_algorithm(samples, sample_rate, frequency):
    n_samples = len(samples)
    print("n_samples: " + str(n_samples))
    f = frequency / sample_rate
    print("f: " + str(f))
    w_real = 2.0 * math.cos(2.0 * math.pi * f)
    w_imag = math.sin(2.0 * math.pi * f)
    print("w_real: " + str(w_real))
    print("w_imag: " + str(w_imag))
    d1, d2 = 0.0, 0.0
    for n in range(n_samples):
        y = samples[n] + w_real * d1 - d2
        d2 = d1
        d1 = y
    result = (0.5 * w_real * d1 - d2, w_imag * d1)
    return result


def goertzel_algorithm_integer_math(samples, constants):
    (w_real_multiplier, w_real_shift, w_imag_multiplier, w_imag_shift) = constants
    n_samples = len(samples)
    print("n_samples: " + str(n_samples))
    d1 = 0
    d2 = 0
    for n in range(n_samples):
        d1_times_w_real_multiplier_64bit = d1 * w_real_multiplier
        y = samples[n] + (d1_times_w_real_multiplier_64bit >> w_real_shift) - d2
        d2 = d1
        d1 = y
        print("n: %d, d1_times_w_real_multiplier_64bit: %d, y: %d, d1: %d, d2: %d" % (n, d1_times_w_real_multiplier_64bit, y, d1, d2))
        record_min_and_max(d1_times_w_real_multiplier_64bit, 64)
        record_min_and_max(y, 32)
        record_min_and_max(d1, 32)
        record_min_and_max(d2, 32)
    d1_times_w_real_multiplier_64bit = w_real_multiplier * d1
    d1_times_w_imag_multiplier_64bit = d1 * w_imag_multiplier
    record_min_and_max(d1_times_w_real_multiplier_64bit, 64)
    record_min_and_max(d1_times_w_imag_multiplier_64bit, 64)
    d1_times_w_real_multiplier_32bit = (d1_times_w_real_multiplier_64bit >> w_real_shift)
    d1_times_w_imag_multiplier_32bit = (d1_times_w_imag_multiplier_64bit >> w_imag_shift)
    record_min_and_max(d1_times_w_real_multiplier_32bit, 32)
    record_min_and_max(d1_times_w_imag_multiplier_32bit, 32)
    result = ((d1_times_w_real_multiplier_32bit >> 1) - d2, d1_times_w_imag_multiplier_32bit)
    return result


def get_magnitude_and_phase(complex_number):
    magnitude = abs(complex_number)
    phase = cmath.phase(complex_number)
    return magnitude, phase


print("N samples: " + str(GOERTZEL_ALGORITHM_N_SAMPLES))
cc = calculate_constants()
cc.calculate(GOERTZEL_ALGORITHM_N_SAMPLES)
cc.print_hash_defines()
cc.save_hash_defines(HASH_DEFINES_SAVE_FILENAME)
print(f"The file called {HASH_DEFINES_SAVE_FILENAME} was written. Now, recompile your project.")

