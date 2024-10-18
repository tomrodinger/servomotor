#!/usr/bin/env python3

# Python program to sanity-check the log file

VERBOSE = False  # Set to True to enable detailed per-line output

def main():
    passed = True
    # Initialize min and max variables for input_a, input_b, expected_result, actual_result
    input_a_min = None
    input_a_max = None
    input_b_min = None
    input_b_max = None
    expected_result_min = None
    expected_result_max = None
    actual_result_min = None
    actual_result_max = None

    with open('test_add_i96.log', 'r') as f:
        lines = f.readlines()
    # Skip the header line
    for line_num, line in enumerate(lines[1:], start=2):  # Start from the second line, line numbers start at 2
        line = line.strip()
        if not line:
            continue  # Skip empty lines
        parts = line.split()
        if len(parts) != 6:
            print(f"Line {line_num}: Invalid number of columns")
            passed = False
            continue
        trial_num_str, input_a_str, input_b_str, expected_result_str, actual_result_str, test_outcome = parts
        try:
            input_a = int(input_a_str)
            input_b = int(input_b_str)
            expected_result = int(expected_result_str)
            actual_result = int(actual_result_str)
        except ValueError as e:
            print(f"Line {line_num}: Invalid integer conversion")
            passed = False
            continue
        computed_expected = input_a + input_b
        if computed_expected != expected_result:
            print(f"Line {line_num}: Computed Expected_Result {computed_expected} does not match Expected_Result {expected_result}")
            passed = False
        if actual_result != expected_result:
            print(f"Line {line_num}: Actual_Result {actual_result} does not match Expected_Result {expected_result}")
            passed = False
        # Update min and max values
        if input_a_min is None or input_a < input_a_min:
            input_a_min = input_a
        if input_a_max is None or input_a > input_a_max:
            input_a_max = input_a
        if input_b_min is None or input_b < input_b_min:
            input_b_min = input_b
        if input_b_max is None or input_b > input_b_max:
            input_b_max = input_b
        if expected_result_min is None or expected_result < expected_result_min:
            expected_result_min = expected_result
        if expected_result_max is None or expected_result > expected_result_max:
            expected_result_max = expected_result
        if actual_result_min is None or actual_result < actual_result_min:
            actual_result_min = actual_result
        if actual_result_max is None or actual_result > actual_result_max:
            actual_result_max = actual_result
        if VERBOSE:
            print(f"input_a: {input_a_str}, input_b: {input_b_str}, expected_result: {expected_result_str}, actual_result: {actual_result_str}, computed_expected: {computed_expected}")
    # After processing all lines, compute bit lengths and print min/max values
    def bit_length(n):
        # For zero, bit_length() returns 0, but we need at least 1 bit to represent zero
        if n == 0:
            return 1
        else:
            return n.bit_length() + 1  # Add one for the sign bit
    print("\nSummary:")
    print(f"Input_a min: {input_a_min}, bits required: {bit_length(input_a_min)}")
    print(f"Input_a max: {input_a_max}, bits required: {bit_length(input_a_max)}")
    print(f"Input_b min: {input_b_min}, bits required: {bit_length(input_b_min)}")
    print(f"Input_b max: {input_b_max}, bits required: {bit_length(input_b_max)}")
    print(f"Expected_Result min: {expected_result_min}, bits required: {bit_length(expected_result_min)}")
    print(f"Expected_Result max: {expected_result_max}, bits required: {bit_length(expected_result_max)}")
    print(f"Actual_Result min: {actual_result_min}, bits required: {bit_length(actual_result_min)}")
    print(f"Actual_Result max: {actual_result_max}, bits required: {bit_length(actual_result_max)}")
    if passed:
        print("\nPASSED")
    else:
        print("\nFAILED")

if __name__ == '__main__':
    main()
