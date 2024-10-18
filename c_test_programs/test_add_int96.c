#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <limits.h>

#define LOG_FILENAME "test_add_i96.log"
#define RUN_TIME_SECONDS 3
#define MAX_N_TRIALS 0 // set to zero for unlimited

typedef struct {
    uint32_t low;
    uint32_t mid;
    int32_t high;
} int96_t;

void add_int96_t(int96_t *a_i96, int64_t *b_i64, int96_t *result) {
    uint64_t sum;
    uint32_t carry;

    // Extract low and mid 32 bits of b_i64
    uint32_t b_low = (uint32_t)(*b_i64);
    uint32_t b_mid = (uint32_t)((*b_i64) >> 32);
    int32_t b_high = (*b_i64 < 0) ? -1 : 0;

    // Add low parts
    sum = (uint64_t)a_i96->low + b_low;
    result->low = (uint32_t)sum;
    carry = sum >> 32;

    // Add mid parts with carry
    sum = (uint64_t)a_i96->mid + b_mid + carry;
    result->mid = (uint32_t)sum;
    carry = sum >> 32;

    // Add high parts with carry
    sum = (int64_t)a_i96->high + b_high + (int64_t)carry;
    result->high = (int32_t)sum;
}

void int128_to_string(__int128_t value, char *str) {
    char temp[50];
    int i = 0;
    int is_negative = value < 0;
    if (is_negative) value = -value;
    do {
        temp[i++] = '0' + (value % 10);
        value /= 10;
    } while (value != 0);
    if (is_negative) temp[i++] = '-';
    int j = 0;
    while (i > 0) {
        str[j++] = temp[--i];
    }
    str[j] = '\0';
}

__int128_t int96_to_int128(int96_t *value) {
    __int128_t result = ((__int128_t)(int64_t)value->high << 64)
                        | ((__int128_t)value->mid << 32)
                        | value->low;
    return result;
}

uint32_t random_uint32(void) {
    // Combine two rand() calls to get a 32-bit number
    uint32_t r = ((uint32_t)rand() << 16) | ((uint32_t)rand() & 0xFFFF);
    return r;
}

int32_t random_int32(void) {
    uint32_t r = random_uint32();
    return (int32_t)r;
}

int main(void)
{
    FILE *fp;
    uint32_t n_passed = 0;
    uint32_t n_failed = 0;
    uint64_t trial_number = 0;

    printf("Test Start\n");

    // Set the random seed based on the current time
    srand(time(NULL));

    // Open file for writing
    fp = fopen(LOG_FILENAME, "w");
    if (fp == NULL) {
        printf("Error opening file %s for writing!\n", LOG_FILENAME);
        return 1;
    }

    // Write header to file
    fprintf(fp, "Trial# Input_a Input_b Expected_Result Actual_Result Test_Outcome\n");

    // Record the start time
    time_t start_time = time(NULL);
    while (1) {
        trial_number++;
        // Define our needed variables
        __int128_t a;
        int64_t b;
        __int128_t expected_result;
        __int128_t actual_result;

        // Generate random values for the inputs for a and b
        int32_t a_high = random_int32();
        uint32_t a_mid = random_uint32();
        uint32_t a_low = random_uint32();
        a = ((__int128_t)a_high << 64) | ((__int128_t)a_mid << 32) | a_low;

        int32_t b_high = random_int32();
        uint32_t b_low = random_uint32();
        b = ((int64_t)b_high << 32) | b_low;

        // Calculate the expected result
        expected_result = a + b;

        // Prepare the inputs for the function
        int96_t a_i96 = {a_low, a_mid, a_high};
        int96_t actual_result_i96;
        add_int96_t(&a_i96, &b, &actual_result_i96);
        actual_result = int96_to_int128(&actual_result_i96);

        // Convert numbers to strings
        char str_a[50], str_b[50], str_expected[50], str_actual[50];
        int128_to_string(a, str_a);
        int128_to_string(b, str_b);
        int128_to_string(expected_result, str_expected);
        int128_to_string(actual_result, str_actual);

        // Write the results to the log file
        fprintf(fp, "%llu %s %s %s %s ",
                trial_number,
                str_a,
                str_b,
                str_expected,
                str_actual);

        if (expected_result == actual_result) {
            fprintf(fp, "PASS\n");
            n_passed++;
        } else {
            fprintf(fp, "FAIL\n");
            n_failed++;
        }

        // Now, let's check if we should stop the test by checking the time elapsed since the start
        time_t current_time = time(NULL);
        if (current_time - start_time >= RUN_TIME_SECONDS) {
            break;
        }
        // Also, break out of the loop if we have reached the maximum number of trials
        if ( (MAX_N_TRIALS > 0) && (trial_number >= MAX_N_TRIALS) ) {
            break;
        }
    }

    // Close file
    fclose(fp);

    printf("Test Complete. Here is a summary:\n");
    printf("Number of trials: %u\n", n_passed + n_failed);
    if (n_failed == 0) {
        printf("All tests passed!\n");
    } else {
        printf("Number of passed tests: %u\n", n_passed);
        printf("Number of failed tests: %u\n", n_failed);
    }

    return 0;
}
