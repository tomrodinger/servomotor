#include <time.h>
#include <stdint.h>
#include <stdio.h>
#include "microsecond_clock.h"
#include "error_handling.h"

static uint64_t previous_64bit_time = 0;

// Get current time in microseconds using CLOCK_MONOTONIC
static uint64_t get_monotonic_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

void microsecond_clock_init(void)
{
    // Nothing to initialize in simulation
}

uint64_t get_microsecond_time(void)
{
    uint64_t current_time = get_monotonic_us();
    
    if(current_time < previous_64bit_time) {
        fatal_error(ERROR_TIME_WENT_BACKWARDS);
    }
    previous_64bit_time = current_time;
    
    return current_time;
}

void reset_microsecond_time(void)
{
    previous_64bit_time = 0;
}

void microsecond_delay(uint32_t microseconds)
{
    struct timespec ts;
    ts.tv_sec = microseconds / 1000000;
    ts.tv_nsec = (microseconds % 1000000) * 1000;
    nanosleep(&ts, NULL);
}