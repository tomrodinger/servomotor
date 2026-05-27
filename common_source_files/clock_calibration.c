#include <stdint.h>
#include "stm32g0xx_hal.h"
#include "clock_calibration.h"
#include "microsecond_clock.h"

#define PROPORTIONAL_CONSTANT 40
#define INTEGRAL_CONSTANT 1

#define MIN_HSI_TRIM_VALUE 62
#define MAX_HSI_TRIM_VALUE 66

uint8_t PI_controller(int32_t error)
{
    int32_t output_value;
    int32_t proportional_term;
    static int32_t integral_term = 64 << 16;

    if(error < -32768) {
        error = -32768;
    }
    else if(error > 32767) {
        error = 32767;
    }
    integral_term += (error * INTEGRAL_CONSTANT);
    proportional_term = error * PROPORTIONAL_CONSTANT;
    output_value = (integral_term + proportional_term) >> 16;
    if(output_value < MIN_HSI_TRIM_VALUE) {
        output_value = MIN_HSI_TRIM_VALUE;
    }
    else if(output_value > MAX_HSI_TRIM_VALUE) {
        output_value = MAX_HSI_TRIM_VALUE;
    }

    return (uint8_t)output_value;
}


int32_t time_sync(uint32_t time_from_master)
{
    // Truncate the device's u64 clock to its low 32 bits before subtracting.
    // The master also only sends the low 32 bits (u32 absolute microseconds),
    // so this is u32 modular subtraction. Cast (int32_t) interprets the
    // wrapped result as a signed error in the [-2^31, +2^31) us range
    // (≈ ±35 min) — correct across the u32 wrap (≈71 min) as long as the
    // true drift is well inside that range and sync is performed regularly.
    uint32_t local_time = (uint32_t)get_microsecond_time();
    int32_t time_error = (int32_t)(time_from_master - local_time);
    uint8_t new_clock_cal_value = PI_controller(time_error);
    RCC->ICSCR = new_clock_cal_value << RCC_ICSCR_HSITRIM_Pos;
    return time_error;
}

uint16_t get_clock_calibration_value(void)
{
    return (uint16_t)(RCC->ICSCR & 0xffff);
}
