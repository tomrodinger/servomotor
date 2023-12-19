#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32g0xx_hal.h"
#include "hall_sensor_calculations.h"
#include "error_handling.h"
#include "ADC.h"
#include "debug_uart.h"
#ifdef PRODUCT_NAME_M1
#include "commutation_table_M1.h"
#include "hall_sensor_constants_M1.h"
#endif
#ifdef PRODUCT_NAME_M2
#include "commutation_table_M2.h"
#include "hall_sensor_constants_M2.h"
#endif
#ifdef PRODUCT_NAME_M3
#include "hall_sensor_constants_M3.h"
#endif
#include "global_variables.h"

#define UINT32_MIDPOINT 2147483648
#define HALL_POSITION_HYSTERESIS 100
#define ONE_REVOLUTION_HALL_COUNTS (SENSOR_SEGMENT_RESOLUTION_DIV_2 * TOTAL_NUMBER_OF_SEGMENTS)
#define ONE_CYCLE_HALL_COUNTS (SENSOR_SEGMENT_RESOLUTION * N_HALL_SENSORS)
//#define DO_DETAILED_PROFILING // uncomment this of you want to do time profiling

static const struct hall_weights_struct hall_weights = HALL_WEIGHTS_INITIALIZER;

extern uint16_t ADC_buffer[DMA_ADC_BUFFER_SIZE];
static int8_t previous_largest_sensor = -2;
static int32_t previous_fraction = 0;
static int32_t sensor_position = 0;
static volatile int32_t latest_sensor_position = 0;
#ifdef DO_DETAILED_PROFILING
static uint16_t time_difference_div = 0;
#endif
static uint8_t hall_sensor_statitics_active = 0;
static hall_sensor_statistics_t hall_sensor_statistics;
static volatile int32_t latest_hall_position = 0;


void adjust_hall_sensor_readings(uint16_t hall_sensor_readings[3], int32_t adjusted_hall_sensor_readings[3])
{
	int32_t d0_shifted = (int32_t)hall_sensor_readings[0] - (int32_t)global_settings.hall1_midline;
	int32_t d1_shifted = (int32_t)hall_sensor_readings[1] - (int32_t)global_settings.hall2_midline;
	int32_t d2_shifted = (int32_t)hall_sensor_readings[2] - (int32_t)global_settings.hall3_midline;
    int32_t d0 = (d0_shifted * hall_weights.h1[0] + d1_shifted * hall_weights.h1[1] + d2_shifted * hall_weights.h1[2]);
    int32_t d1 = (d1_shifted * hall_weights.h2[0] + d2_shifted * hall_weights.h2[1] + d0_shifted * hall_weights.h2[2]);
    int32_t d2 = (d2_shifted * hall_weights.h3[0] + d0_shifted * hall_weights.h3[1] + d1_shifted * hall_weights.h3[2]);
    adjusted_hall_sensor_readings[0] = d0;
    adjusted_hall_sensor_readings[1] = d1;
    adjusted_hall_sensor_readings[2] = d2;
}


get_sensor_position_return_t get_sensor_position(void)
{
    uint16_t hall_sensor_readings[3];
	int32_t d[3];
    int8_t largest_sensor;
    int32_t numerator;
    int32_t denominator;
    int32_t fraction;
#ifdef DO_DETAILED_PROFILING
    static uint16_t start_time;
	static uint16_t end_time;
#endif
    get_sensor_position_return_t hall_position_return_value = {0, 0};

	hall_sensor_readings[0] = ((ADC_buffer[HALL1_ADC_CYCLE_INDEX] + ADC_buffer[HALL1_ADC_CYCLE_INDEX + 8] +
                                ADC_buffer[HALL1_ADC_CYCLE_INDEX + 16] + ADC_buffer[HALL1_ADC_CYCLE_INDEX + 24]) << 3) - HALL_SENSOR_SHIFT;
	hall_sensor_readings[1] = ((ADC_buffer[HALL2_ADC_CYCLE_INDEX] + ADC_buffer[HALL2_ADC_CYCLE_INDEX + 8] +
                                ADC_buffer[HALL2_ADC_CYCLE_INDEX + 16] + ADC_buffer[HALL2_ADC_CYCLE_INDEX + 24]) << 3) - HALL_SENSOR_SHIFT;
	hall_sensor_readings[2] = ((ADC_buffer[HALL3_ADC_CYCLE_INDEX] + ADC_buffer[HALL3_ADC_CYCLE_INDEX + 8] +
                                ADC_buffer[HALL3_ADC_CYCLE_INDEX + 16] + ADC_buffer[HALL3_ADC_CYCLE_INDEX + 24]) << 3) - HALL_SENSOR_SHIFT;

    if(hall_sensor_statitics_active) {
        for(uint8_t h = 0; h < 3; h++) {
            if(hall_sensor_readings[h] > hall_sensor_statistics.max_value[h]) {
                hall_sensor_statistics.max_value[h] = hall_sensor_readings[h];
            }
            if(hall_sensor_readings[h] < hall_sensor_statistics.min_value[h]) {
                hall_sensor_statistics.min_value[h] = hall_sensor_readings[h];
            }
            if(hall_sensor_statistics.n < 0xFFFFFFFF) {
                hall_sensor_statistics.sum[h] += hall_sensor_readings[h];
            }
        }
        if(hall_sensor_statistics.n < 0xFFFFFFFF) {
            hall_sensor_statistics.n++;
        }
    }

	adjust_hall_sensor_readings(hall_sensor_readings, d);

    if((d[0] >= d[1]) && (d[0] >= d[2])) { // check if d[0] is the highest
        largest_sensor = 0;
        numerator = d[1] - d[2];
        if(d[2] > d[1]) {
            denominator = d[0] - d[1];
        }
        else {
            denominator = d[0] - d[2];
        }
    }
    else if ((d[1] >= d[2]) && (d[1] >= d[0])) { // check if d[1] is the highest
        largest_sensor = 1;
        numerator = d[2] - d[0];
        if(d[0] > d[2]) {
            denominator = d[1] - d[2];
        }
        else {
            denominator = d[1] - d[0];
        }
    }
    else {                                    // otherwise d[2] is the highest
        largest_sensor = 2;
        numerator = d[0] - d[1];
        if(d[1] > d[0]) {
            denominator = d[2] - d[0];
        }
        else {
            denominator = d[2] - d[1];
        }
    }

//    numerator >>= 10;
//    denominator >>= 10;

    while((numerator > 32767) || (numerator < -32767)) {
        numerator >>= 1;
        denominator >>= 1;
    }

#ifdef DO_DETAILED_PROFILING
    start_time = TIM14->CNT;
#endif
    // watch out: it seems that this division will give the wrong result if the denominator exceeds the int16_t range
//    #ifdef PRODUCT_NAME_M1
    fraction = numerator * SENSOR_SEGMENT_RESOLUTION_DIV_2 / denominator;
//    #endif
//    #ifdef PRODUCT_NAME_M2
//    fraction = ((numerator * (SENSOR_SEGMENT_RESOLUTION_DIV_2 >> 3) / denominator) << 3);
//    #endif
#ifdef DO_DETAILED_PROFILING
    end_time = TIM14->CNT;
    time_difference_div = end_time - start_time;
#endif
    fraction = fraction + SENSOR_SEGMENT_RESOLUTION_DIV_2;

    if(previous_largest_sensor < 0) {
        hall_position_return_value.change = 0;
        if(previous_largest_sensor == -2) {
            if(largest_sensor == 0) {
                sensor_position = 0;
            }
            else if(largest_sensor == 1) {
                sensor_position = (int32_t)SENSOR_SEGMENT_RESOLUTION;
            }
            else {
                sensor_position = (int32_t)(SENSOR_SEGMENT_RESOLUTION << 1);
            }
            sensor_position += fraction;
        }
        else {
            sensor_position = 0;
        }
        previous_largest_sensor = largest_sensor;
    }
    else {
        hall_position_return_value.change = fraction - previous_fraction;
        if (largest_sensor != previous_largest_sensor) {
            if (largest_sensor - previous_largest_sensor == 1) {
                hall_position_return_value.change += (int32_t)SENSOR_SEGMENT_RESOLUTION;
            }
            else if (largest_sensor - previous_largest_sensor == -1) {
                hall_position_return_value.change -= (int32_t)SENSOR_SEGMENT_RESOLUTION;
            }
            else if (largest_sensor - previous_largest_sensor == -2) {
                hall_position_return_value.change += (int32_t)SENSOR_SEGMENT_RESOLUTION;
            }
            else {
                hall_position_return_value.change -= (int32_t)SENSOR_SEGMENT_RESOLUTION;
            }
            previous_largest_sensor = largest_sensor;
        }
    }
    previous_fraction = fraction;
    sensor_position += hall_position_return_value.change;
//    if(sensor_position >= ONE_CYCLE_HALL_COUNTS) {
//        sensor_position -= ONE_CYCLE_HALL_COUNTS;
//    }
//    else if(sensor_position < 0) {
//        sensor_position += ONE_CYCLE_HALL_COUNTS;
//    }
    latest_sensor_position = sensor_position;
    hall_position_return_value.position = sensor_position;
	return hall_position_return_value;
}


void zero_hall_position(uint8_t keep_sensor_offset)
{
    if(keep_sensor_offset) {
        previous_largest_sensor = -2;
    }
    else {
        previous_largest_sensor = -1;
    }
}


// This function will adjust the sensor position such that it is bound between 0 and upper_bound.
// The the new sensor position will be in the range 0 to upper_bound - 1.
// The tequnique used here to bound it is to subtract or add some multiple of upper_bound.
void bound_the_sensor_position(int32_t upper_bound)
{
    while(1) {
        if(sensor_position < 0) {
            int32_t multiple = (-sensor_position + upper_bound - 1) / upper_bound;
            int32_t adjustment_factor = multiple * upper_bound;
            __disable_irq();
            sensor_position += adjustment_factor;
            __enable_irq();
            continue;
        }
        if(sensor_position >= upper_bound) {
            int32_t multiple = sensor_position / upper_bound;
            int32_t adjustment_factor = multiple * upper_bound;
            __disable_irq();
            sensor_position -= adjustment_factor;
            __enable_irq();
            continue;
        }
        break;
    }
}


void print_sensor_position(void)
{
	char buf[100];
#ifdef DO_DETAILED_PROFILING
	sprintf(buf, "sensor_position: %ld   time_difference_div: %hu\n", latest_sensor_position, time_difference_div);
#else
	sprintf(buf, "sensor_position: %ld\n", latest_sensor_position);
#endif
	print_debug_string(buf);
}


void print_hall_midlines(void)
{
    char buf[100];
    sprintf(buf, "Hall sensor midlines: %u %u %u\n", global_settings.hall1_midline, global_settings.hall2_midline, global_settings.hall3_midline);
    print_debug_string(buf);
}


void get_hall_sensor_statistics(hall_sensor_statistics_t *hall_sensor_statistics_output)
{
	TIM1->DIER &= ~TIM_DIER_UIE; // disable the update interrupt during this operation
    memcpy(hall_sensor_statistics_output, &hall_sensor_statistics, sizeof(hall_sensor_statistics));
    TIM1->DIER |= TIM_DIER_UIE; // enable the update interrupt
}


void hall_sensor_turn_off_statistics(void)
{
    hall_sensor_statitics_active = 0;
}


void hall_sensor_turn_on_and_reset_statistics(void)
{
	TIM1->DIER &= ~TIM_DIER_UIE; // disable the update interrupt during this operation
    for(uint8_t h = 0; h < 3; h++) {
        hall_sensor_statistics.min_value[h] = 0xFFFF;
        hall_sensor_statistics.max_value[h] = 0;
        hall_sensor_statistics.sum[h] = 0;
        hall_sensor_statistics.n = 0;
    }
    hall_sensor_statitics_active = 1;
    TIM1->DIER |= TIM_DIER_UIE; // enable the update interrupt
}
