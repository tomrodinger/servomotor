#ifndef __HALL_SENSOR_CALCULATIONS__
#define __HALL_SENSOR_CALCULATIONS__

#include <stdint.h>
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
#ifdef PRODUCT_NAME_M4
#include "hall_sensor_constants_M4.h"
#endif

typedef struct __attribute__((__packed__)) {
    uint16_t max_value[3];
    uint16_t min_value[3];
    uint64_t sum[3];
    uint32_t n;
} hall_sensor_statistics_t;

typedef struct {
    int32_t position;
    int32_t change;
} get_sensor_position_return_t;

void adjust_hall_sensor_readings(uint16_t hall_sensor_readings[3], int32_t adjusted_hall_sensor_readings[3]);
get_sensor_position_return_t get_sensor_position(void);
void zero_hall_position(uint8_t keep_sensor_offset);
void bound_the_sensor_position(int32_t upper_bound);
void print_sensor_position(void);
void print_hall_midlines(void);
void get_hall_sensor_statistics(hall_sensor_statistics_t *hall_sensor_statistics);
void hall_sensor_turn_off_statistics(void);
void hall_sensor_turn_on_and_reset_statistics(void);

#endif
