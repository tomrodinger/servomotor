#ifndef __HALL_SENSOR_CALCULATIONS__
#define __HALL_SENSOR_CALCULATIONS__

#include <stdint.h>

typedef struct __attribute__((__packed__)) {
    uint16_t max_value[3];
    uint16_t min_value[3];
    uint64_t sum[3];
    uint32_t n;
} hall_sensor_statistics_t;

void adjust_hall_sensor_readings(uint16_t hall_sensor_readings[3], int32_t adjusted_hall_sensor_readings[3]);
int32_t get_hall_position(void);
int32_t get_hall_position_with_hysteresis(void);
int32_t zero_hall_position(void);
void print_hall_position(void);
void print_hall_midlines(void);
void get_hall_sensor_statistics(hall_sensor_statistics_t *hall_sensor_statistics);
void hall_sensor_turn_off_statistics(void);
void hall_sensor_turn_on_and_reset_statistics(void);

#endif
