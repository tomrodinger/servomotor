#ifndef __HALL_SENSOR_CALCULATIONS__
#define __HALL_SENSOR_CALCULATIONS__

#include <stdint.h>

void adjust_hall_sensor_readings(uint16_t hall_sensor_readings[3], int32_t adjusted_hall_sensor_readings[3]);
int32_t get_hall_position(void);
int32_t get_hall_position_with_hysteresis(void);
void zero_hall_position(void);
void print_hall_position(void);
void set_hall_midlines(uint16_t new_hall1_midline, uint16_t new_hall2_midline, uint16_t new_hall3_midline);
void get_hall_midlines(uint16_t *returned_hall1_midline, uint16_t *returned_hall2_midline, uint16_t *returned_hall3_midline);
void print_hall_midlines(void);

#endif
