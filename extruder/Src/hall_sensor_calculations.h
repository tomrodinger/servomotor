#ifndef __HALL_SENSOR_CALCULATIONS__
#define __HALL_SENSOR_CALCULATIONS__

#include <stdint.h>

void adjust_hall_sensor_readings(uint16_t hall_sensor_readings[3], int32_t adjusted_hall_sensor_readings[3]);
int32_t get_hall_position1(void);
int32_t get_hall_position2(void);
int32_t get_hall_position(void);
int32_t get_hall_position_with_hysteresis(void);
void switch_motor(void);
void set_current_motor_index(int index);
void zero_hall_position(void);
void print_hall_position_and_time_difference(void);
void print_hall_sensor_value(void);
void print_hall_midlines(void);
void print_fraction(void);
uint16_t get_smoothed_hall_sensor1_voltage();
uint16_t get_smoothed_hall_sensor2_voltage();
uint16_t get_smoothed_hall_sensor3_voltage();
uint16_t get_smoothed_hall_sensor4_voltage();

#endif
