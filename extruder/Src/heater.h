#ifndef HEATER_H
#define HEATER_H

#include <stdint.h>

void heater_on(void);
void heater_off(void);
void set_heater_temperature(uint16_t new_target_temp_extruder);
uint16_t get_heater_temperature(void);
void heater_logic(void);
void increase_temperature(void);
void decrease_temperature(void);
void print_heater_temperature(void);

#endif