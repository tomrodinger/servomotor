#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include <stdint.h>

int16_t get_temperature_degrees_C(void);
void print_temperature(void);
void check_if_overtemperature(void);

#endif