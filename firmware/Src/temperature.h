#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include <stdint.h>

int16_t get_temperature_degrees_C(void);
void print_temperature(void);
void check_if_overtemperature(void);
void set_overtemperature_test_mode(uint8_t enable); // raise the overtemperature cutoff to ~90 C (production thermal-margin test) or restore the default

#endif