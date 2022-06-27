#ifndef __BED_HEATER_CONTROL_H_
#define __BED_HEATER_CONTROL_H_

#define HEATER_CONTROL_PORT_A_PIN 9

void turn_on_or_off_bed_heater(uint8_t new_state);
void set_bed_temperature(uint16_t temperature_times_10);
uint8_t get_device_status_flags(void);
void bed_heater_calculations(uint32_t current, uint32_t voltage, uint32_t temperature);
void print_bed_resistance(void);
void print_bed_temperature(void);

#endif
