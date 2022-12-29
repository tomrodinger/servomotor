#ifndef __BED_HEATER_CONTROL_H_
#define __BED_HEATER_CONTROL_H_

#include "stdint.h"


#define N_LOADS 8  // There are this many loads connected to port A
#define LOAD_CONTROL_PIN_START_INDEX 4 // The control lines start at this index on port A and are contiguous
#define LOAD_OFF_BSSR_SHIFT_SPEC 0
#define LOAD_ON_BSSR_SHIFT_SPEC 16
#define GPIO_ALL_LOADS_OFF (((1 << N_LOADS) - 1) << LOAD_CONTROL_PIN_START_INDEX) // This has the bits inverted and shifted so it can be applied to the GPIOx->ODR register directly (using |= operation)

typedef struct __attribute__((__packed__)) {
    uint16_t max_millivolts;
    uint16_t min_millivolts;
    uint8_t interval_index;
    uint16_t sequence_count;
    uint16_t n_records_left;
    uint8_t data_record_overflow;
} data_record_t;

typedef enum {
    STOP_IMMEDIATELY_AND_DELETE_ALL_INTERVALS = 0,
    STOPPED,
    HOLD_LOAD_STATE,
    ALLOW_ONLY_CURRENT_INTERVAL_TO_FINISH_THEN_STOP,
    ALLOW_ALL_INTERVALS_TO_FINISH_THEN_STOP,
    RUN_CONTINUOUSLY,
    MANUAL_PULSE,
} interval_run_mode_t;

void coin_cell_control_and_calculations_init(void);
void load_on(uint8_t load_number);
void load_off(uint8_t load_number);
void load_toggle_on_or_off(uint8_t load_number);
uint8_t get_load_state(uint8_t load_number);
void disable_mosfets(void);
uint8_t get_device_status_flags(void);
void apply_pulse(uint32_t pulse_duration_ms, uint8_t load_number);
void print_pulse_stats(void);
void all_loads_off(void);
void add_new_interval(uint16_t interval_add_index, uint8_t load_state, uint32_t duration);
void set_interval_run_mode(interval_run_mode_t run_mode);
volatile uint16_t *get_ADC_buffer_ptr(void);
void print_ADC_values(void);
uint64_t get_picoamp_seconds_count(void);
void print_picoamp_seconds_count(void);
void print_intervals(void);
void print_time_difference(void);
void get_data_record(data_record_t *data_record_out);
void get_data_record_and_print_it(void);
void load_control_and_voltage_calculations(void);

#endif
