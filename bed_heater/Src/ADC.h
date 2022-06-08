#ifndef SRC_ADC_H_
#define SRC_ADC_H_

#define DMA_ADC_BUFFER_SIZE (8 * 4)   // enough space for two complete sets of ADC measurements (8 channels)

void adc_init(void);
void do_one_ADC_conversion_cycle(void);
void check_if_break_condition(void);
void check_if_ADC_watchdog1_exceeded(void);
void check_if_ADC_watchdog2_exceeded(void);
void check_if_ADC_watchdog3_exceeded(void);
uint16_t get_current_sense_value(void);
uint16_t get_24V_sense_value(void);
uint16_t get_temperature_sense_value(void);
uint8_t new_ADC_values_available(void);
void set_analog_watchdog_limits(uint16_t lower_limit, uint16_t upper_limit);


#endif /* SRC_ADC_H_ */
