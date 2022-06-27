#ifndef SRC_ADC_H_
#define SRC_ADC_H_

#define DMA_ADC_BUFFER_REPEAT_CYCLES 32
#define DMA_ADC_BUFFER_VALUES_PER_CYCLE 8
#define DMA_ADC_BUFFER_SIZE (DMA_ADC_BUFFER_VALUES_PER_CYCLE * DMA_ADC_BUFFER_REPEAT_CYCLES)   // enough space for two complete sets of ADC measurements (8 channels)

void adc_init(void);
void do_one_ADC_conversion_cycle(void);
void check_if_break_condition(void);
void check_if_ADC_watchdog1_exceeded(void);
void check_if_ADC_watchdog2_exceeded(void);
void check_if_ADC_watchdog3_exceeded(void);
uint32_t get_current_sense_value(void);
uint32_t get_24V_sense_value(void);
uint32_t get_temperature_sense_value(void);
uint8_t new_ADC_values_available(void);
void print_ADC_values(void);
void set_analog_watchdog_limits(uint16_t lower_limit, uint16_t upper_limit);


#endif /* SRC_ADC_H_ */
