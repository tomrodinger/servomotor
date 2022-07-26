#ifndef SRC_ADC_H_
#define SRC_ADC_H_

#define ADC_CYCLE_INDEXES 8
#define ADC_BUFFER_CYCLE_REPETITIONS 4
#define DMA_ADC_BUFFER_SIZE (ADC_CYCLE_INDEXES * ADC_BUFFER_CYCLE_REPETITIONS)   // enough space for four complete sets of ADC measurements (8 channels)

void adc_init(void);
void check_if_break_condition(void);
void check_if_ADC_watchdog1_exceeded(void);
void check_if_ADC_watchdog2_exceeded(void);
void check_if_ADC_watchdog3_exceeded(void);
uint16_t get_hall_sensor1_voltage(void);
uint16_t get_hall_sensor2_voltage(void);
uint16_t get_hall_sensor3_voltage(void);
uint16_t get_motor_current(void);
uint16_t get_temperature(void);
uint16_t get_supply_voltage(void);
void set_analog_watchdog_limits(uint16_t lower_limit, uint16_t upper_limit);


#endif /* SRC_ADC_H_ */
