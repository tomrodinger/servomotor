#ifndef SRC_ADC_H_
#define SRC_ADC_H_

#define DMA_ADC_BUFFER_SIZE (8 * 4)   // enough space for two complete sets of ADC measurements (8 channels)

void adc_init(void);
void check_if_break_condition(void);
void check_if_ADC_watchdog1_exceeded(void);
void check_if_ADC_watchdog2_exceeded(void);
void check_if_ADC_watchdog3_exceeded(void);
uint16_t get_hall_sensor1_voltage(void);
uint16_t get_hall_sensor2_voltage(void);
uint16_t get_hall_sensor3_voltage(void);
uint16_t get_motor_current(void);


#endif /* SRC_ADC_H_ */
