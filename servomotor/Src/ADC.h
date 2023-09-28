#ifndef SRC_ADC_H_
#define SRC_ADC_H_

#define ADC_CYCLE_INDEXES 8
#define ADC_BUFFER_CYCLE_REPETITIONS 4
#define DMA_ADC_BUFFER_SIZE (ADC_CYCLE_INDEXES * ADC_BUFFER_CYCLE_REPETITIONS)   // enough space for four complete sets of ADC measurements (8 channels)

#define HALL1_ADC_CYCLE_INDEX 1      // hall sensor 1 is connected to this ADC channel index of the 8 channel cycle
#define HALL2_ADC_CYCLE_INDEX 4      // hall sensor 2 is connected to this ADC channel index of the 8 channel cycle
#define HALL3_ADC_CYCLE_INDEX 7      // hall sensor 3 is connected to this ADC channel index of the 8 channel cycle
#define MOTOR_CURRENT_CYCLE_INDEX1 0 // motor current is measured and placed at this index of the 8 channel cycle
#define MOTOR_CURRENT_CYCLE_INDEX2 3 // and also this index
#define MOTOR_CURRENT_CYCLE_INDEX3 6 // and also this index (motor current is measured three times in each ADC cycle)

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
uint16_t get_supply_voltage_ADC_value(void);
uint16_t get_supply_voltage_volts_time_10(void);
void print_supply_voltage(void);
void set_analog_watchdog_limits(uint16_t lower_limit, uint16_t upper_limit);


#endif /* SRC_ADC_H_ */
