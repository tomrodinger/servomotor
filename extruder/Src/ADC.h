#ifndef SRC_ADC_H_
#define SRC_ADC_H_

#define ADC_CYCLE_INDEXES 8
#define ADC_BUFFER_CYCLE_REPETITIONS 4
#define DMA_ADC_BUFFER_SIZE (ADC_CYCLE_INDEXES * ADC_BUFFER_CYCLE_REPETITIONS)   // enough space for four complete sets of ADC measurements (8 channels)

#define HALL1_ADC_CYCLE_INDEX 1       // hall sensor 1 is connected to this index of the 8 channel cycle
#define HALL2_ADC_CYCLE_INDEX 2       // hall sensor 2 is connected to this index of the 8 channel cycle
#define HALL3_ADC_CYCLE_INDEX 4       // hall sensor 3 is connected to this index of the 8 channel cycle
#define HALL4_ADC_CYCLE_INDEX 5       // hall sensor 4 is connected to this index of the 8 channel cycle
#define TEMPERATURE_ADC_CYCLE_INDEX 7 // temperature sensor is connected tothis index of the 8 channel cycle
#define MAX_TEMPERATURE_SENSOR_ADC_VALUE  (4095 * ADC_BUFFER_CYCLE_REPETITIONS) // (2^(ADC_N_BITS) - 1) * ADC_BUFFER_CYCLE_REPETITIONS

void adc_init(void);
void check_if_break_condition(void);
void check_if_ADC_watchdog1_exceeded(void);
void check_if_ADC_watchdog2_exceeded(void);
void check_if_ADC_watchdog3_exceeded(void);
uint16_t get_hall_sensor1_voltage(void);
uint16_t get_hall_sensor2_voltage(void);
uint16_t get_hall_sensor3_voltage(void);
uint16_t get_hall_sensor4_voltage(void);
uint16_t get_temperature_sensor_ADC_value(void);
uint16_t get_motor_current(void);
uint16_t get_temperature(void);
uint16_t get_supply_voltage(void);
void set_analog_watchdog_limits(uint16_t lower_limit, uint16_t upper_limit);


#endif /* SRC_ADC_H_ */
