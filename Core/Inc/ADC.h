#ifndef SRC_ADC_H_
#define SRC_ADC_H_

#define DMA_ADC_BUFFER_SIZE (8 * 4)   // enough space for two complete sets of ADC measurements (8 channels)

void adc_init(void);
void check_if_break_condition(void);
void check_if_ADC_watchdog1_exceeded(void);
void check_if_ADC_watchdog2_exceeded(void);
void check_if_ADC_watchdog3_exceeded(void);


#endif /* SRC_ADC_H_ */
