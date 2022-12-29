#ifndef SRC_ADC_H_
#define SRC_ADC_H_

#define DMA_ADC_BUFFER_REPEAT_CYCLES 1
#define DMA_ADC_BUFFER_VALUES_PER_CYCLE 1
#define DMA_ADC_BUFFER_SIZE (DMA_ADC_BUFFER_VALUES_PER_CYCLE * DMA_ADC_BUFFER_REPEAT_CYCLES)   // enough space for two complete sets of ADC measurements (8 channels)

void adc_init(volatile uint16_t *ADC_buffer);

#endif /* SRC_ADC_H_ */
