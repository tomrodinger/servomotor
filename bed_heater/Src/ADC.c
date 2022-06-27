#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32g0xx_hal.h"
#include "ADC.h"
#include "leds.h"
#include "debug_uart.h"

#define ADC_WATCHDOG_DEFAULT_UPPER_THRESHOLD 1450
#define ADC_WATCHDOG_DEFAULT_LOWER_THRESHOLD 1250

static volatile uint16_t ADC_buffer[DMA_ADC_BUFFER_SIZE];
static volatile uint32_t current = 0;
static volatile uint32_t voltage = 0;
static volatile uint32_t temperature = 0;
static volatile uint8_t new_values_measured_flag = 0;


void adc_init(void)
{
    volatile uint32_t i;

    RCC->AHBENR |= RCC_AHBENR_DMA1EN; // enable the clock to the DMA1
    RCC->APBENR2 |= RCC_APBENR2_ADCEN; // enable the clock to the ADC
    RCC->CCIPR |= (1 << RCC_CCIPR_ADCSEL_Pos); // use the PLLPCLK for the ADC
    ADC->CCR |= (1 << ADC_CCR_PRESC_Pos); // divide the clock source by 2 to get 32 MHz for the ADC clock
//	ADC->CCR |= ADC_CCR_VREFEN; // DEBUG turn on ability to measure internal reference
    ADC1->CFGR1 |= ADC_CFGR1_CHSELRMOD | ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD; // channel selection is done by selecting the exact sequence, continuous conversion mode, if overrun occurs, we take the latest value, 12-bit resolution
//	ADC1->CFGR1 |= ADC_CFGR1_DMACFG; // DMA circular mode (after reaching the end of the buffer, go back to the beginning and keep converting). Comment this out to do one shot mode (fill the buffer to the end then stop).)
//    ADC1->CFGR2 |= (0 << ADC_CFGR2_CKMODE_Pos) | (1 << ADC_CFGR2_OVSR_Pos) | ADC_CFGR2_OVSE; // select the clock mode to take the clock source from the one selected in the RCC->CCIPR register, enable 4x oversampling
//    ADC1->CFGR2 |= (0 << ADC_CFGR2_CKMODE_Pos); // select the clock mode to take the clock source from the one selected in the RCC->CCIPR register, no oversampling
    ADC1->CFGR2 |= (0 << ADC_CFGR2_CKMODE_Pos) | (4 << ADC_CFGR2_OVSS_Pos) | (7 << ADC_CFGR2_OVSR_Pos) | ADC_CFGR2_OVSE; // select the clock mode to take the clock source from the one selected in the RCC->CCIPR register, enable 256x oversampling and 4-bit right shift
    ADC1->SMPR |= (7 << ADC_SMPR_SMP1_Pos); // select 160.5 ADC cycles as the sampling time, the total time for 1 conversion will be 5.4us
                                            // calculation: 1/32000000*(160.5+12.5)*1000000

    ADC1->CFGR1 |= ADC_CFGR1_AWD1EN | ADC_CFGR1_AWD1SGL | (0 << ADC_CFGR1_AWD1CH_Pos);
    ADC1->TR1 = (ADC_WATCHDOG_DEFAULT_LOWER_THRESHOLD << ADC_TR2_LT2_Pos) | (ADC_WATCHDOG_DEFAULT_UPPER_THRESHOLD << ADC_TR2_HT2_Pos); // select the lower and upper threshold for the ADC watchdog 1

    ADC1->TR2 = (ADC_WATCHDOG_DEFAULT_LOWER_THRESHOLD << ADC_TR2_LT2_Pos) | (ADC_WATCHDOG_DEFAULT_UPPER_THRESHOLD << ADC_TR2_HT2_Pos); // select the lower and upper threshold for the ADC watchdog 2
    ADC1->AWD2CR = (1 << 0); // only select channel 0 to be monitored by the watchdog 2, this is the bed heater current measurement

    ADC1->TR3 = (ADC_WATCHDOG_DEFAULT_LOWER_THRESHOLD << ADC_TR2_LT2_Pos) | (ADC_WATCHDOG_DEFAULT_UPPER_THRESHOLD << ADC_TR2_HT2_Pos); // select the lower and upper threshold for the ADC watchdog 3
    ADC1->AWD3CR = (1 << 0); // only select channel 0 to be monitored by the watchdog 3, this is the bed heater current measurement

//    ADC1->CHSELR = (1 << 0) |    // select the channels to be converted, 5 channels total
//    		       (1 << 4) |
//				   (1 << 5) |
//				   (1 << 6) |
//				   (1 << 7);
//    ADC1->CHSELR = (5 << ADC_CHSELR_SQ1_Pos) |    // select the channels to be converted, 5 channels total
//    		       (4 << ADC_CHSELR_SQ2_Pos) |
//				   (6 << ADC_CHSELR_SQ3_Pos) |
//				   (0 << ADC_CHSELR_SQ4_Pos) |
//				   (7 << ADC_CHSELR_SQ5_Pos) |
//				   (0xf << ADC_CHSELR_SQ6_Pos) |  // these last three are not used
//				   (0xf << ADC_CHSELR_SQ7_Pos) |
//				   (0xf << ADC_CHSELR_SQ8_Pos);
    ADC1->CHSELR = (0  << ADC_CHSELR_SQ1_Pos) |    // select the channels to be converted, 8 channels total
    		       (1  << ADC_CHSELR_SQ2_Pos) |
				   (0  << ADC_CHSELR_SQ3_Pos) |
				   (1  << ADC_CHSELR_SQ4_Pos) |
				   (0  << ADC_CHSELR_SQ5_Pos) |
				   (1  << ADC_CHSELR_SQ6_Pos) |
				   (0  << ADC_CHSELR_SQ7_Pos) |
				   (1  << ADC_CHSELR_SQ8_Pos);
    ADC1->CR |= ADC_CR_ADVREGEN; // enable the voltage regulator. this must be done before enabling the ADC

    for(i = 0; i < 100000; i++); // allow time for the voltage regulator to stabilize

    ADC1->CR |= ADC_CR_ADCAL; // calibration must be done before the ADC is enabled
    while(ADC1->CR & ADC_CR_ADCAL); // wait for the calibration to finish before going on

    ADC1->ISR &= ~(ADC_ISR_ADRDY | ADC_ISR_CCRDY); // required to clear this first according to the datasheet. search for "Follow this procedure to enable the ADC".
    ADC1->CR |= ADC_CR_ADEN; // enable the ADC
    while((ADC1->ISR & (ADC_ISR_ADRDY | ADC_ISR_CCRDY)) != (ADC_ISR_ADRDY | ADC_ISR_CCRDY)); // wait for the ADC to become available and channel selection to be acknowledged

    ADC1->CFGR1 |= ADC_CFGR1_DMAEN; // enable of the DMA must be done after calibration (see 15.5.5 in the reference manual)

    DMA1_Channel1->CMAR = (uint32_t)(void*)ADC_buffer;
    DMA1_Channel1->CPAR = (uint32_t)(void*)(&ADC1->DR);
    DMA1_Channel1->CNDTR = (DMA_ADC_BUFFER_SIZE << DMA_CNDTR_NDT_Pos);
	DMA1->IFCR = 0x0fff; // clear all interrupt flags

    DMA1_Channel1->CCR = (1 << DMA_CCR_MSIZE_Pos) | (1 << DMA_CCR_PSIZE_Pos) | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TCIE; // 16 bit transfer, increment memory, circular mode, enable interrupt when DMA buffer is full

    DMAMUX1_Channel0->CCR = (5 << DMAMUX_CxCR_DMAREQ_ID_Pos); // select the ADC as the input to the DMA channel 1
//    DMAMUX1_ChannelStatus->CSR = 0;
//    DMAMUX1_ChannelStatus->CFR = 0;
//    DMAMUX1_RequestGenerator0->RGCR = DMAMUX_RGxCR_GE;

	NVIC_EnableIRQ(DMA1_Channel1_IRQn); // enable the interrupt for the DMA channel 1

    DMA1_Channel1->CCR |= DMA_CCR_EN; // enable the DMA channel as the last step (see section 11.4.3 in the reference manual).

//    ADC->IER |= ADC_IER_EOCIE; // enable the end of conversion interrupt
//    ADC->IER |= ADC_IER_EOSIE; // enable the end of conversion sequence interrupt
}


void do_one_ADC_conversion_cycle(void)
{
    ADC1->CR |= ADC_CR_ADSTART; // start to do the conversions one by one until the DMA buffer is full
}


void DMA1_Channel1_IRQHandler(void)
{
	uint32_t c = 0;
	uint32_t v = 0;
	uint32_t t = 0;
	int32_t i;
//	static uint8_t led_state = 1;
/*
	if(led_state == 0)
	{
		led_state = 1;
		red_LED_on();
	}
	else
	{
		led_state = 0;
		red_LED_off();
	}
*/
	for(i = 0; i < DMA_ADC_BUFFER_SIZE; i += DMA_ADC_BUFFER_VALUES_PER_CYCLE) {
		c += ADC_buffer[i + 0];
		v += ADC_buffer[i + 1];
		c += ADC_buffer[i + 2];
		v += ADC_buffer[i + 3];
		c += ADC_buffer[i + 4];
		v += ADC_buffer[i + 5];
		c += ADC_buffer[i + 6];
		v += ADC_buffer[i + 7];
	}
//	c >>= 4;
//	v >>= 3;
//	t >>= 3;

	current = c;
	voltage = v;
	temperature = t;
	new_values_measured_flag = 1;

	DMA1->IFCR = 0x0fff; // clear all interrupt flags
	do_one_ADC_conversion_cycle();
}


void check_if_break_condition(void)
{
	if(TIM1->SR & TIM_SR_BIF) {
//		red_LED_on();
		TIM1->SR &= ~TIM_SR_BIF;
	}
	else {
//		red_LED_off();
	}
}


void check_if_ADC_watchdog1_exceeded(void)
{
	if(ADC1->ISR & ADC_ISR_AWD1) {
//		red_LED_on();
		ADC1->ISR |= ADC_ISR_AWD1;
	}
	else {
//		red_LED_off();
	}
}


void check_if_ADC_watchdog2_exceeded(void)
{
	if(ADC1->ISR & ADC_ISR_AWD2) {
		red_LED_on();
		ADC1->ISR |= ADC_ISR_AWD2;
	}
	else {
		red_LED_off();
	}
}


void check_if_ADC_watchdog3_exceeded(void)
{
	if(ADC1->ISR & ADC_ISR_AWD3) {
//		red_LED_on();
		ADC1->ISR |= ADC_ISR_AWD3;
	}
	else {
//		red_LED_off();
	}
}


uint32_t get_current_sense_value(void)
{
	return current;
}


uint32_t get_24V_sense_value(void)
{
	return voltage;
}


uint32_t get_temperature_sense_value(void)
{
	return temperature;
}


uint8_t new_ADC_values_available(void)
{
	uint8_t flag = new_values_measured_flag;
	new_values_measured_flag = 0;
	return flag;
}

void print_ADC_values(void)
{
    char buf[50];
    uint32_t c = current;
    uint32_t v = voltage;
    uint32_t t = temperature;
	sprintf(buf, "Bed heater current: %lu\n", c);
	transmit(buf, strlen(buf));
	sprintf(buf, "Bed heater voltage: %lu\n", v);
	transmit(buf, strlen(buf));
	sprintf(buf, "Ambient temperature: %lu\n", t);
	transmit(buf, strlen(buf));
}


void set_analog_watchdog_limits(uint16_t lower_limit, uint16_t upper_limit)
{
    ADC1->TR1 = (lower_limit << ADC_TR2_LT2_Pos) | (upper_limit << ADC_TR2_HT2_Pos); // select the lower and upper threshold for the ADC watchdog 1
    ADC1->TR2 = (lower_limit << ADC_TR2_LT2_Pos) | (upper_limit << ADC_TR2_HT2_Pos); // select the lower and upper threshold for the ADC watchdog 2
    ADC1->TR3 = (lower_limit << ADC_TR2_LT2_Pos) | (upper_limit << ADC_TR2_HT2_Pos); // select the lower and upper threshold for the ADC watchdog 3
}

