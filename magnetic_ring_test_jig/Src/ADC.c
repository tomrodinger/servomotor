#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "stm32g0xx_hal.h"
#include "ADC.h"
#include "leds.h"
#include "error_handling.h"
#include "debug_uart.h"

#define ADC_WATCHDOG_DEFAULT_UPPER_THRESHOLD 65535
#define ADC_WATCHDOG_DEFAULT_LOWER_THRESHOLD 0
#define SUPPLY_VOLTAGE_CALIBRATION_CONSTANT 23664

uint16_t ADC_buffer[DMA_ADC_BUFFER_SIZE];


void adc_init(void)
{
    volatile uint32_t i;

    RCC->AHBENR |= RCC_AHBENR_DMA1EN; // enable the clock to the DMA1
    RCC->APBENR2 |= RCC_APBENR2_ADCEN; // enable the clock to the ADC
    RCC->CCIPR |= (1 << RCC_CCIPR_ADCSEL_Pos); // use the PLLPCLK for the ADC
    ADC->CCR |= (1 << ADC_CCR_PRESC_Pos); // divide the clock source by 2 to get 32 MHz for the ADC clock

    ADC1->CFGR1 |= ADC_CFGR1_CHSELRMOD | ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD | ADC_CFGR1_DMACFG | ADC_CFGR1_DMACFG; // channel selection is done by selecting the exact sequence, continuous conversion mode, if overrun occurs, we take the latest value, DMA circular mode, 12-bit resolution
//    ADC1->CFGR2 |= (0 << ADC_CFGR2_CKMODE_Pos) | (1 << ADC_CFGR2_OVSR_Pos) | ADC_CFGR2_OVSE; // select the clock mode to take the clock source from the one selected in the RCC->CCIPR register, enable 4x oversampling
    ADC1->CFGR2 |= (0 << ADC_CFGR2_CKMODE_Pos); // select the clock mode to take the clock source from the one selected in the RCC->CCIPR register, no oversampling
    ADC1->SMPR |= (0 << ADC_SMPR_SMP1_Pos); // select 1.5 ADC cycles as the sampling time, the total time for 1 conversion will be 0.44us
                                            // calculation: time in microseconds = 1/32000000*(1.5+12.5)*1000000

    ADC1->CFGR1 |= ADC_CFGR1_AWD1EN | ADC_CFGR1_AWD1SGL | (0 << ADC_CFGR1_AWD1CH_Pos); // enable analog watchdog 1 and it will monitor just one channel as selected by AWD1CH bits
    ADC1->TR1 = (ADC_WATCHDOG_DEFAULT_LOWER_THRESHOLD << ADC_TR2_LT2_Pos) | (ADC_WATCHDOG_DEFAULT_UPPER_THRESHOLD << ADC_TR2_HT2_Pos); // select the lower and upper threshold for the ADC watchdog 1

    ADC1->TR2 = (ADC_WATCHDOG_DEFAULT_LOWER_THRESHOLD << ADC_TR2_LT2_Pos) | (ADC_WATCHDOG_DEFAULT_UPPER_THRESHOLD << ADC_TR2_HT2_Pos); // select the lower and upper threshold for the ADC watchdog 2
    ADC1->AWD2CR = (1 << 0); // only select channel 0 to be monitored by the watchdog 2, this is the motor current measurement

    ADC1->TR3 = (ADC_WATCHDOG_DEFAULT_LOWER_THRESHOLD << ADC_TR2_LT2_Pos) | (ADC_WATCHDOG_DEFAULT_UPPER_THRESHOLD << ADC_TR2_HT2_Pos); // select the lower and upper threshold for the ADC watchdog 3
    ADC1->AWD3CR = (1 << 0); // only select channel 0 to be monitored by the watchdog 3, this is the motor current measurement

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
	// select the channels to be converted, 8 channels total supported, we will measure the current multiple times per one cycle of 8
	#ifdef PRODUCT_NAME_M3
    ADC1->CHSELR = (0  << ADC_CHSELR_SQ1_Pos) |  // motor current          (index 0)
    		       (5  << ADC_CHSELR_SQ2_Pos) |  // hall 1                 (index 1)
				   (9  << ADC_CHSELR_SQ3_Pos) |  // 24V line voltage sense (index 2)
				   (0  << ADC_CHSELR_SQ4_Pos) |  // motor current          (index 3)
				   (6  << ADC_CHSELR_SQ5_Pos) |  // hall 2                 (index 4)
				   (4  << ADC_CHSELR_SQ6_Pos) |  // termperature sensor    (index 5)
				   (0  << ADC_CHSELR_SQ7_Pos) |  // motor current          (index 6)
				   (7  << ADC_CHSELR_SQ8_Pos);   // hall 3                 (index 7)
	#else
    ADC1->CHSELR = (0  << ADC_CHSELR_SQ1_Pos) |  // motor current          (index 0)
    		       (5  << ADC_CHSELR_SQ2_Pos) |  // hall 1                 (index 1)
				   (7  << ADC_CHSELR_SQ3_Pos) |  // 24V line voltage sense (index 2)
				   (0  << ADC_CHSELR_SQ4_Pos) |  // motor current          (index 3)
				   (4  << ADC_CHSELR_SQ5_Pos) |  // hall 2                 (index 4)
				   (10 << ADC_CHSELR_SQ6_Pos) |  // termperature sensor    (index 5)
				   (0  << ADC_CHSELR_SQ7_Pos) |  // motor current          (index 6)
				   (6  << ADC_CHSELR_SQ8_Pos);   // hall 3                 (index 7)
	#endif
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
    DMA1_Channel1->CCR = (1 << DMA_CCR_MSIZE_Pos) | (1 << DMA_CCR_PSIZE_Pos) | DMA_CCR_MINC | DMA_CCR_CIRC; // 16 bit transfer, increment memory, circular mode

    DMAMUX1_Channel0->CCR = (5 << DMAMUX_CxCR_DMAREQ_ID_Pos); // select the ADC as the input to the DMA channel 1
//    DMAMUX1_ChannelStatus->CSR = 0;
//    DMAMUX1_ChannelStatus->CFR = 0;
//    DMAMUX1_RequestGenerator0->RGCR = DMAMUX_RGxCR_GE;

    DMA1_Channel1->CCR |= DMA_CCR_EN; // enable the DMA channel as the last step (see section 11.4.3 in the reference manual).

//    ADC->IER |= ADC_IER_EOCIE; // enable the end of conversion interrupt
//    ADC->IER |= ADC_IER_EOSIE; // enable the end of conversion sequence interrupt
    ADC1->CR |= ADC_CR_ADSTART; // start to do the conversions one by one continuously
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

uint16_t get_hall_sensor1_voltage(void)
{
	uint16_t a = ADC_buffer[HALL1_ADC_CYCLE_INDEX + 0] + ADC_buffer[HALL1_ADC_CYCLE_INDEX + 8] + ADC_buffer[HALL1_ADC_CYCLE_INDEX + 16] + ADC_buffer[HALL1_ADC_CYCLE_INDEX + 24];
/*	if(a < 8500 - 2000) {
		fatal_error(ERROR_DEBUG1); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
	}
	if(a > 8500 + 2000) {
		fatal_error(ERROR_DEBUG1); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
	} */
	return a;
}

uint16_t get_hall_sensor2_voltage(void)
{
	uint16_t a = ADC_buffer[HALL2_ADC_CYCLE_INDEX + 0] + ADC_buffer[HALL2_ADC_CYCLE_INDEX + 8] + ADC_buffer[HALL2_ADC_CYCLE_INDEX + 16] + ADC_buffer[HALL2_ADC_CYCLE_INDEX + 24];
/*	if(a < 8500 - 2000) {
		fatal_error(ERROR_DEBUG1); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
	}
	if(a > 8500 + 2000) {
		fatal_error(ERROR_DEBUG1); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
	} */
	return a;
}

uint16_t get_hall_sensor3_voltage(void)
{
	uint16_t a = ADC_buffer[HALL3_ADC_CYCLE_INDEX + 0] + ADC_buffer[HALL3_ADC_CYCLE_INDEX + 8] + ADC_buffer[HALL3_ADC_CYCLE_INDEX + 16] + ADC_buffer[HALL3_ADC_CYCLE_INDEX + 24];
/*	if(a < 8500 - 2000) {
		fatal_error(ERROR_DEBUG1); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
	}
	if(a > 8500 + 2000) {
		fatal_error(ERROR_DEBUG1); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
	} */
	return a;
}


uint16_t get_motor_current(void)
{
	uint32_t current_avg = 0;
	uint16_t buffer_index = 0;
	uint16_t n = 0;
	uint16_t i;

	for(i = 0; i < ADC_BUFFER_CYCLE_REPETITIONS; i++) {
		current_avg += ADC_buffer[buffer_index + MOTOR_CURRENT_CYCLE_INDEX1] +
					   ADC_buffer[buffer_index + MOTOR_CURRENT_CYCLE_INDEX2] +
					   ADC_buffer[buffer_index + MOTOR_CURRENT_CYCLE_INDEX3];
		n += 3;
		buffer_index += ADC_CYCLE_INDEXES;
	}
	current_avg /= n;

//	return 1151; // DEBUG

	return (uint16_t)current_avg;
}


#define TEMPERATURE_ADC_CYCLE_INDEX 5
uint16_t get_temperature(void)
{
	uint16_t a = ADC_buffer[TEMPERATURE_ADC_CYCLE_INDEX + 0] + ADC_buffer[TEMPERATURE_ADC_CYCLE_INDEX + 8] +
	             ADC_buffer[TEMPERATURE_ADC_CYCLE_INDEX + 16] + ADC_buffer[TEMPERATURE_ADC_CYCLE_INDEX + 24];
	return a;
}

#define SUPPLY_VOLTAGE_ADC_CYCLE_INDEX 2
uint16_t get_supply_voltage_ADC_value(void)
{
	uint16_t a = ADC_buffer[SUPPLY_VOLTAGE_ADC_CYCLE_INDEX + 0] + ADC_buffer[SUPPLY_VOLTAGE_ADC_CYCLE_INDEX + 8] +
	             ADC_buffer[SUPPLY_VOLTAGE_ADC_CYCLE_INDEX + 16] + ADC_buffer[SUPPLY_VOLTAGE_ADC_CYCLE_INDEX + 24];
	return a;
}


uint16_t get_supply_voltage_volts_time_10(void)
{
	uint16_t supply_voltage = get_supply_voltage_ADC_value();
	uint32_t supply_voltage_calibrated = (supply_voltage * SUPPLY_VOLTAGE_CALIBRATION_CONSTANT) >> 20;

	return (uint16_t)supply_voltage_calibrated;
}


void print_supply_voltage(void)
{
	char buf[100];
	int16_t supply_voltage = get_supply_voltage_ADC_value();
	sprintf(buf, "Supply voltage (ADC value): %hd\n", supply_voltage);
	print_debug_string(buf);
	int32_t supply_voltage_calibrated = (supply_voltage * SUPPLY_VOLTAGE_CALIBRATION_CONSTANT) >> 20;
	int16_t whole_number = supply_voltage_calibrated / 10;
	int16_t decimal = (supply_voltage_calibrated % 10);
	sprintf(buf, "Supply voltage: %hd.%hu\n", whole_number, decimal);
	print_debug_string(buf);
}


void set_analog_watchdog_limits(uint16_t lower_limit, uint16_t upper_limit)
{
    ADC1->TR1 = (lower_limit << ADC_TR2_LT2_Pos) | (upper_limit << ADC_TR2_HT2_Pos); // select the lower and upper threshold for the ADC watchdog 1
    ADC1->TR2 = (lower_limit << ADC_TR2_LT2_Pos) | (upper_limit << ADC_TR2_HT2_Pos); // select the lower and upper threshold for the ADC watchdog 2
    ADC1->TR3 = (lower_limit << ADC_TR2_LT2_Pos) | (upper_limit << ADC_TR2_HT2_Pos); // select the lower and upper threshold for the ADC watchdog 3
}