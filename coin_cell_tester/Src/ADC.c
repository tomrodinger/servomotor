#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32g0xx_hal.h"
#include "ADC.h"
#include "leds.h"
#include "debug_uart.h"
#include "microsecond_clock.h"


void adc_init(volatile uint16_t *ADC_buffer)
{
    volatile uint32_t i;

    RCC->AHBENR |= RCC_AHBENR_DMA1EN; // enable the clock to the DMA1
    RCC->APBENR2 |= RCC_APBENR2_ADCEN; // enable the clock to the ADC
    RCC->CCIPR |= (1 << RCC_CCIPR_ADCSEL_Pos); // use the PLLPCLK for the ADC
    ADC->CCR |= (1 << ADC_CCR_PRESC_Pos); // divide the clock source by 2 to get 32 MHz for the ADC clock
//  ADC->CCR |= ADC_CCR_VREFEN; // DEBUG turn on ability to measure internal reference
    ADC1->CFGR1 |= ADC_CFGR1_CHSELRMOD | ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD; // channel selection is done by selecting the exact sequence, continuous conversion mode, if overrun occurs, we take the latest value, 12-bit resolution
//  ADC1->CFGR1 |= ADC_CFGR1_DMACFG; // DMA circular mode (after reaching the end of the buffer, go back to the beginning and keep converting). Comment this out to do one shot mode (fill the buffer to the end then stop).)
//    ADC1->CFGR2 |= (0 << ADC_CFGR2_CKMODE_Pos) | (1 << ADC_CFGR2_OVSR_Pos) | ADC_CFGR2_OVSE; // select the clock mode to take the clock source from the one selected in the RCC->CCIPR register, enable 4x oversampling
//    ADC1->CFGR2 |= (0 << ADC_CFGR2_CKMODE_Pos); // select the clock mode to take the clock source from the one selected in the RCC->CCIPR register, no oversampling
    ADC1->CFGR2 |= (0 << ADC_CFGR2_CKMODE_Pos) | (4 << ADC_CFGR2_OVSS_Pos) | (7 << ADC_CFGR2_OVSR_Pos) | ADC_CFGR2_OVSE; // select the clock mode to take the clock source from the one selected in the RCC->CCIPR register, enable 256x oversampling and 4-bit right shift
    ADC1->SMPR |= (6 << ADC_SMPR_SMP1_Pos); // select 79.5 ADC cycles as the sampling time, the total time for 1 conversion (including all oversampling) will be 736us
                                            // calculation: 1/32000000*(79.5+12.5)*256*1000000

    ADC1->CHSELR = (1  << ADC_CHSELR_SQ1_Pos) |    // select the channels to be converted, 8 channels total
                   (1  << ADC_CHSELR_SQ2_Pos) |    // select all to convert channel 1 (PA1), which is the 
                   (1  << ADC_CHSELR_SQ3_Pos) |    // coin cell voltage
                   (1  << ADC_CHSELR_SQ4_Pos) |
                   (1  << ADC_CHSELR_SQ5_Pos) |
                   (1  << ADC_CHSELR_SQ6_Pos) |
                   (1  << ADC_CHSELR_SQ7_Pos) |
                   (15 << ADC_CHSELR_SQ8_Pos);

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
    NVIC_SetPriority(DMA1_Channel1_IRQn, 1); // set the interrupt priority to 1 (which is a bit lower than the highest level of 0)
    NVIC_EnableIRQ(DMA1_Channel1_IRQn); // enable the interrupt for the DMA channel 1
    DMA1_Channel1->CCR |= DMA_CCR_EN; // enable the DMA channel as the last step (see section 11.4.3 in the reference manual).

//    ADC1->CR |= ADC_CR_ADSTART; // start to do the conversions one by one until the DMA buffer is full
}
