#include <stdio.h>
#include "stm32g0xx_hal.h"
#include "error_handling.h"
#include "debug_uart.h"

static int32_t step_count = 0;


void overvoltage_init(void)
{
#ifdef PRODUCT_NAME_M3
	EXTI->EXTICR[2] |= (0x01 << EXTI_EXTICR3_EXTI8_Pos); // Select Port B for EXTI8, which is the overvoltage signal on Servomotor#3
	EXTI->IMR1 |= EXTI_IMR1_IM8; // unmask it
	EXTI->RTSR1 |= EXTI_RTSR1_RT8; // we want to trigger an interrupt on the rising edge of PB8
#else
	EXTI->EXTICR[2] |= (0x01 << EXTI_EXTICR3_EXTI9_Pos); // Select Port B for EXTI9, which is the overvoltage signal on Servomotor#1 and Servomotor#2
	EXTI->IMR1 |= EXTI_IMR1_IM9; // unmask it
	EXTI->RTSR1 |= EXTI_RTSR1_RT9; // we want to trigger an interrupt on the rising edge of PB9
#endif
	NVIC_EnableIRQ(EXTI4_15_IRQn); // enable the interrupt to this external input
}



void step_and_direction_init(void)
{
	EXTI->EXTICR[1] |= (0x01 << EXTI_EXTICR2_EXTI4_Pos); // Select Port B for EXTI4
	EXTI->IMR1 |= EXTI_IMR1_IM4; // unmask it
	EXTI->RTSR1 |= EXTI_RTSR1_RT4; // we want to trigger an interrupt on the riding edge of PB4, which is the A line of the encoder
	NVIC_EnableIRQ(EXTI4_15_IRQn); // enable the interrupt to this external input
}


void EXTI4_15_IRQHandler(void)
{
    // Check for ovevoltage first
#ifdef PRODUCT_NAME_M3
    if(EXTI->RPR1 & EXTI_RPR1_RPIF8) {
        fatal_error(14);
    	EXTI->RPR1 = EXTI_RPR1_RPIF8; // clear the interrupt flag
    }
#else
    if(EXTI->RPR1 & EXTI_RPR1_RPIF9) {
        fatal_error(14);
    	EXTI->RPR1 = EXTI_RPR1_RPIF9; // clear the interrupt flag
    }
#endif
    // then check if the A line from the encoder transitioned
    if(EXTI->RPR1 & EXTI_RPR1_RPIF4) {
        if(GPIOB->IDR & (1 << 5)) {
            step_count += 1;
        }
        else {
            step_count -= 1;
        }
        EXTI->RPR1 = EXTI_RPR1_RPIF4; // clear the interrupt flag

    }
}


int32_t get_external_encoder_position(void)
{
    return step_count;
}


void print_external_encoder_position(void)
{
	char buf[100];
	sprintf(buf, "external encoder position:: %ld\n", step_count);
	print_debug_string(buf);
}