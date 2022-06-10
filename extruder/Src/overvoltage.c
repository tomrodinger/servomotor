#include "stm32g0xx_hal.h"
#include "error_handling.h"

void overvoltage_init(void)
{
	EXTI->EXTICR[3] |= (0x01 << EXTI_EXTICR4_EXTI12_Pos);
	EXTI->IMR1 |= EXTI_IMR1_IM12; // unmask it
	EXTI->RTSR1 |= EXTI_RTSR1_RT12; // we want to trigger an interrupt on the rising edge of PB4
	NVIC_EnableIRQ(EXTI4_15_IRQn); // enable the interrupt to this external input
}

void EXTI4_15_IRQHandler(void)
{
    fatal_error(14);

	EXTI->RPR1 = EXTI_RPR1_RPIF4; // clear the interrupt flag
}

