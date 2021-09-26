#include "stm32g0xx_hal.h"
#include "error_handling.h"

#define MICROSTEPS_PER_STEP 50

static int32_t step_count = 0;

void step_and_direction_init(void)
{
//	EXTI->EXTICR[1] = (0x01 << EXTI_EXTICR2_EXTI4_Pos);
//	EXTI->IMR1 |= EXTI_IMR1_IM4; // unmask it
//	EXTI->RTSR1 = EXTI_RTSR1_RT4; // we want to trigger an interrupt on the riding edge of PB4
//	NVIC_EnableIRQ(EXTI4_15_IRQn); // enable the interrupt to this external input
}

void EXTI4_15_IRQHandler(void)
{
    if(GPIOB->IDR & (1 << 5)) {
    	step_count += MICROSTEPS_PER_STEP;
    }
    else {
    	step_count -= MICROSTEPS_PER_STEP;
    }

//	fatal_error("PB4 interrupt triggered", 2);
	EXTI->RPR1 = EXTI_RPR1_RPIF4; // clear the interrupt flag
}

int32_t get_step_count(void)
{
	int32_t ret;

    __disable_irq();
    ret = step_count;
    step_count = 0;
    __enable_irq();

    return ret;
}
