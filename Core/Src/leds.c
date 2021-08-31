#include "stm32g0xx_hal.h"

void red_LED_on(void) {
    GPIOC->BSRR = 1 << 14;
}

void red_LED_off(void) {
    GPIOC->BSRR = (1 << 14) << 16;
}

void red_LED_toggle(void) {
    if(GPIOC->ODR & (1 << 14)) {
        red_LED_off();
    }
    else {
        red_LED_on();
    }
}

void green_LED_on(void) {
    GPIOC->BSRR = 1 << 15;
}

void green_LED_off(void) {
    GPIOC->BSRR = (1 << 15) << 16;
}

void green_LED_toggle(void) {
    if(GPIOC->ODR & (1 << 15)) {
        green_LED_off();
    }
    else {
        green_LED_on();
    }
}

