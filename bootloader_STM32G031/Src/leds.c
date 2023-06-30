#include "stm32g0xx_hal.h"
#include "leds.h"

inline void red_LED_on(void) {
    GPIOA->BSRR = 1 << 11;
}

inline void red_LED_off(void) {
    GPIOA->BSRR = (1 << 11) << 16;
}

inline void red_LED_toggle(void) {
    if(GPIOA->ODR & (1 << 11)) {
        red_LED_off();
    }
    else {
        red_LED_on();
    }
}

inline void green_LED_on(void) {
    GPIOA->BSRR = 1 << 12;
}

inline void green_LED_off(void) {
    GPIOA->BSRR = (1 << 12) << 16;
}

inline void green_LED_toggle(void) {
    if(GPIOA->ODR & (1 << 12)) {
        green_LED_off();
    }
    else {
        green_LED_on();
    }
}

