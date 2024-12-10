#include <stdint.h>
#include "stm32g0xx_hal.h"
#include "gpio.h"
#include "AT5833.h"


static void delay_us(uint32_t us)
{
    // This function will delay for the specified number of microseconds
    uint32_t delay = us * 4;
    for (volatile int i = 0; i < delay; i++) {
        __asm__("nop");
        __asm__("nop");
        __asm__("nop");
        __asm__("nop");
        __asm__("nop");
        __asm__("nop");
    }
}


static void Delay_ms(uint32_t ms)
{
    // This function will delay for the specified number of milliseconds
    for (int i = 0; i < ms; i++) {
        delay_us(1000);
    }
}


void power_on_AT5833(void)
{
    GPIOB->BSRR = (1 << 0) | (1 << 4) | (1 << 5); // set the two VIO pins high to supply power to the diver chip and set MENABLE high to disable the motor for now
    GPIOA->BSRR = (1 << 15); // set MS1 high
    GPIOC->BSRR = (1 << 6); // set MS2 high
}


void power_off_AT5833(void)
{
    TIM1->CCR1 = 0; // set the PWM duty cycle to 0, which controls the current control reference
    GPIOA->BSRR = ((1 << 0) << 16); // set DIR low
    GPIOA->BSRR = ((1 << 1) << 16); // set STEP low
    GPIOA->BSRR = ((1 << 15) << 16); // set MS1 low
    GPIOC->BSRR = ((1 << 6) << 16); // set MS2 low
    GPIOB->BSRR = (((1 << 0) | (1 << 4) | (1 << 5)) << 16); // set MENABLE low and set the two pins that control the VIO (3.3V power supply to the motor driver chip) low
}


void reset_AT5833(void)
{
    power_off_AT5833();
    Delay_ms(100);
    power_on_AT5833();
    Delay_ms(10);
}


void test_motor_stepping_AT5833(void)
{
    uint32_t acceleration_delay = 1000;
    GPIOA->BSRR = (1 << 0); // set DIR high
    for (int i = 0; i < 100000; i++) {
        GPIOA->BSRR = (1 << 1); // set STEP high
        delay_us(5);
        GPIOA->BSRR = ((1 << 1) << 16); // set STEP low
        delay_us(acceleration_delay);
        if (acceleration_delay > 13 * 2) {
            acceleration_delay--;
        }
    }
}