#include "stm32g0xx_hal.h"
#include "gpio.h"

#define BUTTON_PORT_A_PIN 14 // products M3 and M4
#define TOUCH_BUTTON_PORT_A_PIN 15

static void portA_init(void)
{
    GPIOA->MODER =
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE0_Pos)  | // AIN1: state control of the motor output AOUT1
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE1_Pos)  | // Reset for the motor driver chip (active low)
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE2_Pos)  | // serial port TX
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE3_Pos)  | // serial port RX
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE4_Pos)  | // Temperature sensor (NTC) analog input
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE5_Pos)  | // Hall sensor 1 analog input
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE6_Pos)  | // Hall sensor 2 analog input
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE7_Pos)  | // Hall sensor 3 analog input
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE8_Pos)  | // Motor channel A current control PWM output (PWM1)
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE9_Pos)  | // Don't use. Might be connected tp PA11 internally
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE10_Pos) | // Don't use. Might be connected tp PA12 internally
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE11_Pos) | // Overvoltage setting output (to set the overvoltage threshold) by PWM4
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE12_Pos) | // RS485 Data enable (DE) output
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE13_Pos) | // SWDIO (for programming)
            (MODER_DIGITAL_INPUT      << GPIO_MODER_MODE14_Pos) | // SWCLK (for programming) and button input
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE15_Pos);  // AIN2: state control of the motor output AOUT2

//    GPIOA->OTYPER = (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT1_Pos) | // make all the pins with analog components connected open drain
//                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT3_Pos) | // also, make the RS485 receive pin open drain
//                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT4_Pos) | // may not be necessary
//                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT5_Pos) |
//                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT6_Pos) |
//                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT8_Pos) |
//                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT9_Pos) |
//                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT10_Pos) |
//                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT13_Pos) |
//                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT14_Pos);
    GPIOA->OSPEEDR = 0xffffffff; // make all pins very high speed
    GPIOA->PUPDR = (PUPDR_PULL_UP << GPIO_PUPDR_PUPD3_Pos); // apply pull up on the RS485 receive pin
    GPIOA->BSRR = ((1 << 0) << 16); // set MOSFETs to output low on the motor AOUT1 line
    GPIOA->BSRR = ((1 << 15) << 16); // set MOSFETs to output low on the motor AOUT2 line
    GPIOA->BSRR = (1 << 1); // set the reset pin high so that the motor driver chip can operate
}


static void portB_init(void)
{
    GPIOB->MODER =
            (MODER_DIGITAL_INPUT      << GPIO_MODER_MODE0_Pos)  | // Motor fault indication from driver to MCU (active low)
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE1_Pos)  | // Supply voltage (24V) analog input (after divider)
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE3_Pos)  | // Motor channel B current control PWM output (PWM2)
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE4_Pos)  | // BIN1: state control of the motor output BOUT1
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE5_Pos)  | // BIN2: state control of the motor output BOUT2
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE6_Pos)  | // RS485 Data out
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE7_Pos)  | // RS485 Data receive
            (MODER_DIGITAL_INPUT      << GPIO_MODER_MODE8_Pos)  | // Overvoltage digital input (will shut off motor driver very fast if trigered)
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE9_Pos)  |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE10_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE11_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE12_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE13_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE14_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE15_Pos);

//    GPIOB->OTYPER = (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT1_Pos) | // Make the analog input pins open drain
//    	            (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT4_Pos) |
//    	            (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT5_Pos) |
//    	            (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT7_Pos) | // RX pin make as open drain
//    	            (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT8_Pos);  // Overvoltage digital input make as open drain
    GPIOB->OSPEEDR = 0xffffffff; // make all pins very high speed
    GPIOB->PUPDR = (PUPDR_PULL_UP   << GPIO_PUPDR_PUPD7_Pos) | // RX pin is pull up
                   (PUPDR_PULL_DOWN << GPIO_PUPDR_PUPD8_Pos);  // overvoltage pin is pull down
    GPIOB->BSRR = ((1 << 4) << 16); // set MOSFETs to output low on the motor BOUT1 line
    GPIOB->BSRR = ((1 << 5) << 16); // set MOSFETs to output low on the motor BOUT2 line
}


static void portC_init(void)
{
    GPIOC->MODER =
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE6_Pos)  | // AIO and BIO control line: this allows switching between 0% (high) and 38% (low) of the motor current
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE14_Pos) | // Red LED
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE15_Pos);  // Green LED
//    GPIOC->OTYPER = (0);  // Make the analog input pins open drain
    GPIOC->OSPEEDR = 0xffffffff; // very high speed
    GPIOC->PUPDR = 0; // no pins have pulling resistors
    GPIOC->BSRR = (1 << 0); // set the initial state of the motor to 0% current (MOSFETs off)
}


void GPIO_init_M4(void)
{
    portA_init();
    portB_init();
    portC_init();
}

inline uint8_t get_button_state_M4(void)
{
    return ((GPIOA->IDR & (1 << BUTTON_PORT_A_PIN)) != 0);
}