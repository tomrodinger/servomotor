#include "stm32g0xx_hal.h"
#include "gpio.h"

#define BUTTON_PORT_A_PIN 14 // products M3 and M4
#define TOUCH_BUTTON_PORT_A_PIN 15

static void portA_init(void)
{
    GPIOA->MODER =
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE0_Pos)  | // Direction control of the motor digital output
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE1_Pos)  | // Step motor step control output (to rotate the motor)
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE2_Pos)  | // serial port TX
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE3_Pos)  | // serial port RX
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE4_Pos)  | // Temperature sensor (NTC) analog input
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE5_Pos)  | // Hall sensor 1 analog input
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE6_Pos)  | // Hall sensor 2 analog input
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE7_Pos)  | // Hall sensor 3 analog input
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE8_Pos)  | // Motor current control PWM output
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE9_Pos)  | // Don't use. Might be connected tp PA11 internally
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE10_Pos) | // Don't use. Might be connected tp PA12 internally
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE11_Pos) | // Overvoltage setting output (to set the overvoltage threshold) by PWM
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE12_Pos) | // RS485 Data enable (DE) output
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE13_Pos) | // SWDIO (for programming)
            (MODER_DIGITAL_INPUT      << GPIO_MODER_MODE14_Pos) | // SWCLK (for programming) and button input
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE15_Pos);  // Microstepping setting (0 = 1/64 microstepping, 1 = 1/16 microstepping)

    GPIOA->OTYPER = OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT4_Pos;
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

//    GPIOA->BSRR = ((1 << 15) << 16); // set the microstepping pin sentting MS1 to low, which will set to 1/64 microstepping
    GPIOA->BSRR = (1 << 15); // set the microstepping pin sentting MS1 to high, which will set to 1/16 microstepping
}


static void portB_init(void)
{
    GPIOB->MODER =
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE0_Pos)  | // Motor driver enable (active low)
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE1_Pos)  | // Supply voltage (24V) analog input (after divider)
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE3_Pos)  | // SPREAD setting (Low=StealthChop, High=SpreadCycle)
            (MODER_DIGITAL_INPUT      << GPIO_MODER_MODE4_Pos)  | // DIAG (Diagnostic and StallGuard output. Hi level upon stall detection or driver error.)
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE5_Pos)  | // PDN_UART signal to motor driver (low is power down; this can be used for digital communication as well)
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
    GPIOB->BSRR = (1 << 0) | (1 << 5); // diable the motor and set the PDN_UART pin to high so that the motor driver is not in power down mode
}


static void portC_init(void)
{
    GPIOC->MODER =
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE6_Pos)  | // STDBY input to motor driver (high is standby, low is normal operation)
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE14_Pos) | // Red LED
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE15_Pos);  // Green LED
//    GPIOC->OTYPER = (0);  // Make the analog input pins open drain
    GPIOC->OSPEEDR = 0xffffffff; // very high speed
    GPIOC->PUPDR = 0; // no pins have pulling resistors
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
