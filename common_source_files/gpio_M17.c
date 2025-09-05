#include "stm32g0xx_hal.h"
#include "gpio.h"

#define BUTTON_PORT_A_PIN 13 // products M17 and M17
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
            (MODER_DIGITAL_INPUT      << GPIO_MODER_MODE13_Pos) | // SWDIO (for programming) and button input
            (MODER_DIGITAL_INPUT      << GPIO_MODER_MODE14_Pos) | // SWCLK (for programming)
#if SOFTWARE_COMPATIBILITY_CODE == 2 // this is the version with AT5833 chip and this pin is used for the ~INDEX line
            (MODER_DIGITAL_INPUT     << GPIO_MODER_MODE15_Pos);   // If AT5833 chip: ~INDEX line, to indicate that the motor is at microstep 0 (out of 64)
#else 
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE15_Pos);  // If GC6609 chip: UART communicatioin pin (keep high when not communicating)
                                                                  // If A4988 chip: Stepper motor driver reset (reset low, normal operation high)
                                                                  // V11RC4 and beyong use this pin as MS1 for setting the microstepping mode
#endif
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

#if SOFTWARE_COMPATIBILITY_CODE == 0 // If A4988 chip:
    GPIOA->BSRR = ((1 << 15) << 16); // reset the stepper motor driver
    volatile uint32_t i;
    for(i = 0; i < 1000; i++); // make a delay
    GPIOA->BSRR = (1 << 15); // take the stepper motor driver out of reset by making the reset pin high
#endif
#if SOFTWARE_COMPATIBILITY_CODE >= 3
    GPIOA->BSRR = (1 << 15); // set the MS1 pin high        DEBUG commented out
//    GPIOA->BSRR = ((1 << 15) << 16); // set the MS1 pin low   DEBUG
#endif

}


static void portB_init(void)
{
    GPIOB->MODER =
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE0_Pos)  | // Motor driver enable (active low)
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE1_Pos)  | // Supply voltage (24V) analog input (after divider)
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE3_Pos)  | // In case of AT5833 chip: this is the ~nFault line, which will indicate a condition like undervoltage or overtemperature
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE4_Pos)  | // 3.3V power supply to the IO portion of the motor driver (setting these two pins low will reset the chip)
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE5_Pos)  | // 3.3V power supply to the IO portion of the motor driver (setting these two pins low will reset the chip)
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
//                  (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT4_Pos) |
//                  (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT5_Pos) |
//                  (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT7_Pos) | // RX pin make as open drain
//                  (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT8_Pos);  // Overvoltage digital input make as open drain
    GPIOB->OSPEEDR = 0xffffffff; // make all pins very high speed
    GPIOB->PUPDR = (PUPDR_PULL_UP << GPIO_PUPDR_PUPD7_Pos) | (PUPDR_PULL_DOWN << GPIO_PUPDR_PUPD8_Pos); // RX pin is pull up, overvoltage pin is pull down
#if SOFTWARE_COMPATIBILITY_CODE >= 2 // this is the version with AT5833 chip
    GPIOB->BSRR = ((1 << 0) | (1 << 4) | (1 << 5)); // Make the motor driver enable pin and supply pins high, otherwise the motor driver chip seems to draw too much power and heat up
#endif
}


static void portC_init(void)
{
    GPIOC->MODER =
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE6_Pos)  | // if the SOFTWARE_COMPATIBILITY_CODE is 3 or higher, then this is MS2 used for setting the microstepping mode in the AT5833 chip, otherwise this pin is not connected and not used    DEBUG commented out
//            (MODER_ANALOG_INPUT     << GPIO_MODER_MODE6_Pos)  | // if the SOFTWARE_COMPATIBILITY_CODE is 3 or higher, then this is MS2 used for setting the microstepping mode in the AT5833 chip, otherwise this pin is not connected and not used   DEBUG to allow 256 microsteps interpolation
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE14_Pos) | // Red LED
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE15_Pos);  // Green LED
//    GPIOC->OTYPER = (0);  // Make the analog input pins open drain
    GPIOC->OSPEEDR = 0xffffffff; // very high speed
    GPIOC->PUPDR = 0; // no pins have pulling resistors
    GPIOC->BSRR = (1 << 6); // set the MS2 pin high
}


void GPIO_init_M17(void)
{
    portA_init();
    portB_init();
    portC_init();
}

inline uint8_t get_button_state_M17(void)
{
    return ((GPIOA->IDR & (1 << BUTTON_PORT_A_PIN)) == 0);
}
