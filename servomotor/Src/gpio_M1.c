#include "stm32g0xx_hal.h"
#include "gpio.h"

#define BUTTON_PORT_A_PIN 13 // products M1 and M2
#define TOUCH_BUTTON_PORT_A_PIN 15

static void portA_init(void)
{
    GPIOA->MODER =
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE0_Pos)  | // current measurement channel A
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE1_Pos)  | // MOSFET switch disable
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE2_Pos)  | // serial port TX
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE3_Pos)  | // serial port RX
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE4_Pos)  | // hall sensor 2
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE5_Pos)  | // hall sensor 1
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE6_Pos)  | // hall sensor 3
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE7_Pos)  | // voltage measurement of the power supply
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE8_Pos)  | // PWM 1
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE9_Pos)  |
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE10_Pos) | // PWM 3
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE11_Pos) | // PWM 4
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE12_Pos) | // RS485 drive enable
            (MODER_DIGITAL_INPUT      << GPIO_MODER_MODE13_Pos) | // Button input and also SWDIO (for programming)
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE14_Pos) | // SWCLK (for programming)
            (MODER_DIGITAL_INPUT      << GPIO_MODER_MODE15_Pos);  // touch button

    GPIOA->OTYPER = (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT0_Pos)  | // make all the pins with analog components connected open drain
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT3_Pos)  | // also, make the switch disable pin open drain since there is no resistor between the pin and the base of a transistor
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT4_Pos)  | // also, make the debug UART receive pin open drain
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT5_Pos)  | // may not be necessary
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT6_Pos)  |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT7_Pos)  |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT9_Pos)  |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT13_Pos) |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT14_Pos) |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT15_Pos);
    GPIOA->OSPEEDR = 0xffffffff; // make all pins very high speed
    GPIOA->PUPDR = (PUPDR_PULL_UP << GPIO_PUPDR_PUPD1_Pos) | (PUPDR_PULL_UP << GPIO_PUPDR_PUPD3_Pos); // apply pull up on the switch disable and the UART receive pin
}


static void portB_init(void)
{
    GPIOB->MODER =
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE0_Pos)  |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE1_Pos)  |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE2_Pos)  | // Temperature sensor (NTC) analog input
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE3_Pos)  | // PWM 2
            (MODER_DIGITAL_INPUT      << GPIO_MODER_MODE4_Pos)  | // Encoder A input or step input
            (MODER_DIGITAL_INPUT      << GPIO_MODER_MODE5_Pos)  | // Encoder B input or direction input
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE6_Pos)  | // RS485 Data out
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE7_Pos)  | // RS485 Data receive
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE8_Pos)  |
            (MODER_DIGITAL_INPUT      << GPIO_MODER_MODE9_Pos)  | // overvoltage digital input
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE10_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE11_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE12_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE13_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE14_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE15_Pos);

    GPIOB->OTYPER = (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT4_Pos) | // step input make as open drain
    	            (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT5_Pos) | // direction input make as open drain
    		        (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT7_Pos);  // RX pin make as open drain
    GPIOB->OSPEEDR = 0xffffffff; // make all pins very high speed
    GPIOB->PUPDR = (PUPDR_PULL_UP << GPIO_PUPDR_PUPD4_Pos)   | // Encoder A
                   (PUPDR_PULL_UP << GPIO_PUPDR_PUPD5_Pos)   | // Encoder B
                   (PUPDR_PULL_UP << GPIO_PUPDR_PUPD7_Pos)   | // RX pin is pull up
                   (PUPDR_PULL_DOWN << GPIO_PUPDR_PUPD9_Pos);  // Overvoltage pin is pull down
}


static void portC_init(void)
{
    GPIOC->MODER =
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE0_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE1_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE2_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE3_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE4_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE5_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE6_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE7_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE8_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE9_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE10_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE11_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE12_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE13_Pos) |
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE14_Pos) | // Red LED
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE15_Pos);  // Green LED

    GPIOC->OTYPER = 0; // no pins are open drain
    GPIOC->OSPEEDR = 0xffffffff; // very high speed
    GPIOC->PUPDR = 0; // no pins have pulling resistors
}


static void portD_init(void)
{
    GPIOD->MODER =
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE0_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE1_Pos) | 
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE2_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE3_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE4_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE5_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE6_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE7_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE8_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE9_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE10_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE11_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE12_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE13_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE14_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE15_Pos);

    GPIOD->OTYPER = 0;
    GPIOD->OSPEEDR = 0xffffffff; // very high speed
    GPIOD->PUPDR = 0;
}


void GPIO_init_M1(void)
{
    portA_init();
    portB_init();
    portC_init();
    portD_init();
}

inline uint8_t get_button_state_M1(void)
{
    return ((GPIOA->IDR & (1 << BUTTON_PORT_A_PIN)) == 0);
}
