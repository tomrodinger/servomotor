#include "stm32g0xx_hal.h"
#include "gpio.h"

#define BUTTON_PORT_A_PIN 13

static void portA_init(void)
{
    GPIOA->MODER =
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE0_Pos)  | // Temperature sensor (NTC) analog input
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE1_Pos)  | // Hall sensor 2 analog input
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE2_Pos)  | // Hall sensor 1 analog input
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE3_Pos)  | // Hall sensor 3 analog input
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE4_Pos)  | // Amplified motor current sense voltage (Phase A)
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE5_Pos)  | // Amplified motor current sense voltage (Phase B)
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE6_Pos)  | // Amplified inline motor current sense voltage (Phase B)
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE7_Pos)  | // Motor phase A voltage control PWM output (TIM1_CH1N complementary output)
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE8_Pos)  | // Motor phase A voltage control PWM output (TIM1_CH1)
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE9_Pos)  | // Don't use. Might be connected to PA11 internally
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE10_Pos) | // Don't use. Might be connected to PA12 internally
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE11_Pos) | // Overvoltage setting PWM output (to set the overvoltage threshold) by TIM1_CH4
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE12_Pos) | // RS485 Data enable (DE) output
            (MODER_DIGITAL_INPUT      << GPIO_MODER_MODE13_Pos) | // SWDIO (for programming) and also the test button input (pull low when pushed)
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE14_Pos) | // SWCLK (for programming) and serial port TX
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE15_Pos);  // Serial port RX

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
    GPIOA->PUPDR = (PUPDR_PULL_UP << GPIO_PUPDR_PUPD15_Pos) | (PUPDR_PULL_UP << GPIO_PUPDR_PUPD13_Pos); // Serial RX pin and test button GPIO are pulled up
//    GPIOA->BSRR = ((1 << 1) << 16); // set low initially
}


static void portB_init(void)
{
    GPIOB->MODER =
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE0_Pos)  | // Motor phase B voltage control PWM output (TIM1_CH2N complementary output)
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE1_Pos)  | // Supply voltage (24V) analog input (after divider)
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE3_Pos)  | // Motor phase B voltage control PWM output (TIM1_CH2)
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE4_Pos)  | 
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE5_Pos)  |
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
    GPIOB->PUPDR = (PUPDR_PULL_UP   << GPIO_PUPDR_PUPD7_Pos) | // RS485 data receive pin is pull up
                   (PUPDR_PULL_DOWN << GPIO_PUPDR_PUPD8_Pos);  // overvoltage pin is pull down
}


static void portC_init(void)
{
    GPIOC->MODER =
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE6_Pos)  |
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE14_Pos) | // Red LED
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE15_Pos);  // Green LED
//    GPIOC->OTYPER = (0);  // Make the analog input pins open drain
    GPIOC->OSPEEDR = 0xffffffff; // very high speed
    GPIOC->PUPDR = 0; // no pins have pulling resistors
}


void GPIO_init_M23(void)
{
    portA_init();
    portB_init();
    portC_init();
}

inline uint8_t get_button_state_M23(void)
{
    return ((GPIOA->IDR & (1 << BUTTON_PORT_A_PIN)) == 0);
}
