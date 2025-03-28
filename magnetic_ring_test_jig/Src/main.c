#include "stm32g0xx_hal.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "leds.h"
#include "debug_uart.h"
#include "RS485.h"
#include "mosfets.h"
#include "ADC.h"
#include "PWM.h"
#include "hall_sensor_calculations.h"
#include "motor_control.h"
#include "microsecond_clock.h"
#include "clock_calibration.h"
#include "error_handling.h"
#include "GPIO_interrupts.h"
#include "unique_id.h"
#include "settings.h"
#include "commands.h"
#include "product_info.h"
#include "global_variables.h"
#include "crc32.h"
#include "device_status.h"
#ifdef PRODUCT_NAME_M1
#include "commutation_table_M1.h"
#endif
#ifdef PRODUCT_NAME_M2
#include "commutation_table_M2.h"
#include "hall_sensor_constants_M2.h"
#endif
#ifdef PRODUCT_NAME_M3
#include "commutation_table_M3.h"
#endif


char PRODUCT_DESCRIPTION[] = "Servomotor";

struct __attribute__((__packed__)) firmware_version_struct {
	uint8_t bugfix;
	uint8_t minor;
	uint8_t major;
	uint8_t not_used;
};
#define NOT_USED 0xff
#define MAJOR_FIRMWARE_VERSION 0
#define MINOR_FIRMWARE_VERSION 1
#define BUGFIX_FIRMWARE_VERSION 0
struct firmware_version_struct firmware_version = {MAJOR_FIRMWARE_VERSION, MINOR_FIRMWARE_VERSION, BUGFIX_FIRMWARE_VERSION, NOT_USED};

#define PING_PAYLOAD_SIZE 10

extern uint16_t ADC_buffer[DMA_ADC_BUFFER_SIZE];

static uint64_t my_unique_id;
static int16_t detect_devices_delay = -1;
static uint16_t green_led_action_counter = 0;
static uint8_t n_identify_flashes = 0;

void clock_init(void)
{
    FLASH->ACR = (2 << FLASH_ACR_LATENCY_Pos) | FLASH_ACR_PRFTEN | // set two wait states, enable prefetch,
                 FLASH_ACR_ICEN;                                   // enable instruction cache

    #define M_DIVISION_FACTOR 1       // division factor (1 to 8 are valid values)
    #define N_MULTIPLICATION_FACTOR 8 // multiplication factor  (8 to 86 are valid numbers)
    #define PLLPCLK_DIVISION_FACTOR 2 // valid range is 2 to 32 // this clock must not exceed 122MHz
    #define PLLRCLK_DIVISION_FACTOR 2 // valid range is 2 to 8  // this clock must not exceed 64MHz
    // Input frequency from HSI clock is 16MHz
    // VCO frequency = input frequency * N / M
    // VCO frequency must be in the range 64 to 344MHz
    // input frequency / M must be in the range 2.66 to 16 MHz
    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSI | ((M_DIVISION_FACTOR - 1) << RCC_PLLCFGR_PLLM_Pos) | (N_MULTIPLICATION_FACTOR << RCC_PLLCFGR_PLLN_Pos) |
                 RCC_PLLCFGR_PLLPEN | ((PLLPCLK_DIVISION_FACTOR - 1) << RCC_PLLCFGR_PLLP_Pos) |
                 RCC_PLLCFGR_PLLREN | ((PLLRCLK_DIVISION_FACTOR - 1) << RCC_PLLCFGR_PLLR_Pos);
    RCC->CR = RCC_CR_HSION | RCC_CR_PLLON;
    while((RCC->CR & RCC_CR_HSIRDY) == 0);
    RCC->CFGR = 2 << RCC_CFGR_SW_Pos; // set the system clock to use the PLLRCLK
    while((RCC->CFGR & RCC_CFGR_SW_Msk) != 2);

    RCC->APBENR2 |= (1 << RCC_APBENR2_SYSCFGEN_Pos); // enable the clock to the syscfg (may not be necessary to enable this)
    RCC->IOPENR = 0x3f; // enable the clock to all I/O ports
}

void systick_init(void)
{
    SysTick->LOAD  = 640000;       // set this timer to trigger the interrupt 100 times per second (ie. 64000000 / 100)
    NVIC_SetPriority (SysTick_IRQn, 3); /* set Priority for Systick Interrupt */
    SysTick->VAL   = 0UL;                                             /* Load the SysTick Counter Value */
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                     SysTick_CTRL_TICKINT_Msk   |
                     SysTick_CTRL_ENABLE_Msk;                         /* Enable SysTick IRQ and SysTick Timer */



/*
    SysTick->CTRL = 0; // disable first in case it is already enabled
    SysTick->LOAD  = (uint32_t)(16000000 - 1);              // set reload register to generate interrupt at 4 Hz
//    HAL_NVIC_SetPriority(SysTick_IRQn, TICK_INT_PRIORITY, 0U);
    SysTick->VAL   = 1600000 - 1;                                             // Load the SysTick Counter Value
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                     SysTick_CTRL_TICKINT_Msk   |
                     SysTick_CTRL_ENABLE_Msk;                         // Enable SysTick IRQ and SysTick Timer
*/
}

#define MODER_DIGITAL_INPUT 0
#define MODER_DIGITAL_OUTPUT 1
#define MODER_ALTERNATE_FUNCTION 2
#define MODER_ANALOG_INPUT 3 // this is the default after power up
#define OTYPER_PUSH_PULL 0 // this is the default after power up
#define OTYPER_OPEN_DRAIN 1
#define OSPEEDR_VERY_LOW_SPEED 0 // this is the default except some pins on port A
#define OSPEEDR_LOW_SPEED 1
#define OSPEEDR_HIGH_SPEED 2
#define OSPEEDR_VERY_HIGH_SPEED 3
#define PUPDR_NO_PULL_UP_OR_DOWN 0 // this is the default except on some pins on port A
#define PUPDR_PULL_UP 1
#define PUPDR_PULL_DOWN 2

#ifndef PRODUCT_NAME_M3 // Below are the port pin settings for products M1 and M2
#define BUTTON_PORT_A_PIN 13
#else
#define BUTTON_PORT_A_PIN 14
#endif
#define TOUCH_BUTTON_PORT_A_PIN 15

#ifndef PRODUCT_NAME_M3 // Below are the port pin settings for products M1 and M2
void portA_init(void)
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
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT1_Pos)  | // also, make the switch disable pin open drain since there is no resistor between the pin and the base of a transistor
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT3_Pos)  | // also, make the debug UART receive pin open drain
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT4_Pos)  | // may not be necessary
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT5_Pos)  |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT6_Pos)  |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT7_Pos)  |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT9_Pos)  |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT13_Pos) |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT14_Pos) |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT15_Pos);
    GPIOA->OSPEEDR = 0xffffffff; // make all pins very high speed
    GPIOA->PUPDR = (PUPDR_PULL_UP << GPIO_PUPDR_PUPD1_Pos) | (PUPDR_PULL_UP << GPIO_PUPDR_PUPD3_Pos); // apply pull up on the switch disable and the UART receive pin
}


void portB_init(void)
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


void portC_init(void)
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


void portD_init(void)
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


void GPIO_init(void)
{
    portA_init();
    portB_init();
    portC_init();
    portD_init();
}

#else // Below are the port pin settings for product M3

void portA_init(void)
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
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE15_Pos);  // Stepper motor driver reset (reset low, normal operation high)

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

    GPIOA->BSRR = ((1 << 15) << 16); // reset the stepper motor driver
    volatile uint32_t i;
    for(i = 0; i < 1000; i++); // make a delay
    GPIOA->BSRR = (1 << 15); // take the stepper motor driver out of reset by making the reset pin high
}


void portB_init(void)
{
    GPIOB->MODER =
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE0_Pos)  | // Motor driver enable (active low)
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE1_Pos)  | // Supply voltage (24V) analog input (after divider)
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE3_Pos)  |
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
//    	            (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT4_Pos) |
//    	            (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT5_Pos) |
//    	            (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT7_Pos) | // RX pin make as open drain
//    	            (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT8_Pos);  // Overvoltage digital input make as open drain
    GPIOB->OSPEEDR = 0xffffffff; // make all pins very high speed
    GPIOB->PUPDR = (PUPDR_PULL_UP << GPIO_PUPDR_PUPD7_Pos) | (PUPDR_PULL_DOWN << GPIO_PUPDR_PUPD8_Pos); // RX pin is pull up, overvoltage pin is pull down
}


void portC_init(void)
{
    GPIOC->MODER =
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE6_Pos)  |
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE14_Pos) | // Red LED
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE15_Pos);  // Green LED
//    GPIOC->OTYPER = (0);  // Make the analog input pins open drain
    GPIOC->OSPEEDR = 0xffffffff; // very high speed
    GPIOC->PUPDR = 0; // no pins have pulling resistors
}


void GPIO_init(void)
{
    portA_init();
    portB_init();
    portC_init();
}

#endif


// This interrupt will be called 100 times per second
void SysTick_Handler(void)
{
    #define OVERVOLTAGE_PORT_B_PIN 12

    if(n_identify_flashes == 0) {
        if(green_led_action_counter < 99) {
            green_LED_off();
            green_led_action_counter++;
        }
        else if(green_led_action_counter < 100) {
            green_LED_on();
            green_led_action_counter++;
        }
        else {
            green_led_action_counter = 0;
        }
    }
    else {
        if(green_led_action_counter < 3) {
            green_LED_on();
            green_led_action_counter++;
        }
        else if(green_led_action_counter < 8) {
            green_LED_off();
            green_led_action_counter++;
        }
        else {
            green_led_action_counter = 0;
            n_identify_flashes--;
        }
    }

    if(detect_devices_delay > 0) {
        detect_devices_delay--;
    }

//    if((GPIOB->IDR & (1 << OVERVOLTAGE_PORT_B_PIN)) == 0) {
//        red_LED_off();
//    }
//    else {
//        red_LED_on();
//    }
}


void process_packet(void)
{
    uint8_t command;
    uint16_t payload_size;
    void *payload;
    uint8_t is_broadcast;

    // There is a distinct possibility that the new packet is not for us. We will know after we see the return value of rs485_get_next_packet.
    if (!rs485_get_next_packet(&command, &payload_size, &payload, &is_broadcast)) {
        // The new packet, whatever it is, is not of interest to us and we need to clear it and return out of here
        rs485_done_with_this_packet();
        return;
    }

    if(!rs485_validate_packet_crc32()) {
        // CRC32 validation failed, allow next command and return
        rs485_done_with_this_packet();
        return;
    }

//    print_number("Received a command with length: ", commandLen);
    if((axis == global_settings.my_alias) || (axis == ALL_ALIAS)) {
//        print_number("Axis:", axis);
//        print_number("command:", command);
        switch(command) {
        case CAPTURE_HALL_SENSOR_DATA_COMMAND:
            {
                uint8_t capture_length = payload[0];
                rs485_done_with_this_packet();
                if(!is_broadcast) {
                    rs485_transmit(NO_ERROR_RESPONSE, 3);
                }
                start_calibration(capture_length);
            }
            break;
        case RESET_TIME_COMMAND:
            rs485_done_with_this_packet();
        	reset_time();
            if(!is_broadcast) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case GET_CURRENT_TIME_COMMAND:
            rs485_done_with_this_packet();
            if(!is_broadcast) {
                struct __attribute__((__packed__)) {
                    uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                    uint64_t local_time;
                    uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
                } time_reply;
                time_reply.local_time = get_microsecond_time();
                rs485_finalize_and_transmit_packet(&time_reply, sizeof(time_reply), crc32_enabled);
            }
            break;
        case TIME_SYNC_COMMAND:
            {
                uint64_t time_from_master = 0;
                memcpy(&time_from_master, payload, 6);
                rs485_done_with_this_packet();
                int32_t time_error = time_sync(time_from_master);
                uint16_t clock_calibration_value = get_clock_calibration_value();
                if(!is_broadcast) {
                    struct __attribute__((__packed__)) {
                        uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                        int32_t time_error;
                        uint16_t clock_calibration_value;
                        uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
                    } time_sync_reply;
                    time_sync_reply.time_error = time_error;
                    time_sync_reply.clock_calibration_value = clock_calibration_value;
                    rs485_finalize_and_transmit_packet(&time_sync_reply, sizeof(time_sync_reply), crc32_enabled);
                }
            }
            break;
        case GET_UPDATE_FREQUENCY_COMMAND:
            {
                rs485_done_with_this_packet();
                uint32_t frequency = get_update_frequency();
                if(!is_broadcast) {
                    struct __attribute__((__packed__)) {
                        uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                        uint32_t frequency;
                        uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
                    } frequency_reply;
                    frequency_reply.frequency = frequency;
                    rs485_finalize_and_transmit_packet(&frequency_reply, sizeof(frequency_reply), crc32_enabled);
                }
            }
            break;
        case DETECT_DEVICES_COMMAND:
            rs485_done_with_this_packet();
        	detect_devices_delay = get_random_number(99);
			break;
        case SET_DEVICE_ALIAS_COMMAND:
            {
                uint64_t unique_id = ((int64_t*)payload)[0];
                uint8_t new_alias = payload[8];
                rs485_done_with_this_packet();
                if(unique_id == my_unique_id) {
                    rs485_transmit(NO_ERROR_RESPONSE, 3); 
                    print_number("Unique ID matches. Will save the alias and reset. New alias:", (uint16_t)new_alias);
                    microsecond_delay(5000); // 5ms should be enough time to transmit the above debug message, which is about 100 bytes, at baud rate of 230400
                    global_settings.my_alias = new_alias;
                    save_global_settings(); // this will never return because the device will reset after writing the new settings to flash
                }
            }
            break;
        case GET_PRODUCT_INFO_COMMAND:
            rs485_done_with_this_packet();
			if(!is_broadcast) {
				struct __attribute__((__packed__)) {
					uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
					uint8_t product_info_length;
					struct product_info_struct product_info;
					uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
				} product_info_reply;
				product_info_reply.product_info_length = sizeof(struct product_info_struct);
				memcpy(&product_info_reply.product_info, (struct product_info_struct *)(PRODUCT_INFO_MEMORY_LOCATION), sizeof(struct product_info_struct));
				rs485_finalize_and_transmit_packet(&product_info_reply, sizeof(product_info_reply), crc32_enabled);
			}
			break;
        case GET_PRODUCT_DESCRIPTION_COMMAND:
            rs485_done_with_this_packet();
			if(!is_broadcast) {
				struct __attribute__((__packed__)) {
					uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
					uint8_t product_description_length;
					char product_description[sizeof(PRODUCT_DESCRIPTION)];
					uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
				} product_description_reply;
				product_description_reply.product_description_length = sizeof(PRODUCT_DESCRIPTION);
				memcpy(product_description_reply.product_description, &PRODUCT_DESCRIPTION, sizeof(PRODUCT_DESCRIPTION));
				rs485_finalize_and_transmit_packet(&product_description_reply, sizeof(product_description_reply), crc32_enabled);
			}
			break;
        case GET_FIRMWARE_VERSION_COMMAND:
            rs485_done_with_this_packet();
			if(!is_broadcast) {
				struct __attribute__((__packed__)) {
					uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
					uint8_t firmware_version_length;
					char firmware_version_data[sizeof(firmware_version)];
					uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
				} firmware_version_reply;
				firmware_version_reply.firmware_version_length = sizeof(firmware_version);
				memcpy(firmware_version_reply.firmware_version_data, &firmware_version, sizeof(firmware_version));
				rs485_finalize_and_transmit_packet(&firmware_version_reply, sizeof(firmware_version_reply), crc32_enabled);
			}
			break;
        case SYSTEM_RESET_COMMAND:
            NVIC_SystemReset();
            break;
        case PING_COMMAND:
            {
                struct __attribute__((__packed__)) {
                    uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                    uint8_t ping_payload[PING_PAYLOAD_SIZE];
                    uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
                } ping_reply;
                memcpy(ping_reply.ping_payload, payload, PING_PAYLOAD_SIZE);
                rs485_done_with_this_packet();
                if(!is_broadcast) {
                    rs485_finalize_and_transmit_packet(&ping_reply, sizeof(ping_reply), crc32_enabled);
                }
            }
            break;
        case GET_SUPPLY_VOLTAGE_COMMAND:
            rs485_done_with_this_packet();
            {
                struct __attribute__((__packed__)) {
                    uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                    uint16_t supply_voltage;
                    uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
                } supply_voltage_reply;
                if(!is_broadcast) {
                    supply_voltage_reply.supply_voltage = get_supply_voltage_volts_time_10();
                    rs485_finalize_and_transmit_packet(&supply_voltage_reply, sizeof(supply_voltage_reply), crc32_enabled);
                }
            }
            break;
        case IDENTIFY_COMMAND:
            {
                uint64_t unique_id = ((int64_t*)payload)[0];
                rs485_done_with_this_packet();
                if(unique_id == my_unique_id) {
                    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk; // temporarily disable the SysTick interrupt
                    green_led_action_counter = 0;
                    n_identify_flashes = 30;
                    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; // reneable the interrupt
                    print_debug_string("Identifying\n");
                    rs485_transmit(NO_ERROR_RESPONSE, 3);
                }
                break;
            }
        }
    }
    else {
        rs485_done_with_this_packet();
    }
}

void transmit_unique_id(void)
{
    crc32_init();
    calculate_crc32_buffer((uint8_t*)&my_unique_id, 8);
    uint32_t crc32 = calculate_crc32_u8(global_settings.my_alias); // and also the alias (1 byte)
    rs485_transmit(RESPONSE_CHARACTER_TEXT "\x01\x0d", 3);
    rs485_transmit(&my_unique_id, 8);
    rs485_transmit(&global_settings.my_alias, 1);
    rs485_transmit(&crc32, 4);
}


void process_debug_uart_commands(void)
{
    uint8_t command_debug_uart = get_command_debug_uart();

    if(command_debug_uart != 0) {
    	switch(command_debug_uart) {
    	case 'p':
            print_debug_string("\n");
            print_hall_sensor_data();
            print_motor_temperature();
            print_supply_voltage();
			break;
    	case 'S':
			print_debug_string("Saving settings\n");
            save_global_settings();
    		break;
        case 'r':
            NVIC_SystemReset();
            break;
		}
    	command_debug_uart = 0;
	}
}


void button_logic(void)
{
    static uint64_t press_start_time = 0;
    uint32_t time_pressed_down = 0;
    static uint8_t press_flag = 0;

#ifdef PRODUCT_NAME_M1
    if((GPIOA->IDR & (1 << BUTTON_PORT_A_PIN)) == 0) {
#else
    if((GPIOA->IDR & (1 << BUTTON_PORT_A_PIN)) != 0) {
#endif
    	if(press_flag == 0) {
			press_start_time = get_microsecond_time();
			press_flag = 1;
    	}
    }
    else if(press_flag) {
    	press_flag = 0;
    	time_pressed_down = (uint32_t)(get_microsecond_time() - press_start_time);
    	if(time_pressed_down > 15000000) {
    		time_pressed_down = 15000000;
    	}
    	time_pressed_down = time_pressed_down / 1000;

    	if(time_pressed_down > 0) {
    		print_number("Button press time: ", time_pressed_down);
    	}

		if(time_pressed_down >= 15000) {
//            start_calibration(0);
        }
		else if(time_pressed_down >= 2000) {
//			start_go_to_closed_loop_mode();
		}
		else if(time_pressed_down >= 300) {
//            check_current_sensor_and_enable_mosfets();
//			add_trapezoid_move_to_queue(BUTTON_PRESS_MOTOR_MOVE_DISTANCE * 2, get_update_frequency() * 2);
		}
		else if(time_pressed_down >= 50) {
//        	check_current_sensor_and_enable_mosfets();
//			add_trapezoid_move_to_queue(-BUTTON_PRESS_MOTOR_MOVE_DISTANCE * 2, get_update_frequency() * 2);
		}
    }
}

void print_start_message(void)
{
//	char buff[200];
	uint32_t my_unique_id_u32_array[2];

	memcpy(my_unique_id_u32_array, &my_unique_id, sizeof(my_unique_id));

    print_debug_string("Applicaiton start\n");
}

int main(void)
{
    clock_init();
    systick_init();
    GPIO_init();
    debug_uart_init();
    rs485_init();
    adc_init();
    pwm_init();
    overvoltage_init();
    #ifndef PRODUCT_NAME_M3
    step_and_direction_init();
    #endif

    SCB->VTOR = 0x2800; // vector table is moved to where the application starts, which is after the bootloader

    my_unique_id = get_unique_id();

    load_global_settings(); // load the settings from non-volatile memory into ram for faster access
    if(global_settings.commutation_position_offset == 0xffffffff) {
        global_settings.commutation_position_offset = DEFAULT_COMMUTATION_POSITION_OFFSET;
    }

    microsecond_clock_init();

    __enable_irq();

    print_start_message();

//    rs485_transmit("Start\n", 6);

    while(1) {
        if(rs485_has_a_packet()) {
            if(detect_devices_delay >= 0) { // if a DETECT_DEVICES_COMMAND has been issued then we will ignore all other commands until the delay is over and we send out the unique ID
                rs485_done_with_this_packet();
            }
            else {
                process_packet();
            }
        }

        if(detect_devices_delay == 0) {
            print_debug_string("Transmitting unique ID\n");
            transmit_unique_id();
            detect_devices_delay--;
    	}

        process_debug_uart_commands();
        button_logic();
    }
}
