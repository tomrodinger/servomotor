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
#include "microsecond_clock.h"
#include "clock_calibration.h"
#include "error_handling.h"
#include "unique_id.h"
#include "settings.h"
#include "commands.h"
#include "product_info.h"
#include "global_variables.h"
#include "device_status.h"
#include "bed_heater_control.h"


char PRODUCT_DESCRIPTION[] = "Print bed heater";

struct __attribute__((__packed__)) firmware_version_struct {
    uint8_t bugfix;
    uint8_t minor;
    uint8_t major;
    uint8_t not_used;
};
struct firmware_version_struct firmware_version = {0, 8, 0, 0};

#define PING_PAYLOAD_SIZE 10

extern uint16_t ADC_buffer[DMA_ADC_BUFFER_SIZE];

static uint64_t my_unique_id;
static int16_t detect_devices_delay = -1;

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

void portA_init(void)
{
    #define BUTTON_PORT_A_PIN 13
    #define FAN1_CONTROL_PORT_A_PIN 7
    #define FAN2_CONTROL_PORT_A_PIN 8

    GPIOA->BSRR = (1 << HEATER_CONTROL_PORT_A_PIN) | (1 << FAN1_CONTROL_PORT_A_PIN) | (1 << FAN2_CONTROL_PORT_A_PIN);

    GPIOA->MODER =
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE0_Pos)  | // current measurement channel A
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE1_Pos)  | // 24V line voltage measurement ADC input
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE2_Pos)  | // serial port TX
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE3_Pos)  | // serial port RX
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE4_Pos)  |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE5_Pos)  |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE6_Pos)  |
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE7_Pos)  | // fan 1 digital control
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE8_Pos)  | // fan 2 digital control
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE9_Pos)  | // heater control digital output
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE10_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE11_Pos) |
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE12_Pos) | // RS485 drive enable
            (MODER_DIGITAL_INPUT      << GPIO_MODER_MODE13_Pos) | // Button input and also SWDIO (for programming)
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE14_Pos) | // SWCLK (for programming)
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE15_Pos);

    GPIOA->OTYPER = (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT0_Pos) | // make all the pins with analog components connected open drain
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT1_Pos) | // also, make the debug UART receive pin open drain
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT3_Pos) | // may not be necessary
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT4_Pos) |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT5_Pos) |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT6_Pos) |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT7_Pos) |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT8_Pos) |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT9_Pos) |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT10_Pos) |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT11_Pos) |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT13_Pos) |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT14_Pos) |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT15_Pos);
    GPIOA->OSPEEDR = 0xffffffff; // make all pins very high speed
    GPIOA->PUPDR = (PUPDR_PULL_UP << GPIO_PUPDR_PUPD3_Pos); // apply pull up on the UART receive pin
}


void portB_init(void)
{
    GPIOB->MODER =
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE0_Pos)  |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE1_Pos)  |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE2_Pos)  | // ambient temperature measurement ADC input
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE3_Pos)  |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE4_Pos)  |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE5_Pos)  |
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE6_Pos)  | // RS485 Data out
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE7_Pos)  | // RS485 Data receive
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE8_Pos)  |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE9_Pos)  |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE10_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE11_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE12_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE13_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE14_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE15_Pos);

    GPIOB->OTYPER = (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT7_Pos);  // RX pin make as open drain
    GPIOB->OSPEEDR = 0xffffffff; // make all pins very high speed
    GPIOB->PUPDR = (PUPDR_PULL_UP << GPIO_PUPDR_PUPD7_Pos); // RX pin is pull up
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


// This interrupt will be called 100 times per second
void SysTick_Handler(void)
{
    #define OVERVOLTAGE_PORT_B_PIN 12

    static uint16_t toggle_counter = 0;

    toggle_counter++;
    if(toggle_counter >= 50) {
        green_LED_toggle();
        toggle_counter = 0;
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

    uint64_t local_time;
    uint64_t time_from_master = 0;
    uint64_t unique_id;
    uint8_t new_alias;
    uint8_t ping_response_buffer[PING_PAYLOAD_SIZE + 3];

    // There is a distinct possibility that the new packet is not for us. We will know after we see the return value of rs485_get_next_packet.
    if (!rs485_get_next_packet(&command, &payload_size, &payload, &is_broadcast)) {
        // The new packet, whatever it is, is not of interest to us and we need to clear it and return out of here
        rs485_done_with_this_packet();
        return;
    }

    if(!validate_command_crc32()) {
        // CRC32 validation failed, allow next command and return
        rs485_done_with_this_packet();
        return;
    }

//    print_number("Received a command with length: ", commandLen);
    if((axis == global_settings.my_alias) || (axis == ALL_ALIAS)) {
//        print_number("Axis:", axis);
//        print_number("command:", command);
        switch(command) {
        case RESET_TIME_COMMAND:
            rs485_done_with_this_packet();
            reset_microsecond_time();
            if(!is_broadcast) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case GET_CURRENT_TIME_COMMAND:
            rs485_done_with_this_packet();
            if(!is_broadcast) {
                local_time = get_microsecond_time();
                rs485_transmit(RESPONSE_CHARACTER_TEXT "\x01\x06", 3);
                rs485_transmit(&local_time, 6);
            }
            break;
        case TIME_SYNC_COMMAND:
            memcpy(&time_from_master, payload, 6);
            rs485_done_with_this_packet();
            int32_t time_error = time_sync(time_from_master);
            uint16_t clock_calibration_value = get_clock_calibration_value();
            if(!is_broadcast) {
                rs485_transmit(RESPONSE_CHARACTER_TEXT "\x01\x06", 3);
                rs485_transmit(&time_error, 4);
                rs485_transmit(&clock_calibration_value, 2);
            }
            break;
        case GET_STATUS_COMMAND:
            rs485_done_with_this_packet();
            if(!is_broadcast) {
                uint8_t device_status_flags = get_device_status_flags();
                set_device_status_flags(device_status_flags);
                rs485_transmit(get_device_status(), sizeof(struct device_status_struct));
            }
            break;
        case DETECT_DEVICES_COMMAND:
            rs485_done_with_this_packet();
            detect_devices_delay = get_random_number(99);
            break;
        case SET_DEVICE_ALIAS_COMMAND:
            unique_id = ((int64_t*)payload)[0];
            new_alias = payload[8];
            rs485_done_with_this_packet();
            if(unique_id == my_unique_id) {
                transmit("Unique ID matches\n", 18);
                global_settings.my_alias = new_alias;
                save_global_settings();
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case GET_PRODUCT_INFO_COMMAND:
            rs485_done_with_this_packet();
            if(!is_broadcast) {
                rs485_transmit(RESPONSE_CHARACTER_TEXT "\x01", 2);
                uint8_t product_info_length = sizeof(struct product_info_struct);
                struct product_info_struct *product_info = (struct product_info_struct *)(PRODUCT_INFO_MEMORY_LOCATION);
                rs485_transmit(&product_info_length, 1);
                rs485_transmit(product_info, sizeof(struct product_info_struct));
            }
            break;
        case GET_PRODUCT_DESCRIPTION_COMMAND:
            rs485_done_with_this_packet();
            if(!is_broadcast) {
                rs485_transmit(RESPONSE_CHARACTER_TEXT "\x01", 2);
                uint8_t product_description_length = sizeof(PRODUCT_DESCRIPTION);
                rs485_transmit(&product_description_length, 1);
                rs485_transmit(&PRODUCT_DESCRIPTION, sizeof(PRODUCT_DESCRIPTION));
            }
            break;
        case GET_FIRMWARE_VERSION_COMMAND:
            rs485_done_with_this_packet();
            if(!is_broadcast) {
                rs485_transmit(RESPONSE_CHARACTER_TEXT "\x01", 2);
                uint8_t firmware_version_length = sizeof(firmware_version);
                rs485_transmit(&firmware_version_length, 1);
                rs485_transmit(&firmware_version, sizeof(firmware_version));
            }
            break;
        case SYSTEM_RESET_COMMAND:
            NVIC_SystemReset();
            break;
        case PING_COMMAND:
            memcpy(ping_response_buffer + 3, payload, PING_PAYLOAD_SIZE);
            rs485_done_with_this_packet();
            if(!is_broadcast) {
                ping_response_buffer[0] = RESPONSE_CHARACTER;
                ping_response_buffer[1] = '\x01';
                ping_response_buffer[2] = PING_PAYLOAD_SIZE;
                rs485_transmit(ping_response_buffer, PING_PAYLOAD_SIZE + 3);
            }
            break;
        }
    }
    else {
        rs485_done_with_this_packet();
    }
}

void transmit_unique_id(void)
{
    uint32_t crc32 = 0x04030201;
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
        case 'c':
            do_one_ADC_conversion_cycle();
            break;
        case 'p':
            print_ADC_values();
            print_bed_resistance();
            print_bed_temperature();
            break;
        case 'e':
            turn_on_or_off_bed_heater(1);
            #define ENABLE_BED_HEATER_MESSAGE "Enabling the bed heater. Press capital E to disable it.\n"
            transmit(ENABLE_BED_HEATER_MESSAGE, sizeof(ENABLE_BED_HEATER_MESSAGE) - 1);
            break;
        case 'E':
            turn_on_or_off_bed_heater(0);
            #define DISABLE_BED_HEATER_MESSAGE "Disabling the bed heater. Press small e to enable it.\n"
            transmit(DISABLE_BED_HEATER_MESSAGE, sizeof(DISABLE_BED_HEATER_MESSAGE) - 1);
            break;
        case 'S':
            transmit("Saving settings\n", 16);
            save_global_settings();
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

    if((GPIOA->IDR & (1 << BUTTON_PORT_A_PIN)) == 0) {
        if(press_flag == 0) {
            press_start_time = get_microsecond_time();
            press_flag = 1;
        }
    }
    else if(press_flag) {
        press_flag = 0;
        time_pressed_down = (uint32_t)(get_microsecond_time() - press_start_time);
        if(time_pressed_down > 5000000) {
            time_pressed_down = 5000000;
        }
        time_pressed_down = time_pressed_down / 1000;

        if(time_pressed_down > 0) {
            print_number("Button press time: ", time_pressed_down);
        }

        if(time_pressed_down >= 2000) {

        }
        else if(time_pressed_down >= 300) {

        }
        else if(time_pressed_down >= 50) {

        }
    }
}

void print_start_message(void)
{
    uint32_t my_unique_id_u32_array[2];

    memcpy(my_unique_id_u32_array, &my_unique_id, sizeof(my_unique_id));

    transmit("Applicaiton start\n", 18);
}


void process_ADC_values(void)
{
    uint32_t current;
    uint32_t voltage;
    uint32_t temperature;

    if(new_ADC_values_available()) {
        current = get_current_sense_value();
        voltage = get_24V_sense_value();
        temperature = get_temperature_sense_value();
        bed_heater_calculations(current, voltage, temperature);
    }
}

int main(void)
{
    clock_init();
    systick_init();
    portA_init();
    portB_init();
    portC_init();
    portD_init();
    debug_uart_init();
    rs485_init();
    microsecond_clock_init();
    adc_init();
//    pwm_init();

    SCB->VTOR = 0x2800; // vector table is moved to where the application starts, which is after the bootloader

    my_unique_id = get_unique_id();

    load_global_settings(); // load the settings from non-volatile memory into ram for faster access

    __enable_irq();

    print_start_message();

//    rs485_transmit("Start\n", 6);

    while(1) {
//      check_if_break_condition();
//      check_if_ADC_watchdog2_exceeded();

        if(rs485_has_a_packet()) {
            process_packet();
        }

        if(detect_devices_delay == 0) {
            transmit("Transmitting unique ID\n", 23);
            transmit_unique_id();
            detect_devices_delay--;
        }

        process_debug_uart_commands();
        button_logic();

        process_ADC_values();
    }
}
