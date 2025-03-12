#include "stm32g0xx_hal.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "leds.h"
#include "debug_uart.h"
#include "RS485.h"
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
#include "coin_cell_control_and_calculations.h"


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
extern char selectedAxis;
extern uint8_t command;
extern uint8_t valueBuffer[MAX_VALUE_BUFFER_LENGTH];
extern volatile uint8_t commandReceived;

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

    GPIOA->ODR |= GPIO_ALL_LOADS_OFF; // set all the pins that control the loads high (to turn off the loads)

    GPIOA->MODER =
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE0_Pos)  |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE1_Pos)  | // Coin cell voltage of positive terminal
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE2_Pos)  | // serial port TX
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE3_Pos)  | // serial port RX
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE4_Pos)  | // Load control channel 1
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE5_Pos)  | // Load control channel 2
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE6_Pos)  | // Load control channel 3
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE7_Pos)  | // Load control channel 4
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE8_Pos)  | // Load control channel 5
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE9_Pos)  | // Load control channel 6
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE10_Pos) | // Load control channel 7
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE11_Pos) | // Load control channel 8
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE12_Pos) | // RS485 drive enable
            (MODER_DIGITAL_INPUT      << GPIO_MODER_MODE13_Pos) | // Button input and also SWDIO (for programming)
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE14_Pos) | // SWCLK (for programming)
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE15_Pos);

    GPIOA->OTYPER = (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT0_Pos) | // make all the pins with analog components connected open drain
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT1_Pos) | // also, make the debug UART receive pin open drain
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT3_Pos) | // for controlling the loads, the pin is connected to the base of the transistor
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT4_Pos) | // To turn on a load, we need to pull the line low. To turn off the load, we need
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT5_Pos) | // to let the line float high. So, we need to use open drain.
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
    GPIOB->PUPDR = (PUPDR_PULL_UP << GPIO_PUPDR_PUPD7_Pos); // RS485 receive pin is pull up
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


void processCommand(uint8_t axis, uint8_t command, uint8_t *parameters)
{
    uint64_t local_time;
    uint64_t time_from_master = 0;
    uint64_t unique_id;
    uint8_t new_alias;
    uint8_t ping_response_buffer[PING_PAYLOAD_SIZE + 3];

    #define MAX_MULTI_MOVES (sizeof(uint32_t) * 8) // maximum number of moves in a multi move command. this number should be the number of bits in an uint32_t
    struct move_parameters_struct {
        union { // if the bitfield shows a 0 for this move then this parameter represents the acceleration, otherwise it represents the velocity
            int32_t acceleration;
            int32_t velocity;
        };
        int32_t time_steps;
    };
    struct multi_move_command_buffer_struct {
        uint32_t move_type_bits; // a bit field specifying the type of each move: 0 = move with acceleration; 1 = move with velocuty
        struct move_parameters_struct move_parameters[MAX_MULTI_MOVES];
    };

//    print_number("Received a command with length: ", commandLen);
    if((axis == global_settings.my_alias) || (axis == ALL_ALIAS)) {
//        print_number("Axis:", axis);
//        print_number("command:", command);
        switch(command) {
        case ADD_NEW_INTERVAL_COMMAND:
        {
            uint16_t interval_add_index = ((int16_t*)parameters)[0];
            uint8_t load_state = parameters[2];
            uint32_t duration;
            memcpy(&duration, parameters + 3, sizeof(uint32_t));
            rs485_allow_next_command();
            add_new_interval(interval_add_index, load_state, duration);
            if(axis != ALL_ALIAS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        }
        case SET_INTERVAL_RUN_MODE_COMMAND:
        {
            interval_run_mode_t run_mode = (interval_run_mode_t)parameters[0];
            rs485_allow_next_command();
            if(run_mode >= MANUAL_PULSE) {
                fatal_error(ERROR_INVALID_RUN_MODE); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
            }
            set_interval_run_mode(run_mode);
            if(axis != ALL_ALIAS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        }
        case RESET_TIME_COMMAND:
            rs485_allow_next_command();
            reset_microsecond_time();
            if(axis != ALL_ALIAS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case GET_CURRENT_TIME_COMMAND:
            rs485_allow_next_command();
            if(axis != ALL_ALIAS) {
                local_time = get_microsecond_time();
                rs485_transmit(ENCODED_RESPONSE_CHARACTER_TEXT "\x01\x06", 3);
                rs485_transmit(&local_time, 6);
            }
            break;
        case TIME_SYNC_COMMAND:
            memcpy(&time_from_master, parameters, 6);
            rs485_allow_next_command();
            transmit("Time sync\n", 10);
            int32_t time_error = time_sync(time_from_master);
            uint16_t clock_calibration_value = get_clock_calibration_value();
            if(axis != ALL_ALIAS) {
                rs485_transmit(ENCODED_RESPONSE_CHARACTER_TEXT "\x01\x06", 3);
                rs485_transmit(&time_error, 4);
                rs485_transmit(&clock_calibration_value, 2);
            }
            break;
        case GET_STATUS_COMMAND:
            rs485_allow_next_command();
            if(axis != ALL_ALIAS) {
                uint8_t device_status_flags = get_device_status_flags();
                set_device_status_flags(device_status_flags);
                rs485_transmit(get_device_status(), sizeof(struct device_status_struct));
            }
            break;
        case DETECT_DEVICES_COMMAND:
            rs485_allow_next_command();
            detect_devices_delay = get_random_number(99);
            break;
        case SET_DEVICE_ALIAS_COMMAND:
            unique_id = ((int64_t*)parameters)[0];
            new_alias = parameters[8];
            rs485_allow_next_command();
            if(unique_id == my_unique_id) {
                transmit("Unique ID matches\n", 18);
                global_settings.my_alias = new_alias;
                save_global_settings();
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case GET_PRODUCT_INFO_COMMAND:
            rs485_allow_next_command();
            if(axis != ALL_ALIAS) {
                rs485_transmit(ENCODED_RESPONSE_CHARACTER_TEXT "\x01", 2);
                uint8_t product_info_length = sizeof(struct product_info_struct);
                struct product_info_struct *product_info = (struct product_info_struct *)(PRODUCT_INFO_MEMORY_LOCATION);
                rs485_transmit(&product_info_length, 1);
                rs485_transmit(product_info, sizeof(struct product_info_struct));
            }
            break;
        case GET_PRODUCT_DESCRIPTION_COMMAND:
            rs485_allow_next_command();
            if(axis != ALL_ALIAS) {
                rs485_transmit(ENCODED_RESPONSE_CHARACTER_TEXT "\x01", 2);
                uint8_t product_description_length = sizeof(PRODUCT_DESCRIPTION);
                rs485_transmit(&product_description_length, 1);
                rs485_transmit(&PRODUCT_DESCRIPTION, sizeof(PRODUCT_DESCRIPTION));
            }
            break;
        case GET_FIRMWARE_VERSION_COMMAND:
            rs485_allow_next_command();
            if(axis != ALL_ALIAS) {
                rs485_transmit(ENCODED_RESPONSE_CHARACTER_TEXT "\x01", 2);
                uint8_t firmware_version_length = sizeof(firmware_version);
                rs485_transmit(&firmware_version_length, 1);
                rs485_transmit(&firmware_version, sizeof(firmware_version));
            }
            break;
        case SYSTEM_RESET_COMMAND:
            NVIC_SystemReset();
            break;
        case PING_COMMAND:
            memcpy(ping_response_buffer + 3, parameters, PING_PAYLOAD_SIZE);
            rs485_allow_next_command();
            transmit("Ping\n", 5);
            if(axis != ALL_ALIAS) {
                ping_response_buffer[0] = ENCODED_RESPONSE_CHARACTER;
                ping_response_buffer[1] = '\x01';
                ping_response_buffer[2] = PING_PAYLOAD_SIZE;
                rs485_transmit(ping_response_buffer, PING_PAYLOAD_SIZE + 3);
            }
            break;
        case GET_DATA_RECORD_COMMAND:
        {
            uint8_t get_data_record_response_buffer[3 + sizeof(data_record_t)];
            rs485_allow_next_command();
            transmit("Get data record\n", 16);
            if(axis != ALL_ALIAS) {
                get_data_record_response_buffer[0] = ENCODED_RESPONSE_CHARACTER;
                get_data_record_response_buffer[1] = '\x01';
                get_data_record_response_buffer[2] = sizeof(data_record_t);
                get_data_record((data_record_t *)(get_data_record_response_buffer + 3));
                rs485_transmit(get_data_record_response_buffer, 3 + sizeof(data_record_t));
            }
            break;
        }
        case GET_PICOAMP_SECONDS_COMMAND:
        {
            uint64_t picoamp_seconds_count;
            uint8_t get_data_record_response_buffer[3 + sizeof(uint64_t)];
            rs485_allow_next_command();
            transmit("Get picoamp seconds\n", 20);
            if(axis != ALL_ALIAS) {
                get_data_record_response_buffer[0] = ENCODED_RESPONSE_CHARACTER;
                get_data_record_response_buffer[1] = '\x01';
                get_data_record_response_buffer[2] = sizeof(uint64_t);
                picoamp_seconds_count = get_picoamp_seconds_count();
                memcpy(get_data_record_response_buffer + 3, &picoamp_seconds_count, sizeof(uint64_t));
                rs485_transmit(get_data_record_response_buffer, 3 + sizeof(uint64_t));
            }
            break;
        }
        }
    }
    else {
        rs485_allow_next_command();
    }
}

void transmit_unique_id(void)
{
    uint32_t crc32 = 0x04030201;
    rs485_transmit(ENCODED_RESPONSE_CHARACTER_TEXT "\x01\x0d", 3);
    rs485_transmit(&my_unique_id, 8);
    rs485_transmit(&global_settings.my_alias, 1);
    rs485_transmit(&crc32, 4);
}


static char toggle_load_message[] = "Toggling load X on or off\n";

void process_debug_uart_commands(void)
{
    uint8_t command_debug_uart = get_command_debug_uart();

    if(command_debug_uart != 0) {
        switch(command_debug_uart) {
        case 'p':
            print_ADC_values();
            print_pulse_stats();
            print_picoamp_seconds_count();
            print_intervals();
            print_time_difference();
            break;
        case 'u':
            apply_pulse(500, 2);
            transmit("Applying pulse\n", 15);
            break;
        case 'g':
            get_data_record_and_print_it();
            break;
        case '1':
            load_toggle_on_or_off(command_debug_uart - '1');
            toggle_load_message[14] = command_debug_uart;
            transmit(toggle_load_message, sizeof(toggle_load_message) - 1);
            break;
        case '2':
            load_toggle_on_or_off(command_debug_uart - '1');
            toggle_load_message[14] = command_debug_uart;
            transmit(toggle_load_message, sizeof(toggle_load_message) - 1);
            break;
        case '3':
            load_toggle_on_or_off(command_debug_uart - '1');
            toggle_load_message[14] = command_debug_uart;
            transmit(toggle_load_message, sizeof(toggle_load_message) - 1);
            break;
        case '4':
            load_toggle_on_or_off(command_debug_uart - '1');
            toggle_load_message[14] = command_debug_uart;
            transmit(toggle_load_message, sizeof(toggle_load_message) - 1);
            break;
        case '5':
            load_toggle_on_or_off(command_debug_uart - '1');
            toggle_load_message[14] = command_debug_uart;
            transmit(toggle_load_message, sizeof(toggle_load_message) - 1);
            break;
        case '6':
            load_toggle_on_or_off(command_debug_uart - '1');
            toggle_load_message[14] = command_debug_uart;
            transmit(toggle_load_message, sizeof(toggle_load_message) - 1);
            break;
        case '7':
            load_toggle_on_or_off(command_debug_uart - '1');
            toggle_load_message[14] = command_debug_uart;
            transmit(toggle_load_message, sizeof(toggle_load_message) - 1);
            break;
        case '8':
            load_toggle_on_or_off(command_debug_uart - '1');
            toggle_load_message[14] = command_debug_uart;
            transmit(toggle_load_message, sizeof(toggle_load_message) - 1);
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
    adc_init(get_ADC_buffer_ptr());
    coin_cell_control_and_calculations_init();

    SCB->VTOR = 0x2800; // vector table is moved to where the application starts, which is after the bootloader

    my_unique_id = get_unique_id();

    load_global_settings(); // load the settings from non-volatile memory into ram for faster access

    __enable_irq();

    print_start_message();

//    rs485_transmit("Start\n", 6);

    while(1) {
//      check_if_break_condition();
//      check_if_ADC_watchdog2_exceeded();

        if(commandReceived) {
            processCommand(selectedAxis, command, valueBuffer);
        }

        if(detect_devices_delay == 0) {
            transmit("Transmitting unique ID\n", 23);
            transmit_unique_id();
            detect_devices_delay--;
        }

        process_debug_uart_commands();
        button_logic();

        print_millivolts_if_changed();
    }
}
