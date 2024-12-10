#include "stm32g0xx_hal.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "gpio.h"
#include "leds.h"
#include "debug_uart.h"
#include "RS485.h"
#include "mosfets.h"
#include "ADC.h"
#include "temperature.h"
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
#ifdef PRODUCT_NAME_M4
#include "commutation_table_M4.h"
#endif


char PRODUCT_DESCRIPTION[] = "Servomotor";

struct __attribute__((__packed__)) firmware_version_struct {
    uint8_t bugfix;
    uint8_t minor;
    uint8_t major;
    uint8_t not_used;
};
#define MAJOR_FIRMWARE_VERSION 0
#define MINOR_FIRMWARE_VERSION 9
#define BUGFIX_FIRMWARE_VERSION 0
#define DEVELOPMENT_FIRMWARE_VERSION 1 // this is the least significant number when it comes to versioning and is the last number on the right when printed in human readable form
struct firmware_version_struct firmware_version = {DEVELOPMENT_FIRMWARE_VERSION, BUGFIX_FIRMWARE_VERSION, MINOR_FIRMWARE_VERSION, MAJOR_FIRMWARE_VERSION};

#define BUTTON_PRESS_MOTOR_MOVE_DISTANCE ONE_REVOLUTION_MICROSTEPS

#define PING_PAYLOAD_SIZE 10

extern uint16_t ADC_buffer[DMA_ADC_BUFFER_SIZE];
extern char selectedAxis;
extern uint8_t command;
extern uint8_t valueBuffer[MAX_VALUE_BUFFER_LENGTH];
extern volatile uint8_t commandReceived;

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
    SysTick->VAL   = 1600000 - 1;                                             // Load the SysTick Counter Value
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                     SysTick_CTRL_TICKINT_Msk   |
                     SysTick_CTRL_ENABLE_Msk;                         // Enable SysTick IRQ and SysTick Timer
*/
}


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


void set_led_test_mode(uint8_t colours)
{
    __disable_irq();
    disable_mosfets();
    if (colours & 1) {
        green_LED_on();
    }
    if (colours & 2) {
        red_LED_on();
    }
    while(1);
}

void processCommand(uint8_t axis, uint8_t command, uint8_t *parameters)
{
    uint64_t local_time;
    uint64_t time_from_master = 0;
    uint8_t capture_type;
    uint8_t n_items_in_queue;
    int32_t acceleration;
    int32_t velocity;
    uint32_t time_steps;
    uint32_t frequency;
    uint64_t unique_id;
    uint8_t new_alias;
    uint16_t new_maximum_motor_current;
    uint16_t new_maximum_motor_regen_current;
    uint8_t n_moves_in_this_command;
    int32_t max_homing_travel_displacement;
    uint32_t max_homing_time;
    uint8_t ping_response_buffer[PING_PAYLOAD_SIZE + 3];
    uint8_t control_hall_sensor_statistics_subcommand;

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
        case DISABLE_MOSFETS_COMMAND:
            rs485_allow_next_command();
            disable_mosfets();
            if(axis != ALL_ALIAS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case ENABLE_MOSFETS_COMMAND:
            rs485_allow_next_command();
            check_current_sensor_and_enable_mosfets();
            if(axis != ALL_ALIAS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case TRAPEZOID_MOVE_COMMAND:
            {
                int32_t trapezoid_move_displacement = ((int32_t*)parameters)[0];
                uint32_t trapezoid_move_time = ((uint32_t*)parameters)[1];
                rs485_allow_next_command();
                add_trapezoid_move_to_queue(trapezoid_move_displacement, trapezoid_move_time);
            }
            if(axis != ALL_ALIAS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case SET_MAX_VELOCITY_COMMAND:
            {
                uint32_t max_velocity = *(uint32_t*)parameters;
                rs485_allow_next_command();
                set_max_velocity(max_velocity);
            }
            if(axis != ALL_ALIAS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case GO_TO_POSITION_COMMAND:
            {
                int32_t end_position = ((int32_t*)parameters)[0];
                uint32_t move_time = ((uint32_t*)parameters)[1];
                rs485_allow_next_command();
                add_go_to_position_to_queue(end_position, move_time);
            }
            if(axis != ALL_ALIAS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case SET_MAX_ACCELERATION_COMMAND:
            {
                uint32_t max_acceleration = *(uint32_t*)parameters;
                rs485_allow_next_command();
                set_max_acceleration(max_acceleration);
            }
            if(axis != ALL_ALIAS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case START_CALIBRATION_COMMAND:
            rs485_allow_next_command();
            start_calibration(0);
            break;
        case CAPTURE_HALL_SENSOR_DATA_COMMAND:
            capture_type = parameters[0];
            rs485_allow_next_command();
            if(axis != ALL_ALIAS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            if(capture_type == CAPTURE_HALL_SENSOR_READINGS_WHILE_TURNING) {
                start_calibration(1);
            }
            else {
                start_capture(capture_type);
            }
            break;
        case RESET_TIME_COMMAND:
            rs485_allow_next_command();
            reset_time();
            if(axis != ALL_ALIAS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case GET_CURRENT_TIME_COMMAND:
            rs485_allow_next_command();
            if(axis != ALL_ALIAS) {
                local_time = get_microsecond_time();
                rs485_transmit(RESPONSE_CHARACTER_TEXT "\x01\x06", 3);
                rs485_transmit(&local_time, 6);
            }
            break;
        case TIME_SYNC_COMMAND:
            memcpy(&time_from_master, parameters, 6);
            rs485_allow_next_command();
            int32_t time_error = time_sync(time_from_master);
            uint16_t clock_calibration_value = get_clock_calibration_value();
            if(axis != ALL_ALIAS) {
                rs485_transmit(RESPONSE_CHARACTER_TEXT "\x01\x06", 3);
                rs485_transmit(&time_error, 4);
                rs485_transmit(&clock_calibration_value, 2);
            }
            break;
        case GET_N_ITEMS_IN_QUEUE_COMMAND:
            rs485_allow_next_command();
            n_items_in_queue = get_n_items_in_queue();
            if(axis != ALL_ALIAS) {
                rs485_transmit(RESPONSE_CHARACTER_TEXT "\x01\x01", 3);
                rs485_transmit(&n_items_in_queue, 1);
            }
            break;
        case EMERGENCY_STOP_COMMAND:
            rs485_allow_next_command();
            emergency_stop();
            if(axis != ALL_ALIAS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case ZERO_POSITION_COMMAND:
            rs485_allow_next_command();
            zero_position();
            if(axis != ALL_ALIAS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case HOMING_COMMAND:
            max_homing_travel_displacement = ((int32_t*)parameters)[0];
            max_homing_time = ((uint32_t*)parameters)[1];
            rs485_allow_next_command();
            char buf[100];
            sprintf(buf, "homing: max_homing_travel_displacement: %ld   max_homing_time: %lu\n", max_homing_travel_displacement, max_homing_time);
            print_debug_string(buf);
            start_homing(max_homing_travel_displacement, max_homing_time);
            if(axis != ALL_ALIAS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case GET_POSITION_COMMAND:
            rs485_allow_next_command();
            {
//                int32_t motor_position;
                int64_t motor_position;
                if(axis != ALL_ALIAS) {
                    motor_position = get_motor_position();
//                    rs485_transmit(RESPONSE_CHARACTER_TEXT "\x01\x04", 3);
                    rs485_transmit(RESPONSE_CHARACTER_TEXT "\x01\x08", 3);
                    rs485_transmit(&motor_position, sizeof(motor_position));
                }
            }
            break;
        case GET_HALL_SENSOR_POSITION_COMMAND:
            rs485_allow_next_command();
            {
//                int32_t hall_sensor_position;
                int64_t hall_sensor_position;
                if(axis != ALL_ALIAS) {
                    hall_sensor_position = get_hall_position();
//                    rs485_transmit(RESPONSE_CHARACTER_TEXT "\x01\x04", 3);
                    rs485_transmit(RESPONSE_CHARACTER_TEXT "\x01\x08", 3);
                    rs485_transmit(&hall_sensor_position, sizeof(hall_sensor_position));
                }
            }
            break;
        case GET_COMPREHENSIVE_POSITION_COMMAND:
            rs485_allow_next_command();
            {
                struct __attribute__((__packed__)) {
                    uint8_t header[3];
//                    int32_t motor_position;
                    int64_t motor_position;
//                    int32_t hall_sensor_position;
                    int64_t hall_sensor_position;
                    int32_t external_encoder_position;
                } comprehensive_position;
                if(axis != ALL_ALIAS) {
                    comprehensive_position.header[0] = RESPONSE_CHARACTER;
                    comprehensive_position.header[1] = 1;
                    comprehensive_position.header[2] = sizeof(comprehensive_position) - 3;
                    comprehensive_position.motor_position = get_motor_position();
                    comprehensive_position.hall_sensor_position = get_hall_position();
                    comprehensive_position.external_encoder_position = get_external_encoder_position();
                    rs485_transmit(&comprehensive_position, sizeof(comprehensive_position));
                }
            }
            break;
        case GET_STATUS_COMMAND:
            rs485_allow_next_command();
            if(axis != ALL_ALIAS) {
                uint8_t motor_status_flags = get_motor_status_flags();
                set_device_status_flags(motor_status_flags);
                rs485_transmit(get_device_status(), sizeof(struct device_status_struct));
                sprintf(buf, "Get status: %hu\n", motor_status_flags);
                print_debug_string(buf);
            }
            break;
        case GO_TO_CLOSED_LOOP_COMMAND:
            rs485_allow_next_command();
            start_go_to_closed_loop_mode();
            if(axis != ALL_ALIAS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case GET_UPDATE_FREQUENCY_COMMAND:
            rs485_allow_next_command();
            frequency = get_update_frequency();
            if(axis != ALL_ALIAS) {
                rs485_transmit(RESPONSE_CHARACTER_TEXT "\x01\x04", 3);
                rs485_transmit(&frequency, 4);
            }
            break;
        case MOVE_WITH_ACCELERATION_COMMAND:
            acceleration = ((int32_t*)parameters)[0];
            time_steps = ((uint32_t*)parameters)[1];
            rs485_allow_next_command();
//            sprintf(buf, "move_with_acceleration: %ld %lu\n", acceleration, time_steps);
//            print_debug_string(buf);
            add_to_queue(acceleration, time_steps, MOVE_WITH_ACCELERATION);
            if(axis != ALL_ALIAS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
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
                rs485_transmit(NO_ERROR_RESPONSE, 3); 
                print_number("Unique ID matches. Will save the alias and reset. New alias:", (uint16_t)new_alias);
                microsecond_delay(5000); // 5ms should be enough time to transmit the above debug message, which is about 100 bytes, at baud rate of 230400
                global_settings.my_alias = new_alias;
                save_global_settings(); // this will never return because the device will reset after writing the new settings to flash
            }
            break;
        case GET_PRODUCT_INFO_COMMAND:
            rs485_allow_next_command();
            if(axis != ALL_ALIAS) {
                rs485_transmit(RESPONSE_CHARACTER_TEXT "\x01", 2);
                uint8_t product_info_length = sizeof(struct product_info_struct);
                struct product_info_struct *product_info = (struct product_info_struct *)(PRODUCT_INFO_MEMORY_LOCATION);
                rs485_transmit(&product_info_length, 1);
                rs485_transmit(product_info, sizeof(struct product_info_struct));
            }
            break;
        case GET_PRODUCT_DESCRIPTION_COMMAND:
            rs485_allow_next_command();
            if(axis != ALL_ALIAS) {
                rs485_transmit(RESPONSE_CHARACTER_TEXT "\x01", 2);
                uint8_t product_description_length = sizeof(PRODUCT_DESCRIPTION);
                rs485_transmit(&product_description_length, 1);
                rs485_transmit(&PRODUCT_DESCRIPTION, sizeof(PRODUCT_DESCRIPTION));
            }
            break;
        case GET_FIRMWARE_VERSION_COMMAND:
            rs485_allow_next_command();
            if(axis != ALL_ALIAS) {
                rs485_transmit(RESPONSE_CHARACTER_TEXT "\x01", 2);
                uint8_t firmware_version_length = sizeof(firmware_version);
                rs485_transmit(&firmware_version_length, 1);
                rs485_transmit(&firmware_version, sizeof(firmware_version));
            }
            break;
        case MOVE_WITH_VELOCITY_COMMAND:
            velocity = ((int32_t*)parameters)[0];
            time_steps = ((uint32_t*)parameters)[1];
            rs485_allow_next_command();
//            sprintf(buf, "move_with_velocity: %ld %lu\n", velocity, time_steps);
//            print_debug_string(buf);
            add_to_queue(velocity, time_steps, MOVE_WITH_VELOCITY);
            if(axis != ALL_ALIAS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case SYSTEM_RESET_COMMAND:
            NVIC_SystemReset();
            break;
        case SET_MAXIMUM_MOTOR_CURRENT:
            new_maximum_motor_current = ((uint16_t*)parameters)[0];
            new_maximum_motor_regen_current = ((uint16_t*)parameters)[1];
            rs485_allow_next_command();
            set_max_motor_current(new_maximum_motor_current, new_maximum_motor_regen_current);
            if(axis != ALL_ALIAS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case MULTI_MOVE_COMMAND:
            n_moves_in_this_command = ((int8_t*)parameters)[0];
            struct multi_move_command_buffer_struct multi_move_command_buffer;
            memcpy(&multi_move_command_buffer, parameters + 1, sizeof(int32_t) + n_moves_in_this_command * (sizeof(int32_t) + sizeof(int32_t)));
            rs485_allow_next_command();
//            char buf[100];
//            sprintf(buf, "multimove: %hu   %lu\n", n_moves_in_this_command, multi_move_command_buffer.move_type_bits);
//            print_debug_string(buf);
            if(n_moves_in_this_command > MAX_MULTI_MOVES) {
                fatal_error(ERROR_MULTI_MOVE_MORE_THAN_32_MOVES); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
            }
            for(uint8_t i = 0; i < n_moves_in_this_command; i++) {
                if((multi_move_command_buffer.move_type_bits & 1) == 0) {
                    add_to_queue(multi_move_command_buffer.move_parameters[i].acceleration, multi_move_command_buffer.move_parameters[i].time_steps, MOVE_WITH_ACCELERATION);
//                    sprintf(buf, "added_to_queue: acceleration: %ld  time_steps: %lu\n", multi_move_command_buffer.move_parameters[i].acceleration, multi_move_command_buffer.move_parameters[i].time_steps);
//                    print_debug_string(buf);
                } else {
                    add_to_queue(multi_move_command_buffer.move_parameters[i].velocity, multi_move_command_buffer.move_parameters[i].time_steps, MOVE_WITH_VELOCITY);
//                    sprintf(buf, "added_to_queue: velocity: %ld  time_steps: %lu\n", multi_move_command_buffer.move_parameters[i].velocity, multi_move_command_buffer.move_parameters[i].time_steps);
//                    print_debug_string(buf);
                }
                multi_move_command_buffer.move_type_bits >>= 1;
            }
            if(axis != ALL_ALIAS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case SET_SAFETY_LIMITS_COMMAND:
            {
        //            lower_limit = ((int32_t*)parameters)[0];
        //            upper_limit = ((int32_t*)parameters)[1];
                int64_t lower_limit = ((int64_t*)parameters)[0];
                int64_t upper_limit = ((int64_t*)parameters)[1];
                rs485_allow_next_command();
        //            sprintf(buf, "movement limits %ld to %ld\n", lower_limit, upper_limit);
        //            print_debug_string(buf);
                set_movement_limits(lower_limit, upper_limit);
                print_int64("lower_limit:", lower_limit);
                print_int64("upper_limit:", upper_limit);
                if(axis != ALL_ALIAS) {
                    rs485_transmit(NO_ERROR_RESPONSE, 3);
                }
            }
            break;
        case ADD_TO_QUEUE_TEST_COMMAND:
            {
                struct __attribute__((__packed__)) {
                    uint8_t header[3];
                    add_to_queue_test_results_t add_to_queue_test_results;
                } add_to_queue_test_reply;
                add_to_queue_test(((int32_t*)parameters)[0], ((uint32_t*)parameters)[1], ((uint8_t*)parameters)[8], &add_to_queue_test_reply.add_to_queue_test_results);
                rs485_allow_next_command();
                if(axis != ALL_ALIAS) {
                    add_to_queue_test_reply.header[0] = RESPONSE_CHARACTER;
                    add_to_queue_test_reply.header[1] = 1;
                    add_to_queue_test_reply.header[2] = sizeof(add_to_queue_test_reply) - sizeof(add_to_queue_test_reply.header);
                    rs485_transmit(&add_to_queue_test_reply, sizeof(add_to_queue_test_reply));
                }
            }
            break;
        case PING_COMMAND:
            memcpy(ping_response_buffer + 3, parameters, PING_PAYLOAD_SIZE);
            rs485_allow_next_command();
            if(axis != ALL_ALIAS) {
                ping_response_buffer[0] = RESPONSE_CHARACTER;
                ping_response_buffer[1] = '\x01';
                ping_response_buffer[2] = PING_PAYLOAD_SIZE;
                rs485_transmit(ping_response_buffer, PING_PAYLOAD_SIZE + 3);
            }
            break;
        case CONTROL_HALL_SENSOR_STATISTICS_COMMAND:
            control_hall_sensor_statistics_subcommand = ((int8_t*)parameters)[0];
            rs485_allow_next_command();
            if(axis != ALL_ALIAS) {
                if(control_hall_sensor_statistics_subcommand == 0) {
                    hall_sensor_turn_off_statistics();
                }
                if(control_hall_sensor_statistics_subcommand == 1) {
                    hall_sensor_turn_on_and_reset_statistics();
                }
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case GET_HALL_SENSOR_STATISTICS_COMMAND:
            rs485_allow_next_command();
            if(axis != ALL_ALIAS) {
                struct __attribute__((__packed__)) {
                    uint8_t axis;
                    uint8_t command;
                    uint8_t size;
                    hall_sensor_statistics_t hall_sensor_statistics;
                } hall_sensor_statistics_full_response;
                get_hall_sensor_statistics(&hall_sensor_statistics_full_response.hall_sensor_statistics);
                hall_sensor_statistics_full_response.axis = RESPONSE_CHARACTER;
                hall_sensor_statistics_full_response.command = '\x01';
                hall_sensor_statistics_full_response.size = sizeof(hall_sensor_statistics_t);
                rs485_transmit(&hall_sensor_statistics_full_response, sizeof(hall_sensor_statistics_full_response));
            }
            break;
        case READ_MULTIPURPOSE_BUFFER_COMMAND:
            rs485_allow_next_command();
            if(axis != ALL_ALIAS) {
                uint8_t data_type;
                uint16_t data_size;
                uint16_t data_size_left_to_send;
                uint8_t *multipurpose_data_ptr;
                get_multipurpose_data(&data_type, &data_size, &multipurpose_data_ptr);
                if (data_type != 0) {
                    struct __attribute__((__packed__)) {
                        uint8_t axis;
                        uint8_t command;
                        uint8_t size;
                        uint16_t extended_size;
                        uint8_t data_type;
                    } multipurpose_data_response_header;
                    multipurpose_data_response_header.axis = RESPONSE_CHARACTER;
                    multipurpose_data_response_header.command = '\x01';
                    multipurpose_data_response_header.size = 255; // when size is 255 then we get the actual size from the next 16 bit number (thus allowing sizes up to 65535)
                    multipurpose_data_response_header.extended_size = sizeof(data_type) + data_size;
                    multipurpose_data_response_header.data_type = data_type;
                    rs485_transmit(&multipurpose_data_response_header, sizeof(multipurpose_data_response_header));
                    data_size_left_to_send = data_size;
                    while(data_size_left_to_send > 0) {
                        uint16_t data_size_this_packet = data_size_left_to_send;
                        if(data_size_this_packet > TRANSMIT_BUFFER_SIZE) {
                            data_size_this_packet = TRANSMIT_BUFFER_SIZE;
                        }
                        rs485_transmit(multipurpose_data_ptr, data_size_this_packet);
                        multipurpose_data_ptr += data_size_this_packet;
                        data_size_left_to_send -= data_size_this_packet;
                    }
                    rs485_wait_for_transmit_done();
                    clear_multipurpose_data();
                }
                print_debug_string("Read multipurpose data\n");
                sprintf(buf, "Type is %hu, Size is %hu\n", data_type, data_size);
                print_debug_string(buf);
            }
            break;
        case GET_SUPPLY_VOLTAGE_COMMAND:
            rs485_allow_next_command();
            {
                struct __attribute__((__packed__)) {
                    uint8_t header[3];
                    uint16_t supply_voltage;
                } supply_voltage_reply;
                if(axis != ALL_ALIAS) {
                    supply_voltage_reply.header[0] = RESPONSE_CHARACTER;
                    supply_voltage_reply.header[1] = 1;
                    supply_voltage_reply.header[2] = sizeof(supply_voltage_reply) - 3;
                    supply_voltage_reply.supply_voltage = get_supply_voltage_volts_times_10();
                    rs485_transmit(&supply_voltage_reply, sizeof(supply_voltage_reply));
                }
            }
            break;
        case GET_MAX_PID_ERROR_COMMAND:
            rs485_allow_next_command();
            {
                struct __attribute__((__packed__)) {
                    uint8_t header[3];
                    int32_t min_PID_error;
                    int32_t max_PID_error;
                } get_max_pid_error_reply;
                if(axis != ALL_ALIAS) {
                    get_max_pid_error_reply.header[0] = RESPONSE_CHARACTER;
                    get_max_pid_error_reply.header[1] = 1;
                    get_max_pid_error_reply.header[2] = sizeof(get_max_pid_error_reply) - 3;
                    int32_t min_PID_error;
                    int32_t max_PID_error;
                    get_max_PID_error(&min_PID_error, &max_PID_error);
                    get_max_pid_error_reply.min_PID_error = min_PID_error;
                    get_max_pid_error_reply.max_PID_error = max_PID_error;
                    rs485_transmit(&get_max_pid_error_reply, sizeof(get_max_pid_error_reply));
                }
            }
            break;
        case TEST_MODE_COMMAND:
            {
                uint8_t test_mode = parameters[0];
                rs485_allow_next_command();
                if (test_mode == 0) {
                    set_motor_test_mode(0);
                    set_led_test_mode(0);
                }
                else if (test_mode < 10) {
                    set_motor_test_mode(test_mode);
                }
                else {
                    set_led_test_mode(test_mode - 10);
                }
                if(axis != ALL_ALIAS) {
                    rs485_transmit(NO_ERROR_RESPONSE, 3);
                }
                sprintf(buf, "Setting the test mode to %hu\n", test_mode);
                print_debug_string(buf);
            }
            break;
        case VIBRATE_COMMAND:
            {
                uint8_t vibration_level = parameters[0];
                rs485_allow_next_command();
                vibrate(vibration_level);
                if(axis != ALL_ALIAS) {
                    rs485_transmit(NO_ERROR_RESPONSE, 3);
                }
                if(vibration_level == 0) {
                    print_debug_string("Turning off vibration\n");
                }
                else {
                    sprintf(buf, "Turning on vibration with level %hu\n", vibration_level);
                    print_debug_string(buf);
                }
            }
            break;
        case IDENTIFY_COMMAND:
            {
                unique_id = ((int64_t*)parameters)[0];
                rs485_allow_next_command();
                if(unique_id == my_unique_id) {
                    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk; // temporarily disable the SysTick interrupt
                    green_led_action_counter = 0;
                    n_identify_flashes = 30;
                    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; // reneable the interrupt
                    print_debug_string("Identifying\n");
                    rs485_transmit(NO_ERROR_RESPONSE, 3);
                }
            }
            break;
        case GET_TEMPERATURE_COMMAND:
            rs485_allow_next_command();
            {
                struct __attribute__((__packed__)) {
                    uint8_t header[3];
                    int16_t temperature;
                } get_temperature_reply;
                if(axis != ALL_ALIAS) {
                    get_temperature_reply.header[0] = RESPONSE_CHARACTER;
                    get_temperature_reply.header[1] = 1;
                    get_temperature_reply.header[2] = sizeof(get_temperature_reply) - sizeof(get_temperature_reply.header);
                    get_temperature_reply.temperature = get_temperature_degrees_C();
                    rs485_transmit(&get_temperature_reply, sizeof(get_temperature_reply));
                }
            }
            break;
        case SET_PID_CONSTANTS_COMMAND:
            {
                uint32_t p = ((uint32_t*)parameters)[0];
                uint32_t i = ((uint32_t*)parameters)[1];
                uint32_t d = ((uint32_t*)parameters)[2];
                rs485_allow_next_command();
                sprintf(buf, "PID constants: %lu, %lu, %lu\n", p, i, d);
                print_debug_string(buf);
                set_pid_constants(p, i, d);
                if(axis != ALL_ALIAS) {
                    rs485_transmit(NO_ERROR_RESPONSE, 3);
                }
            }
            break;
        case SET_MAX_ALLOWABLE_POSITION_DEVIATION:
            {
                int64_t new_max_allowable_position_deviation = llabs(((int64_t*)parameters)[0]);
                rs485_allow_next_command();
                set_max_allowable_position_deviation(new_max_allowable_position_deviation);
                if(axis != ALL_ALIAS) {
                    rs485_transmit(NO_ERROR_RESPONSE, 3);
                }
            }
            break;
        case GET_DEBUG_VALUES_COMMAND:
            {
                rs485_allow_next_command();
                if(axis != ALL_ALIAS) {
                    int64_t max_acceleration;
                    int64_t max_velocity;
                    int64_t currenbt_velocity;
                    int32_t measured_velocity;
                    uint32_t n_time_steps;
                    int64_t debug_value1;
                    int64_t debug_value2;
                    int64_t debug_value3;
                    int64_t debug_value4;
                    uint16_t all_motor_control_calulations_profiler_time;
                    uint16_t all_motor_control_calulations_profiler_max_time;
                    uint16_t get_sensor_position_profiler_time;
                    uint16_t get_sensor_position_profiler_max_time;
                    uint16_t compute_velocity_profiler_time;
                    uint16_t compute_velocity_profiler_max_time;
                    uint16_t motor_movement_calculations_profiler_time;
                    uint16_t motor_movement_calculations_profiler_max_time;
                    uint16_t motor_phase_calculations_profiler_time;
                    uint16_t motor_phase_calculations_profiler_max_time;
                    uint16_t motor_control_loop_period_profiler_time;
                    uint16_t motor_control_loop_period_profiler_max_time;
                    uint16_t hall_sensor1_voltage;
                    uint16_t hall_sensor2_voltage;
                    uint16_t hall_sensor3_voltage;
                    uint32_t commutation_position_offset;
                    uint8_t motor_phases_reversed;
                    int32_t max_hall_position_delta;
                    int32_t min_hall_position_delta;
                    int32_t average_hall_position_delta;
                    uint8_t motor_pwm_voltage;

                    get_motor_control_debug_values(&max_acceleration, &max_velocity, &currenbt_velocity, &measured_velocity, &n_time_steps, &debug_value1, &debug_value2, &debug_value3, &debug_value4);
                    get_profiled_times(&all_motor_control_calulations_profiler_time, &all_motor_control_calulations_profiler_max_time,
                                        &get_sensor_position_profiler_time, &get_sensor_position_profiler_max_time,
                                        &compute_velocity_profiler_time, &compute_velocity_profiler_max_time,
                                        &motor_movement_calculations_profiler_time, &motor_movement_calculations_profiler_max_time,
                                        &motor_phase_calculations_profiler_time, &motor_phase_calculations_profiler_max_time, 
                                        &motor_control_loop_period_profiler_time, &motor_control_loop_period_profiler_max_time);
                    get_hall_sensor_data(&hall_sensor1_voltage, &hall_sensor2_voltage, &hall_sensor3_voltage, &commutation_position_offset, &motor_phases_reversed);
                    get_hall_position_delta_stats(&max_hall_position_delta, &min_hall_position_delta, &average_hall_position_delta);
                    get_motor_pwm_voltage(&motor_pwm_voltage);

                    struct __attribute__((__packed__)) {
                        uint8_t header[3];
                        int64_t max_acceleration;
                        int64_t max_velocity;
                        int64_t currenbt_velocity;
                        int32_t measured_velocity;
                        uint32_t n_time_steps;
                        int64_t debug_value1;
                        int64_t debug_value2;
                        int64_t debug_value3;
                        int64_t debug_value4;
                        uint16_t all_motor_control_calulations_profiler_time;
                        uint16_t all_motor_control_calulations_profiler_max_time;
                        uint16_t get_sensor_position_profiler_time;
                        uint16_t get_sensor_position_profiler_max_time;
                        uint16_t compute_velocity_profiler_time;
                        uint16_t compute_velocity_profiler_max_time;
                        uint16_t motor_movement_calculations_profiler_time;
                        uint16_t motor_movement_calculations_profiler_max_time;
                        uint16_t motor_phase_calculations_profiler_time;
                        uint16_t motor_phase_calculations_profiler_max_time;
                        uint16_t motor_control_loop_period_profiler_time;
                        uint16_t motor_control_loop_period_profiler_max_time;
                        uint16_t hall_sensor1_voltage;
                        uint16_t hall_sensor2_voltage;
                        uint16_t hall_sensor3_voltage;
                        uint32_t commutation_position_offset;
                        uint8_t motor_phases_reversed;
                        int32_t max_hall_position_delta;
                        int32_t min_hall_position_delta;
                        int32_t average_hall_position_delta;
                        uint8_t motor_pwm_voltage;
                    } get_debug_values_reply;
                    get_debug_values_reply.header[0] = RESPONSE_CHARACTER;
                    get_debug_values_reply.header[1] = 1;
                    get_debug_values_reply.header[2] = sizeof(get_debug_values_reply) - sizeof(get_debug_values_reply.header);
                    get_debug_values_reply.max_acceleration = max_acceleration;
                    get_debug_values_reply.max_velocity = max_velocity;
                    get_debug_values_reply.currenbt_velocity = currenbt_velocity;
                    get_debug_values_reply.measured_velocity = measured_velocity;
                    get_debug_values_reply.n_time_steps = n_time_steps;
                    get_debug_values_reply.debug_value1 = debug_value1;
                    get_debug_values_reply.debug_value2 = debug_value2;
                    get_debug_values_reply.debug_value3 = debug_value3;
                    get_debug_values_reply.debug_value4 = debug_value4;
                    get_debug_values_reply.all_motor_control_calulations_profiler_time = all_motor_control_calulations_profiler_time;
                    get_debug_values_reply.all_motor_control_calulations_profiler_max_time = all_motor_control_calulations_profiler_max_time;
                    get_debug_values_reply.get_sensor_position_profiler_time = get_sensor_position_profiler_time;
                    get_debug_values_reply.get_sensor_position_profiler_max_time = get_sensor_position_profiler_max_time;
                    get_debug_values_reply.compute_velocity_profiler_time = compute_velocity_profiler_time;
                    get_debug_values_reply.compute_velocity_profiler_max_time = compute_velocity_profiler_max_time;
                    get_debug_values_reply.motor_movement_calculations_profiler_time = motor_movement_calculations_profiler_time;
                    get_debug_values_reply.motor_movement_calculations_profiler_max_time = motor_movement_calculations_profiler_max_time;
                    get_debug_values_reply.motor_phase_calculations_profiler_time = motor_phase_calculations_profiler_time;
                    get_debug_values_reply.motor_phase_calculations_profiler_max_time = motor_phase_calculations_profiler_max_time;
                    get_debug_values_reply.motor_control_loop_period_profiler_time = motor_control_loop_period_profiler_time;
                    get_debug_values_reply.motor_control_loop_period_profiler_max_time = motor_control_loop_period_profiler_max_time;
                    get_debug_values_reply.hall_sensor1_voltage = hall_sensor1_voltage;
                    get_debug_values_reply.hall_sensor2_voltage = hall_sensor2_voltage;
                    get_debug_values_reply.hall_sensor3_voltage = hall_sensor3_voltage;
                    get_debug_values_reply.commutation_position_offset = commutation_position_offset;
                    get_debug_values_reply.motor_phases_reversed = motor_phases_reversed;
                    get_debug_values_reply.max_hall_position_delta = max_hall_position_delta;
                    get_debug_values_reply.min_hall_position_delta = min_hall_position_delta;
                    get_debug_values_reply.average_hall_position_delta = average_hall_position_delta;
                    get_debug_values_reply.motor_pwm_voltage = motor_pwm_voltage;
                    rs485_transmit(&get_debug_values_reply, sizeof(get_debug_values_reply));
                }
            }
            break;
        }
    }
    else {
        rs485_allow_next_command();
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
    uint8_t command_debug_uart = get_command_from_debug_uart();

    if(command_debug_uart != 0) {
        disable_or_enable_debug_printing(1);
        switch(command_debug_uart) {
        case 'z':
            zero_position();
            break;
        case 'c':
            start_go_to_closed_loop_mode();
            break;
        case 'C':
            start_calibration(0);
            break;
        case 'a':
            start_calibration(1); // we will let it print the output so we can capture it and plot it
            break;
        case 'f':
            start_fast_capture_data();
            break;
        case 't':
            fast_capture_until_trigger();
            break;
        case 'p':
            print_debug_string("\n");
            print_position();
            print_sensor_position();
            print_external_encoder_position();
//            print_PID_data();
            print_queue_stats();
            print_current_movement();
            print_velocity();
            print_time_difference();
            print_hall_sensor_data();
            print_hall_position_delta_stats();
            print_motor_pwm_voltage();
            print_motor_status();
            print_temperature();
            print_supply_voltage();
            break;
        case 'P':
            print_motor_current();
            break;
        case 'I':
            increase_commutation_offset();
            break;
        case 'D':
            decrease_commutation_offset();
            break;
        case 'i':
            increase_motor_pwm_voltage();
            break;
        case 'd':
            decrease_motor_pwm_voltage();
            break;
        case 'e':
            check_current_sensor_and_enable_mosfets();
            break;
        case 'E':
            disable_mosfets();
            print_debug_string("Disabling MOSFETs\n");
            break;
        case 'o':
            if(get_motor_control_mode() == CLOSED_LOOP_POSITION_CONTROL) {
                set_motor_control_mode(OPEN_LOOP_POSITION_CONTROL);
                print_debug_string("Open loop position control\n");
            }
            else {
                set_motor_control_mode(CLOSED_LOOP_POSITION_CONTROL);
                print_debug_string("Closed loop position control\n");
            }
            break;
        case 'v':
            if(get_motor_control_mode() == OPEN_LOOP_PWM_VOLTAGE_CONTROL) {
                set_motor_control_mode(CLOSED_LOOP_POSITION_CONTROL);
                print_debug_string("Closed loop position control\n");
            }
            else {
                set_motor_control_mode(OPEN_LOOP_PWM_VOLTAGE_CONTROL);
                print_debug_string("Open loop PWM voltage control\n");
            }
            break;
        case 'h':
            start_homing(6451200, 620000);
            break;
        case 'H':
            start_homing(-6451200, 620000);
            break;
        case 'S':
            print_debug_string("Saving settings\n");
            save_global_settings();
            break;
        case 'r':
            NVIC_SystemReset();
            break;
        case 'g':
            add_trapezoid_move_to_queue(BUTTON_PRESS_MOTOR_MOVE_DISTANCE * 1, get_update_frequency() * 1);
            break;
        case 'G':
            add_trapezoid_move_to_queue(-BUTTON_PRESS_MOTOR_MOVE_DISTANCE * 1, get_update_frequency() * 1);
            break;
        case 'V':
            vibrate(1);
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

    if(get_button_state()) {
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
            start_calibration(0);
        }
        else if(time_pressed_down >= 2000) {
            start_go_to_closed_loop_mode();
        }
        else if(time_pressed_down >= 300) {
            check_current_sensor_and_enable_mosfets();
            add_trapezoid_move_to_queue(BUTTON_PRESS_MOTOR_MOVE_DISTANCE * 2, get_update_frequency() * 2);
        }
        else if(time_pressed_down >= 50) {
            check_current_sensor_and_enable_mosfets();
            add_trapezoid_move_to_queue(-BUTTON_PRESS_MOTOR_MOVE_DISTANCE * 2, get_update_frequency() * 2);
        }
    }
}


void print_start_message(void)
{
//  char buff[200];
    uint32_t my_unique_id_u32_array[2];

    memcpy(my_unique_id_u32_array, &my_unique_id, sizeof(my_unique_id));

    print_debug_string("Applicaiton start\n");
//    sprintf(buff, "Unique ID: 0x%08lX%08lX\n", my_unique_id_u32_array[1], my_unique_id_u32_array[0]);
//    print_debug_string(buff);
//    if((global_settings.my_alias >= 33) && (global_settings.my_alias <= 126)) {
//        sprintf(buff, "Alias: %c\n", global_settings.my_alias);
//    }
//    else {
//        sprintf(buff, "Alias: 0x%02hx\n", global_settings.my_alias);
//    }
//    print_debug_string(buff);
    print_hall_midlines();
    print_max_motor_current_settings();
    print_commutation_position_offset();
}

int main(void)
{
    clock_init();
    systick_init();
    #if defined(PRODUCT_NAME_M3) && defined(GC6609)
    power_off_GC6609();
    #endif
    GPIO_init();
    debug_uart_init();
    rs485_init();
    adc_init();
    pwm_init();
    motor_control_init();
    overvoltage_init();
    #if defined(PRODUCT_NAME_M1) || defined(PRODUCT_NAME_M2)
    step_and_direction_init();
    #endif
    #if defined(PRODUCT_NAME_M3) && defined(GC6609)
    reset_GC6609();
    init_GC6609_through_UART();
    #endif

    SCB->VTOR = 0x2800; // vector table is moved to where the application starts, which is after the bootloader

    my_unique_id = get_unique_id();

    load_global_settings(); // load the settings from non-volatile memory into ram for faster access
    if(global_settings.commutation_position_offset == 0xffffffff) {
        global_settings.commutation_position_offset = DEFAULT_COMMUTATION_POSITION_OFFSET;
    }
    set_commutation_position_offset(global_settings.commutation_position_offset);

    microsecond_clock_init();

//    set_max_motor_current(700, 700); // DEBUG
//    check_current_sensor_and_enable_mosfets(); // DEBUG
//    test_motor_stepping(); // DEBUG

    __enable_irq();

    set_pid_constants(PROPORTIONAL_CONSTANT_PID, INTEGRAL_CONSTANT_PID, DERIVATIVE_CONSTANT_PID); // note that within this function is a call to __disable_irq() and __enable_irq()

    print_start_message();

    disable_or_enable_debug_printing(0); // from now on we will not print anythign to the debug UART unless the user interacts with the debug UART by sending a command

    while(1) {
        if(commandReceived) {
            if(detect_devices_delay >= 0) { // if a DETECT_DEVICES_COMMAND has been issued then we will ignore all other commands until the delay is over and we send out the unique ID
                rs485_allow_next_command();
            }
            else {
                processCommand(selectedAxis, command, valueBuffer);
            }
        }

        if(detect_devices_delay == 0) {
            print_debug_string("Transmitting unique ID\n");
            transmit_unique_id();
            detect_devices_delay--;
        }

        if(process_calibration_data()) {
            disable_motor_control_loop();
            save_global_settings();
            print_debug_string("\nResetting\n\n");
            microsecond_delay(10000);
            NVIC_SystemReset();
        }

        if(is_fast_capture_data_result_ready()) {
            print_fast_capture_data_result();
        }

        check_if_actual_vs_desired_position_deviated_too_much();
#ifdef PRODUCT_NAME_M1
        process_go_to_closed_loop_data();
#endif

        process_debug_uart_commands();
        button_logic();

#ifdef PRODUCT_NAME_M1
        if((GPIOA->IDR & (1 << TOUCH_BUTTON_PORT_A_PIN)) == 0) {
            red_LED_on();
        }
        else {
            red_LED_off();
        }
#endif

//      check_if_break_condition();
#if defined(PRODUCT_NAME_M1) || defined(PRODUCT_NAME_M2) 
        check_if_ADC_watchdog2_exceeded();
#endif
#if defined(PRODUCT_NAME_M1)
    check_if_ADC_watchdog2_exceeded();
    check_if_ADC_watchdog3_exceeded();
#endif
        check_if_overtemperature();
    }
}
