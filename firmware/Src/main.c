#include "stm32g0xx_hal.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
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
#include "crc32.h"
#include "product_info.h"
#include "global_variables.h"
#include "device_status.h"
#include "portable_stdint.h"
#ifdef PRODUCT_NAME_M1
#include "commutation_table_M1.h"
#endif
#ifdef PRODUCT_NAME_M2
#include "commutation_table_M2.h"
#include "hall_sensor_constants_M2.h"
#endif
#ifdef PRODUCT_NAME_M17
#include "commutation_table_M17.h"
#endif
#ifdef PRODUCT_NAME_M23
#include "commutation_table_M23.h"
#endif

// Simulation-only printf function that compiles to nothing in firmware builds and optionally
// compiles to nothing if the SIMULATOR_DEBUG_PRINTING is not defined (ie. you are not debugging)
//#define SIMULATOR_DEBUG_PRINTING
#ifdef MOTOR_SIMULATION
#ifdef SIMULATOR_DEBUG_PRINTING
#include <stdio.h>
#define simulation_printf(...) printf(__VA_ARGS__)
#else
#define simulation_printf(...) ((void)0)
#endif
#else
#define simulation_printf(...) ((void)0)
#endif

//#define CERTIFICATION_TEST_MODE // if this is uncommented then the motor will start spinning soon after power is applied and will continue spinning indefinitely

char PRODUCT_DESCRIPTION[] = "Servomotor";

// Here we store the version information for the main firmware (not the bootloader firmware)
struct __attribute__((__packed__)) firmware_version_struct {
    uint8_t development;
    uint8_t bugfix;
    uint8_t minor;
    uint8_t major;
};
#define MAJOR_FIRMWARE_VERSION 0
#define MINOR_FIRMWARE_VERSION 14
#define BUGFIX_FIRMWARE_VERSION 2
#define DEVELOPMENT_FIRMWARE_VERSION 0 // this is the least significant number when it comes to versioning and is the last number on the right when printed in human readable form
struct firmware_version_struct firmware_version = {DEVELOPMENT_FIRMWARE_VERSION, BUGFIX_FIRMWARE_VERSION, MINOR_FIRMWARE_VERSION, MAJOR_FIRMWARE_VERSION};

#define BUTTON_PRESS_MOTOR_MOVE_DISTANCE ONE_REVOLUTION_MICROSTEPS

#define PING_PAYLOAD_SIZE 10

#define DEFAULT_MIDLINE 34000 // Later it might be good to have this constant autodetermined and placed into hall_sensor_constants_M17.h

extern volatile uint16_t ADC_buffer[DMA_ADC_BUFFER_SIZE];

static uint64_t my_unique_id;
static int16_t detect_devices_delay_to_answer = -1;
static int16_t detect_devices_delay_to_reenable_packet_processing = -1;
static uint32_t hall_sensor_n_points_to_capture = 0;
static uint8_t hall_sensor_point_size = 0;
static uint16_t captured_point_division_factor = 1;
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

    if(detect_devices_delay_to_answer > 0) {
        detect_devices_delay_to_answer--;
    }
    if(detect_devices_delay_to_reenable_packet_processing >= 0) {
        detect_devices_delay_to_reenable_packet_processing--;
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


void copy_input_parameters_and_check_size(void *destination, const void *source, uint32_t destination_size, uint32_t source_size)
{
    if (destination_size != source_size) {
        fatal_error(ERROR_COMMAND_SIZE_WRONG);
    }
    memcpy(destination, source, destination_size);
}

void check_payload_size_is_zero(uint16_t payload_size)
{
    if (payload_size != 0) {
        fatal_error(ERROR_COMMAND_SIZE_WRONG);
    }
}


void process_packet(void)
{
    uint8_t command;
    uint16_t payload_size;
    uint8_t *payload;
    uint8_t is_broadcast;

    // There is a distinct possibility that the new packet is not for us. We will know after we see the return value of rs485_get_next_packet.
    if (!rs485_get_next_packet(&command, &payload_size, &payload, &is_broadcast)) {
        // The new packet, whatever it is, is not of interest to us and we need to clear it and return out of here
        rs485_done_with_this_packet();
        return;
    }

    #ifdef MOTOR_SIMULATION
    printf("Received a valid packet. The payload size is %hu\n", payload_size);
    #endif

    if(!rs485_validate_packet_crc32()) {
        // CRC32 validation failed, allow next command and return
        rs485_done_with_this_packet();
        return;
    }

    if (!is_broadcast) {
        if_fatal_error_then_respond(); // set up the fatal error logic such that if a fatal error occurs during the processing of this command then we will respond back with the fatal error code
    }

    #ifdef MOTOR_SIMULATION
    printf("Command received that is valid for this device: Command=%02u is_broadcast = %hhu\n", command, is_broadcast);
    #endif

    switch(command) {
    case DISABLE_MOSFETS_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        disable_mosfets();
        rs485_transmit_no_error_packet(is_broadcast);
        break;
    case ENABLE_MOSFETS_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        check_current_sensor_and_enable_mosfets();
        rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
        break;
    case TRAPEZOID_MOVE_COMMAND:
        {
            struct __attribute__((__packed__)) {
                int32_t displacement;
                uint32_t time;
            } trapezoid_move_inputs;
            copy_input_parameters_and_check_size(&trapezoid_move_inputs, payload, sizeof(trapezoid_move_inputs), payload_size);
            rs485_done_with_this_packet();
            add_trapezoid_move_to_queue(trapezoid_move_inputs.displacement, trapezoid_move_inputs.time);
        }
        rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
        break;
    case SET_MAX_VELOCITY_COMMAND:
        {
            uint32_t max_velocity;
            copy_input_parameters_and_check_size(&max_velocity, payload, sizeof(max_velocity), payload_size);
            rs485_done_with_this_packet();
            set_max_velocity(max_velocity);
        }
        rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
        break;
    case GO_TO_POSITION_COMMAND:
        {
            struct __attribute__((__packed__)) {
                int32_t end_position;
                uint32_t move_time;
            } go_to_position_input;
            copy_input_parameters_and_check_size(&go_to_position_input, payload, sizeof(go_to_position_input), payload_size);
            rs485_done_with_this_packet();
            add_go_to_position_to_queue(go_to_position_input.end_position, go_to_position_input.move_time);
        }
        rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
        break;
    case SET_MAX_ACCELERATION_COMMAND:
        {
            uint32_t max_acceleration;
            copy_input_parameters_and_check_size(&max_acceleration, payload, sizeof(max_acceleration), payload_size);
            rs485_done_with_this_packet();
            set_max_acceleration(max_acceleration);
        }
        rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
        break;
    case START_CALIBRATION_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
        start_calibration(0);
        break;
    case CAPTURE_HALL_SENSOR_DATA_COMMAND:
        {
            typedef struct __attribute__((__packed__)) {
                uint8_t capture_type;
                uint32_t n_points_to_capture;
                uint8_t channels_to_capture_bitmask;
                uint16_t time_steps_per_sample;
                uint16_t n_samples_to_sum;
                uint16_t division_factor;
            } capture_hall_sensor_data_input_t;
            capture_hall_sensor_data_input_t capture_hall_sensor_data_input;
            copy_input_parameters_and_check_size(&capture_hall_sensor_data_input, payload, sizeof(capture_hall_sensor_data_input), payload_size);
            rs485_done_with_this_packet();
            if ( !((capture_hall_sensor_data_input.capture_type == 1) ||
                   (capture_hall_sensor_data_input.capture_type == 2) ||
                   (capture_hall_sensor_data_input.capture_type == 3))  ) {
                fatal_error(ERROR_CAPTURE_BAD_PARAMETERS);
            }
            if ( (capture_hall_sensor_data_input.n_points_to_capture == 0) ||
                 (capture_hall_sensor_data_input.time_steps_per_sample == 0) ||
                 (capture_hall_sensor_data_input.n_samples_to_sum == 0) ||
                 (capture_hall_sensor_data_input.division_factor == 0)         ) {
                fatal_error(ERROR_CAPTURE_BAD_PARAMETERS);
            }
            if (((capture_hall_sensor_data_input.channels_to_capture_bitmask & 7) == 0) ||
                ((capture_hall_sensor_data_input.channels_to_capture_bitmask & (~7)) != 0)) {
                fatal_error(ERROR_CAPTURE_BAD_PARAMETERS);
            }
            uint16_t n_channels = 0;
            for (uint16_t i = 0; i < 3; i++) {
                if (capture_hall_sensor_data_input.channels_to_capture_bitmask & (1 << i)) {
                    n_channels++;
                }
            }
            hall_sensor_point_size = n_channels * sizeof(uint16_t);
            uint32_t payload_size = capture_hall_sensor_data_input.n_points_to_capture * hall_sensor_point_size;
            if (payload_size > 65535 - 5 - sizeof(uint32_t)) {
                fatal_error(ERROR_CAPTURE_PAYLOAD_TOO_BIG);
            }
            simulation_printf("Starting the RS485 packet. payload_size = %u\n", payload_size);
            rs485_start_the_packet((uint16_t)payload_size);
            hall_sensor_n_points_to_capture = capture_hall_sensor_data_input.n_points_to_capture;
            captured_point_division_factor = capture_hall_sensor_data_input.division_factor;
            simulation_printf("Starting the capture:");
            simulation_printf("   capture_type = %hhu\n",                capture_hall_sensor_data_input.capture_type);
            simulation_printf("   n_points_to_capture = %u\n",           capture_hall_sensor_data_input.n_points_to_capture);
            simulation_printf("   channels_to_capture_bitmask = %hhu\n", capture_hall_sensor_data_input.channels_to_capture_bitmask);
            simulation_printf("   time_steps_per_sample = %hu\n",        capture_hall_sensor_data_input.time_steps_per_sample);
            simulation_printf("   n_samples_to_sum = %hu\n",             capture_hall_sensor_data_input.n_samples_to_sum);
            simulation_printf("   n_channels = %hu\n",                     n_channels);
            simulation_printf("   hall_sensor_n_points_to_capture = %u\n", hall_sensor_n_points_to_capture);
            simulation_printf("   captured_point_division_factor = %hu\n", captured_point_division_factor);
            simulation_printf("   hall_sensor_point_size = %hhu\n",        hall_sensor_point_size);
            print_debug_string("Capture start\n");
            start_or_stop_capture(capture_hall_sensor_data_input.capture_type,
                                  capture_hall_sensor_data_input.channels_to_capture_bitmask,
                                  capture_hall_sensor_data_input.time_steps_per_sample,
                                  capture_hall_sensor_data_input.n_samples_to_sum);
        }
        break;
    case RESET_TIME_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        reset_time();
        rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
        break;
    case GET_CURRENT_TIME_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        if(!is_broadcast) {
            struct __attribute__((__packed__)) {
                uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                uint64_t current_time;
                uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
            } current_time_reply;
            current_time_reply.current_time = get_microsecond_time();
            rs485_finalize_and_transmit_packet(&current_time_reply, sizeof(current_time_reply));
        }
        break;
    case TIME_SYNC_COMMAND:
        {
            uint64_t time_from_master;
            copy_input_parameters_and_check_size(&time_from_master, payload, 6, payload_size);
            rs485_done_with_this_packet();
            if(!is_broadcast) {
                struct __attribute__((__packed__)) {
                    uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                    int32_t time_error;
                    uint16_t clock_calibration_value;
                    uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
                } time_sync_reply;
                time_sync_reply.time_error = time_sync(time_from_master);
                time_sync_reply.clock_calibration_value = get_clock_calibration_value();
                rs485_finalize_and_transmit_packet(&time_sync_reply, sizeof(time_sync_reply));
            }
        }
        break;
    case GET_N_ITEMS_IN_QUEUE_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        if(!is_broadcast) {
            struct __attribute__((__packed__)) {
                uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                uint8_t n_items_in_queue;
                uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
            } queue_items_reply;
            queue_items_reply.n_items_in_queue = get_n_items_in_queue();
            rs485_finalize_and_transmit_packet(&queue_items_reply, sizeof(queue_items_reply));
        }
        break;
    case EMERGENCY_STOP_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        emergency_stop();
        rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
        break;
    case ZERO_POSITION_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        zero_position();
        rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
        break;
    case HOMING_COMMAND:
        {
            struct __attribute__((__packed__)) {
                int32_t max_homing_travel_displacement;
                uint32_t max_homing_time;
            } homing_input;
            copy_input_parameters_and_check_size(&homing_input, payload, sizeof(homing_input), payload_size);
            rs485_done_with_this_packet();
            char buf[100];
            sprintf(buf, "homing: max_homing_travel_displacement: " _PRId32 "   max_homing_time: " _PRIu32 "\n", homing_input.max_homing_travel_displacement, homing_input.max_homing_time);
            print_debug_string(buf);
            start_homing(homing_input.max_homing_travel_displacement, homing_input.max_homing_time);
            rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
        }
        break;
    case GET_POSITION_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        if(!is_broadcast) {
            struct __attribute__((__packed__)) {
                uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                int64_t motor_position;
                uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
            } position_reply;
            position_reply.motor_position = get_motor_position();
            rs485_finalize_and_transmit_packet(&position_reply, sizeof(position_reply));
        }
        break;
    case GET_HALL_SENSOR_POSITION_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        if(!is_broadcast) {
            struct __attribute__((__packed__)) {
                uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                int64_t hall_sensor_position;
                uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
            } hall_position_reply;
            hall_position_reply.hall_sensor_position = get_hall_position();
            rs485_finalize_and_transmit_packet(&hall_position_reply, sizeof(hall_position_reply));
        }
        break;
    case GET_COMPREHENSIVE_POSITION_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        {
            struct __attribute__((__packed__)) {
                uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                int64_t motor_position;
                int64_t hall_sensor_position;
                int32_t external_encoder_position;
                uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
            } comprehensive_position;
            if(!is_broadcast) {
                comprehensive_position.motor_position = get_motor_position();
                comprehensive_position.hall_sensor_position = get_hall_position();
                comprehensive_position.external_encoder_position = get_external_encoder_position();
                rs485_finalize_and_transmit_packet(&comprehensive_position, sizeof(comprehensive_position));
            }
        }
        break;
    case GET_STATUS_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        if(!is_broadcast) {
            struct __attribute__((__packed__)) {
                uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                struct device_status_struct status;
                uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
            } status_reply;
            uint16_t motor_status_flags = get_motor_status_flags();
            set_device_status_flags(motor_status_flags);
            memcpy(&status_reply.status, get_device_status(), sizeof(struct device_status_struct));
            rs485_finalize_and_transmit_packet(&status_reply, sizeof(status_reply));
            char buf[100];
            sprintf(buf, "Get status: %hu\n", motor_status_flags);
            print_debug_string(buf);
        }
        break;
    case GO_TO_CLOSED_LOOP_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        start_go_to_closed_loop_mode();
        rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
        break;
    case GET_PRODUCT_SPECS_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        if(!is_broadcast) {
            struct __attribute__((__packed__)) {
                uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                uint32_t frequency;
                uint32_t counts_per_rotation;
                uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
            } get_product_specs_reply;
            get_product_specs_reply.frequency = get_update_frequency();
            get_product_specs_reply.counts_per_rotation = ONE_REVOLUTION_MICROSTEPS;
            rs485_finalize_and_transmit_packet(&get_product_specs_reply, sizeof(get_product_specs_reply));
        }
        break;
    case MOVE_WITH_ACCELERATION_COMMAND:
        {
            struct __attribute__((__packed__)) {
                int32_t acceleration;
                uint32_t time_steps;
            } acceleration_input;
            copy_input_parameters_and_check_size(&acceleration_input, payload, sizeof(acceleration_input), payload_size);
            rs485_done_with_this_packet();
        //            char buf[100];
        //            sprintf(buf, "move_with_acceleration: %ld %lu\n", acceleration_input.acceleration, acceleration_input.time_steps);
        //            print_debug_string(buf);
            add_to_queue(acceleration_input.acceleration, acceleration_input.time_steps, MOVE_WITH_ACCELERATION);
            rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
        }
        break;
    case DETECT_DEVICES_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        detect_devices_delay_to_reenable_packet_processing = 100;
        detect_devices_delay_to_answer = get_random_number(95);
        break;
    case SET_DEVICE_ALIAS_COMMAND:
        {
            uint8_t new_alias;
            copy_input_parameters_and_check_size(&new_alias, payload, sizeof(new_alias), payload_size);
            rs485_done_with_this_packet();
            if ( (new_alias == EXTENDED_ADDRESSING              ) ||
                 (new_alias == RESPONSE_CHARACTER_CRC32_ENABLED ) ||
                 (new_alias == RESPONSE_CHARACTER_CRC32_DISABLED)    ) {
                fatal_error(ERROR_BAD_ALIAS);
            }
            rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
            print_number("Unique ID matches. Will save the alias and reset. New alias:", (uint16_t)new_alias);
            rs485_wait_for_transmit_done(); // make sure that the no error packet is sent out
            microsecond_delay(5000); // 5ms should be enough time to transmit the above debug message
            global_settings.my_alias = new_alias;
            save_global_settings(); // this will never return because the device will reset after writing the new settings to flash
        }
        break;
    case GET_PRODUCT_INFO_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        if(!is_broadcast) {
            struct __attribute__((__packed__)) {
                uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                struct product_info_struct product_info;
                uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
            } product_info_reply;
            struct product_info_struct *product_info = get_product_info();
            memcpy(&product_info_reply.product_info, product_info, sizeof(struct product_info_struct));
            rs485_finalize_and_transmit_packet(&product_info_reply, sizeof(product_info_reply));
        }
        break;
    case GET_PRODUCT_DESCRIPTION_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        if(!is_broadcast) {
            struct __attribute__((__packed__)) {
                uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                char product_description[sizeof(PRODUCT_DESCRIPTION)];
                uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
            } product_description_reply;
            memcpy(product_description_reply.product_description, PRODUCT_DESCRIPTION, sizeof(PRODUCT_DESCRIPTION));
            rs485_finalize_and_transmit_packet(&product_description_reply, sizeof(product_description_reply));
        }
        break;
    case GET_FIRMWARE_VERSION_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        if(!is_broadcast) {
            struct __attribute__((__packed__)) {
                uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                struct firmware_version_struct firmware_version_data;
                uint8_t in_bootloader;
                uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
            } firmware_version_reply;
            memcpy(&firmware_version_reply.firmware_version_data, &firmware_version, sizeof(firmware_version));
            firmware_version_reply.in_bootloader = 0; // 0 indicates that we are running the main firmware (not the bootloader) and we are returning the firmware version of the main firmware
            rs485_finalize_and_transmit_packet(&firmware_version_reply, sizeof(firmware_version_reply));
        }
        break;
    case MOVE_WITH_VELOCITY_COMMAND:
        {
            struct __attribute__((__packed__)) {
                int32_t velocity;
                uint32_t time_steps;
            } velocity_input;
            copy_input_parameters_and_check_size(&velocity_input, payload, sizeof(velocity_input), payload_size);
            rs485_done_with_this_packet();
    //            char buf[100];
    //            sprintf(buf, "move_with_velocity: %ld %lu\n", velocity_input.velocity, velocity_input.time_steps);
    //            print_debug_string(buf);
            add_to_queue(velocity_input.velocity, velocity_input.time_steps, MOVE_WITH_VELOCITY);
            rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
        }
        break;
    case SYSTEM_RESET_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
        rs485_wait_for_transmit_done(); // make sure that the no error packet is sent out before resetting the device
        NVIC_SystemReset();
        break;
    case SET_MAXIMUM_MOTOR_CURRENT:
        {
            struct __attribute__((__packed__)) {
                uint16_t new_maximum_motor_current;
                uint16_t new_maximum_motor_regen_current;
            } current_input;
            copy_input_parameters_and_check_size(&current_input, payload, sizeof(current_input), payload_size);
            rs485_done_with_this_packet();
            set_max_motor_current(current_input.new_maximum_motor_current, current_input.new_maximum_motor_regen_current);
            rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
        }
        break;
    case MULTIMOVE_COMMAND:
        {
            typedef struct __attribute__((__packed__)) {
                union { // if the bitfield shows a 0 for this move then this parameter represents the acceleration, otherwise it represents the velocity
                    int32_t acceleration;
                    int32_t velocity;
                };
                int32_t time_steps;
            } move_parameters_t;

            // First check if we have at least one byte for n_moves_in_this_command
            if (payload_size < 1) {
                fatal_error(ERROR_COMMAND_SIZE_WRONG);
            }
            uint8_t n_moves_in_this_command = ((int8_t*)payload)[0];
            #ifdef MOTOR_SIMULATION
            printf("n_moves_in_this_command: %hhu\n", n_moves_in_this_command);
            #endif
            typedef struct __attribute__((__packed__)) {
                uint32_t move_type_bits; // a bit field specifying the type of each move: 0 = move with acceleration; 1 = move with velocuty
                move_parameters_t move_parameters[MAX_MULTIMOVES];
//                move_parameters_t move_parameters[n_moves_in_this_command];
            } multi_move_command_buffer_t;
            multi_move_command_buffer_t multi_move_command_buffer;
            
            // Calculate the expected payload size: 1 byte for n_moves + 4 bytes for move_type_bits + n_moves * 8 bytes for parameters
            uint16_t expected_size = sizeof(multi_move_command_buffer.move_type_bits) + n_moves_in_this_command * sizeof(multi_move_command_buffer.move_parameters[0]);
            #ifdef MOTOR_SIMULATION
            printf("expected_size: %hu   payload_size - 1: %hu\n", expected_size, (uint16_t)(payload_size - 1));
            #endif
//            uint16_t expected_size = sizeof(n_moves_in_this_command) + sizeof(multi_move_command_buffer);
            copy_input_parameters_and_check_size(&multi_move_command_buffer, payload + 1, expected_size, payload_size - 1);
            rs485_done_with_this_packet();
    //            char buf[100];
    //            sprintf(buf, "multimove: %hu   %lu\n", n_moves_in_this_command, multi_move_command_buffer.move_type_bits);
    //            print_debug_string(buf);
            if(n_moves_in_this_command > MAX_MULTIMOVES) {
                fatal_error(ERROR_MULTIMOVE_MORE_THAN_32_MOVES); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
            }
            for(uint8_t i = 0; i < n_moves_in_this_command; i++) {
                if((multi_move_command_buffer.move_type_bits & 1) == 0) {
                    add_to_queue(multi_move_command_buffer.move_parameters[i].acceleration, multi_move_command_buffer.move_parameters[i].time_steps, MOVE_WITH_ACCELERATION);
    //                    char buf[100];
    //                    sprintf(buf, "added_to_queue: acceleration: %ld  time_steps: %lu\n", multi_move_command_buffer.move_parameters[i].acceleration, multi_move_command_buffer.move_parameters[i].time_steps);
    //                    print_debug_string(buf);
                } else {
                    add_to_queue(multi_move_command_buffer.move_parameters[i].velocity, multi_move_command_buffer.move_parameters[i].time_steps, MOVE_WITH_VELOCITY);
    //                    char buf[100];
    //                    sprintf(buf, "added_to_queue: velocity: %ld  time_steps: %lu\n", multi_move_command_buffer.move_parameters[i].velocity, multi_move_command_buffer.move_parameters[i].time_steps);
    //                    print_debug_string(buf);
                }
                multi_move_command_buffer.move_type_bits >>= 1;
            }
            rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
        }
        break;
    case SET_SAFETY_LIMITS_COMMAND:
        {
            struct __attribute__((__packed__)) {
                int64_t lower_limit;
                int64_t upper_limit;
            } limits_input;
            copy_input_parameters_and_check_size(&limits_input, payload, sizeof(limits_input), payload_size);
            rs485_done_with_this_packet();
    //            char buf[100];
    //            sprintf(buf, "movement limits %ld to %ld\n", limits_input.lower_limit, limits_input.upper_limit);
    //            print_debug_string(buf);
            set_movement_limits(limits_input.lower_limit, limits_input.upper_limit);
            print_int64("lower_limit:", limits_input.lower_limit);
            print_int64("upper_limit:", limits_input.upper_limit);
            rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
        }
        break;
    case ADD_TO_QUEUE_TEST_COMMAND:
        {
            struct __attribute__((__packed__)) {
                int32_t param1;
                uint32_t param2;
                uint8_t param3;
            } queue_test_input;
            copy_input_parameters_and_check_size(&queue_test_input, payload, sizeof(queue_test_input), payload_size);
            rs485_done_with_this_packet();
            
            struct __attribute__((__packed__)) {
                uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                add_to_queue_test_results_t add_to_queue_test_results;
                uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
            } add_to_queue_test_reply;
            add_to_queue_test(queue_test_input.param1, queue_test_input.param2, queue_test_input.param3, &add_to_queue_test_reply.add_to_queue_test_results);
            if(!is_broadcast) {
                rs485_finalize_and_transmit_packet(&add_to_queue_test_reply, sizeof(add_to_queue_test_reply));
            }
        }
        break;
    case PING_COMMAND:
        {
            struct __attribute__((__packed__)) {
                uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                uint8_t ping_payload[PING_PAYLOAD_SIZE];
                uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
            } ping_reply;
            copy_input_parameters_and_check_size(ping_reply.ping_payload, payload, sizeof(ping_reply.ping_payload), payload_size);
            rs485_done_with_this_packet();
            if(!is_broadcast) {
                rs485_finalize_and_transmit_packet(&ping_reply, sizeof(ping_reply));
            }
        }
        break;
    case CONTROL_HALL_SENSOR_STATISTICS_COMMAND:
        {
            uint8_t control_hall_sensor_statistics_subcommand;
            copy_input_parameters_and_check_size(&control_hall_sensor_statistics_subcommand, payload, sizeof(control_hall_sensor_statistics_subcommand), payload_size);
            rs485_done_with_this_packet();
            if(!is_broadcast) {
                if(control_hall_sensor_statistics_subcommand == 0) {
                    hall_sensor_turn_off_statistics();
                }
                if(control_hall_sensor_statistics_subcommand == 1) {
                    hall_sensor_turn_on_and_reset_statistics();
                }
                rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
            }
        }
        break;
    case GET_HALL_SENSOR_STATISTICS_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        if(!is_broadcast) {
            struct __attribute__((__packed__)) {
                uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                hall_sensor_statistics_t hall_sensor_statistics;
                uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
            } hall_sensor_statistics_full_response;
            get_hall_sensor_statistics(&hall_sensor_statistics_full_response.hall_sensor_statistics);
            rs485_finalize_and_transmit_packet(&hall_sensor_statistics_full_response, sizeof(hall_sensor_statistics_full_response));
        }
        break;
    case READ_MULTIPURPOSE_BUFFER_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        if(!is_broadcast) {
            uint8_t data_type;
            uint16_t data_size;
            uint16_t data_size_left_to_send;
            uint8_t *multipurpose_data_ptr;
            uint32_t crc32;
            get_multipurpose_data(&data_type, &data_size, &multipurpose_data_ptr);
            if (data_type != 0) {
                struct __attribute__((__packed__)) {
                    uint8_t packet_size;
                    uint16_t extended_packet_size;
                    uint8_t response_character;
                    uint8_t error_code;
                    uint8_t data_type;
                } multipurpose_data_response_header;
                multipurpose_data_response_header.packet_size = 255; // when size is 255 then we get the actual size from the next 16 bit number (thus allowing sizes up to 65535)
                multipurpose_data_response_header.extended_packet_size = sizeof(multipurpose_data_response_header) + data_size;
                if (rs485_is_crc32_enabled()) {
                    crc32_init();
                    multipurpose_data_response_header.extended_packet_size += sizeof(crc32);
                    multipurpose_data_response_header.response_character = RESPONSE_CHARACTER_CRC32_ENABLED;
                }
                else {
                    multipurpose_data_response_header.response_character = RESPONSE_CHARACTER_CRC32_DISABLED;
                }
                multipurpose_data_response_header.error_code = ERROR_CODE_NO_ERROR;
                multipurpose_data_response_header.data_type = data_type;
                rs485_transmit(&multipurpose_data_response_header, sizeof(multipurpose_data_response_header));
                if (rs485_is_crc32_enabled()) {
                    crc32 = calculate_crc32_buffer(&multipurpose_data_response_header, sizeof(multipurpose_data_response_header));
                }
                data_size_left_to_send = data_size;
                while(data_size_left_to_send > 0) {
                    uint16_t data_size_this_packet = data_size_left_to_send;
                    if(data_size_this_packet > TRANSMIT_BUFFER_SIZE) {
                        data_size_this_packet = TRANSMIT_BUFFER_SIZE;
                    }
                    if (rs485_is_crc32_enabled()) {
                        crc32 = calculate_crc32_buffer_without_reinit(multipurpose_data_ptr, data_size_this_packet);
                    }
                    rs485_transmit(multipurpose_data_ptr, data_size_this_packet);
                    multipurpose_data_ptr += data_size_this_packet;
                    data_size_left_to_send -= data_size_this_packet;
                }
                if (rs485_is_crc32_enabled()) {
                    rs485_transmit(&crc32, sizeof(crc32));
                }
                rs485_wait_for_transmit_done();
                clear_multipurpose_data();
            }
            print_debug_string("Read multipurpose data\n");
            char buf[100];
            sprintf(buf, "Type is %hu, Size is %hu\n", data_type, data_size);
            print_debug_string(buf);
        }
        break;
    case GET_SUPPLY_VOLTAGE_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        {
            struct __attribute__((__packed__)) {
                uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                uint16_t supply_voltage;
                uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
            } supply_voltage_reply;
            if(!is_broadcast) {
                supply_voltage_reply.supply_voltage = get_supply_voltage_volts_times_10();
                rs485_finalize_and_transmit_packet(&supply_voltage_reply, sizeof(supply_voltage_reply));
            }
        }
        break;
    case GET_MAX_PID_ERROR_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        {
            struct __attribute__((__packed__)) {
                uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                int32_t min_PID_error;
                int32_t max_PID_error;
                uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
            } get_max_pid_error_reply;
            if(!is_broadcast) {
                int32_t min_PID_error;
                int32_t max_PID_error;
                get_max_PID_error(&min_PID_error, &max_PID_error);
                get_max_pid_error_reply.min_PID_error = min_PID_error;
                get_max_pid_error_reply.max_PID_error = max_PID_error;
                rs485_finalize_and_transmit_packet(&get_max_pid_error_reply, sizeof(get_max_pid_error_reply));
            }
        }
        break;
    case TEST_MODE_COMMAND:
        {
            uint8_t test_mode;
            uint8_t led_test_mode_active = 0;
            uint8_t led_test_mode = 0;
            copy_input_parameters_and_check_size(&test_mode, payload, sizeof(test_mode), payload_size);
            rs485_done_with_this_packet();
            if (test_mode == 0) {
                set_motor_test_mode(0);
                set_led_test_mode(0);
            }
            else if (test_mode < 10) {
                set_motor_test_mode(test_mode);
            }
            else if (test_mode < 10 + 4) {
                led_test_mode_active = 1;
                // test modes 10..13 map to LED bitmasks 0..3 (green=bit0, red=bit1)
                led_test_mode = test_mode - 10;
            }
            else if (test_mode < 14 + 60) { // test modes 14 to 73 are for triggering fatal errors 0 to 59, for testing if fatal errors are working correctly
                fatal_error(test_mode - 14);
            }
            else {
                fatal_error(ERROR_INVALID_TEST_MODE);
            }
            char buf[100];
            sprintf(buf, "Setting the test mode to %hu\n", test_mode);
            print_debug_string(buf);
            rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
            rs485_wait_for_transmit_done(); // make sure that the no error packet is sent out before resetting the device
            if (led_test_mode_active) {
                set_led_test_mode(led_test_mode); // we will not exit from this function, so it is important to call this after achnowledging the transmission
            }
        }
        break;
    case VIBRATE_COMMAND:
        {
            uint8_t vibration_level;
            copy_input_parameters_and_check_size(&vibration_level, payload, sizeof(vibration_level), payload_size);
            rs485_done_with_this_packet();
            vibrate(vibration_level);
            rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
            if(vibration_level == 0) {
                print_debug_string("Turning off vibration\n");
            }
            else {
                char buf[100];
                sprintf(buf, "Turning on vibration with level %hu\n", vibration_level);
                print_debug_string(buf);
            }
        }
        break;
    case IDENTIFY_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk; // temporarily disable the SysTick interrupt
        green_led_action_counter = 0;
        n_identify_flashes = 30;
        SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; // reneable the interrupt
        print_debug_string("Identifying\n");
        rs485_transmit_no_error_packet(is_broadcast);
        break;
    case GET_TEMPERATURE_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        {
            struct __attribute__((__packed__)) {
                uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                int16_t temperature;
                uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
            } get_temperature_reply;
            if(!is_broadcast) {
                get_temperature_reply.temperature = get_temperature_degrees_C();
                rs485_finalize_and_transmit_packet(&get_temperature_reply, sizeof(get_temperature_reply));
            }
        }
        break;
    case SET_PID_CONSTANTS_COMMAND:
        {
            struct __attribute__((__packed__)) {
                uint32_t p;
                uint32_t i;
                uint32_t d;
            } pid_constants;
            copy_input_parameters_and_check_size(&pid_constants, payload, sizeof(pid_constants), payload_size);
            rs485_done_with_this_packet();
            char buf[100];
            sprintf(buf, "PID constants: " _PRIu32 ", " _PRIu32 ", " _PRIu32 "\n", pid_constants.p, pid_constants.i, pid_constants.d);
            print_debug_string(buf);
            set_pid_constants(pid_constants.p, pid_constants.i, pid_constants.d);
            rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
        }
        break;
    case SET_MAX_ALLOWABLE_POSITION_DEVIATION:
        {
            int64_t new_max_allowable_position_deviation;
            copy_input_parameters_and_check_size(&new_max_allowable_position_deviation, payload, sizeof(new_max_allowable_position_deviation), payload_size);
            rs485_done_with_this_packet();
            set_max_allowable_position_deviation(llabs(new_max_allowable_position_deviation));
            rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
        }
        break;
    case GET_DEBUG_VALUES_COMMAND:
        {
            check_payload_size_is_zero(payload_size);
            rs485_done_with_this_packet();
            if(!is_broadcast) {
                int64_t max_acceleration;
                int64_t max_velocity;
                int64_t current_velocity;
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

                get_motor_control_debug_values(&max_acceleration, &max_velocity, &current_velocity, &measured_velocity, &n_time_steps, &debug_value1, &debug_value2, &debug_value3, &debug_value4);
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
                    uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                    int64_t max_acceleration;
                    int64_t max_velocity;
                    int64_t current_velocity;
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
                    uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
                } get_debug_values_reply;
                get_debug_values_reply.max_acceleration = max_acceleration;
                get_debug_values_reply.max_velocity = max_velocity;
                get_debug_values_reply.current_velocity = current_velocity;
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
                rs485_finalize_and_transmit_packet(&get_debug_values_reply, sizeof(get_debug_values_reply));
            }
        }
        break;
    case CRC32_CONTROL_COMMAND:
        {
            uint8_t crc32_enable_new_state;
            copy_input_parameters_and_check_size(&crc32_enable_new_state, payload, sizeof(crc32_enable_new_state), payload_size);
            rs485_done_with_this_packet();
            rs485_set_crc32_enable(crc32_enable_new_state);
            rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
        }
        break;        
    case GET_COMMUNICATION_STATISTICS_COMMAND:
        {
            uint8_t reset_statistics;
            copy_input_parameters_and_check_size(&reset_statistics, payload, sizeof(reset_statistics), payload_size);
            rs485_done_with_this_packet();
            {            
                if(!is_broadcast) {
                    struct __attribute__((__packed__)) {
                        uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                        rs485_error_statistics_t statistics;
                        uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
                    } crc32_error_count_reply;
                    crc32_error_count_reply.statistics = rs485_get_error_statistics_and_optionally_reset(reset_statistics);
                    rs485_finalize_and_transmit_packet(&crc32_error_count_reply, sizeof(crc32_error_count_reply));
                }
            }
        }
        break;        
    default:
        rs485_done_with_this_packet();
        break;
    }

    if_fatal_error_then_dont_respond(); // we no longer need the fatal error logic to respond back with the fatal error code if we hit a fatal error, because the command was full processed and a response was sent back already
}


void transmit_detect_devices_response(void)
{
    struct __attribute__((__packed__)) {
        uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
        uint64_t unique_id;
        uint8_t alias;
        uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
    } detect_devices_reply;
    detect_devices_reply.unique_id = my_unique_id;
    detect_devices_reply.alias = global_settings.my_alias;
    rs485_finalize_and_transmit_packet(&detect_devices_reply, sizeof(detect_devices_reply));
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

#ifdef MOTOR_SIMULATION
// When building for the actual motor firmware, this is a no-op.
// When building the simulator, this initializes the motor simulation environment.
void motor_simulator_init(void);
#endif

#ifdef MOTOR_SIMULATION
int main_simulation(void)
#else
int main(void)
#endif
{   
    #ifndef MOTOR_SIMULATION
    clock_init();
    systick_init();
    #endif
    #if defined(PRODUCT_NAME_M17) && defined(GC6609)
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
    #if defined(PRODUCT_NAME_M17) && defined(GC6609)
    reset_GC6609();
    init_GC6609_through_UART();
    #endif

    SCB->VTOR = 0x2800; // vector table is moved to where the application starts, which is after the bootloader

    my_unique_id = get_unique_id();

    load_global_settings(); // load the settings from non-volatile memory into ram for faster access

    if(global_settings.hall1_midline == 0xffff) {
        global_settings.hall1_midline = DEFAULT_MIDLINE;
    }
    if(global_settings.hall2_midline == 0xffff) {
        global_settings.hall2_midline = DEFAULT_MIDLINE;
    }
    if(global_settings.hall3_midline == 0xffff) {
        global_settings.hall3_midline = DEFAULT_MIDLINE;
    }
    // The below adjustments to the midline are to handle the case where the calibration was done using an
    // older firmware and now the new firmware is being used without recalibrating. Thw way that midlines
    // and hall sensor readings are processed had been simplified in firmware version
    if(global_settings.hall1_midline >= 23300) {
        global_settings.hall1_midline = ((uint32_t)global_settings.hall1_midline + HALL_SENSOR_SHIFT) >> 3;
    }
    if(global_settings.hall2_midline >= 23300) {
        global_settings.hall2_midline = ((uint32_t)global_settings.hall2_midline + HALL_SENSOR_SHIFT) >> 3;
    }
    if(global_settings.hall3_midline >= 23300) {
        global_settings.hall3_midline = ((uint32_t)global_settings.hall3_midline + HALL_SENSOR_SHIFT) >> 3;
    }
    // end of adjustments to midlines because of firmware change

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
        #ifdef MOTOR_SIMULATION
        // When building for the actual motor firmware, this is a no-op.
        // When building the simulator, this runs the simulator's logic, timing, and visualization code.
        motor_simulator_visualization();
        #endif

        if(rs485_has_a_packet()) {
            if((detect_devices_delay_to_reenable_packet_processing >= 0) || (hall_sensor_n_points_to_capture > 0)) { // if a "Detect devices" has been issued then we will ignore all other commands until the delay is over and we send out the unique ID. Also, if we are capturing the hall sensor data then also we will ignore commands because if there are commands then that is wrong use of the protocol.
                rs485_done_with_this_packet();
            }
            else {
                process_packet();
                #ifdef MOTOR_SIMULATION
                extern volatile int gResetProgress;
                if (gResetProgress > 0) {
                    break;
                }
                #endif
            }
        }

        if(detect_devices_delay_to_answer == 0) {
            print_debug_string("Transmitting unique ID\n");
            transmit_detect_devices_response();
            detect_devices_delay_to_answer--;
        }

        if(hall_sensor_n_points_to_capture > 0) {
            captured_point_t captured_point;
            if(get_hall_sensor_captured_point(&captured_point, captured_point_division_factor)) {
                simulation_printf("hall_sensor_n_points_to_capture = %u   hall_sensor_point_size = %hhu   transmitting it\n", hall_sensor_n_points_to_capture, hall_sensor_point_size);
                rs485_continue_the_packet(&captured_point, hall_sensor_point_size);
                hall_sensor_n_points_to_capture--;
                if (hall_sensor_n_points_to_capture == 0) {
                    simulation_printf("Stopping the capture now\n");
                    start_or_stop_capture(0, 0, 0, 0);
                    rs485_end_the_packet();
                    print_debug_string("Capture finished\n");
                }
            }
        }

        if(process_calibration_data()) {
            disable_motor_control_loop();
            save_global_settings();
            print_debug_string("\nResetting\n\n");
            microsecond_delay(10000);
            NVIC_SystemReset();
            #ifdef MOTOR_SIMULATION
            break;
            #endif
        }

        if(is_fast_capture_data_result_ready()) {
            print_fast_capture_data_result();
        }

        #if !defined(CERTIFICATION_TEST_MODE) && !defined(PROGRAMMING_TEST_JIG_MODE)
        check_if_actual_vs_desired_position_deviated_too_much();
        #endif

#ifdef PRODUCT_NAME_M1
        process_go_to_closed_loop_data();
#endif

        process_debug_uart_commands();
        #ifdef MOTOR_SIMULATION
        extern volatile int gResetProgress;
        if (gResetProgress > 0) {
            break;
        }
        #endif
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

        #ifdef CERTIFICATION_TEST_MODE
        if (get_n_items_in_queue() == 0) {
            if (is_mosfets_enabled() == 0) {
                check_current_sensor_and_enable_mosfets();
            }
            add_trapezoid_move_to_queue(BUTTON_PRESS_MOTOR_MOVE_DISTANCE * 60 * 10, get_update_frequency() * 60 * 10);    
        }
        #endif
    }
    return 0;
}

#ifdef MOTOR_SIMULATION
void main_simulation_init(void)
{
    detect_devices_delay_to_answer = -1;
    detect_devices_delay_to_reenable_packet_processing = -1;
    green_led_action_counter = 0;
    n_identify_flashes = 0;

    hall_sensor_n_points_to_capture = 0;
    hall_sensor_point_size = 0;
    captured_point_division_factor = 1;
}
#endif
