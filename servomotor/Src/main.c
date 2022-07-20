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
#include "CommutationTable.h"
#include "hall_sensor_calculations.h"
#include "motor_control.h"
#include "microsecond_clock.h"
#include "clock_calibration.h"
#include "error_handling.h"
#include "step_direction_input.h"
#include "overvoltage.h"
#include "unique_id.h"
#include "settings.h"
#include "commands.h"
#include "product_info.h"
#include "LookupTableZ.h"
#include "global_variables.h"
#include "device_status.h"


char PRODUCT_DESCRIPTION[] = "Servomotor";

struct __attribute__((__packed__)) firmware_version_struct {
	uint8_t bugfix;
	uint8_t minor;
	uint8_t major;
	uint8_t not_used;
};
struct firmware_version_struct firmware_version = {0, 8, 0, 0};

#define BUTTON_PRESS_MOTOR_MOVE_DISTANCE (N_COMMUTATION_STEPS * N_COMMUTATION_SUB_STEPS * ONE_REVOLUTION_STEPS)

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
    #define TOUCH_BUTTON_PORT_A_PIN 15

    GPIOA->MODER =
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE0_Pos)  | // current measurement channel A
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE1_Pos)  | // MOSFET switch disable
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE2_Pos)  | // serial port TX
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE3_Pos)  | // serial port RX
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE4_Pos)  | // hall sensor 2
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE5_Pos)  | // hall sensor 1
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE6_Pos)  | // hall sensor 3
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE7_Pos)  | // current measurement channel B
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE8_Pos)  | // PWM 1
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE9_Pos)  |
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE10_Pos) | // PWM 3
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE11_Pos) | // PWM 4
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE12_Pos) | // RS485 drive enable
            (MODER_DIGITAL_INPUT      << GPIO_MODER_MODE13_Pos) | // Button input and also SWDIO (for programming)
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE14_Pos) | // SWCLK (for programming)
            (MODER_DIGITAL_INPUT      << GPIO_MODER_MODE15_Pos);  // touch button

    GPIOA->OTYPER = (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT0_Pos) | // make all the pins with analog components connected open drain
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT3_Pos) | // also, make the debug UART receive pin open drain
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT4_Pos) | // may not be necessary
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT5_Pos) |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT6_Pos) |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT7_Pos) |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT9_Pos) |
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
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE2_Pos)  |
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE3_Pos)  | // PWM 2
            (MODER_DIGITAL_INPUT      << GPIO_MODER_MODE4_Pos)  | // Encoder A input or step input
            (MODER_DIGITAL_INPUT      << GPIO_MODER_MODE5_Pos)  | // Encoder B input or direction input
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE6_Pos)  | // RS485 Data out
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE7_Pos)  | // RS485 Data receive
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE8_Pos)  |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE9_Pos)  |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE10_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE11_Pos) |
            (MODER_DIGITAL_INPUT      << GPIO_MODER_MODE12_Pos) | // overvoltage digital input
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE13_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE14_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE15_Pos);

    GPIOB->OTYPER = (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT4_Pos) | // step input make as open drain
    	            (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT5_Pos) | // direction input make as open drain
    		        (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT7_Pos);  // RX pin make as open drain
    GPIOB->OSPEEDR = 0xffffffff; // make all pins very high speed
    GPIOB->PUPDR = (PUPDR_PULL_UP << GPIO_PUPDR_PUPD7_Pos) | (PUPDR_PULL_DOWN << GPIO_PUPDR_PUPD12_Pos); // RX pin is pull up, overvoltage pin is pull down
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
//    int32_t end_position;
//    uint32_t end_time;
//    int32_t desired_position;
    uint32_t max_velocity;
    uint32_t max_acceleration;
    uint8_t capture_type;
    uint8_t n_items_in_queue;
    int32_t motor_position;
    int32_t acceleration;
    int32_t velocity;
    uint32_t time_steps;
    uint32_t frequency;
    uint64_t unique_id;
    uint8_t new_alias;
    int32_t trapezoid_move_displacement;
    uint32_t trapezoid_move_time;
    uint16_t new_maximum_motor_current;
    uint16_t new_maximum_motor_regen_current;
    uint8_t n_moves_in_this_command;
    int32_t max_homing_travel_displacement;
    uint32_t max_homing_time;
    int32_t lower_limit;
    int32_t upper_limit;
    add_to_queue_test_results_t add_to_queue_test_results;
    uint8_t ping_response_buffer[PING_PAYLOAD_SIZE + 3];
    uint8_t control_hall_sensor_statistics_subcommand;
    struct __attribute__((__packed__)) {
        uint8_t axis;
        uint8_t command;
        uint8_t size;
        hall_sensor_statistics_t hall_sensor_statistics;
    } hall_sensor_statistics_full_response;

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
            enable_mosfets();
            if(axis != ALL_ALIAS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case TRAPEZOID_MOVE_COMMAND:
            trapezoid_move_displacement = ((int32_t*)parameters)[0];
            trapezoid_move_time = ((uint32_t*)parameters)[1];
            rs485_allow_next_command();
            add_trapezoid_move_to_queue(trapezoid_move_displacement, trapezoid_move_time);
            if(axis != ALL_ALIAS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case SET_MAX_VELOCITY_COMMAND:
            max_velocity = *(uint32_t*)parameters;
            rs485_allow_next_command();
            set_max_velocity(max_velocity);
            if(axis != ALL_ALIAS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case SET_POSITION_AND_FINISH_TIME_COMMAND:
//            end_position = ((int32_t*)parameters)[0];
//            end_time = ((int32_t*)parameters)[1];
            rs485_allow_next_command();
//            add_to_queue(end_position, end_time);
            if(axis != ALL_ALIAS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case SET_MAX_ACCELERATION_COMMAND:
            max_acceleration = *(uint32_t*)parameters;
            rs485_allow_next_command();
            set_max_acceleration(max_acceleration);
            if(axis != ALL_ALIAS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case START_CALIBRATION_COMMAND:
            rs485_allow_next_command();
            enable_mosfets();
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
                rs485_transmit("R\x01\x06", 3);
                rs485_transmit(&local_time, 6);
            }
            break;
        case TIME_SYNC_COMMAND:
        	memcpy(&time_from_master, parameters, 6);
            rs485_allow_next_command();
        	int32_t time_error = time_sync(time_from_master);
        	uint16_t clock_calibration_value = get_clock_calibration_value();
            if(axis != ALL_ALIAS) {
                rs485_transmit("R\x01\x06", 3);
                rs485_transmit(&time_error, 4);
                rs485_transmit(&clock_calibration_value, 2);
            }
            break;
        case GET_N_ITEMS_IN_QUEUE_COMMAND:
            rs485_allow_next_command();
        	n_items_in_queue = get_n_items_in_queue();
            if(axis != ALL_ALIAS) {
                rs485_transmit("R\x01\x01", 3);
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
            zero_position_and_hall_sensor();
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
            transmit(buf, strlen(buf));
            start_homing(max_homing_travel_displacement, max_homing_time);
            if(axis != ALL_ALIAS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case GET_POSITION_COMMAND:
            rs485_allow_next_command();
            if(axis != ALL_ALIAS) {
            	motor_position = get_actual_motor_position();
                rs485_transmit("R\x01\x04", 3);
           		rs485_transmit(&motor_position, sizeof(motor_position));
            }
            break;
        case GET_STATUS_COMMAND:
            rs485_allow_next_command();
            if(axis != ALL_ALIAS) {
            	uint8_t motor_status_flags = get_motor_status_flags();
                set_device_status_flags(motor_status_flags);
                rs485_transmit(get_device_status(), sizeof(struct device_status_struct));
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
				rs485_transmit("R\x01\x04", 3);
				rs485_transmit(&frequency, 4);
			}
			break;
        case MOVE_WITH_ACCELERATION_COMMAND:
            acceleration = ((int32_t*)parameters)[0];
            time_steps = ((uint32_t*)parameters)[1];
            rs485_allow_next_command();
//            sprintf(buf, "move_with_acceleration: %ld %lu\n", acceleration, time_steps);
//            transmit(buf, strlen(buf));
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
               	transmit("Unique ID matches\n", 18);
        		global_settings.my_alias = new_alias;
           		save_global_settings();
                rs485_transmit(NO_ERROR_RESPONSE, 3);
        	}
        	break;
        case GET_PRODUCT_INFO_COMMAND:
            rs485_allow_next_command();
			if(axis != ALL_ALIAS) {
				rs485_transmit("R\x01", 2);
				uint8_t product_info_length = sizeof(struct product_info_struct);
                struct product_info_struct *product_info = (struct product_info_struct *)(PRODUCT_INFO_MEMORY_LOCATION);
				rs485_transmit(&product_info_length, 1);
				rs485_transmit(product_info, sizeof(struct product_info_struct));
			}
			break;
        case GET_PRODUCT_DESCRIPTION_COMMAND:
            rs485_allow_next_command();
			if(axis != ALL_ALIAS) {
				rs485_transmit("R\x01", 2);
				uint8_t product_description_length = sizeof(PRODUCT_DESCRIPTION);
				rs485_transmit(&product_description_length, 1);
				rs485_transmit(&PRODUCT_DESCRIPTION, sizeof(PRODUCT_DESCRIPTION));
			}
			break;
        case GET_FIRMWARE_VERSION_COMMAND:
            rs485_allow_next_command();
			if(axis != ALL_ALIAS) {
				rs485_transmit("R\x01", 2);
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
//            transmit(buf, strlen(buf));
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
            save_global_settings();
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
//            transmit(buf, strlen(buf));
            if(n_moves_in_this_command > MAX_MULTI_MOVES) {
                fatal_error(24);
            }
            for(uint8_t i = 0; i < n_moves_in_this_command; i++) {
                if((multi_move_command_buffer.move_type_bits & 1) == 0) {
                    add_to_queue(multi_move_command_buffer.move_parameters[i].acceleration, multi_move_command_buffer.move_parameters[i].time_steps, MOVE_WITH_ACCELERATION);
//                    sprintf(buf, "added_to_queue: acceleration: %ld  time_steps: %lu\n", multi_move_command_buffer.move_parameters[i].acceleration, multi_move_command_buffer.move_parameters[i].time_steps);
//                    transmit(buf, strlen(buf));
                } else {
                    add_to_queue(multi_move_command_buffer.move_parameters[i].velocity, multi_move_command_buffer.move_parameters[i].time_steps, MOVE_WITH_VELOCITY);
//                    sprintf(buf, "added_to_queue: velocity: %ld  time_steps: %lu\n", multi_move_command_buffer.move_parameters[i].velocity, multi_move_command_buffer.move_parameters[i].time_steps);
//                    transmit(buf, strlen(buf));
                }
                multi_move_command_buffer.move_type_bits >>= 1;
            }
			if(axis != ALL_ALIAS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
			}
			break;
        case SET_SAFETY_LIMITS_COMMAND:
            lower_limit = ((int32_t*)parameters)[0];
            upper_limit = ((uint32_t*)parameters)[1];
            rs485_allow_next_command();
            sprintf(buf, "movement limits %ld to %ld\n", lower_limit, upper_limit);
            transmit(buf, strlen(buf));
            set_movement_limits(lower_limit, upper_limit);
			if(axis != ALL_ALIAS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
			}
			break;
        case ADD_TO_QUEUE_TEST_COMMAND:
            add_to_queue_test(((int32_t*)parameters)[0], ((uint32_t*)parameters)[1], ((uint8_t*)parameters)[8], &add_to_queue_test_results);
            rs485_allow_next_command();
			if(axis != ALL_ALIAS) {
                rs485_transmit("R\x01\x10", 3);
                rs485_transmit(&add_to_queue_test_results, sizeof(add_to_queue_test_results));
			}
			break;
        case PING_COMMAND:
        	memcpy(ping_response_buffer + 3, parameters, PING_PAYLOAD_SIZE);
            rs485_allow_next_command();
            if(axis != ALL_ALIAS) {
                ping_response_buffer[0] = 'R';
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
            get_hall_sensor_statistics(&hall_sensor_statistics_full_response.hall_sensor_statistics);
            if(axis != ALL_ALIAS) {
                hall_sensor_statistics_full_response.axis = 'R';
                hall_sensor_statistics_full_response.command = '\x01';
                hall_sensor_statistics_full_response.size = sizeof(hall_sensor_statistics_t);
                rs485_transmit(&hall_sensor_statistics_full_response, sizeof(hall_sensor_statistics_full_response));
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
    uint32_t crc32 = 0x04030201;
    rs485_transmit("R\x01\x0d", 3);
    rs485_transmit(&my_unique_id, 8);
    rs485_transmit(&global_settings.my_alias, 1);
    rs485_transmit(&crc32, 4);
}


void process_debug_uart_commands(void)
{
    uint8_t command_debug_uart = get_command_debug_uart();

    if(command_debug_uart != 0) {
    	switch(command_debug_uart) {
    	case 'z':
    		zero_position_and_hall_sensor();
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
            transmit("\n", 1);
    		print_position();
    		print_hall_position();
    		print_queue_stats();
    		print_current_movement();
    		print_velocity();
    		print_time_difference();
            print_hall_sensor_data();
            print_hall_position_delta_stats();
            print_motor_status();
			break;
    	case 'P':
    		print_motor_current();
    		break;
    	case 'i':
    		increase_motor_pwm_voltage();
    		break;
    	case 'd':
    		decrease_motor_pwm_voltage();
    		break;
    	case 'e':
    		enable_mosfets();
			transmit("Enabling MOSFETs\n", 17);
    		break;
    	case 'E':
    		disable_mosfets();
			transmit("Disabling MOSFETs\n", 18);
    		break;
    	case 'o':
    		if(get_motor_control_mode() == CLOSED_LOOP_POSITION_CONTROL) {
    			set_motor_control_mode(OPEN_LOOP_POSITION_CONTROL);
				transmit("Open loop position control\n", 27);
    		}
    		else {
    			set_motor_control_mode(CLOSED_LOOP_POSITION_CONTROL);
				transmit("Closed loop position control\n", 29);
    		}
    		break;
    	case 'v':
    		if(get_motor_control_mode() == OPEN_LOOP_PWM_VOLTAGE_CONTROL) {
    			set_motor_control_mode(CLOSED_LOOP_POSITION_CONTROL);
				transmit("Closed loop position control\n", 29);
    		}
    		else {
    			set_motor_control_mode(OPEN_LOOP_PWM_VOLTAGE_CONTROL);
				transmit("Open loop PWM voltage control\n", 30);
    		}
    		break;
    	case 'h':
            start_homing(6451200, 620000);
    		break;
    	case 'H':
            start_homing(-6451200, 620000);
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
			start_go_to_closed_loop_mode();
		}
		else if(time_pressed_down >= 300) {
			enable_mosfets();
			add_trapezoid_move_to_queue(BUTTON_PRESS_MOTOR_MOVE_DISTANCE * 2, get_update_frequency() * 2);
		}
		else if(time_pressed_down >= 50) {
			enable_mosfets();
			add_trapezoid_move_to_queue(-BUTTON_PRESS_MOTOR_MOVE_DISTANCE * 2, get_update_frequency() * 2);
		}
    }
}

void print_start_message(void)
{
//	char buff[200];
	uint32_t my_unique_id_u32_array[2];

	memcpy(my_unique_id_u32_array, &my_unique_id, sizeof(my_unique_id));

    transmit("Applicaiton start\n", 18);
//    sprintf(buff, "Unique ID: 0x%08lX%08lX\n", my_unique_id_u32_array[1], my_unique_id_u32_array[0]);
//    transmit(buff, strlen(buff));
//    if((global_settings.my_alias >= 33) && (global_settings.my_alias <= 126)) {
//        sprintf(buff, "Alias: %c\n", global_settings.my_alias);
//    }
//    else {
//        sprintf(buff, "Alias: 0x%02hx\n", global_settings.my_alias);
//    }
//    transmit(buff, strlen(buff));
    print_hall_midlines();
    print_max_motor_current_settings();
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
    pwm_init();
    step_and_direction_init();
    overvoltage_init();

    SCB->VTOR = 0x2800; // vector table is moved to where the application starts, which is after the bootloader

    my_unique_id = get_unique_id();

    load_global_settings(); // load the settings from non-volatile memory into ram for faster access
    set_motor_current_baseline(); // make sure to call this before set_max_motor_current() and make sure to call it when the MOSFETs are disabled. at the start of the program is fine
    if(global_settings.max_motor_pwm_voltage == 0xffff) {
        global_settings.max_motor_pwm_voltage = DEFAULT_MAX_MOTOR_PWM_VOLTAGE;
    }
    if(global_settings.max_motor_regen_pwm_voltage == 0xffff) {
        global_settings.max_motor_regen_pwm_voltage = DEFAULT_MAX_MOTOR_REGEN_PWM_VOLTAGE;
    }
    set_max_motor_current(global_settings.max_motor_pwm_voltage, global_settings.max_motor_regen_pwm_voltage);

    __enable_irq();

    print_start_message();

//    rs485_transmit("Start\n", 6);

    while(1) {
//    	check_if_break_condition();
    	check_if_ADC_watchdog2_exceeded();

    	if(commandReceived) {
            processCommand(selectedAxis, command, valueBuffer);
        }

        if(detect_devices_delay == 0) {
            transmit("Transmitting unique ID\n", 23);
            transmit_unique_id();
            detect_devices_delay--;
    	}

        if(is_calibration_data_available()) {
            disable_motor_control_loop();
            process_calibration_data();
            transmit("Saving the settings\n", 20);
            save_global_settings();
            transmit("\nResetting\n\n", 12);
            microsecond_delay(10000);
            NVIC_SystemReset();
        }

        if(is_fast_capture_data_result_ready()) {
            print_fast_capture_data_result();
        }


        process_debug_uart_commands();
        button_logic();

        if((GPIOA->IDR & (1 << TOUCH_BUTTON_PORT_A_PIN)) == 0) {
            red_LED_on();
        }
        else {
            red_LED_off();
        }
    }
}
