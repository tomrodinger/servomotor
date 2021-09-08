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

#define DISABLE_MOSFETS_COMMAND 0
#define ENABLE_MOSFETS_COMMAND 1
#define SET_POSITION_AND_MOVE_COMMAND 2
#define SET_VELOCITY_COMMAND 3
#define SET_POSITION_AND_FINISH_TIME_COMMAND 4
#define SET_ACCELERATION_COMMAND 5
#define START_CALIBRATION_COMMAND 6
#define CAPTURE_HALL_SENSOR_DATA_COMMAND 7
#define RESET_TIME_COMMAND 8
#define GET_CURRENT_TIME_COMMAND 9
#define TIME_SYNC_COMMAND 10
#define GET_N_ITEMS_IN_QUEUE_COMMAND 11
#define EMERGENCY_STOP_COMMAND 12
#define ZERO_POSITION_COMMAND 13
#define HOMING_COMMAND 14
#define GET_POSITION_COMMAND 15
#define GET_STATUS_COMMAND 16
#define GO_TO_CLOSED_LOOP_COMMAND 17
#define NO_ERROR_RESPONSE "R\x00\x00"

#define BUTTON_PRESS_MOTOR_MOVE_DISTANCE (N_COMMUTATION_STEPS * N_COMMUTATION_SUB_STEPS * 7)

extern uint16_t ADC_buffer[DMA_ADC_BUFFER_SIZE];
extern uint32_t USART2_timout_timer;
extern char selectedAxis;
extern uint8_t command;
extern uint8_t valueBuffer[MAX_VALUE_BUFFER_LENGTH];
extern volatile uint8_t commandReceived;

void SysTick_Handler(void)
{
    green_LED_toggle();
    if(USART2_timout_timer < 2) {
    	USART2_timout_timer++;
    }
//	red_LED_off();
}

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
    SysTick->LOAD  = 16000000;                         /* set reload register */
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

    GPIOA->MODER =
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE0_Pos)  | // current measurement channel A
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE1_Pos)  | // RS485 drive enable
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE2_Pos)  | // RS485 Data out
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE3_Pos)  | // RS485 Data receive
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE4_Pos)  | // hall sensor 2
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE5_Pos)  | // hall sensor 1
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE6_Pos)  | // hall sensor 3
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE7_Pos)  | // current measurement channel B
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE8_Pos)  | // PWM 1
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE9_Pos)  |
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE10_Pos) | // PWM 3
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE11_Pos) | // PWM 4
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE12_Pos) | // MOSFET switch disable
            (MODER_DIGITAL_INPUT      << GPIO_MODER_MODE13_Pos) | // Button input and also SWDIO (for programming)
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE14_Pos) | // SWCLK (for programming)
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE15_Pos);  // Potentiometer input

    GPIOA->OTYPER = (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT0_Pos) | // make all the pins with analog components connected open drain
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT3_Pos) | // also, make the RS485 receive pin open drain
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT4_Pos) | // may not be necessary
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT5_Pos) |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT6_Pos) |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT7_Pos) |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT15_Pos);
    GPIOA->OSPEEDR = 0xffffffff; // make all pins very high speed
    GPIOA->PUPDR = (PUPDR_PULL_UP << GPIO_PUPDR_PUPD3_Pos); // apply pull up on the RS485 receive pin
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
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE6_Pos)  | // serial port TX
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE7_Pos)  | // serial port RX
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE8_Pos)  |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE9_Pos)  |
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
    GPIOB->PUPDR = (PUPDR_PULL_UP << GPIO_PUPDR_PUPD7_Pos); // no pull up or down except RX pin is pull up
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



void processCommand(uint8_t axis, uint8_t command, uint8_t *parameters)
{
    uint64_t local_time;
    uint64_t time_from_master = 0;
    int32_t end_position;
    uint32_t end_time;
    int32_t desired_position;
    int32_t max_velocity;
    uint16_t max_acceleration;
    uint8_t capture_type;
	uint8_t n_items_in_queue;
	int32_t motor_position;
	uint8_t buf[5];

//    print_number("Received a command with length: ", commandLen);
    if((axis == MY_AXIS) || (axis == ALL_AXIS)) {
//        print_number("Axis:", axis);
//        print_number("command:", command);
        switch(command) {
        case DISABLE_MOSFETS_COMMAND:
            disable_mosfets();
            if(axis != ALL_AXIS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case ENABLE_MOSFETS_COMMAND:
            enable_mosfets();
            if(axis != ALL_AXIS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case SET_POSITION_AND_MOVE_COMMAND:
            end_position = ((int32_t*)parameters)[0];
            local_time = get_microsecond_time();
            desired_position = get_desired_position();
            max_velocity = get_max_velocity();
            end_time = abs((int32_t)(end_position - desired_position)) / max_velocity + local_time;
            add_to_queue(end_position, end_time);
//            print_number("Set position: ", (uint16_t)desired_position);
            if(axis != ALL_AXIS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case SET_VELOCITY_COMMAND:
            max_velocity = *(int32_t*)parameters;
            if(max_velocity > MAX_VELOCITY) {
                max_velocity = MAX_VELOCITY;
            }
            set_max_velocity(max_velocity);
            if(axis != ALL_AXIS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case SET_POSITION_AND_FINISH_TIME_COMMAND:
            end_position = ((int32_t*)parameters)[0];
            end_time = ((int32_t*)parameters)[1];
            add_to_queue(end_position, end_time);
//            print_number("Set position: ", (uint16_t)desired_position);
            if(axis != ALL_AXIS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case SET_ACCELERATION_COMMAND:
            max_acceleration = *(uint16_t*)parameters;
            if(max_acceleration > MAX_ACCELERATION) {
            	max_acceleration = MAX_ACCELERATION;
            }
            set_max_acceleration(max_acceleration);
            if(axis != ALL_AXIS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case START_CALIBRATION_COMMAND:
            enable_mosfets();
            start_calibration(1);
            break;
        case CAPTURE_HALL_SENSOR_DATA_COMMAND:
        	capture_type = parameters[0];
            if(axis != ALL_AXIS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            start_capture(capture_type);
            break;
        case RESET_TIME_COMMAND:
        	reset_time();
            if(axis != ALL_AXIS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case GET_CURRENT_TIME_COMMAND:
            if(axis != ALL_AXIS) {
                local_time = get_microsecond_time();
                rs485_transmit("R\x01\x06", 3);
                rs485_transmit(&local_time, 6);
            }
            break;
        case TIME_SYNC_COMMAND:
        	memcpy(&time_from_master, parameters, 6);
        	int32_t time_error = time_sync(time_from_master);
        	uint16_t clock_calibration_value = get_clock_calibration_value();
            if(axis != ALL_AXIS) {
                rs485_transmit("R\x01\x06", 3);
                rs485_transmit(&time_error, 4);
                rs485_transmit(&clock_calibration_value, 2);
            }
            break;
        case GET_N_ITEMS_IN_QUEUE_COMMAND:
        	n_items_in_queue = get_n_items_in_queue();
            if(axis != ALL_AXIS) {
                rs485_transmit("R\x01\x01", 3);
                rs485_transmit(&n_items_in_queue, 1);
            }
            break;
        case EMERGENCY_STOP_COMMAND:
        	emergency_stop();
            if(axis != ALL_AXIS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case ZERO_POSITION_COMMAND:
            zero_position_and_hall_sensor();
			if(axis != ALL_AXIS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case HOMING_COMMAND:
            enable_mosfets();
//            int32_t max_homing_travel_displacement = ((int32_t*)parameters)[0];
            //start_homing(max_homing_travel_displacement);
            if(axis != ALL_AXIS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        case GET_POSITION_COMMAND:
            if(axis != ALL_AXIS) {
            	motor_position = get_actual_motor_position();
                rs485_transmit("R\x01\x04", 3);
           		rs485_transmit(&motor_position, sizeof(motor_position));
            }
            break;
        case GET_STATUS_COMMAND:
            if(axis != ALL_AXIS) {
            	get_motor_status(buf);
                rs485_transmit("R\x01\x05", 3);
        		rs485_transmit(buf, sizeof(buf));
            }
            break;
        case GO_TO_CLOSED_LOOP_COMMAND:
        	start_go_to_closed_loop_mode();
            if(axis != ALL_AXIS) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
            }
            break;
        }
    }
}


void process_debug_uart_commands(void)
{
    uint8_t command_uart1 = get_command_uart1();

    if(command_uart1 != 0) {
    	switch(command_uart1) {
    	case 'z':
    		zero_position_and_hall_sensor();
    		break;
    	case 'c':
			start_calibration(0);
			break;
    	case 'p':
    		print_position();
    		print_hall_position();
    		print_queue_stats();
    		print_current_movement();
    		print_velocity();
    		print_time_difference();
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
//    		start_homing(1000000000);
    		break;
    	case 'H':
//    		start_homing(-1000000000);
    		break;
		}
    	command_uart1 = 0;
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
			move_n_steps_in_m_time(-BUTTON_PRESS_MOTOR_MOVE_DISTANCE * 2, 5000000);
		}
		else if(time_pressed_down >= 50) {
			enable_mosfets();
			move_n_steps_in_m_time(BUTTON_PRESS_MOTOR_MOVE_DISTANCE * 2, 5000000);
		}
    }
}


int main(void)
{
//	volatile int i;
//	char buff[200];

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


    __enable_irq();

    transmit("Program Start\n", 14);
    rs485_transmit("Start\n", 6);

    while(1) {
//    	check_if_break_condition();
//    	check_if_ADC_watchdog2_exceeded();

    	if(commandReceived) {
            processCommand(selectedAxis, command, valueBuffer);
            commandReceived = 0;
        }

        process_debug_uart_commands();
        button_logic();
    }
}
