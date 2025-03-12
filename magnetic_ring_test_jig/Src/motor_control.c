#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "stm32g0xx_hal.h"
#include "debug_uart.h"
#include "microsecond_clock.h"
#include "ADC.h"
#include "hall_sensor_calculations.h"
#include "error_handling.h"
#include "RS485.h"
#include "motor_control.h"
#include "leds.h"
#include "device_status.h"
#include "global_variables.h"
#include "PWM.h"


#define HALL_SENSOR_SHIFT 30000
#define HALL_SENSOR_CAPTURE_LENGTH_MULTIPLIER 1000 // # make sure that this value is the same as in the magnetic_ring_test_jig.py program

static uint32_t hall_sensor_capture_countdown = 0;
static uint32_t hall1_sum;
static uint32_t hall2_sum;
static uint32_t hall3_sum;
static uint8_t avg_counter;

// a packed structure to store the response with the hall sensor data
typedef struct __attribute__((packed)) {
	uint8_t alias;
	uint8_t command;
	uint8_t length;
	uint16_t hall1;
	uint16_t hall2;
	uint16_t hall3;
} hall_sensor_data_t;
static hall_sensor_data_t hall_sensor_data;

void start_calibration(uint8_t capture_length)
{
	if(capture_length == 0) {
		hall_sensor_capture_countdown = 0;
		return;
	}

	hall_sensor_data.alias = ENCODED_RESPONSE_CHARACTER;
	hall_sensor_data.command = '1';
	hall_sensor_data.length = 6;

	uint32_t new_hall_sensor_capture_countdown = (uint32_t)capture_length * HALL_SENSOR_CAPTURE_LENGTH_MULTIPLIER;

    TIM1->DIER &= ~TIM_DIER_UIE; // disable the update interrupt during this operation
	hall1_sum = 0;
	hall2_sum = 0;
	hall3_sum = 0;
	avg_counter = 0;
	hall_sensor_capture_countdown = new_hall_sensor_capture_countdown;
	TIM1->DIER |= TIM_DIER_UIE; // enable the update interrupt
}


void print_motor_temperature(void)
{
	char buf[100];
	int16_t temperature = get_temperature();
	sprintf(buf, "Temperature: %hd\n", temperature);
	print_debug_string(buf);
}


void print_hall_sensor_data(void)
{
	char buf[100];
	uint16_t hall1 = get_hall_sensor1_voltage();
	uint16_t hall2 = get_hall_sensor2_voltage();
	uint16_t hall3 = get_hall_sensor3_voltage();

	sprintf(buf, "hall1: %hu   hall2: %hu   hall3: %hu\n", hall1, hall2, hall3);
	print_debug_string(buf);
}


void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
    if(hall_sensor_capture_countdown > 0) {
		hall1_sum += get_hall_sensor1_voltage();
		hall2_sum += get_hall_sensor2_voltage();
		hall3_sum += get_hall_sensor3_voltage();
		avg_counter++;
		if(avg_counter == 32) {
			hall_sensor_data.hall1 = (hall1_sum >> 1) - HALL_SENSOR_SHIFT;
			hall_sensor_data.hall2 = (hall2_sum >> 1) - HALL_SENSOR_SHIFT;
			hall_sensor_data.hall3 = (hall3_sum >> 1) - HALL_SENSOR_SHIFT;
			rs485_transmit(&hall_sensor_data, sizeof(hall_sensor_data_t));
			avg_counter = 0;
			hall1_sum = 0;
			hall2_sum = 0;
			hall3_sum = 0;
			hall_sensor_capture_countdown--;
		}
    }
	TIM1->SR = 0; // clear the interrupt flag
}


uint32_t get_update_frequency(void)
{
	return PWM_FREQUENCY >> 1;
}


void reset_time(void)
{
    TIM1->DIER &= ~TIM_DIER_UIE; // disable the update interrupt during this operation
    reset_microsecond_time();
    TIM1->DIER |= TIM_DIER_UIE; // enable the update interrupt
}

