#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "stm32g0xx_hal.h"
#include "ADC.h"
#include "leds.h"
#include "error_handling.h"
#include "debug_uart.h"

#define SUPPLY_VOLTAGE_CALIBRATION_CONSTANT 23664
#define EXPECTED_MOTOR_CURRENT_BASELINE 1152

#define MAX_UINT16 ((1 << 16) - 1)

uint16_t hysteretic_motor_current = 0;

void adc_init(void)
{
}


uint16_t get_hall_sensor1_voltage(void)
{
	uint16_t a = 0 * 4;
	return a;
}

uint16_t get_hall_sensor2_voltage(void)
{
	uint16_t a = 0 * 4;
	return a;
}

uint16_t get_hall_sensor3_voltage(void)
{
	uint16_t a = 0 * 4;
	return a;
}


uint16_t get_temperature_ADC_value(void)
{
	uint16_t a = 0 * 4;
	return a;
}

uint16_t get_supply_voltage_ADC_value(void)
{
	double motor_supply_voltage = 30.0;
	double reference_voltage = 3.3;
	double voltage_divider_R1 = 18000;
	double voltage_divider_R2 = 1500;
	double voltage_divider_gain = voltage_divider_R2 / (voltage_divider_R1 + voltage_divider_R2);
	uint32_t ADC_bits = 12;
	double ADC_value = motor_supply_voltage * voltage_divider_gain / reference_voltage * (1 << ADC_bits) * 4;
	if (ADC_value > MAX_UINT16) {
		printf("ERROR: ADC_value exceeded maximum value\n");
		exit(1);
	}
//	printf("ADC value: %lf\n", ADC_value);
	return (uint16_t)(ADC_value);
}


uint16_t get_supply_voltage_volts_time_10(void)
{
	uint16_t supply_voltage = get_supply_voltage_ADC_value();
	uint32_t supply_voltage_calibrated = (supply_voltage * SUPPLY_VOLTAGE_CALIBRATION_CONSTANT) >> 20;

	return (uint16_t)supply_voltage_calibrated;
}


void print_supply_voltage(void)
{
	char buf[100];
	int16_t supply_voltage = get_supply_voltage_ADC_value();
	sprintf(buf, "Supply voltage (ADC value): %hd\n", supply_voltage);
	print_debug_string(buf);
	int32_t supply_voltage_calibrated = (supply_voltage * SUPPLY_VOLTAGE_CALIBRATION_CONSTANT) >> 20;
	int16_t whole_number = supply_voltage_calibrated / 10;
	int16_t decimal = (supply_voltage_calibrated % 10);
	sprintf(buf, "Supply voltage: %hd.%hu\n", whole_number, decimal);
	print_debug_string(buf);
}


void set_analog_watchdog_limits(uint16_t lower_limit, uint16_t upper_limit)
{
}


void turn_on_mosfet_for_first_time(void)
{
}


void set_hysteretic_motor_current(uint16_t new_hysteretic_motor_current)
{
	hysteretic_motor_current = new_hysteretic_motor_current;
}


uint16_t get_motor_current(void)
{
	return EXPECTED_MOTOR_CURRENT_BASELINE;
}

void set_hysteretic_motor_current_to_off(void)
{
}
