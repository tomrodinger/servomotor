#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "stm32g0xx_hal.h"
#include "../firmware/Src/ADC.h"
#include "leds.h"
#include "error_handling.h"
#include "debug_uart.h"
#include <math.h>
#include "motor_hal.h"
#include "hall_sensor_calculations.h"

#define ENCODER_ROTATIONS_PER_SHAFT_ROTATION (TOTAL_NUMBER_OF_SEGMENTS / N_HALL_SENSORS / 2)
#define SUPPLY_VOLTAGE_CALIBRATION_CONSTANT 23664
#define HALL_SENSOR_SINE_AMPLITUDE 3000
#define HALL_SENSOR_SINE_OFFSET 32768

#define MAX_UINT16 ((1 << 16) - 1)

// ADC buffer used by hall sensor calculations
uint16_t ADC_buffer[DMA_ADC_BUFFER_SIZE] = {0};

void adc_init(void)
{
    // Initialize ADC buffer with simulated values
    for(int i = 0; i < DMA_ADC_BUFFER_SIZE; i += ADC_CYCLE_INDEXES) {
        ADC_buffer[i + MOTOR_CURRENT_PHASE_A_CYCLE_INDEX] = 1350;  // Motor current
        ADC_buffer[i + HALL1_ADC_CYCLE_INDEX] = 32768;            // Hall 1
        ADC_buffer[i + SUPPLY_VOLTAGE_ADC_CYCLE_INDEX] = 65535;   // Supply voltage
        ADC_buffer[i + MOTOR_CURRENT_PHASE_B_CYCLE_INDEX] = 1350; // Motor current
        ADC_buffer[i + HALL2_ADC_CYCLE_INDEX] = 32768;           // Hall 2
        ADC_buffer[i + TEMPERATURE_ADC_CYCLE_INDEX] = 32768;     // Temperature
        ADC_buffer[i + 6] = 1350;                                // Motor current
        ADC_buffer[i + HALL3_ADC_CYCLE_INDEX] = 32768;          // Hall 3
    }
}


uint16_t get_hall_sensor1_voltage(void)
{
    double angleDeg = MotorHAL_GetPosition();
    double angleRad = angleDeg * M_PI / 180.0;
    double sensor = HALL_SENSOR_SINE_OFFSET + HALL_SENSOR_SINE_AMPLITUDE * sin(angleRad * ENCODER_ROTATIONS_PER_SHAFT_ROTATION);
    if(sensor < 0) sensor = 0;
    if(sensor > 65535) sensor = 65535;
    return (uint16_t)sensor;
}

uint16_t get_hall_sensor2_voltage(void)
{
    double angleDeg = MotorHAL_GetPosition();
    double angleRad = angleDeg * M_PI / 180.0;
    double sensor = HALL_SENSOR_SINE_OFFSET + HALL_SENSOR_SINE_AMPLITUDE * sin(angleRad * ENCODER_ROTATIONS_PER_SHAFT_ROTATION - (2.0 * M_PI / 3.0));
    if(sensor < 0) sensor = 0;
    if(sensor > 65535) sensor = 65535;
    return (uint16_t)sensor;
}

uint16_t get_hall_sensor3_voltage(void)
{
    double angleDeg = MotorHAL_GetPosition();
    double angleRad = angleDeg * M_PI / 180.0;
    double sensor = HALL_SENSOR_SINE_OFFSET + HALL_SENSOR_SINE_AMPLITUDE * sin(angleRad * ENCODER_ROTATIONS_PER_SHAFT_ROTATION - (4.0 * M_PI / 3.0));
    if(sensor < 0) sensor = 0;
    if(sensor > 65535) sensor = 65535;
    return (uint16_t)sensor;
}


uint16_t get_temperature_ADC_value(void)
{
    // Return value above OVERHEAT_TEMPERATURE_THRESHOLD_ADC_VALUE (11900)
    // to avoid triggering overheat protection
    return 16000;  // Safe temperature around 35°C
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


uint16_t get_supply_voltage_volts_times_10(void)
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

uint16_t get_motor_current(void)
{
    // Return simulated motor current around the expected baseline of 1350
    // This matches EXPECTED_MOTOR_CURRENT_BASELINE from motor_control.c
    return 1350;
}
 
// -----------------------------------------------------------------------------
// Simulate ADC conversion for hall sensor channels.
// This function recalculates the sinusoidal analog values for the three hall sensors,
// based on the current motor angle (in degrees) and writes those values into each ADC DMA buffer cycle.
// Each hall sensor value is written to its corresponding ADC buffer index.
void simulate_ADC_hall_sensor_values(void)
{
    uint16_t hall1 = get_hall_sensor1_voltage();
    uint16_t hall2 = get_hall_sensor2_voltage();
    uint16_t hall3 = get_hall_sensor3_voltage();
    for (int i = 0; i < DMA_ADC_BUFFER_SIZE; i += ADC_CYCLE_INDEXES) {
         ADC_buffer[i + HALL1_ADC_CYCLE_INDEX] = hall1;
         ADC_buffer[i + HALL2_ADC_CYCLE_INDEX] = hall2;
         ADC_buffer[i + HALL3_ADC_CYCLE_INDEX] = hall3;
    }
}
