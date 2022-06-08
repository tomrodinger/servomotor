#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32g0xx_hal.h"
#include "bed_heater_control.h"
#include "ADC.h"
#include "debug_uart.h"

static uint8_t bed_heater_enabled = 0;
static uint32_t bed_heater_resistance = 0;
static uint16_t desired_temperature_times_10 = 0;
static uint16_t actual_temperature_time_10 = 0;


static uint16_t calculate_bed_temperature_times_10(void)
{
    return 0;
}


void turn_on_or_off_bed_heater(uint8_t new_state)
{
    if(new_state) {
        // setting the pin low will enable the bed heater MOSFET
        // (the MOSFET driver take an inverted input control signal)
        GPIOA->BSRR = (1 << HEATER_CONTROL_PORT_A_PIN) << 16;
        bed_heater_enabled = 1;
    }
    else {
        // setting the pin low will enable the bed heater MOSFET
        // (the MOSFET driver take an inverted input control signal)
        GPIOA->BSRR = 1 << HEATER_CONTROL_PORT_A_PIN;
        bed_heater_enabled = 0;
    }
}


void set_bed_temperature(uint16_t new_temperature_times_10)
{
    desired_temperature_times_10 = new_temperature_times_10;
}


uint8_t get_device_status_flags(void)
{
    if(bed_heater_enabled) {
        return 1;
    }
    else {
        return 0;
    }
}


void bed_heater_calculations(uint16_t current, uint16_t voltage, uint16_t temperature)
{
    uint32_t resistance = (uint32_t)voltage * 65535 / current;
    bed_heater_resistance = resistance; 
}


void print_bed_current(void)
{
    char buf[40];
    uint16_t current = get_current_sense_value();
	sprintf(buf, "Bed heater current: %hu\n", current);
	transmit(buf, strlen(buf));
}


void print_bed_voltage(void)
{
    char buf[40];
    uint16_t voltage = get_24V_sense_value();
	sprintf(buf, "Bed heater voltage: %hu\n", voltage);
	transmit(buf, strlen(buf));
}


void print_ambient_temperature(void)
{
    char buf[40];
    uint16_t temperature = get_temperature_sense_value();
	sprintf(buf, "Ambient temperature: %hu\n", temperature);
	transmit(buf, strlen(buf));
}


void print_bed_resistance(void)
{
    char buf[50];
	sprintf(buf, "Bed heater resistance: %lu\n", bed_heater_resistance);
	transmit(buf, strlen(buf));
}


void print_bed_temperature(void)
{
    char buf[40];
	sprintf(buf, "Bed temperature: %hu\n", actual_temperature_time_10);
	transmit(buf, strlen(buf));
}

// This is the interrupt routine that is called each time the PWM timer (timer 1) overflows back to 0
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{

	TIM1->SR = 0; // clear the interrupt flag
}

