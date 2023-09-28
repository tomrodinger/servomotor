#include <stdio.h>
#include <string.h>
#include "stm32g0xx_hal.h"
#include "heater.h"
#include "debug_uart.h"
#include "adc.h"


#define HEATER_PORT_A_PIN 9
#define TARGET_HEATER_TEMPERATURE_INCREMENT 100

static uint8_t heater_enabled = 0;
static uint16_t target_heater_temperature = 0;
static uint32_t heating_counter = 0;


void heater_on(void) {
    GPIOA->ODR |= (1 << HEATER_PORT_A_PIN);
    // GPIOB->ODR |= (1 << HEATER_PORT_B_PIN);
}


void heater_off(void) {
    GPIOA->ODR &= ~(1 << HEATER_PORT_A_PIN);
    // GPIOB->ODR &= ~(1 << HEATER_PORT_B_PIN);
}


void set_heater_temperature(uint16_t new_target_heater_temperature)
{
    char buf[40];
    target_heater_temperature = new_target_heater_temperature;
    if(target_heater_temperature == 0) {
        heater_enabled = 0;
    }
    else {
        heater_enabled = 1;
    }
    sprintf(buf, "Temperature setting: %d\n", (int)target_heater_temperature);
    transmit(buf, strlen(buf));
}


uint16_t get_heater_temperature(void)
{
    uint16_t ADC_value = get_temperature_sensor_ADC_value();    
    return MAX_TEMPERATURE_SENSOR_ADC_VALUE - ADC_value;
}


void heater_logic(void)
{
    uint16_t measured_heater_temperature = get_heater_temperature();
    heating_counter = (heating_counter + 1) & 1023;
    if(heating_counter % 1023 != 0) {
        return;
    }
    // print_hall_position();
    // print_hall_sensor_value();
    // char buff[10];
    // sprintf(buff, "%d\n", measured_heater_temperature);
    // transmit(buff, strlen(buff));
    if((heater_enabled) && (measured_heater_temperature < target_heater_temperature)) {
        GPIOA->ODR |= (1 << HEATER_PORT_A_PIN); // turn on the heater
        // GPIOB->ODR |= (1 << HEATER_PORT_B_PIN);
    }
    else {
        GPIOA->ODR &= ~(1 << HEATER_PORT_A_PIN); // turn off the heater
        // GPIOB->ODR &= ~(1 << HEATER_PORT_B_PIN);
    }
}


void increase_temperature(void)
{
    char buf[40];
    if(target_heater_temperature < MAX_TEMPERATURE_SENSOR_ADC_VALUE - TARGET_HEATER_TEMPERATURE_INCREMENT) {
        target_heater_temperature += TARGET_HEATER_TEMPERATURE_INCREMENT;
    }
    heater_enabled = 1;
    sprintf(buf, "Temperature setting: %d\n", (int)target_heater_temperature);
    transmit(buf, strlen(buf));
}


void decrease_temperature(void)
{
    char buf[40];
    if(target_heater_temperature > TARGET_HEATER_TEMPERATURE_INCREMENT) {
        target_heater_temperature -= TARGET_HEATER_TEMPERATURE_INCREMENT;
    }
    else {
        target_heater_temperature = 0;
        heater_enabled = 0;
    }
    sprintf(buf, "Temperature setting: %d\n", (int)target_heater_temperature);
    transmit(buf, strlen(buf));
}


void print_heater_temperature(void)
{
	char buf[50];
	int16_t temperature = get_heater_temperature();
	sprintf(buf, "Heater temperature: %d\n", (int)temperature);
	transmit(buf, strlen(buf));
}
