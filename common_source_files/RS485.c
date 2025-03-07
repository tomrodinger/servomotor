#include "stm32g0xx_hal.h"
#include <string.h>
#include "RS485.h"
#include "error_handling.h"
#include "leds.h"
#include <stdlib.h>
#include <stdio.h>
#include "global_variables.h"
#include "device_status.h"

static volatile char transmitBuffer[TRANSMIT_BUFFER_SIZE];
static volatile uint8_t transmitIndex = 0;
static volatile uint8_t transmitCount = 0;

static uint16_t valueLength;
static uint16_t nReceivedBytes = 0;
static uint16_t receiveIndex;

volatile uint8_t selectedAxis;
volatile uint8_t command;
volatile uint8_t valueBuffer[MAX_VALUE_BUFFER_LENGTH];
volatile uint8_t commandReceived = 0;

void rs485_init(void)
{
    commandReceived = 0;
    RCC->APBENR2 |= RCC_APBENR2_USART1EN_Msk; // enable the clock to the UART1 peripheral
    RCC->CCIPR |= 1 << RCC_CCIPR_USART1SEL_Pos; // select SYSCLK as the clock source
    
    GPIOB->AFR[0] |= (0 << GPIO_AFRL_AFSEL6_Pos) | // for PB6, choose alternative function 0 (USART1_TX)
                     (0 << GPIO_AFRL_AFSEL7_Pos);  // for PB7, choose alternative function 0 (USART1_RX)
    GPIOA->AFR[1] |= (1 << GPIO_AFRH_AFSEL12_Pos); // for PA12, choose alternative function 1 (USART1_DE)

    USART1->BRR = 278; // set baud to 230400 @ 64MHz SYSCLK
    USART1->CR1 = (0 << USART_CR1_DEAT_Pos) | (0 << USART_CR1_DEDT_Pos) | USART_CR1_FIFOEN | USART_CR1_RXNEIE_RXFNEIE; // set timing parameters for the drive enable, enable the FIFO mode, enable the receive interrupt
    USART1->CR2 = USART_CR2_RTOEN; // enable the timeout feature  (this is supported on USART1 but not supported on USART2)
    USART1->RTOR = ((230400 / 10) << USART_RTOR_RTO_Pos); // set the timeout t0 0.1 s)
    USART1->CR3 = (0 << USART_CR3_DEP_Pos) | USART_CR3_DEM | USART_CR3_EIE; // drive enable is active high, enable the drive enable, an interrupt will happen if there is an error (like overrun or framing error or noise error), 
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; // enable transmitter, receiver, and the uart
    NVIC_SetPriority(USART1_IRQn, 2); // third highest priority. needs to be lower than the motor control interrupt (PWM.c) but otherwise this is more important than things like systick
    NVIC_EnableIRQ(USART1_IRQn);
}

void rs485_allow_next_command(void)
{
    commandReceived = 0;
    USART1->CR1 |= USART_CR1_RXNEIE_RXFNEIE; // enable receive interrupt
}

void USART1_IRQHandler(void)
{
    // check for errors like framing error, overrun error, and noise error
    if(USART1->ISR & USART_ISR_FE) {
        fatal_error(ERROR_FRAMING); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
    }
    if(USART1->ISR & USART_ISR_ORE) {
        fatal_error(ERROR_OVERRUN); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
    }
    if(USART1->ISR & USART_ISR_NE) {
        fatal_error(ERROR_NOISE); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
    }

    if((USART1->ISR & USART_ISR_RXNE_RXFNE) && (USART1->CR1 & USART_CR1_RXNEIE_RXFNEIE)) {
        if(USART1->ISR & USART_ISR_RTOF) {
            nReceivedBytes = 0;
            USART1->ICR |= USART_ICR_RTOCF; // clear the timeout flag
        }
        uint8_t receivedByte;
        receivedByte = USART1->RDR;
        #ifdef MOTOR_SIMULATION
        USART1->ISR &= ~USART_ISR_RXNE_RXFNE; // clear this bit to indicate that we have read the received byte (done automatically in the real hardware but not in the simulated hardware)
        #endif
        if(nReceivedBytes < 65535) {
            nReceivedBytes++;
        }
        else {
            fatal_error(ERROR_TOO_MANY_BYTES); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
        }
        if(commandReceived) {
            fatal_error(ERROR_COMMAND_OVERFLOW); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
        }
        if(nReceivedBytes == 1) {
            selectedAxis = receivedByte;
        }
        else if(nReceivedBytes == 2) {
            command = receivedByte;
        }
        else {
            if(nReceivedBytes == 3) {
                valueLength = receivedByte;
                receiveIndex = 0;
            }
            else if((nReceivedBytes == 5) && (valueLength == 255)) {
                valueLength = (receivedByte << 8) + valueBuffer[0];
                receiveIndex = 0;
            }
            else if(receiveIndex < MAX_VALUE_BUFFER_LENGTH) {
                valueBuffer[receiveIndex++] = receivedByte;
            }

            if(receiveIndex >= valueLength) {
                if(selectedAxis != RESPONSE_CHARACTER && (selectedAxis == global_settings.my_alias || selectedAxis == 255)) {
                    if(valueLength <= MAX_VALUE_BUFFER_LENGTH) {
                        USART1->CR1 &= ~USART_CR1_RXNEIE_RXFNEIE; // disable receive interrupt until this command is processed
                                                                    // in the mean time, received bytes will be put in the hardware fifo
                        commandReceived = 1;
                    }
                    else {
                        fatal_error(ERROR_COMMAND_TOO_LONG); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
                    }
                }
                nReceivedBytes = 0;
            }
        }
    }

    if(nReceivedBytes == 0) {
        red_LED_off();
    }
    else {
        red_LED_on();
    }

    while((USART1->ISR & USART_ISR_TXE_TXFNF_Msk) && (transmitCount > 0)) {
        USART1->TDR = transmitBuffer[transmitIndex];
#ifdef MOTOR_SIMULATION
        // In simulation, we need to clear TXE after writing to TDR
        // (In real hardware, this is done by the USART hardware)
        USART1->ISR &= ~USART_ISR_TXE_TXFNF_Msk;
#endif
        transmitCount--;
        transmitIndex++;
    }

    if(transmitCount == 0) {
        USART1->CR1 &= ~USART_CR1_TXFEIE; // nothing more to transmit, so disable the interrupt
    }
}


void rs485_transmit(void *s, uint8_t len)
{
    if(len == 0) {
        return;
    }
    if(len > TRANSMIT_BUFFER_SIZE) {
        len = TRANSMIT_BUFFER_SIZE;
    }
    while(transmitCount > 0); // wait for previous transmission to finish
    memcpy((void*)transmitBuffer, s, len); // we discard the volatile qualifier here. i am not actually sure if this is safe or not.
    transmitIndex = 0;
    transmitCount = len;

    while((USART1->ISR & USART_ISR_TXE_TXFNF_Msk) && (transmitCount > 0)) {
        USART1->TDR = transmitBuffer[transmitIndex];
#ifdef MOTOR_SIMULATION
        // In simulation, we need to clear TXE after writing to TDR
        // (In real hardware, this is done by the USART hardware)
        USART1->ISR &= ~USART_ISR_TXE_TXFNF_Msk;
#endif
        transmitCount--;
        transmitIndex++;
    }
    if(transmitCount > 0) {
        USART1->CR1 |= USART_CR1_TXFEIE; // we have more to transmit and the buffer must be full,
                                         // so enable the interrupt to handle the rest of the transmission
                                         // once the buffer becomes empty
    }
}

void rs485_wait_for_transmit_done(void)
{
    while(transmitCount > 0); // wait for previous transmission to finish
}

#ifdef MOTOR_SIMULATION
/**
 * Initialize all static variables in RS485.c for simulator use
 * This function should be called when the simulator starts to ensure
 * all static variables are properly initialized
 */
void RS485_simulator_init(void)
{
    printf("RS485_simulator_init() called\n");
    
    // Initialize transmit buffer variables
    transmitIndex = 0;
    transmitCount = 0;
    
    // Initialize static variables that were moved from USART1_IRQHandler
    valueLength = 0;
    nReceivedBytes = 0;
    receiveIndex = 0;
    
    // Initialize command handling variables
    commandReceived = 0;
    selectedAxis = 0;
    command = 0;
    
    // Clear value buffer
    for (int i = 0; i < MAX_VALUE_BUFFER_LENGTH; i++) {
        valueBuffer[i] = 0;
    }
    
    printf("RS485 module reset complete\n");
}

void print_nReceivedBytes(void)
{
    printf("nReceivedBytes = %hu\n", nReceivedBytes);
}

#endif
