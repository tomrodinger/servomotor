#include "stm32g0xx_hal.h"
#include <string.h>
#include "RS485.h"
#include "error_handling.h"
#include "leds.h"
#include <stdlib.h>
#include <stdio.h>

#define TRANSMIT_BUFFER_SIZE 150

uint8_t my_alias = 255;
static char transmitBuffer[TRANSMIT_BUFFER_SIZE];
static volatile uint8_t transmitIndex = 0;
static volatile uint8_t transmitCount = 0;
static uint16_t valueLength;
static uint16_t nReceivedBytes = 0;
static uint16_t receiveIndex;
volatile uint32_t USART2_timout_timer = 0;

char volatile selectedAxis;
uint8_t command;
uint8_t valueBuffer[MAX_VALUE_BUFFER_LENGTH];
volatile uint8_t commandReceived = 0;

void rs485_init(void)
{
    RCC->APBENR1 |= RCC_APBENR1_USART2EN_Msk; // enable the clock to the UART2 peripheral
//    RCC->CCIPR |= 1 << RCC_CCIPR_USART2SEL_Pos; // select SYSCLK as the clock source
    GPIOA->AFR[0] = (1 << GPIO_AFRL_AFSEL1_Pos) | (1 << GPIO_AFRL_AFSEL2_Pos) | (1 << GPIO_AFRL_AFSEL3_Pos); // choose the right alternate function to put on these pins
    USART2->BRR = 278; // set baud to 115200 @ 64MHz SYSCLK
    USART2->CR1 = (0 << USART_CR1_DEAT_Pos) | (0 << USART_CR1_DEDT_Pos) | USART_CR1_RXNEIE_RXFNEIE; // set timing parameters for the drive enable, enable the receive interrupt
//    HAL_NVIC_SetPriority(USART2_IRQn, TICK_INT_PRIORITY, 0U); // DEBUG
//    USART2->CR2 = USART_CR2_RTOEN; // enable the timeout feature  (this is not supported on UART2)
    USART2->CR3 = (0 << USART_CR3_DEP_Pos) | USART_CR3_DEM; // drive enable is active high, enable the drive enable
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; // enable transmitter, receiver, and the uart
    NVIC_SetPriority(USART2_IRQn, 0);
    NVIC_EnableIRQ(USART2_IRQn);
}


void rs485_drive_enable(void)
{
    GPIOA->BSRR = 1 << 1;
}


void rs485_drive_disable(void)
{
    GPIOA->BSRR = (1 << 1) << 16;
}



void USART2_IRQHandler(void)
{
    char message[100];

    if(USART2->ISR & USART_ISR_RXNE_RXFNE) {
        if(USART2_timout_timer >= USART2_TIMEOUT) {
            nReceivedBytes = 0;
        }
        uint8_t receivedByte;
        receivedByte = USART2->RDR;
        if(nReceivedBytes < 65535) {
            nReceivedBytes++;
        }
        else {
            fatal_error("too many bytes", 1);
        }
        USART2_timout_timer = 0;
        if(!commandReceived) {
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
                else {
                    if(receiveIndex < MAX_VALUE_BUFFER_LENGTH) {
                        valueBuffer[receiveIndex] = receivedByte;
                    }
                    else {
                        sprintf(message, "full: %hu %hu %u %hu %hu", selectedAxis, command, (unsigned int)receiveIndex, valueBuffer[receiveIndex-2], valueBuffer[receiveIndex-1]);
                        fatal_error(message, 1);
                    }
                    receiveIndex++;
//                    if(receiveIndex == 1281) {
//                        sprintf(message, "recd: %hu %hu %u %hu %hu", selectedAxis, command, (unsigned int)receiveIndex, valueBuffer[receiveIndex-2], valueBuffer[receiveIndex-1]);
//                        fatal_error(message, 1);
//                    }
                }

                if(receiveIndex >= valueLength) {
                    if(selectedAxis != 'R' && selectedAxis == my_alias) {
                        if(valueLength <= MAX_VALUE_BUFFER_LENGTH) {
                            commandReceived = 1;
                        }
                        else {
                            sprintf(message, "too long: %hu %hu %u %hu %hu", selectedAxis, command, (unsigned int)receiveIndex, valueBuffer[receiveIndex-2], valueBuffer[receiveIndex-1]);
                            fatal_error(message, 1);
                        }
                    }
                    nReceivedBytes = 0;
                }
            }
        }
        else {
            fatal_error("command overflow", 5);
        }
    }

    while((USART2->ISR & USART_CR1_TXEIE_TXFNFIE_Msk) && (transmitCount > 0)) {
    	red_LED_on();
        USART2->TDR = transmitBuffer[transmitIndex];
        transmitCount--;
        transmitIndex++;
    }

    if(transmitCount == 0) {
        USART2->CR1 &= ~USART_CR1_TXEIE_TXFNFIE_Msk; // nothing more to transmit, so disable the interrupt
//        if(USART2->ISR & USART_ISR_TC) {
//            USART2->CR1 &= ~USART_CR1_TCIE; // disable the interrupt for "transmission complete" and
//            rs485_drive_disable();          // disable the RS485 drive
//        }
    }

    red_LED_off();
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
    memcpy(transmitBuffer, s, len);
    transmitIndex = 0;
    transmitCount = len;

//    rs485_drive_enable();
//    USART2->CR1 |= USART_CR1_TCIE; // enable the interrupt for "transmission complete". we will use it to disable the RS485 drive enable.

    while((USART2->ISR & USART_ISR_TXE_TXFNF_Msk) && (transmitCount > 0)) {
        USART2->TDR = transmitBuffer[transmitIndex];
        transmitCount--;
        transmitIndex++;
    }
    if(transmitCount > 0) {
        USART2->CR1 |= USART_CR1_TXEIE_TXFNFIE_Msk; // we have more to transmit and the buffer must be full,
                                                    // so enable the interrupt to handle the rest of the transmission
                                                    // once the buffer becomes empty
    }
}
