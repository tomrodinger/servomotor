#include "stm32g0xx_hal.h"
#include <string.h>
#include "RS485.h"
#include "error_handling.h"
#include "leds.h"
#include <stdlib.h>
#include <stdio.h>
#include "global_variables.h"

#define TRANSMIT_BUFFER_SIZE 150

static char transmitBuffer[TRANSMIT_BUFFER_SIZE];
static volatile uint8_t transmitIndex = 0;
static volatile uint8_t transmitCount = 0;
static uint16_t valueLength;
static uint16_t nReceivedBytes = 0;
static uint16_t receiveIndex;
volatile uint32_t USART1_timout_timer = 0;

char volatile selectedAxis;
uint8_t command;
uint8_t valueBuffer[MAX_VALUE_BUFFER_LENGTH];
volatile uint8_t commandReceived = 0;

void rs485_init(void)
{
    RCC->APBENR2 |= RCC_APBENR2_USART1EN_Msk; // enable the clock to the UART1 peripheral
    RCC->CCIPR |= 1 << RCC_CCIPR_USART1SEL_Pos; // select SYSCLK as the clock source
    
    GPIOB->AFR[0] |= (0 << GPIO_AFRL_AFSEL6_Pos) | // for PB6, choose alternative function 0 (USART1_TX)
                     (0 << GPIO_AFRL_AFSEL7_Pos);  // for PB7, choose alternative function 0 (USART1_RX)
    GPIOA->AFR[1] |= (1 << GPIO_AFRH_AFSEL12_Pos); // for PA12, choose alternative function 1 (USART1_DE)

    USART1->BRR = 278; // set baud to 115200 @ 64MHz SYSCLK
    USART1->CR1 = (0 << USART_CR1_DEAT_Pos) | (0 << USART_CR1_DEDT_Pos) | USART_CR1_FIFOEN | USART_CR1_RXNEIE_RXFNEIE; // set timing parameters for the drive enable, enable the FIFO mode, enable the receive interrupt
//    HAL_NVIC_SetPriority(USART1_IRQn, TICK_INT_PRIORITY, 0U); // DEBUG
//    USART1->CR2 = USART_CR2_RTOEN; // enable the timeout feature  (this is supported on USART1 but not supported on USART2)
    USART1->RTOR = (255 << USART_RTOR_RTO_Pos); // set the maximum timeout (255 bits)
    USART1->CR3 = (0 << USART_CR3_DEP_Pos) | USART_CR3_DEM; // drive enable is active high, enable the drive enable
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; // enable transmitter, receiver, and the uart
    NVIC_SetPriority(USART1_IRQn, 1); // pretty high priority but lower than the motor control interrupt
    NVIC_EnableIRQ(USART1_IRQn);
}


void USART1_IRQHandler(void)
{
    char message[100];

    if(USART1->ISR & USART_ISR_RXNE_RXFNE) {
//        if((USART1_timout_timer >= USART1_TIMEOUT) || (USART1->ISR & USART_ISR_RTOF)) {
//            nReceivedBytes = 0;
//            USART1->ICR |= USART_ICR_RTOCF; // clear the timeout flag
//        }
        if(USART1_timout_timer >= USART1_TIMEOUT) {
            nReceivedBytes = 0;
        }
        uint8_t receivedByte;
        receivedByte = USART1->RDR;
        if(nReceivedBytes < 65535) {
            nReceivedBytes++;
        }
        else {
            fatal_error("too many bytes", 1);
        }
        USART1_timout_timer = 0;
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
                }

                if(receiveIndex >= valueLength) {
                    if(selectedAxis != 'R' && (selectedAxis == my_alias || selectedAxis == 255)) {
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

    while((USART1->ISR & USART_ISR_TXE_TXFNF_Msk) && (transmitCount > 0)) {
    	red_LED_on();
        USART1->TDR = transmitBuffer[transmitIndex];
        transmitCount--;
        transmitIndex++;
    }

    if(transmitCount == 0) {
        USART1->CR1 &= ~USART_CR1_TXFEIE; // nothing more to transmit, so disable the interrupt
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

    while((USART1->ISR & USART_ISR_TXE_TXFNF_Msk) && (transmitCount > 0)) {
        USART1->TDR = transmitBuffer[transmitIndex];
        transmitCount--;
        transmitIndex++;
    }
    if(transmitCount > 0) {
        USART1->CR1 |= USART_CR1_TXFEIE; // we have more to transmit and the buffer must be full,
                                         // so enable the interrupt to handle the rest of the transmission
                                         // once the buffer becomes empty
    }
}
