#include "stm32g0xx_hal.h"
#include <string.h>
#include "RS485.h"
#include "error_handling.h"
#include "leds.h"

#define TRANSMIT_BUFFER_SIZE 150
#define MAX_VALUE_LENGTH 200 // Maximum length that the value can be in the entire protocol (all possible devices, present and future)

uint8_t my_alias = 'Z';
static char transmitBuffer[TRANSMIT_BUFFER_SIZE];
static volatile uint8_t transmitIndex = 0;
static volatile uint8_t transmitCount = 0;
static uint8_t receiveIndex = 0;
static uint8_t valueLength;
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
    NVIC_SetPriority(USART2_IRQn, 2);
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
    if(USART2->ISR & USART_ISR_RXNE_RXFNE) {
        if(USART2_timout_timer >= USART2_TIMEOUT) {
            receiveIndex = 0;
        }
        uint8_t receivedByte;
        receivedByte = USART2->RDR;
        USART2_timout_timer = 0;
        if(!commandReceived) {
            if(receiveIndex == 0) {
				selectedAxis = receivedByte;
				receiveIndex++;
            }
            else if(receiveIndex == 1) {
                command = receivedByte;
                receiveIndex++;
            }
            else {
                if(receiveIndex == 2) {
                    valueLength = receivedByte;
                    receiveIndex++;
                    if(valueLength > MAX_VALUE_LENGTH) {
                        receiveIndex = 0;
                    }
                }
                else if(receiveIndex - 3 < MAX_VALUE_BUFFER_LENGTH) {
                    valueBuffer[receiveIndex - 3] = receivedByte;
                    if(receiveIndex < 255) {
                        receiveIndex++;
                    }
                }
                if(receiveIndex - 3 >= valueLength) {
                    if(valueLength <= MAX_VALUE_BUFFER_LENGTH) {
                        commandReceived = 1;
                    }
                    receiveIndex = 0;
                }
            }
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

