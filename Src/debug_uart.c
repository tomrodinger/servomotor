#include "stm32g0xx_hal.h"
#include <stdint.h>
#include <string.h>

#define TRANSMIT_BUFFER_SIZE 150
static char transmitBuffer[TRANSMIT_BUFFER_SIZE];
static volatile uint8_t transmitIndex = 0;
static volatile uint8_t transmitCount = 0;
static volatile uint8_t command_uart1 = 0;


void debug_uart_init(void)
{
    RCC->APBENR2 |= RCC_APBENR2_USART1EN_Msk; // enable the clock to the UART1 peripheral
    RCC->CCIPR |= 1 << RCC_CCIPR_USART1SEL_Pos; // select SYSCLK as the clock source
    USART1->BRR = 278; // set baud to 115200 @ 64MHz SYSCLK
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_FIFOEN |
                  USART_CR1_RXNEIE_RXFNEIE; // enable transmitter, receiver, FIFO mode
//    HAL_NVIC_SetPriority(USART1_IRQn, TICK_INT_PRIORITY, 0U); // DEBUG
    NVIC_SetPriority(USART1_IRQn, 3);
    NVIC_EnableIRQ(USART1_IRQn);
}


void USART1_IRQHandler(void)
{
    if(USART1->ISR & USART_ISR_RXNE_RXFNE) {
        uint8_t receivedByte;
        receivedByte = USART1->RDR;
        if((receivedByte == 'z') || (receivedByte == 'c') || (receivedByte == 'p') || (receivedByte == 'P') ||
           (receivedByte == 'i') || (receivedByte == 'd') || (receivedByte == 'e') || (receivedByte == 'E') ||
		   (receivedByte == 'o') || (receivedByte == 'v') || (receivedByte == 'h') || (receivedByte == 'H') ||
           (receivedByte == 'S') || (receivedByte == 'C') || (receivedByte == 'a') || (receivedByte == 'f')) {
            command_uart1 = receivedByte;
        }
    }

    while((USART1->ISR & USART_ISR_TXE_TXFNF_Msk) && (transmitCount > 0)) {
        USART1->TDR = transmitBuffer[transmitIndex];
        transmitCount--;
        transmitIndex++;
    }
    if(transmitCount == 0) {
        USART1->CR1 &= ~USART_CR1_TXFEIE; // nothing more to transmit, so disable the interrupt
    }
}


void transmit(void *s, uint8_t len)
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

//    USART1->ICR = USART_ICR_TXFECF_Msk; // clear the transmit interrupt flag
//    USART1->TDR = transmitBuffer[0]; // start transmitting the first byte
                               // the rest of the transmission will be completed by the UART interrupt
}

void transmit_without_interrupts(char *message, uint8_t len)
{
	int32_t i;
	for(i = 0; i < len; i++) {
		while(!(USART1->ISR & USART_ISR_TXE_TXFNF_Msk));
		USART1->TDR = message[i];
	}
}


// the output will be right justified and padded with spaces
// there is no terminator byte
void convert_uint16_to_byte_array(char *output, uint16_t input)
{
    static int8_t i;
    static int8_t leading_zero = 1;

    #define N_DIGITS 5 // a uint16_t has maximum 5 digits
    for(i = N_DIGITS - 1; i >= 0; i--) {
        if(input == 0) {
            if(leading_zero) {
                output[i] = '0';
            }
            else {
                output[i] = ' ';
            }
        }
        else {
            output[i] = input % 10 + '0';
            input /= 10;
        }
        leading_zero = 0;
    }
}


void print_number(char *message_prefix, uint16_t n)
{
    static char message[100];
    uint8_t message_prefix_len = strlen(message_prefix);
    strcpy(message, message_prefix);
    convert_uint16_to_byte_array(message + message_prefix_len, n);
    message[message_prefix_len + N_DIGITS] = '\n';
    transmit(message, message_prefix_len + N_DIGITS + 1);
}



// the output will be right justified and padded with spaces
// there is no terminator byte
void convert_int64_to_byte_array(char *output, int64_t input)
{
    int8_t i;
    int8_t leading_zero = 1;
    uint8_t negative = 0;
    
    if(input < 0) {
        negative = 1;
        input = -input;
    }

    #define N_DIGITS_INT64 20 // a int64_t has maximum 20 digits
    for(i = N_DIGITS_INT64 - 1; i >= 0; i--) {
        if(input == 0) {
            if(leading_zero) {
                output[i] = '0';
            }
            else if(negative) {
                output[i] = '-';
                negative = 0;
            }
            else {
                output[i] = ' ';
            }
        }
        else {
            output[i] = input % 10 + '0';
            input /= 10;
        }
        leading_zero = 0;
    }
}


void print_int64(char *message_prefix, int64_t n)
{
    static char message[115];
    uint8_t message_prefix_len = strlen(message_prefix);
    strcpy(message, message_prefix);
    convert_int64_to_byte_array(message + message_prefix_len, n);
    message[message_prefix_len + N_DIGITS_INT64] = '\n';
    transmit(message, message_prefix_len + N_DIGITS_INT64 + 1);
}


uint8_t get_command_uart1(void)
{
	uint8_t tmp = command_uart1;
	if(tmp != 0) {
		command_uart1 = 0;
	}
	return tmp;
}

