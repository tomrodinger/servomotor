#include "stm32g0xx_hal.h"
#include <stdint.h>
#include <string.h>
#ifdef MOTOR_SIMULATION
#include <stdio.h>
#endif

#define TRANSMIT_BUFFER_SIZE 150
static char transmitBuffer[TRANSMIT_BUFFER_SIZE];
static volatile uint8_t transmitIndex = 0;
static volatile uint8_t transmitCount = 0;
static volatile uint8_t command_debug_uart = 0;
static uint8_t debug_printing_enabled = 1;


void debug_uart_init(void)
{
    RCC->APBENR1 |= RCC_APBENR1_USART2EN_Msk; // enable the clock to the UART2 peripheral
//    RCC->CCIPR |= 1 << RCC_CCIPR_USART2SEL_Pos; // select SYSCLK as the clock source

    GPIOA->AFR[0] |= (1 << GPIO_AFRL_AFSEL2_Pos) | // for PA2, choose alternative function 1 (USART2_TX)
                     (1 << GPIO_AFRL_AFSEL3_Pos);  // for PA3, choose alternative function 1 (USART2_RX)

    USART2->BRR = 278; // set baud to 115200 @ 64MHz SYSCLK
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_RXNEIE_RXFNEIE; // enable transmitter, receiver, and the receive interrupt
    NVIC_SetPriority(USART2_IRQn, 3); // lowest priority for the debug UART
    NVIC_EnableIRQ(USART2_IRQn);
}


void disable_or_enable_debug_printing(uint8_t enable)
{
    debug_printing_enabled = enable;
}


void USART2_IRQHandler(void)
{
    if(USART2->ISR & USART_ISR_RXNE_RXFNE) {
        uint8_t receivedByte;
        receivedByte = USART2->RDR;
        command_debug_uart = receivedByte;
    }

    while((USART2->ISR & USART_ISR_TXE_TXFNF_Msk) && (transmitCount > 0)) {
        USART2->TDR = transmitBuffer[transmitIndex];
        transmitCount--;
        transmitIndex++;
    }
    if(transmitCount == 0) {
        USART2->CR1 &= ~USART_CR1_TXEIE_TXFNFIE_Msk; // nothing more to transmit, so disable the interrupt
    }
}


void transmit(void *s, uint8_t len)
{
    #ifdef MOTOR_SIMULATION
    char buf[len + 1];
    memcpy(buf, s, len);
    buf[len] = 0;
    printf("%s", buf);
    return;

    #endif
    if (debug_printing_enabled == 0) {
        return;
    }
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

//    USART2->ICR = USART_ICR_TXFECF_Msk; // clear the transmit interrupt flag
//    USART2->TDR = transmitBuffer[0]; // start transmitting the first byte
                               // the rest of the transmission will be completed by the UART interrupt
}

// print a null terminated string to the debug port
void print_debug_string(void *s)
{
    #ifdef MOTOR_SIMULATION
    printf("%s", (char*)s);
    return;
    #endif

    if (debug_printing_enabled == 0) {
        return;
    }
    transmit(s, strlen(s));
}


void transmit_without_interrupts(const char *message, uint8_t len)
{
    if (debug_printing_enabled == 0) {
        return;
    }
    for(int32_t i = 0; i < len; i++) {
        while(!(USART2->ISR & USART_ISR_TXE_TXFNF_Msk));
        USART2->TDR = message[i];
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
    if (debug_printing_enabled == 0) {
        return;
    }
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
    if (debug_printing_enabled == 0) {
        return;
    }
    static char message[115];
    uint8_t message_prefix_len = strlen(message_prefix);
    strcpy(message, message_prefix);
    convert_int64_to_byte_array(message + message_prefix_len, n);
    message[message_prefix_len + N_DIGITS_INT64] = '\n';
    transmit(message, message_prefix_len + N_DIGITS_INT64 + 1);
}


uint8_t get_command_from_debug_uart(void)
{
    uint8_t tmp = command_debug_uart;
    if(tmp != 0) {
        command_debug_uart = 0;
    }
    return tmp;
}

