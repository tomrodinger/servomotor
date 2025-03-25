#include "stm32g0xx_hal.h"
#include <string.h>
#include <config.h>
#include "RS485.h"
#include "error_handling.h"
#include "leds.h"
#include <stdlib.h>
#include <stdio.h>
#include "global_variables.h"
#include "device_status.h"
#include "unique_id.h"
#include "crc32.h"

static volatile char transmitBuffer[TRANSMIT_BUFFER_SIZE];
static volatile uint8_t transmitIndex;
static volatile uint8_t transmitCount;

// The receive buffer will hold the following:
// 1. encoded size of the entire packet (maximum 3 bytes but usually 1 byte)
// 2. address of the device being addressed (maximum 9 bytes but usually 1 byte)
// 3. command (1 byte)
// 4. value buffer (maximum MAX_VALUE_BUFFER_LENGTH bytes, which is different in the main firmware vs. the bootloader and is defined in the respective config.h file)
// 5. CRC32 (4 bytes or if disabled then 0 bytes)
// The maximum size for all of this therefore is as follows:
#define MAX_RECEIVE_BUFFER_SIZE (3 + 9 + 1 + MAX_VALUE_BUFFER_LENGTH + 4)
static volatile uint8_t receiveBuffers[N_RECEIVE_BUFFERS][MAX_RECEIVE_BUFFER_SIZE];
static volatile uint8_t receiveBufferReadPosition;
static volatile uint8_t receiveBufferWritePosition;
static volatile uint8_t receiveBuffersUsed;
static volatile uint16_t receiveIndex;
static volatile uint8_t firstByteInvalid;
static volatile uint16_t receivePacketSize;
static volatile uint8_t receiveBufferOverflow;

#define UNKNOWN_VALUE_LENGTH 65535

#if 0
static uint16_t nReceivedBytes = 0;
volatile uint8_t selectedAxis;
volatile uint8_t command;
volatile uint8_t valueBuffer[MAX_VALUE_BUFFER_LENGTH];
volatile uint16_t valueLength;
volatile uint8_t commandReceived = 0;
volatile uint8_t in_extended_addressing_mode = 0;
static volatile uint8_t unique_id_mismatch = 0; // Flag to track if unique ID doesn't match (0 = match so far)
static volatile uint8_t unique_id_bytes_received = 0;
#endif


// This function will be called when the device first starts and it will also be called
// if a fatal error occurs so as to make sure tha the UART is reinitialized and working
// even if something abnormal happed that caused the fatal error
void rs485_init(void)
{
    // reinitialize all these variables related to 
    transmitIndex = 0;
    transmitCount = 0;
    receiveBufferReadPosition = 0;
    receiveBufferWritePosition = 0;
    receiveBuffersUsed = 0;
    receiveIndex = 0;
    firstByteInvalid = 0;
    receivePacketSize = 0;
    receiveBufferOverflow = 0;

    //commandReceived = 0;
    //unique_id_mismatch = 0; // Initialize to no mismatch (will be set to 1 if any byte doesn't match)
    //unique_id_bytes_received = 0;
    
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

uint8_t rs485_has_a_packet(void)
{
    return receiveBuffersUsed > 0;
}

uint8_t rs485_get_next_packet(uint8_t *command, uint16_t *payload_size, uint8_t **payload, uint8_t *is_broadcast)
{
    if (receiveBuffersUsed == 0) {
        return 0; // nothing in the buffer
    }
    uint16_t packet_size = decode_first_byte(receiveBuffers[receiveBufferReadPosition][0]);
    uint8_t parsing_index = 1;
    if (packet_size == DECODED_FIRST_BYTE_EXTENDED_SIZE) {
        packet_size = ((uint16_t)receiveBuffers[receiveBufferReadPosition][parsing_index + 1] << 8) | receiveBuffers[receiveBufferReadPosition][parsing_index];
        parsing_index += 2;
    }
    uint8_t deviceID = receiveBuffers[receiveBufferReadPosition][parsing_index];
    if (deviceID == RESPONSE_CHARACTER) { 
        return 0; // discard a packet that is from some other device that is responding to a command
    }
    parsing_index++;
    if (deviceID == ALL_ALIAS) { // this is a broadcast packet and so it it valid for us
        *is_broadcast = 1;
    }
    else {
        *is_broadcast = 0;
        if (deviceID == EXTENDED_ADDRESSING) {
            uint64_t our_device_id = get_unique_id();
            uint64_t *received_device_id = (uint64_t *)&receiveBuffers[receiveBufferReadPosition][parsing_index];
            if (*received_device_id != our_device_id) {
                return 0; // extended addressing was used but the unique ID did not match and so this packet is not for us
            }
            parsing_index += UNIQUE_ID_SIZE;
        }
        else {
            if (deviceID != global_settings.my_alias) {
                return 0; // normal address was used but the alias did not match ours and so this packet is not for us
            }
        }
    }
    parsing_index++;
    if (packet_size < parsing_index + 1) { // add an extra required byte for the command 
        return 0; // this packet is invalid because the indicated size of the packet is too small to hold all the information needed to properly decode it
    }
    *command = receiveBuffers[receiveBufferReadPosition][parsing_index];
    parsing_index++;
    *payload_size = packet_size - parsing_index;
    *payload = (void*)(&receiveBuffers[receiveBufferReadPosition][parsing_index]);
    return 1;
}

uint8_t rs485_validate_packet_crc32(void)
{
    uint16_t size_size = 1;
    uint16_t packet_size = decode_first_byte(receiveBuffers[receiveBufferReadPosition][0]);
    if (packet_size == DECODED_FIRST_BYTE_EXTENDED_SIZE) {
        packet_size = ((uint16_t)receiveBuffers[receiveBufferReadPosition][2] << 8) | receiveBuffers[receiveBufferReadPosition][1];
        size_size += 2;
    }
    uint32_t received_crc32 = *(uint32_t*)&receiveBuffers[receiveBufferReadPosition][packet_size - sizeof(uint32_t)];
    if (packet_size < sizeof(uint32_t)) {
        return 0; // there are less than 4 bytes in this packet and so it cannot contain a CRC32 and is therefore invalid or lacking a CRC32
    }
    uint16_t crc32_calculation_n_bytes = size_size + packet_size - sizeof(uint32_t); // we will include the size bytes but not the crc32 bytes in the CRC32 calculation

    crc32_init(); // reset the CRC32 calculation unit
    uint32_t calculated_crc32 = calculate_crc32_buffer((void*)&receiveBuffers[receiveBufferReadPosition][0], crc32_calculation_n_bytes);

    if (calculated_crc32 != received_crc32) {
        return 0; // the CRC32 does not match and so the packet is invalid
    }
    return 1; // the CRC32 matches and so the packet is valid
}

void rs485_done_with_this_packet(void)
{
    __disable_irq();
    if (receiveBuffersUsed > 0) {
        receiveBuffersUsed--;
        receiveBufferReadPosition = (receiveBufferReadPosition + 1) % N_RECEIVE_BUFFERS;
    }
    USART1->CR1 |= USART_CR1_RXNEIE_RXFNEIE; // enable receive interrupt
    __enable_irq();
}

void USART1_IRQHandler(void)
{
    // In this interrupt handler we first handle receive related logic
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
            receiveIndex = 0;
            firstByteInvalid = 0;
            USART1->ICR |= USART_ICR_RTOCF; // clear the timeout flag
        }

        uint8_t receivedByte;
        receivedByte = USART1->RDR;
        #ifdef MOTOR_SIMULATION
        USART1->ISR &= ~USART_ISR_RXNE_RXFNE; // clear this bit to indicate that we have read the received byte (done automatically in the real hardware but not in the simulated hardware)
        printf("* * * Received a byte: %hhu\n", receivedByte);
        #endif

        if(receiveBuffersUsed >= N_RECEIVE_BUFFERS) {
            fatal_error(ERROR_COMMAND_OVERFLOW);
        }
        if (receiveIndex == 0) {
            if(!is_valid_first_byte_format(receivedByte)) {
                firstByteInvalid = 1;
            }
            receivePacketSize = decode_first_byte(receivedByte);
            receiveBufferOverflow = 0;
        }
        else if ((receiveIndex == 2) && (receivePacketSize == DECODED_FIRST_BYTE_EXTENDED_SIZE)) {
            receivePacketSize = ((uint16_t)receivedByte << 8) | receiveBuffers[receiveBufferWritePosition][1]; // this packet has an extended size that is taken from the second and third bytes of the packet
        }
        if (receiveIndex < MAX_RECEIVE_BUFFER_SIZE) {
            receiveBuffers[receiveBufferWritePosition][receiveIndex] = receivedByte;
            receiveIndex++;
        }
        else {
            receiveBufferOverflow = 1; // our buffer could not hold this (too long) packet, so we will assume that this packet is not for us and we will drop it after it fully went through
        }
        if(receiveIndex == receivePacketSize) {
            receiveIndex = 0;
            if (!receiveBufferOverflow && !firstByteInvalid) {
                receiveBufferWritePosition = (receiveBufferWritePosition + 1) % N_RECEIVE_BUFFERS;
                receiveBuffersUsed++;
            }
        }
    }

    // And now secondly in this interrupt handler we handle transmit related logic
    while((USART1->ISR & USART_ISR_TXE_TXFNF_Msk) && (transmitCount > 0)) {
        USART1->TDR = transmitBuffer[transmitIndex];
#ifdef MOTOR_SIMULATION
        // In simulation, we need to clear TXE after writing to TDR
        // (In real hardware, this is done by the USART hardware)
        USART1->ISR &= ~USART_ISR_TXE_TXFNF_Msk;
        printf("* * * Transmitted a byte: %hhu\n", transmitBuffer[transmitIndex]);
#endif
        transmitCount--;
        transmitIndex++;
    }

    if(transmitCount == 0) {
        USART1->CR1 &= ~USART_CR1_TXFEIE; // nothing more to transmit, so disable the interrupt
    }

    // And finally control the red LED such that it flashes during the time that a command is being received
    if(receiveIndex > 0) {
        red_LED_on();
    }
    else {
        red_LED_off();
    }

    #ifdef MOTOR_SIMULATION
    // For debugging purposes, lets print out the contents of all of these internal state variables (in this order):
    // static volatile uint8_t receiveBufferReadPosition;
    // static volatile uint8_t receiveBufferWritePosition;
    // static volatile uint8_t receiveBuffersUsed;
    // static volatile uint16_t receiveIndex;
    // static volatile uint16_t receivePacketSize;
    // static volatile uint8_t receiveBufferOverflow;
    // static volatile uint8_t receiveBuffers[N_RECEIVE_BUFFERS][MAX_RECEIVE_BUFFER_SIZE];
    // static volatile uint8_t transmitIndex;
    // static volatile uint8_t transmitCount;
    // static volatile char transmitBuffer[TRANSMIT_BUFFER_SIZE];
    printf("* * * Finished USART1_IRQHandler. This is the current complete internal state:\n");
    printf("         receiveBufferReadPosition:  %hhu\n", receiveBufferReadPosition);
    printf("         receiveBufferWritePosition: %hhu\n", receiveBufferWritePosition);
    printf("         receiveBuffersUsed:         %hhu\n", receiveBuffersUsed);
    printf("         receiveIndex:               %hu\n",  receiveIndex);
    printf("         receivePacketSize:          %hu\n",  receivePacketSize);
    printf("         receiveBufferOverflow:      %hhu\n", receiveBufferOverflow);
    for (uint32_t j = 0; j < N_RECEIVE_BUFFERS; j++) {
        printf("         receiveBuffers[%d]:", j);
        for (uint32_t i = 0; i < receiveIndex; i++) {
            printf(" %hhu", receiveBuffers[j][i]);
        }
        printf("\n");
    }
    printf("         transmitIndex:         %hhu\n", transmitIndex);
    printf("         transmitCount:         %hhu\n", transmitCount);
    printf("         transmitBuffer:");
    for (uint32_t i = transmitIndex; i < transmitIndex + transmitCount; i++) {
        printf(" %hhu", transmitBuffer[i]);
    }
    printf("\n");
    #endif

}

#if 0
void OLD_rs485_done_with_this_packet(void)
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
        // Error checks for received bytes count and command overflow
        if(nReceivedBytes >= 65535) {
            fatal_error(ERROR_TOO_MANY_BYTES);
        }
        if(commandReceived) {
            fatal_error(ERROR_COMMAND_OVERFLOW);
        }
        
        nReceivedBytes++;

        // Process the first byte (device ID or extended addressing indicator)
        if(nReceivedBytes == 1) {
            if(!is_valid_first_byte_format(receivedByte)) {
                fatal_error(ERROR_INVALID_FIRST_BYTE);
            }
            selectedAxis = decode_first_byte(receivedByte);
            in_extended_addressing_mode = (selectedAxis == EXTENDED_ADDRESSING);
            unique_id_bytes_received = 0;
            
            // Set unique_id_mismatch based on addressing mode
            if(in_extended_addressing_mode) {
                unique_id_mismatch = 0; // For extended addressing, we'll check bytes as they arrive
            } else {
                // For standard addressing, check if this message is for us
                unique_id_mismatch = (selectedAxis == RESPONSE_CHARACTER || 
                                    (selectedAxis != global_settings.my_alias && 
                                     selectedAxis != ALL_ALIAS)) ? 1 : 0;
            }
            valueLength = UNKNOWN_VALUE_LENGTH; // valueLength needs to be initialized to some high number so that the condition if(receiveIndex >= valueLength) is always false before the correct valueLength is determined
            return; // Return to wait for next byte
        }
        
        // Process based on addressing mode
        if(in_extended_addressing_mode) {
            // Extended addressing mode
            if(unique_id_bytes_received < UNIQUE_ID_SIZE) {
                // Check each unique ID byte as it arrives
                uint64_t device_id = get_unique_id();
                uint8_t expected_byte = (device_id >> (unique_id_bytes_received * 8)) & 0xFF;
                
                // If any byte doesn't match, set the mismatch flag to 1
                if(receivedByte != expected_byte) {
                    unique_id_mismatch = 1;
                }

                unique_id_bytes_received++;
            }
            else if(unique_id_bytes_received == UNIQUE_ID_SIZE) {
                // Received all unique ID bytes, now get command
                command = receivedByte;
                unique_id_bytes_received++;
            }
            else if(unique_id_bytes_received == UNIQUE_ID_SIZE + 1) {
                // Get value length
                valueLength = receivedByte;
                receiveIndex = 0;
                unique_id_bytes_received++;
            }
            else if((unique_id_bytes_received == UNIQUE_ID_SIZE + 3) && (valueLength == 255)) {
                // Handle extended value length
                valueLength = (receivedByte << 8) + valueBuffer[0];
                receiveIndex = 0;
            }
            else if(receiveIndex < MAX_VALUE_BUFFER_LENGTH) {
                // Store value bytes
                valueBuffer[receiveIndex++] = receivedByte;
            }
        }
        else {
            // Standard addressing mode
            if(nReceivedBytes == 2) {
                // Second byte is the command
                command = receivedByte;
            }
            else if(nReceivedBytes == 3) {
                // Third byte is the value length
                valueLength = receivedByte;
                receiveIndex = 0;
            }
            else if((nReceivedBytes == 5) && (valueLength == 255)) {
                // Extended value length
                valueLength = (receivedByte << 8) + valueBuffer[0];
                receiveIndex = 0;
            }
            else if(receiveIndex < MAX_VALUE_BUFFER_LENGTH) {
                // Store value bytes
                valueBuffer[receiveIndex++] = receivedByte;
            }
        }
        
        // Check if we've received all bytes for the current command
        if(receiveIndex >= valueLength) { // This condition must remain false before the correct valueLength is received. That is why we initialize valueLength to UNKNOWN_VALUE_LENGTH, which is a big number
            // Process command if it's addressed to this device (no mismatch)
            if(!unique_id_mismatch) {
                if(valueLength <= MAX_VALUE_BUFFER_LENGTH) {
                    USART1->CR1 &= ~USART_CR1_RXNEIE_RXFNEIE; // Disable receive interrupt until command is processed
                    commandReceived = 1;
                }
                else {
                    fatal_error(ERROR_COMMAND_TOO_LONG);
                }
            }
            
            nReceivedBytes = 0; // Reset state for next command
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
#endif

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

uint8_t rs485_is_transmit_done(void)
{
    return (transmitCount == 0);
}


// We need to do two things before finally transmitting the packet:
// 1. If CRC32 is enabled then we need to calculate the CRC32 of the structure and place it at the end of the structure
// 2. Fill in the packet size in the header (the first byte) so that it includes the CRC32 if the CRC32 is enabled or excludes it otherwise
// Please note that the structure that you pass in must contain the space for the crc32 if crc32_enabled is true
void rs485_finalize_and_transmit_packet(void *data, uint16_t structure_size, uint8_t crc32_enabled)
{
    if(!crc32_enabled) {
        structure_size -= sizeof(uint32_t); // remove the size of the crc32 value, because we will not be sending it
        rs485_transmit(data, structure_size);
    }
    else {
        uint32_t crc32 = calculate_crc32_buffer((uint8_t*)data, structure_size - sizeof(uint32_t));
        *(uint32_t *)(((uint8_t*)data) + (structure_size - sizeof(uint32_t))) = crc32; // place the crc32 as the last 4 bytes of the packet
    }
    ((uint8_t*)data)[0] = structure_size;
    ((uint8_t*)data)[1] = RESPONSE_CHARACTER;
    ((uint8_t*)data)[2] = 1;
    rs485_transmit(data, structure_size);
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
    
    // Set up the USART1 interrupt status register such that no interrupt flags are set
    // except for the USART_ISR_TXE_TXFNF_Msk flag, which indicates that the transmit buffer is empty and the UART
    // is ready to accept a new byte for sending out
    // Hardware does this in the real hardware, but we need to do it in the simulator
    USART1->ISR = USART_ISR_TXE_TXFNF_Msk;

    printf("RS485 module reset complete\n");
}

void print_receiveIndex(void)
{
    printf("receiveIndex = %hu\n", receiveIndex);
}

#endif

