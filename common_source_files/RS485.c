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

// Simulation-only printf function that compiles to nothing in firmware builds and optionally
// compiles to nothing if the SIMULATOR_DEBUG_PRINTING is not defined (ie. you are not debugging)
//#define SIMULATOR_DEBUG_PRINTING
#ifdef MOTOR_SIMULATION
#ifdef SIMULATOR_DEBUG_PRINTING
#include <stdio.h>
#define simulation_printf(...) printf(__VA_ARGS__)
#else
#define simulation_printf(...) ((void)0)
#endif
#else
#define simulation_printf(...) ((void)0)
#endif

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

static uint8_t crc32_enabled = 1;
static uint32_t crc32_error_count = 0;
static uint32_t packet_decode_error_count = 0;

#define UNKNOWN_VALUE_LENGTH 65535

#if MOTOR_SIMULATION
#include "microsecond_clock.h"
// This function checks the time difference between now and the last time that a byte was received from the UART
// If this time difference is more than 10ms then it will set the timeout interrupt flag of the UART
// add static variable here to leep track of the last time that a byte was received from the UART
static uint64_t last_byte_received_time = 0;

void check_for_UART_timeout_sim(void)
{
    USART1->ISR &= ~USART_ISR_RTOF; // clear this flag at first, but set it again if there is a timeout
    
    // Get current time in microseconds
    uint64_t current_time = get_microsecond_time();
    
    // If this is the first call, initialize the last byte time
    if (last_byte_received_time == 0) {
        last_byte_received_time = current_time;
        return;
    }
    
    // Check if it's been more than 10ms (10,000 microseconds) since the last byte
    if (current_time - last_byte_received_time > 10000) {
        // Set the timeout flag
        USART1->ISR |= USART_ISR_RTOF;
        simulation_printf("A timeout on the UART was detected in this simulation. Setting the timeout flag of the UART.\n");
    }
    
    // Update the last byte time whenever a new byte is received
    // This happens when the RXNE flag is set, indicating data is available
    if (USART1->ISR & USART_ISR_RXNE_RXFNE) {
        last_byte_received_time = current_time;
    }
}
#endif

#ifdef MOTOR_SIMULATION
void print_internal_state(void)
{
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
    simulation_printf("         receiveBufferReadPosition:  %hhu\n", receiveBufferReadPosition);
    simulation_printf("         receiveBufferWritePosition: %hhu\n", receiveBufferWritePosition);
    simulation_printf("         receiveBuffersUsed:         %hhu\n", receiveBuffersUsed);
    simulation_printf("         receiveIndex:               %hu\n",  receiveIndex);
    simulation_printf("         firstByteInvalid:           %hhu\n",  firstByteInvalid);    
    simulation_printf("         receivePacketSize:          %hu\n",  receivePacketSize);
    simulation_printf("         receiveBufferOverflow:      %hhu\n", receiveBufferOverflow);
    for (uint32_t j = 0; j < N_RECEIVE_BUFFERS; j++) {
        simulation_printf("         receiveBuffers[%d]:", j);
        for (uint32_t i = 0; i < receiveIndex; i++) {
            simulation_printf(" %hhu", receiveBuffers[j][i]);
        }
        simulation_printf("\n");
    }
    simulation_printf("         transmitIndex:         %hhu\n", transmitIndex);
    simulation_printf("         transmitCount:         %hhu\n", transmitCount);
    simulation_printf("         transmitBuffer:");
    for (uint32_t i = transmitIndex; i < transmitIndex + transmitCount; i++) {
        simulation_printf(" %hhu", transmitBuffer[i]);
    }
    simulation_printf("\n");
}
#endif

// This function will be called when the device first starts and it will also be called
// if a fatal error occurs so as to make sure tha the UART is reinitialized and working
// even if something abnormal happed that caused the fatal error
void rs485_init(void)
{
    // reinitialize all these variables related to transmit and receive
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

void rs485_set_crc32_enable(uint8_t crc32_enable_new_state)
{
    crc32_enabled = crc32_enable_new_state;
}

uint8_t rs485_is_crc32_enabled(void)
{
    return crc32_enabled;
}

rs485_error_statistics_t rs485_get_error_statistics_and_optionally_reset(uint8_t reset)
{
    rs485_error_statistics_t stats = {
        .crc32_error_count = crc32_error_count,
        .packet_decode_error_count = packet_decode_error_count
    };

    if (reset) {
        crc32_error_count = 0;
        packet_decode_error_count = 0;
    }
    return stats;
}

uint8_t rs485_has_a_packet(void)
{
    return receiveBuffersUsed > 0;
}

uint8_t rs485_get_next_packet(uint8_t *command, uint16_t *payload_size, uint8_t **payload, uint8_t *is_broadcast)
{
    simulation_printf("* * * rs485_get_next_packet() called\n");
    if (receiveBuffersUsed == 0) {
        simulation_printf("         nothing in the buffer. returning.\n");
        return 0;
    }
    uint16_t packet_size = decode_first_byte(receiveBuffers[receiveBufferReadPosition][0]);
    uint16_t bytes_parsed = 1;
    simulation_printf("         packet_size = %hu   bytes_parsed = %hu\n", packet_size, bytes_parsed);
    if (packet_size == DECODED_FIRST_BYTE_EXTENDED_SIZE) {
        packet_size = ((uint16_t)receiveBuffers[receiveBufferReadPosition][bytes_parsed + 1] << 8) | receiveBuffers[receiveBufferReadPosition][bytes_parsed];
        bytes_parsed += 2;
        simulation_printf("         packet_size = %hu   bytes_parsed = %hu\n", packet_size, bytes_parsed);
    }
    uint8_t deviceID = receiveBuffers[receiveBufferReadPosition][bytes_parsed];
    simulation_printf("         deviceID = %hhu\n", deviceID);
    if ((deviceID == RESPONSE_CHARACTER_CRC32_ENABLED) || (deviceID == RESPONSE_CHARACTER_CRC32_DISABLED)) { 
        simulation_printf("         deviceID is one of the response characters (%hhu). returning.\n", deviceID);
        return 0; // discard a packet that is from some other device that is responding to a command
    }
    bytes_parsed++;
    simulation_printf("         bytes_parsed = %hu\n", bytes_parsed);
    if (deviceID == ALL_ALIAS) { // this is a broadcast packet and so it it valid for us
        *is_broadcast = 1;
        simulation_printf("         deviceID is the broadcast character (%u). setting is_broadcast to %hhu.\n", ALL_ALIAS, *is_broadcast);
    }
    else {
        *is_broadcast = 0;
        simulation_printf("         deviceID is not the broadcast character (%u). setting is_broadcast to %hhu.\n", ALL_ALIAS, *is_broadcast);
        if (deviceID == EXTENDED_ADDRESSING) {
            uint64_t our_device_id = get_unique_id();
            uint64_t *received_unique_id = (uint64_t *)&receiveBuffers[receiveBufferReadPosition][bytes_parsed];
            simulation_printf("         deviceID indicates extended addressing with the character (%u)   our_device_id = %llX   *received_unique_id = %llX.\n", EXTENDED_ADDRESSING, our_device_id, *received_unique_id);
            if (memcmp(received_unique_id, &our_device_id, sizeof(our_device_id)) != 0) {
                simulation_printf("         *received_unique_id != our_device_id. returning.\n");
                return 0; // extended addressing was used but the unique ID did not match and so this packet is not for us
            }
            bytes_parsed += UNIQUE_ID_SIZE;
            simulation_printf("         bytes_parsed = %hu\n", bytes_parsed);
        }
        else {
            if (deviceID != global_settings.my_alias) {
                simulation_printf("         deviceID is not the alias of this device (%hhu). returning.\n", global_settings.my_alias);
                return 0; // normal address was used but the alias did not match ours and so this packet is not for us
            }
        }
    }
    uint16_t crc32_size = (crc32_enabled) ? sizeof(uint32_t) : 0;
    if (packet_size < bytes_parsed + sizeof(*command) + crc32_size) { // we need to make sure that we have more bytes available in the packet for the command and (if enabled) the CRC32
        simulation_printf("         packet_size < bytes_parsed + sizeof(*command) + crc32_size   packet_size = %hu   bytes_parsed = %hu   sizeof(*command) = %lu   crc32_size = %hu   returning.\n", packet_size, bytes_parsed, sizeof(*command), crc32_size);
        packet_decode_error_count++;
        return 0; // this packet is invalid because the indicated size of the packet is too small to hold all the information needed to properly decode it
    }
    *command = receiveBuffers[receiveBufferReadPosition][bytes_parsed];
    bytes_parsed++;
    *payload_size = packet_size - bytes_parsed - crc32_size;
    *payload = (uint8_t *)(&receiveBuffers[receiveBufferReadPosition][bytes_parsed]);
    #ifdef MOTOR_SIMULATION
    simulation_printf("         *command = %hhu   bytes_parsed = %hu   *payload_size = %hu   payload:", *command, bytes_parsed, *payload_size);
    for (uint32_t i = 0; i < *payload_size; i++) {
        simulation_printf(" %hhu", (*payload)[i]);
    }
    simulation_printf("\n");
    simulation_printf("         payload (hex):");
    for (uint32_t i = 0; i < *payload_size; i++) {
        simulation_printf(" %hhX", (*payload)[i]);
    }
    simulation_printf("\n");
    #endif
    return 1;
}


uint8_t rs485_validate_packet_crc32(void)
{
    if (!crc32_enabled) {
        return 1;
    }

    #if defined(MOTOR_SIMULATION) && defined(SIMULATOR_DEBUG_PRINTING)
    simulation_printf("* * * rs485_validate_packet_crc32() called\n");
    uint16_t size_size = 1;
    #endif

    uint16_t packet_size = decode_first_byte(receiveBuffers[receiveBufferReadPosition][0]);
    simulation_printf("         packet_size = %hu   size_size = %hu\n", packet_size, size_size);
    if (packet_size == DECODED_FIRST_BYTE_EXTENDED_SIZE) {
        packet_size = ((uint16_t)receiveBuffers[receiveBufferReadPosition][2] << 8) | receiveBuffers[receiveBufferReadPosition][1];
        #if defined(MOTOR_SIMULATION) && defined(SIMULATOR_DEBUG_PRINTING)
        size_size += 2;
        simulation_printf("         packet_size = %hu   size_size = %hu\n", packet_size, size_size);
        #endif
    }

    uint32_t received_crc32;
    memcpy(&received_crc32, (void *)&receiveBuffers[receiveBufferReadPosition][packet_size - sizeof(uint32_t)], sizeof(uint32_t));
    simulation_printf("         received_crc32 = %X\n", received_crc32);
    if (packet_size < sizeof(uint32_t)) {
        #ifdef MOTOR_SIMULATION
        simulation_printf("         ******** I EXPECT THIS ERROR TO NECER HAPPEN because packets too small to contain a CRC32 are discarded in the rs485_get_next_packet() function   packet_size < sizeof(uint32_t)   packet_size = %hu\n", packet_size);
        exit(1);
        #endif
        packet_decode_error_count++;
        return 0; // there are less than 4 bytes in this packet and so it cannot contain a CRC32 and is therefore invalid or lacking a CRC32
    }
    uint16_t crc32_calculation_n_bytes = packet_size - sizeof(uint32_t); // we will include the size bytes but not the crc32 bytes in the CRC32 calculation

    #ifdef MOTOR_SIMULATION
    simulation_printf("         calculating the CRC32 of the following buffer (size is %hu):", crc32_calculation_n_bytes);
    for (uint32_t i = 0; i < crc32_calculation_n_bytes; i++) {
        simulation_printf(" %hhX", receiveBuffers[receiveBufferReadPosition][i]);
    }
    simulation_printf("\n");
    #endif
    uint32_t calculated_crc32 = calculate_crc32_buffer((void*)&receiveBuffers[receiveBufferReadPosition][0], crc32_calculation_n_bytes);

    simulation_printf("         crc32_calculation_n_bytes = %hu   calculated_crc32 = %X\n", crc32_calculation_n_bytes, calculated_crc32);
    if (calculated_crc32 != received_crc32) {
        simulation_printf("         calculated_crc32 != received_crc32   calculated_crc32 = %X   received_crc32 = %X   returning\n", calculated_crc32, received_crc32);
        crc32_error_count++;
        return 0; // the CRC32 does not match and so the packet is invalid
    }
    simulation_printf("         returning 1 (because the CRC32 check passed)\n");
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
    #ifdef MOTOR_SIMULATION
    simulation_printf("* * * rs485_done_with_this_packet function finished its work. This is the current complete internal state:\n");
    print_internal_state();
    #endif

}

void USART1_IRQHandler(void)
{
    #ifdef MOTOR_SIMULATION
    uint8_t want_print_internal_state = 0;
    #endif

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
        #if MOTOR_SIMULATION
        check_for_UART_timeout_sim();
        #endif
        if(USART1->ISR & USART_ISR_RTOF) {
            receiveIndex = 0;
            firstByteInvalid = 0;
            USART1->ICR |= USART_ICR_RTOCF; // clear the timeout flag
        }

        uint8_t receivedByte;
        receivedByte = USART1->RDR;
        #ifdef MOTOR_SIMULATION
        USART1->ISR &= ~USART_ISR_RXNE_RXFNE; // clear this bit to indicate that we have read the received byte (done automatically in the real hardware but not in the simulated hardware)
        simulation_printf("* * * Received a byte: %hhu\n", receivedByte);
        want_print_internal_state = 1;
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
        }
        else {
            receiveBufferOverflow = 1; // our buffer could not hold this (too long) packet, so we will assume that this packet is not for us and we will drop it after it fully went through
        }
        receiveIndex++;
        if(receiveIndex >= receivePacketSize) {
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
        simulation_printf("* * * Transmitted a byte: %hhu\n", transmitBuffer[transmitIndex]);
        want_print_internal_state = 1;
#endif
        transmitCount--;
        transmitIndex++;
    }

    if(transmitCount == 0) {
        USART1->CR1 &= ~USART_CR1_TXFEIE; // nothing more to transmit, so disable the interrupt
    }

    // And finally control the red LED such that it flashes during the time that a command is being received
    if(receiveIndex > 0) {
        green_LED_on();
    }
    else {
        green_LED_off();
    }

    #ifdef MOTOR_SIMULATION
    if (want_print_internal_state) {
        #include <pthread.h>
        simulation_printf("* * * Finished USART1_IRQHandler. The thread ID is %ld This is the current complete internal state:\n", (unsigned long)(pthread_self()));
//        simulation_printf("* * * Finished USART1_IRQHandler. This is the current complete internal state:\n");
        print_internal_state();
    }
    #endif
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
        simulation_printf("* * * Transmitted a byte: %hhu\n", transmitBuffer[transmitIndex]);
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
    while((transmitCount > 0) || ((USART1->ISR & USART_ISR_TC_Msk) == 0)); // wait for previous transmission to finish (buffer becomes empty and last data item writen to USART->TDR has been transmitted out of the shift register)
}

uint8_t rs485_is_transmit_done(void)
{
    return (transmitCount == 0);
}

void rs485_transmit_no_error_packet(uint8_t is_broadcast)
{
    if (is_broadcast) {
        return; // if the received message was a broadcasted message then we will not respond (to prevent a collision on the bus if there are multiple devices)
    }
    if (crc32_enabled) {
        #if 0
        struct __attribute__((__packed__)) {
            uint8_t size;
            uint8_t response_character;
            uint8_t command;
            uint32_t crc32;
        } response_with_crc32;
        response_with_crc32.size = encode_first_byte(7);
        response_with_crc32.response_character = RESPONSE_CHARACTER_CRC32_ENABLED;
        response_with_crc32.command = 0;
        response_with_crc32.crc32 = calculate_crc32_buffer((uint8_t*)&response_with_crc32, sizeof(response_with_crc32) - sizeof(uint32_t));
        rs485_transmit(&response_with_crc32, sizeof(response_with_crc32));
        #endif
        rs485_transmit(NO_ERROR_RESPONSE_CRC32_ENABLED, 7); // because this response will be use frequently, I am hard coding the extra precomputed CRC32 bytes. the above commented out code calculates the CRC32 on the fly.
    }
    else {
        rs485_transmit(NO_ERROR_RESPONSE_CRC32_DISABLED, 3);
    }
}


// We need to do two things before finally transmitting the packet:
// 1. If CRC32 is enabled then we need to calculate the CRC32 of the structure and place it at the end of the structure
// 2. Fill in the packet size in the header (the first byte) so that it includes the CRC32 if the CRC32 is enabled or excludes it otherwise
// Please note that the structure that you pass in must contain the space for the crc32 if crc32_enabled is true
void rs485_finalize_and_transmit_packet(void *data, uint16_t structure_size)
{
    if(crc32_enabled) {
        ((uint8_t*)data)[1] = RESPONSE_CHARACTER_CRC32_ENABLED;
    }
    else {
        structure_size -= sizeof(uint32_t); // remove the size of the crc32 value, because we will not be sending it
        ((uint8_t*)data)[1] = RESPONSE_CHARACTER_CRC32_DISABLED;
    }
    ((uint8_t*)data)[0] = encode_first_byte(structure_size);
    ((uint8_t*)data)[2] = 1;

    if(crc32_enabled) {
        uint32_t crc32 = calculate_crc32_buffer((uint8_t*)data, structure_size - sizeof(uint32_t));
        memcpy(((uint8_t*)data) + (structure_size - sizeof(uint32_t)), &crc32, sizeof(uint32_t));
        #ifdef MOTOR_SIMULATION
        uint16_t crc32_calculation_n_bytes = structure_size - sizeof(uint32_t);
        simulation_printf("         calculating the CRC32 of the following buffer (size is %hu):", crc32_calculation_n_bytes);
        for (uint32_t i = 0; i < crc32_calculation_n_bytes; i++) {
            simulation_printf(" %hhX", ((uint8_t*)data)[i]);
        }
        simulation_printf("\n");
        simulation_printf("         calculated_crc32 = %X\n", crc32);
        #endif
    }

    #ifdef MOTOR_SIMULATION
    simulation_printf("* * * Transmitting the following packet after it has been finalized:");
    for (uint32_t i = 0; i < structure_size; i++) {
        simulation_printf(" %hhu", ((uint8_t *)data)[i]);
    }
    simulation_printf("\n");
    #endif

    rs485_transmit(data, structure_size);
}

void rs485_start_the_packet(uint16_t payload_size)
{
    struct __attribute__((__packed__)) {
        uint8_t packet_size;
        uint16_t extended_size;
        uint8_t response_character;
        uint8_t has_data_flag;
    } header;
    header.packet_size = FIRST_BYTE_EXTENDED_SIZE;
    header.extended_size = sizeof(header) + payload_size;
    header.has_data_flag = 1;
    if(crc32_enabled) {
        header.response_character = RESPONSE_CHARACTER_CRC32_ENABLED;
        header.extended_size += sizeof(uint32_t); // add the bytes for the CRC32 to the packet size info
        calculate_crc32_buffer(&header, sizeof(header));
    }
    else {
        header.response_character = RESPONSE_CHARACTER_CRC32_DISABLED;
    }
    rs485_transmit(&header, sizeof(header));
}

void rs485_continue_the_packet(void *s, uint8_t len)
{
    if(crc32_enabled) {
        calculate_crc32_buffer_without_reinit(s, len);
    }
    rs485_transmit(s, len);
}

void rs485_end_the_packet(void)
{
    if(crc32_enabled) {
        uint32_t crc32 = get_crc32();
        rs485_transmit(&crc32, sizeof(crc32));
    }
}

#ifdef MOTOR_SIMULATION
/**
 * Initialize all static variables in RS485.c for simulator use
 * This function should be called when the simulator starts to ensure
 * all static variables are properly initialized
 */
void RS485_simulator_init(void)
{
    simulation_printf("RS485_simulator_init() called\n");
    // Set up the USART1 interrupt status register such that no interrupt flags are set
    // except for the USART_ISR_TXE_TXFNF_Msk flag, which indicates that the transmit buffer is empty and the UART
    // is ready to accept a new byte for sending out and the USART_ISR_TC_Msk flag, which indicates that the
    // transmission is complete and we can start more transmissions
    // Hardware does this in the real hardware, but we need to do it in the simulator
    USART1->ISR = USART_ISR_TXE_TXFNF_Msk | USART_ISR_TC_Msk;
    crc32_enabled = 1;
    crc32_error_count = 0;
    packet_decode_error_count = 0;
    simulation_printf("RS485 module reset complete\n");
}

void print_receiveIndex(void)
{
    simulation_printf("receiveIndex = %hu\n", receiveIndex);
}

#endif

