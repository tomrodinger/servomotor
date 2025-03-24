
#ifndef SRC_RS485_H_
#define SRC_RS485_H_

#include <config.h>
#include "settings.h"
#include "product_info.h"

// Device ID bit manipulation constants
#define DEVICE_ID_LSB_MASK 0x01 // Mask for checking if LSB is set
#define DEVICE_ID_SHIFT 1 // Number of bits to shift for device ID interpretation
// Reserved aliases with special meaning
#define ALL_ALIAS 127 // to address all devices on the bus at the same time
#define RESPONSE_CHARACTER 126 // this is the character that is used to indicate that the response is coming from the device that is being addressed
#define EXTENDED_ADDRESSING 125 // this is the character that is used to indicate that we will use extended addressing
#define ENCODED_RESPONSE_CHARACTER (RESPONSE_CHARACTER << DEVICE_ID_SHIFT) | DEVICE_ID_LSB_MASK // this is what will be sent on the line as the first byte of a message
#define ENCODED_RESPONSE_CHARACTER_TEXT "\xFD" // this needs to be set according to the text version of the ENCODED_RESPONSE_CHARACTER
#define NO_ERROR_RESPONSE (ENCODED_RESPONSE_CHARACTER_TEXT "\x00\x00")

// Include unique_id.h for UNIQUE_ID_SIZE definition
#include "unique_id.h"

#define USART1_TIMEOUT 5 // one count is 10ms, so this is a timeout of about 50 ms

#define TRANSMIT_BUFFER_SIZE 150 // this is the maximum number of bytes that can be transmitted with one call to rs485_transmit()

// Function to encode a device ID by setting LSB to 1
static inline uint8_t encode_device_id(uint8_t device_id) {
    return (device_id << DEVICE_ID_SHIFT) | DEVICE_ID_LSB_MASK;
}

// Function to decode a device ID by shifting right
static inline uint8_t decode_device_id(uint8_t encoded_id) {
    return encoded_id >> DEVICE_ID_SHIFT;
}

// Function to check if LSB is set (valid device ID format)
static inline uint8_t is_valid_device_id_format(uint8_t encoded_id) {
    return (encoded_id & DEVICE_ID_LSB_MASK) == DEVICE_ID_LSB_MASK;
}

void rs485_init(void);
void rs485_allow_next_command(void);
void rs485_transmit(void *s, uint8_t len);
void rs485_wait_for_transmit_done(void);






#endif /* SRC_RS485_H_ */
