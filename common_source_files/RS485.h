
#ifndef SRC_RS485_H_
#define SRC_RS485_H_

#include <config.h>
#include "settings.h"
#include "product_info.h"

// Device ID bit manipulation constants
#define FIRST_BYTE_LSB_MASK 0x01 // Mask for checking if LSB is set
#define FIRST_BYTE_SHIFT 1 // Number of bits to shift for device ID interpretation
#define FIRST_BYTE_EXTENDED_SIZE 255
#define DECODED_FIRST_BYTE_EXTENDED_SIZE (FIRST_BYTE_EXTENDED_SIZE >> FIRST_BYTE_SHIFT) // If the first byte is this after decoding then we need to use extended sizing, which is that we take the packet size from the next two bytes (size can be 0 to 65535)
// Reserved aliases with special meaning
#define ALL_ALIAS 255 // to address all devices on the bus at the same time
#define EXTENDED_ADDRESSING 254 // this is the character that is used to indicate that we will use extended addressing
#define RESPONSE_CHARACTER_CRC32_ENABLED 253 // this is the character that is used to indicate that the response is coming from the device that is being addressed (and a 4-byte CRC32 is appended)
#define RESPONSE_CHARACTER_CRC32_DISABLED 252 // this is the character that is used to indicate that the response is coming from the device that is being addressed (and there is no CRC32 is appended)
#define RESPONSE_CHARACTER_CRC32_ENABLED_TEXT "\xFD" // this needs to be set according to the text version of the RESPONSE_CHARACTER_CRC32_ENABLED
#define RESPONSE_CHARACTER_CRC32_DISABLED_TEXT "\xFC" // this needs to be set according to the text version of the RESPONSE_CHARACTER_CRC32_DISABLED
//#define NO_ERROR_RESPONSE_CRC32_ENABLED ("\x0F\xFD\x00\xDF\x01\x0F\x55") // the first \x0F is what is sent on the line and it will be decoded to mean 7 bytes by the other end
//#define NO_ERROR_RESPONSE_CRC32_DISABLED ("\x07" RESPONSE_CHARACTER_CRC32_DISABLED_TEXT "\x00") // the \x07 is what is sent on the line and it will be decoded to mean 3 bytes by the other end
#define NO_ERROR_RESPONSE_CRC32_ENABLED ("\x0d" RESPONSE_CHARACTER_CRC32_ENABLED_TEXT "\x13\xe2\x7b\x37") // the first \x0d is what is sent on the line and it will be decoded to mean 6 bytes by the other end
#define NO_ERROR_RESPONSE_CRC32_DISABLED ("\x05" RESPONSE_CHARACTER_CRC32_DISABLED_TEXT) // the \x05 is what is sent on the line and it will be decoded to mean 2 bytes by the other end
#define ERROR_CODE_NO_ERROR 0

// Include unique_id.h for UNIQUE_ID_SIZE definition
#include "unique_id.h"

#define USART1_TIMEOUT 5 // one count is 10ms, so this is a timeout of about 50 ms

#define TRANSMIT_BUFFER_SIZE 150 // this is the maximum number of bytes that can be transmitted with one call to rs485_transmit()

// Function to encode a first byte by setting LSB to 1
static inline uint8_t encode_first_byte(uint8_t first_byte) {
    return (first_byte << FIRST_BYTE_SHIFT) | FIRST_BYTE_LSB_MASK;
}

// Function to decode a first byte by shifting right
static inline uint8_t decode_first_byte(uint8_t encoded_first_byte) {
    return encoded_first_byte >> FIRST_BYTE_SHIFT;
}

// Function to check if LSB is set (valid first byte format)
static inline uint8_t is_valid_first_byte_format(uint8_t encoded_first_byte) {
    return (encoded_first_byte & FIRST_BYTE_LSB_MASK) == FIRST_BYTE_LSB_MASK;
}

typedef struct {
    uint32_t crc32_error_count;
    uint32_t packet_decode_error_count;
    uint32_t first_bit_error_count;
    uint32_t framing_error_count;
    uint32_t overrun_error_count;
    uint32_t noise_error_count;
} rs485_error_statistics_t;

void rs485_init(void);
void rs485_set_crc32_enable(uint8_t crc32_enable_new_state);
uint8_t rs485_is_crc32_enabled(void);
rs485_error_statistics_t rs485_get_error_statistics_and_optionally_reset(uint8_t reset);
uint8_t rs485_has_a_packet(void);
uint8_t rs485_get_next_packet(uint8_t *command, uint16_t *payload_size, uint8_t **payload, uint8_t *is_broadcast);
uint8_t rs485_validate_packet_crc32(void);
void rs485_done_with_this_packet_dont_disable_enable_irq(void);
void rs485_done_with_this_packet(void);
void rs485_transmit(void *s, uint8_t len);
void rs485_wait_for_transmit_done(void);
uint8_t rs485_transmit_not_done(void);
void rs485_transmit_no_error_packet(uint8_t is_broadcast);
void rs485_transmit_error_packet(uint8_t error_code);
void rs485_finalize_and_transmit_packet(void *data, uint16_t structure_size);
void rs485_start_the_packet(uint16_t payload_size);
void rs485_continue_the_packet(void *s, uint8_t len);
void rs485_end_the_packet(void);


#endif /* SRC_RS485_H_ */
