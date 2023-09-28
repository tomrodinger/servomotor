#ifndef SRC_DEBUG_UART_H_
#define SRC_DEBUG_UART_H_

#include <stdint.h>

void debug_uart_init(void);
void convert_uint16_to_byte_array(char *output, uint16_t input);
void transmit(void *s, uint8_t len);
void print_debug_string(void *s);
void transmit_without_interrupts(const char *message, uint8_t len);
void print_number(char *message_prefix, uint16_t n);
void print_int64(char *message_prefix, int64_t n);
uint8_t get_command_debug_uart(void);


#endif /* SRC_DEBUG_UART_H_ */
