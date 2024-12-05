#ifndef __GC6609_H__
#define __GC6609_H__

#define GC6609 // After July 23, 2024, we are using the GC6609 stepper motor driver chip rather than the A4988 chip

#define bool uint8_t
#define true 1
#define false 0

#define test_motor_stepping() test_motor_stepping_GC6609()

void init_GC6609_through_UART(void);
void power_on_GC6609(void);
void power_off_GC6609(void);
void reset_GC6609(void);
bool GC6608_UART_bit_bang_read_registers(uint8_t *returned_data, uint16_t *returned_data_size);
void test_motor_stepping_GC6609(void);

#endif