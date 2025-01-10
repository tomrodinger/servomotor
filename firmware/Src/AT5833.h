#ifndef __AT5833_H__
#define __AT5833_H__

#define AT5833 // experimental usage of this driver chip (Oct. 18, 2024)

//#define bool uint8_t
#define true 1
#define false 0

#define test_motor_stepping() test_motor_stepping_AT5833()

void power_on_AT5833(void);
void power_off_AT5833(void);
void reset_AT5833(void);
void test_motor_stepping_AT5833(void);

#endif