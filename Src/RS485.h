
#ifndef SRC_RS485_H_
#define SRC_RS485_H_

#include "settings.h"

#define ALL_ALIAS 255 // to address all devices on the bus at the same time

#define USART1_TIMEOUT 5 // one count is 10ms, so this is a timeout of about 50 ms

#define MAX_VALUE_BUFFER_LENGTH (FLASH_PAGE_SIZE + 1) // Maximum length of any value that this controller might need to process
                                                      // Values longer than this may be valid for other devices and will be ignored

void rs485_init(void);
void rs485_transmit(void *s, uint8_t len);


#endif /* SRC_RS485_H_ */
