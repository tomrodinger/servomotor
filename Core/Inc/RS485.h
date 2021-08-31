
#ifndef SRC_RS485_H_
#define SRC_RS485_H_

#define MAX_VALUE_BUFFER_LENGTH 8 // Maximum length of any value that this controller might need to process
                                  // Values longer than this may be valid for other devices and will be ignored

void rs485_init(void);
void rs485_transmit(void *s, uint8_t len);


#endif /* SRC_RS485_H_ */
