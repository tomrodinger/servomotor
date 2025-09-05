#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>

#define MODER_DIGITAL_INPUT 0
#define MODER_DIGITAL_OUTPUT 1
#define MODER_ALTERNATE_FUNCTION 2
#define MODER_ANALOG_INPUT 3 // this is the default after power up
#define OTYPER_PUSH_PULL 0 // this is the default after power up
#define OTYPER_OPEN_DRAIN 1
#define OSPEEDR_VERY_LOW_SPEED 0 // this is the default except some pins on port A
#define OSPEEDR_LOW_SPEED 1
#define OSPEEDR_HIGH_SPEED 2
#define OSPEEDR_VERY_HIGH_SPEED 3
#define PUPDR_NO_PULL_UP_OR_DOWN 0 // this is the default except on some pins on port A
#define PUPDR_PULL_UP 1
#define PUPDR_PULL_DOWN 2

// Check for valid product define
#if defined(PRODUCT_NAME_M1)
    #define GPIO_init() GPIO_init_M1()
    #define get_button_state() get_button_state_M1()
#elif defined(PRODUCT_NAME_M2)
    #define GPIO_init() GPIO_init_M2()
    #define get_button_state() get_button_state_M2()
#elif defined(PRODUCT_NAME_M17)
    #define GPIO_init() GPIO_init_M17()
    #define get_button_state() get_button_state_M17()
#elif defined(PRODUCT_NAME_M23)
    #define GPIO_init() GPIO_init_M23()
    #define get_button_state() get_button_state_M23()
#else
    #error "Invalid or missing PRODUCT_NAME_X define. Must be one of: PRODUCT_NAME_M1, PRODUCT_NAME_M2, PRODUCT_NAME_M17, PRODUCT_NAME_M23"
#endif

void GPIO_init_M1(void);
void GPIO_init_M2(void);
void GPIO_init_M17(void);
void GPIO_init_M23(void);
uint8_t get_button_state_M1(void);
uint8_t get_button_state_M2(void);
uint8_t get_button_state_M17(void);
uint8_t get_button_state_M23(void);

#endif // GPIO_H
