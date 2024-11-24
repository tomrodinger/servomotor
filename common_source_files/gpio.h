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

#ifdef PRODUCT_NAME_M1
#define GPIO_init() GPIO_init_M1()
#define get_button_state() get_button_state_M1()
#endif
#ifdef PRODUCT_NAME_M2
#define GPIO_init() GPIO_init_M2()
#define get_button_state() get_button_state_M2()
#endif
#ifdef PRODUCT_NAME_M3
#define GPIO_init() GPIO_init_M3()
#define get_button_state() get_button_state_M3()
#endif
#ifdef PRODUCT_NAME_M4
#define GPIO_init() GPIO_init_M4()
#define get_button_state() get_button_state_M4()
#endif

void GPIO_init_M1(void);
void GPIO_init_M2(void);
void GPIO_init_M3(void);
void GPIO_init_M4(void);
uint8_t get_button_state_M1(void);
uint8_t get_button_state_M2(void);
uint8_t get_button_state_M3(void);
uint8_t get_button_state_M4(void);
