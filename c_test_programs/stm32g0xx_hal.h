#ifndef __STM32G0xx_HAL_H
#define __STM32G0xx_HAL_H

#include <stdint.h>

struct GPIO_TypeDef {
    volatile uint32_t MODER;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    // Add other relevant GPIO registers here
};

struct TIM_TypeDef {
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t CR1;
    volatile uint32_t SR;
    volatile uint32_t DIER;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    // Add other relevant Timer registers here
};

struct RCC_TypeDef {
    volatile uint32_t APBENR2; // Add relevant RCC registers
};

extern struct GPIO_TypeDef* GPIOA;
extern struct GPIO_TypeDef* GPIOB;
extern struct TIM_TypeDef* TIM16;
extern struct TIM_TypeDef* TIM1;
extern struct TIM_TypeDef* TIM3;
extern struct RCC_TypeDef* RCC;


#define TIM_DIER_UIE 0
#define FLASH_PAGE_SIZE 0
#define RCC_APBENR2_TIM16EN 0
#define TIM_CR1_CEN 0
#define TIM16_IRQn 0
#define TIM16_IRQn 0

void NVIC_SetPriority(int irq, int priority);
void NVIC_EnableIRQ(int irq);
void __disable_irq(void);
void __enable_irq(void);

#endif // __STM32G0xx_HAL_H
