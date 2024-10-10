#include "stm32g0xx_hal.h"

struct GPIO_TypeDef GPIOA_var;
struct GPIO_TypeDef GPIOB_var;
struct TIM_TypeDef TIM16_var;
struct TIM_TypeDef TIM1_var;
struct TIM_TypeDef TIM3_var;
struct RCC_TypeDef RCC_var;

struct GPIO_TypeDef* GPIOA = &GPIOA_var;
struct GPIO_TypeDef* GPIOB = &GPIOB_var;
struct TIM_TypeDef* TIM16 = &TIM16_var;
struct TIM_TypeDef* TIM1 = &TIM1_var;
struct TIM_TypeDef* TIM3 = &TIM3_var;
struct RCC_TypeDef* RCC = &RCC_var;

void NVIC_SetPriority(int irq, int priority) {}
void NVIC_EnableIRQ(int irq) {}
void __disable_irq(void) {}
void __enable_irq(void) {}
