#include "stm32g0xx_hal.h"

// Peripheral instance definitions

#include <stdlib.h>
#include <stdio.h>

// Static peripheral instances and variables
uint32_t SystemCoreClock = 64000000U;  // After PLL configuration in clock_init()
const uint8_t AHBPrescTable[16] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U};

// Mock RCC registers with proper initialization
static RCC_TypeDef RCC_var = {
    .CR = RCC_CR_HSIRDY | RCC_CR_PLLON,  // HSI and PLL ready
    .PLLCFGR = 0,
    .CFGR = (2 << RCC_CFGR_SW_Pos),  // System clock switched to PLL (SW = 0b10)
    .IOPENR = 0x3f,  // Enable clocks for all GPIO ports (A-F)
    .APBENR2 = RCC_APBENR2_USART1EN_Msk | RCC_APBENR2_TIM16EN_Msk | RCC_APBENR2_TIM1EN_Msk | RCC_APBENR2_ADCEN_Msk,  // Enable USART1, TIM16, TIM1, and ADC clocks
    .APBENR1 = RCC_APBENR1_TIM3EN_Msk,  // Enable TIM3 clock
    .AHBENR = RCC_AHBENR_DMA1EN_Msk  // Enable DMA1 clock
};
// Mock GPIO registers
static GPIO_TypeDef GPIOA_var = {
    .MODER = (2U << (12 * 2)) |  // PA12 in AF mode for USART1_DE
             (2U << (8 * 2)) |   // PA8 in AF mode for TIM1_CH1
             (2U << (10 * 2)) |  // PA10 in AF mode for TIM1_CH3
             (2U << (11 * 2)) |  // PA11 in AF mode for TIM1_CH4
             (3U << (0 * 2)) |   // PA0 in analog mode for motor current
             (3U << (4 * 2)) |   // PA4 in analog mode for hall 2
             (3U << (5 * 2)) |   // PA5 in analog mode for hall 1
             (3U << (6 * 2)) |   // PA6 in analog mode for hall 3
             (3U << (7 * 2)) |   // PA7 in analog mode for supply voltage
             (3U << (9 * 2)) |   // PA9 in analog mode for temperature
             (1U << (15 * 2)),   // PA15 in output mode for motor enable
    .OTYPER = 0,   // Push-pull by default
    .OSPEEDR = 0,  // Low speed by default
    .PUPDR = 0,    // No pull-up/pull-down
    .IDR = 0,      // Input data register
    .ODR = 0,      // Output data register
    .BSRR = 0,     // Bit set/reset register
    .LCKR = 0,     // Configuration lock register
    .AFR = {0, 
            (1U << 16) |         // PA12 AF1 for USART1_DE
            (2U << 0) |          // PA8 AF2 for TIM1_CH1
            (2U << 8) |          // PA10 AF2 for TIM1_CH3
            (2U << 12)},         // PA11 AF2 for TIM1_CH4
    .BRR = 0       // Bit reset register
};
static GPIO_TypeDef GPIOB_var = {
    .MODER = (2U << 12) | (2U << 14) |  // PB6/PB7 in AF mode for USART1_TX/RX
             (2U << (3 * 2)) |          // PB3 in AF mode for TIM1_CH2
             (2U << (4 * 2)) |          // PB4 in AF mode for TIM3_CH1
             (2U << (5 * 2)) |          // PB5 in AF mode for TIM3_CH2
             (1U << (0 * 2)) |          // PB0 in output mode for stepper motor enable
             (0U << (1 * 2)) |          // PB1 in input mode for fault detection
             (1U << (2 * 2)),           // PB2 in output mode for motor sleep control
    .OTYPER = 0,
    .OSPEEDR = 0,
    .PUPDR = 0,
    .IDR = 0,
    .ODR = 0,
    .BSRR = (1 << 0),                   // Set PB0 high initially (MOSFETs disabled for M3)
    .LCKR = 0,
    .AFR = {(1U << (3 * 4)) |          // PB3 AF1 for TIM1_CH2
            (1U << (4 * 4)) |          // PB4 AF1 for TIM3_CH1
            (1U << (5 * 4)), 0},       // PB5 AF1 for TIM3_CH2
    .BRR = 0
};
static GPIO_TypeDef GPIOC_var = {
    .MODER = (1U << (6 * 2)),  // PC6 in output mode for red LED
    .OTYPER = 0,   // Push-pull output
    .OSPEEDR = 0,  // Low speed
    .PUPDR = 0,    // No pull-up/pull-down
    .IDR = 0,      // Input data register
    .ODR = 0,      // Output data register (LED off by default)
    .BSRR = 0,     // Bit set/reset register
    .LCKR = 0,     // Configuration lock register
    .AFR = {0, 0}, // No alternate functions
    .BRR = 0       // Bit reset register
};
static GPIO_TypeDef GPIOD_var = {
    .MODER = 0, .OTYPER = 0, .OSPEEDR = 0, .PUPDR = 0,
    .IDR = 0, .ODR = 0, .BSRR = 0, .LCKR = 0,
    .AFR = {0, 0}, .BRR = 0
};
// Mock EXTI registers
static EXTI_TypeDef EXTI_var = {
    .IMR1 = 0,
    .EMR1 = 0,
    .RTSR1 = 0,
    .FTSR1 = 0
};

// Mock ADC registers
static ADC_TypeDef ADC1_var = {
    .ISR = ADC_ISR_ADRDY | ADC_ISR_CCRDY,  // ADC ready and channel config ready
    .IER = ADC_IER_AWD2IE | ADC_IER_AWD3IE,  // Enable watchdog interrupts
    .CR = ADC_CR_ADVREGEN,  // Enable voltage regulator
    .CFGR1 = ADC_CFGR1_CHSELRMOD | ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD | ADC_CFGR1_DMACFG | ADC_CFGR1_DMAEN,  // Continuous mode, DMA circular mode
    .CFGR2 = 0,  // No oversampling
    .SMPR = 0,  // 1.5 ADC cycles sampling time
    .CHSELR = (0 << ADC_CHSELR_SQ1_Pos) |  // Motor current
              (5 << ADC_CHSELR_SQ2_Pos) |  // Hall 1
              (7 << ADC_CHSELR_SQ3_Pos) |  // Supply voltage
              (0 << ADC_CHSELR_SQ4_Pos) |  // Motor current
              (4 << ADC_CHSELR_SQ5_Pos) |  // Hall 2
              (10 << ADC_CHSELR_SQ6_Pos) | // Temperature
              (0 << ADC_CHSELR_SQ7_Pos) |  // Motor current
              (6 << ADC_CHSELR_SQ8_Pos),   // Hall 3
    .DR = 0,  // Data register
    .AWD2CR = 1,  // Monitor channel 0 (motor current) with watchdog 2
    .AWD3CR = (1 << 3),  // Monitor channel 3 with watchdog 3
    .TR2 = (0 << ADC_TR2_LT2_Pos) | (65535 << ADC_TR2_HT2_Pos),  // Watchdog thresholds
    .TR3 = (0 << ADC_TR3_LT3_Pos) | (65535 << ADC_TR3_HT3_Pos)   // Watchdog thresholds
};

static ADC_Common_TypeDef ADC_common_var = {
    .CCR = (1 << ADC_CCR_PRESC_Pos)  // Divide clock by 2 for 32MHz ADC clock
};

// Mock DMA registers
static DMA_Channel_TypeDef DMA1_Channel1_var = {
    .CCR = (1 << DMA_CCR_MSIZE_Pos) |  // 16-bit memory size
           (1 << DMA_CCR_PSIZE_Pos) |  // 16-bit peripheral size
           DMA_CCR_MINC |              // Memory increment mode
           DMA_CCR_CIRC |              // Circular mode
           DMA_CCR_EN,                 // Channel enabled
    .CNDTR = 32,  // Number of data items to transfer
    .CPAR = 0,    // Will be set to ADC1->DR by adc_init()
    .CMAR = 0     // Will be set to ADC_buffer by adc_init()
};

static DMAMUX_Channel_TypeDef DMAMUX1_Channel0_var = {
    .CCR = (5 << DMAMUX_CxCR_DMAREQ_ID_Pos)  // Select ADC as DMA request source
};

// Mock Timer registers
static TIM_TypeDef TIM1_var = {
    .CR1 = TIM_CR1_CEN,  // Counter enabled
    .CR2 = 0,
    .SMCR = 0,
    .DIER = 0,
    .SR = 0,
    .EGR = 0,
    .CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | (6 << TIM_CCMR1_OC2M_Pos),  // PWM mode 1
    .CCMR2 = (6 << TIM_CCMR2_OC3M_Pos) | (6 << TIM_CCMR2_OC4M_Pos),  // PWM mode 1
    .CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E,  // Enable outputs
    .CNT = 0,
    .PSC = 0,  // No prescaler
    .ARR = 65535,  // Max period
    .CCR1 = 0,
    .CCR2 = 0,
    .CCR3 = 0,
    .CCR4 = 0,
    .BDTR = TIM_BDTR_MOE | TIM_BDTR_AOE  // Main output enabled, automatic output enabled
};

static TIM_TypeDef TIM3_var = {
    .CR1 = TIM_CR1_CEN,  // Counter enabled
    .CR2 = 0,
    .SMCR = 0,
    .DIER = 0,
    .SR = 0,
    .CNT = 0,
    .PSC = 0,  // No prescaler
    .ARR = 65535,  // Max period
    .CCR1 = 0,
    .CCR2 = 0,
    .CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | (6 << TIM_CCMR1_OC2M_Pos),  // PWM mode 1
    .CCER = TIM_CCER_CC1E | TIM_CCER_CC2E  // Enable outputs
};

static TIM_TypeDef TIM14_var = {
    .CR1 = TIM_CR1_CEN,  // Counter enabled
    .CR2 = 0,
    .SMCR = 0,
    .DIER = 0,
    .SR = 0
};

static TIM_TypeDef TIM16_var = {
    .CR1 = TIM_CR1_CEN,  // Counter enabled
    .CR2 = 0,
    .SMCR = 0,
    .DIER = TIM_DIER_UIE,  // Update interrupt enabled
    .SR = 0,
    .CNT = 0,
    .PSC = 0,  // No prescaler
    .ARR = 2048  // Auto-reload value for 31.25kHz (64MHz/2048)
};

// Mock USART registers
static USART_TypeDef USART1_var = {
    .CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_FIFOEN | USART_CR1_RXNEIE_RXFNEIE | (0 << USART_CR1_DEAT_Pos) | (0 << USART_CR1_DEDT_Pos),
    .CR2 = USART_CR2_RTOEN,  // Enable timeout
    .CR3 = USART_CR3_DEM | USART_CR3_EIE | (0 << USART_CR3_DEP_Pos),  // Drive enable active high, enable drive enable, enable error interrupt
    .BRR = 278,  // 230400 baud @ 64MHz
    .ISR = USART_ISR_TXE_TXFNF_Msk,  // TX empty by default
    .RTOR = (230400 / 10) << USART_RTOR_RTO_Pos,  // 0.1s timeout
    .RDR = 0,  // Receive data register
    .TDR = 0,  // Transmit data register
    .ICR = 0,  // Interrupt clear register
    .RQR = 0   // Request register
};

static USART_TypeDef USART2_var = {
    .CR1 = 0,
    .CR2 = 0,
    .CR3 = 0,
    .BRR = 0,
    .ISR = USART_ISR_TXE_TXFNF_Msk  // TX empty by default
};

// Mock CRC registers
static CRC_TypeDef CRC_var = {
    .DR = 0,
    .IDR = 0,
    .CR = 0
};
// Mock FLASH registers
static FLASH_TypeDef FLASH_var = {
    .ACR = 0  // Will be set by clock_init()
};

// Mock SCB registers
static SCB_Type SCB_var = {
    .VTOR = 0,  // Will be set by firmware
    .SHPR[0] = 0,  // System handlers 4-7 priority
    .SHPR[1] = 0,  // System handlers 8-11 priority
    .SHPR[2] = 0   // System handlers 12-15 priority
};

// Mock NVIC registers
static NVIC_Type NVIC_var = {
    .ISER = {1U << TIM16_IRQn | 1U << ADC1_IRQn | 1U << USART1_IRQn},  // Enable TIM16, ADC1, and USART1 interrupts
    .IPR = {
        [TIM16_IRQn] = 1U << 6,  // TIM16 priority 1
        [ADC1_IRQn] = 0U << 6,   // ADC1 priority 0 (highest)
        [USART1_IRQn] = 2U << 6  // USART1 priority 2
    }
};

// Mock SysTick registers
static SysTick_Type SysTick_var = {
    .CTRL = 0,
    .LOAD = 0,
    .VAL = 0,
    .CALIB = 0
};

// Define the peripheral pointers
RCC_TypeDef *const RCC = &RCC_var;
GPIO_TypeDef *const GPIOA = &GPIOA_var;
GPIO_TypeDef *const GPIOB = &GPIOB_var;
GPIO_TypeDef *const GPIOC = &GPIOC_var;
GPIO_TypeDef *const GPIOD = &GPIOD_var;
EXTI_TypeDef *const EXTI = &EXTI_var;
ADC_TypeDef *const ADC1 = &ADC1_var;
ADC_Common_TypeDef *const ADC = &ADC_common_var;
DMA_Channel_TypeDef *const DMA1_Channel1 = &DMA1_Channel1_var;
DMAMUX_Channel_TypeDef *const DMAMUX1_Channel0 = &DMAMUX1_Channel0_var;
TIM_TypeDef *const TIM1 = &TIM1_var;
TIM_TypeDef *const TIM3 = &TIM3_var;
TIM_TypeDef *const TIM14 = &TIM14_var;
TIM_TypeDef *const TIM16 = &TIM16_var;
USART_TypeDef *const USART1 = &USART1_var;
USART_TypeDef *const USART2 = &USART2_var;
CRC_TypeDef *const CRC = &CRC_var;
FLASH_TypeDef *const FLASH = &FLASH_var;
SCB_Type *const SCB = &SCB_var;
NVIC_Type *const NVIC = &NVIC_var;
SysTick_Type *const SysTick = &SysTick_var;

// HAL variables
volatile uint32_t uwTick = 0;
uint32_t uwTickPrio = 0;
HAL_TickFreqTypeDef uwTickFreq = HAL_TICK_FREQ_DEFAULT;

// Flag to track if interrupts are enabled
volatile uint8_t g_interrupts_enabled = 0;  // Interrupts disabled by default

HAL_StatusTypeDef HAL_RCC_OscConfig(void* RCC_OscInitStruct) {
    return HAL_OK;
}

HAL_StatusTypeDef HAL_RCC_ClockConfig(void* RCC_ClkInitStruct, uint32_t FLatency) {
    return HAL_OK;
}

void HAL_RCC_GetOscConfig(void* RCC_OscInitStruct) {
}

void HAL_RCC_GetClockConfig(void* RCC_ClkInitStruct, uint32_t* pFLatency) {
}

uint32_t HAL_GetTick(void) {
    return uwTick;
}

void HAL_Delay(uint32_t Delay) {
    uint32_t tickstart = HAL_GetTick();
    uint32_t wait = Delay;

    // Add a freq to guarantee minimum wait
    if (wait < HAL_MAX_DELAY) {
        wait += 1;
    }

    while((HAL_GetTick() - tickstart) < wait) {
    }
}

void HAL_IncTick(void) {
    uwTick++;
}

void HAL_SuspendTick(void) {
}

void HAL_ResumeTick(void) {
}

uint32_t HAL_GetTickPrio(void) {
    return uwTickPrio;
}

HAL_StatusTypeDef HAL_SetTickFreq(HAL_TickFreqTypeDef Freq) {
    HAL_StatusTypeDef status = HAL_OK;
    HAL_TickFreqTypeDef prevTickFreq;

    if (uwTickFreq != Freq) {
        prevTickFreq = uwTickFreq;
        uwTickFreq = Freq;

        if (HAL_InitTick(uwTickPrio) != HAL_OK) {
            uwTickFreq = prevTickFreq;
            status = HAL_ERROR;
        }
    }

    return status;
}

HAL_TickFreqTypeDef HAL_GetTickFreq(void) {
    return uwTickFreq;
}

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority) {
    uwTickPrio = TickPriority;
    return HAL_OK;
}

// Implementation of core functions
void __enable_irq(void) {
    g_interrupts_enabled = 1;
}

void __disable_irq(void) {
    g_interrupts_enabled = 0;
}

// External declarations for reset flags
extern volatile int gExitFatalError;
extern volatile int gResetProgress;

// Implementation of NVIC_SystemReset
void NVIC_SystemReset(void) {
    printf("NVIC_SystemReset called\n");
    g_interrupts_enabled = 0;
    gResetProgress = 1; // Step 1: Waiting for fatal_error and TIM16_IRQHandler to exit
    gExitFatalError = 1;  // Set flag to exit fatal error state
    printf("gResetProgress = %d, gExitFatalError = %d\n",
           gResetProgress, gExitFatalError);
}
