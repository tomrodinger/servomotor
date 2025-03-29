/**
  ******************************************************************************
  * @file    stm32g0xx_hal.h
  * @author  MCD Application Team
  * @brief   This file contains all the functions prototypes for the HAL
  *          module driver.
  ******************************************************************************
  */

#ifndef STM32G0xx_HAL_H
#define STM32G0xx_HAL_H

// External declarations for interrupt handlers implemented in RS485.c and motor_control.c
extern void USART1_IRQHandler(void);
extern void TIM16_IRQHandler(void);

// When building for the actual motor firmware, these are no-ops.
// When building the simulator, these functions handle the simulation environment:
// - motor_simulator_init(): initializes the simulation
// - motor_simulator_logic(): runs simulation logic, timing, and visualization
void motor_simulator_init(void);
void motor_simulator_visualization(void);

#ifdef __cplusplus
extern "C" {
#endif

// Priority definitions
#define TICK_INT_PRIORITY            ((uint32_t)3U)  // SysTick interrupt priority
#define SYSTICK_CLKSOURCE_HCLK      0U              // SysTick clock source

#include <stdint.h>
#include <stdbool.h>

#define __IO volatile

// Register bit definitions
// RCC defines
#define RCC_AHBENR_DMA1EN              (1 << 0)
#define RCC_AHBENR_DMA1EN_Msk          RCC_AHBENR_DMA1EN
#define RCC_AHBENR_CRCEN               (1 << 12)  // CRC clock enable
#define RCC_APBENR2_ADCEN              (1 << 20)
#define RCC_APBENR2_ADCEN_Msk          RCC_APBENR2_ADCEN
#define RCC_APBENR2_TIM1EN             (1 << 11)  // TIM1 clock enable
#define RCC_APBENR2_TIM1EN_Msk         RCC_APBENR2_TIM1EN
#define RCC_APBENR2_TIM14EN            (1 << 15)  // TIM14 clock enable
#define RCC_APBENR2_TIM16EN            (1 << 17)  // TIM16 clock enable
#define RCC_APBENR2_TIM16EN_Msk        RCC_APBENR2_TIM16EN
#define RCC_APBENR2_USART1EN_Msk       (1 << 14)  // USART1 clock enable
#define RCC_APBENR1_USART2EN_Msk       (1 << 17)  // USART2 clock enable
#define RCC_APBENR1_TIM3EN             (1 << 1)   // TIM3 clock enable
#define RCC_APBENR1_TIM3EN_Msk         RCC_APBENR1_TIM3EN

// RCC Internal Clock Source Calibration Register defines
#define RCC_ICSCR_HSITRIM_Pos          24U        // HSI clock trimming
#define RCC_CCIPR_ADCSEL_Pos           30
#define RCC_CCIPR_ADCSEL_Msk           (0x3 << RCC_CCIPR_ADCSEL_Pos)

// ADC defines
#define ADC_CCR_PRESC_Pos              18
#define ADC_CCR_PRESC_Msk              (0x7 << ADC_CCR_PRESC_Pos)

#define ADC_CFGR1_CHSELRMOD            (1 << 21)
#define ADC_CFGR1_CONT                 (1 << 13)
#define ADC_CFGR1_OVRMOD               (1 << 12)
#define ADC_CFGR1_DMACFG               (1 << 1)
#define ADC_CFGR1_AWD1EN               (1 << 23)
#define ADC_CFGR1_AWD1SGL              (1 << 22)
#define ADC_CFGR1_AWD1CH_Pos           26
#define ADC_CFGR1_DMAEN                (1 << 0)

#define ADC_CFGR2_CKMODE_Pos           30
#define ADC_SMPR_SMP1_Pos              0

#define ADC_TR2_LT2_Pos                0
#define ADC_TR2_HT2_Pos                16
#define ADC_TR3_LT3_Pos                0
#define ADC_TR3_HT3_Pos                16

#define ADC_CHSELR_SQ1_Pos             0
#define ADC_CHSELR_SQ2_Pos             4
#define ADC_CHSELR_SQ3_Pos             8
#define ADC_CHSELR_SQ4_Pos             12
#define ADC_CHSELR_SQ5_Pos             16
#define ADC_CHSELR_SQ6_Pos             20
#define ADC_CHSELR_SQ7_Pos             24
#define ADC_CHSELR_SQ8_Pos             28

#define ADC_CR_ADVREGEN                (1 << 28)
#define ADC_CR_ADCAL                   (1 << 31)
#define ADC_CR_ADEN                    (1 << 0)
#define ADC_CR_ADSTART                 (1 << 2)

#define ADC_ISR_ADRDY                  (1 << 0)
#define ADC_ISR_CCRDY                  (1 << 13)
#define ADC_ISR_AWD2                   (1 << 8)
#define ADC_ISR_AWD3                   (1 << 9)
#define ADC_IER_AWD2IE                 (1 << 8)  // Analog watchdog 2 interrupt enable
#define ADC_IER_AWD3IE                 (1 << 9)  // Analog watchdog 3 interrupt enable

// Timer Output Compare Mode defines
#define TIM_CCMR1_OC2M_Pos            12U
#define TIM_CCMR2_OC3M_Pos            4U
#define TIM_CCMR2_OC4M_Pos            12U

// Timer Capture/Compare Enable defines
#define TIM_CCER_CC2E                  (1 << 4)  // Capture/Compare 2 output enable
#define TIM_CCER_CC3E                  (1 << 8)  // Capture/Compare 3 output enable

// Timer Break and Dead-time register defines
#define TIM_BDTR_AOE                   (1 << 14) // Automatic output enable

// DMA defines
#define DMA_CNDTR_NDT_Pos              0U        // Number of data to transfer position
#define DMA_CNDTR_NDT_Msk              (0xFFFF << DMA_CNDTR_NDT_Pos)

#define DMA_CCR_EN                     (1 << 0)  // Channel enable
#define DMA_CCR_TCIE                   (1 << 1)  // Transfer complete interrupt enable
#define DMA_CCR_HTIE                   (1 << 2)  // Half transfer interrupt enable
#define DMA_CCR_TEIE                   (1 << 3)  // Transfer error interrupt enable
#define DMA_CCR_DIR                    (1 << 4)  // Data transfer direction
#define DMA_CCR_CIRC                   (1 << 5)  // Circular mode
#define DMA_CCR_PINC                   (1 << 6)  // Peripheral increment mode
#define DMA_CCR_MINC                   (1 << 7)  // Memory increment mode
#define DMA_CCR_PSIZE_Pos              8U        // Peripheral size
#define DMA_CCR_MSIZE_Pos              10U       // Memory size
#define DMA_CCR_PL_Pos                 12U       // Channel priority level
#define DMA_CCR_MEM2MEM                (1 << 14) // Memory to memory mode

// EXTI defines
#define EXTI_IMR1_IM4                  (1 << 4)  // Interrupt Mask on line 4
#define EXTI_IMR1_IM8                  (1 << 8)  // Interrupt Mask on line 8
#define EXTI_RTSR1_RT4                 (1 << 4)  // Rising trigger event on line 4
#define EXTI_RTSR1_RT8                 (1 << 8)  // Rising trigger event on line 8
#define EXTI_RPR1_RPIF4                (1 << 4)  // Rising edge pending flag on line 4
#define EXTI_RPR1_RPIF8                (1 << 8)  // Rising edge pending flag on line 8
#define EXTI_EXTICR2_EXTI4_Pos         0U        // EXTI4 configuration bits position
#define EXTI_EXTICR3_EXTI8_Pos         0U        // EXTI8 configuration bits position

// Basic type definitions
typedef enum IRQn {
    NonMaskableInt_IRQn = -14,
    HardFault_IRQn = -13,
    SysTick_IRQn = -1,
    EXTI4_15_IRQn = 7,    // Added EXTI IRQ
    ADC1_IRQn = 12,       // Added ADC1 IRQ
    TIM14_IRQn = 19,      // Added TIM14 IRQ
    TIM16_IRQn = 21,      // Added TIM16 IRQ
    USART2_IRQn = 26,     // Added USART2 IRQ
    USART1_IRQn = 27
} IRQn_Type;

// NVIC function declarations
static inline void NVIC_EnableIRQ(IRQn_Type IRQn) {
    if ((int32_t)(IRQn) >= 0) {
        // Implementation for simulator
    }
}

static inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority) {
    if ((int32_t)(IRQn) >= 0) {
        // Implementation for simulator
    }
}

void NVIC_SystemReset(void);

// GPIO defines
#define GPIO_MODER_MODE0_Pos           0U
#define GPIO_MODER_MODE1_Pos           2U
#define GPIO_MODER_MODE2_Pos           4U
#define GPIO_MODER_MODE3_Pos           6U
#define GPIO_MODER_MODE4_Pos           8U
#define GPIO_MODER_MODE5_Pos           10U
#define GPIO_MODER_MODE6_Pos           12U
#define GPIO_MODER_MODE7_Pos           14U
#define GPIO_MODER_MODE8_Pos           16U
#define GPIO_MODER_MODE9_Pos           18U
#define GPIO_MODER_MODE10_Pos          20U
#define GPIO_MODER_MODE11_Pos          22U
#define GPIO_MODER_MODE12_Pos          24U
#define GPIO_MODER_MODE13_Pos          26U
#define GPIO_MODER_MODE14_Pos          28U
#define GPIO_MODER_MODE15_Pos          30U

// GPIO Output Type Register (OTYPER) bit positions
#define GPIO_OTYPER_OT0_Pos            0U
#define GPIO_OTYPER_OT1_Pos            1U
#define GPIO_OTYPER_OT2_Pos            2U
#define GPIO_OTYPER_OT3_Pos            3U
#define GPIO_OTYPER_OT4_Pos            4U
#define GPIO_OTYPER_OT5_Pos            5U
#define GPIO_OTYPER_OT6_Pos            6U
#define GPIO_OTYPER_OT7_Pos            7U
#define GPIO_OTYPER_OT8_Pos            8U
#define GPIO_OTYPER_OT9_Pos            9U
#define GPIO_OTYPER_OT10_Pos           10U
#define GPIO_OTYPER_OT11_Pos           11U
#define GPIO_OTYPER_OT12_Pos           12U
#define GPIO_OTYPER_OT13_Pos           13U
#define GPIO_OTYPER_OT14_Pos           14U
#define GPIO_OTYPER_OT15_Pos           15U

// GPIO Pull-up/Pull-down Register (PUPDR) bit positions
#define GPIO_PUPDR_PUPD0_Pos           0U
#define GPIO_PUPDR_PUPD1_Pos           2U
#define GPIO_PUPDR_PUPD2_Pos           4U
#define GPIO_PUPDR_PUPD3_Pos           6U
#define GPIO_PUPDR_PUPD4_Pos           8U
#define GPIO_PUPDR_PUPD5_Pos           10U
#define GPIO_PUPDR_PUPD6_Pos           12U
#define GPIO_PUPDR_PUPD7_Pos           14U
#define GPIO_PUPDR_PUPD8_Pos           16U
#define GPIO_PUPDR_PUPD9_Pos           18U
#define GPIO_PUPDR_PUPD10_Pos          20U
#define GPIO_PUPDR_PUPD11_Pos          22U
#define GPIO_PUPDR_PUPD12_Pos          24U
#define GPIO_PUPDR_PUPD13_Pos          26U
#define GPIO_PUPDR_PUPD14_Pos          28U
#define GPIO_PUPDR_PUPD15_Pos          30U

// GPIO AFR (Alternate Function Register) defines
#define GPIO_AFRH_AFSEL8_Pos          0U
#define GPIO_AFRH_AFSEL11_Pos         12U
#define GPIO_AFRH_AFSEL12_Pos         16U
#define GPIO_AFRL_AFSEL2_Pos          8U
#define GPIO_AFRL_AFSEL3_Pos          12U
#define GPIO_AFRL_AFSEL6_Pos          24U
#define GPIO_AFRL_AFSEL7_Pos          28U

// Timer defines
#define TIM_SR_UIF                     (1 << 0)  // Update interrupt flag
#define TIM_SR_CC1IF                   (1 << 1)  // Capture/Compare 1 interrupt flag

// Timer Control defines
#define TIM_CR1_CEN                    (1 << 0)  // Counter enable
#define TIM_CR1_UIFREMAP              (1 << 11) // UIF status bit remap
#define TIM_EGR_UG                     (1 << 0)  // Update generation
#define TIM_DIER_UIE                   (1 << 0)  // Update interrupt enable

// Timer Output Compare Mode defines
#define TIM_CCMR1_OC1M_Pos            4U
#define TIM_CCMR2_OC4M_Pos            12U

// Timer Capture/Compare Enable defines
#define TIM_CCER_CC1E                  (1 << 0)  // Capture/Compare 1 output enable
#define TIM_CCER_CC4E                  (1 << 12) // Capture/Compare 4 output enable

// Timer Break and Dead-time register defines
#define TIM_BDTR_MOE                   (1 << 15) // Main output enable
#define TIM_SR_CC2IF                   (1 << 2)  // Capture/Compare 2 interrupt flag
#define TIM_SR_CC3IF                   (1 << 3)  // Capture/Compare 3 interrupt flag
#define TIM_SR_CC4IF                   (1 << 4)  // Capture/Compare 4 interrupt flag
#define TIM_SR_COMIF                   (1 << 5)  // COM interrupt flag
#define TIM_SR_TIF                     (1 << 6)  // Trigger interrupt flag
#define TIM_SR_BIF                     (1 << 7)  // Break interrupt flag
#define TIM_SR_CC1OF                   (1 << 9)  // Capture/Compare 1 overcapture flag
#define TIM_SR_CC2OF                   (1 << 10) // Capture/Compare 2 overcapture flag
#define TIM_SR_CC3OF                   (1 << 11) // Capture/Compare 3 overcapture flag
#define TIM_SR_CC4OF                   (1 << 12) // Capture/Compare 4 overcapture flag

// DMAMUX defines
#define DMAMUX_CxCR_DMAREQ_ID_Pos      0U
#define DMAMUX_CxCR_DMAREQ_ID_Msk      (0xFF << DMAMUX_CxCR_DMAREQ_ID_Pos)
#define DMAMUX_CxCR_SOIE               (1 << 8)  // Synchronization overrun interrupt enable
#define DMAMUX_CxCR_EGE                (1 << 9)  // Event generation enable
#define DMAMUX_CxCR_SE                 (1 << 16) // Synchronization enable
#define DMAMUX_CxCR_SPOL_Pos           17U       // Synchronization polarity
#define DMAMUX_CxCR_NBREQ_Pos          19U       // Number of DMA requests minus 1 to forward
#define DMAMUX_CxCR_SYNC_ID_Pos        24U       // Synchronization identification

// Basic defines
#define __ASM __asm
#define __STATIC_INLINE static inline
#define __PACKED __attribute__((packed))
#define __WEAK __attribute__((weak))
#define HAL_MAX_DELAY      0xFFFFFFFFU
#define __COMPILER_BARRIER() __asm volatile ("" ::: "memory")
#define __NOP() __asm volatile ("nop")

// Flash defines
#define FLASH_PAGE_SIZE    2048U  // 2KB page size for STM32G031

// Flash ACR (Access Control Register) defines
#define FLASH_ACR_LATENCY_Pos         0U
#define FLASH_ACR_PRFTEN              (1 << 8)   // Prefetch enable
#define FLASH_ACR_ICEN                (1 << 9)   // Instruction cache enable

// Flash Status Register (SR) defines
#define FLASH_SR_BSY1_Msk             (1 << 0)   // Flash busy
#define FLASH_SR_EOP_Msk              (1 << 16)  // End of operation

// Flash Control Register (CR) defines
#define FLASH_CR_PG                   (1 << 0)   // Programming
#define FLASH_CR_PER                  (1 << 1)   // Page erase
#define FLASH_CR_STRT                 (1 << 16)  // Start operation
#define FLASH_CR_EOPIE                (1 << 24)  // End of operation interrupt enable
#define FLASH_CR_LOCK                 (1 << 31)  // Lock
#define FLASH_CR_LOCK_Msk             FLASH_CR_LOCK
#define FLASH_CR_PNB_Pos              3U         // Page number position

// CRC Control Register (CR) defines
#define CRC_CR_RESET_Msk              (1 << 0)   // Reset CRC calculation unit
#define CRC_CR_REV_OUT_Msk            (1 << 7)   // Reverse output data
#define CRC_CR_REV_IN_0               (1 << 5)   // Reverse input data bit 0
#define CRC_CR_REV_IN_1               (1 << 6)   // Reverse input data bit 1

// RCC PLL configuration defines
#define RCC_PLLCFGR_PLLSRC_HSI       0x2U
#define RCC_PLLCFGR_PLLM_Pos         4U
#define RCC_PLLCFGR_PLLN_Pos         8U
#define RCC_PLLCFGR_PLLP_Pos         17U
#define RCC_PLLCFGR_PLLR_Pos         25U
#define RCC_PLLCFGR_PLLPEN           (1 << 16)
#define RCC_PLLCFGR_PLLREN           (1 << 24)

// RCC Clock Control defines
#define RCC_CR_HSION                  (1 << 8)
#define RCC_CR_HSIRDY                 (1 << 10)
#define RCC_CR_PLLON                  (1 << 24)

// RCC Clock Configuration defines
#define RCC_CFGR_SW_Pos              0U
#define RCC_CFGR_SW_Msk              (0x7U << RCC_CFGR_SW_Pos)
#define RCC_CCIPR_USART1SEL_Pos      16U

// USART defines
#define USART_CR1_UE                  (1 << 0)   // USART enable
#define USART_CR1_RE                  (1 << 2)   // Receiver enable
#define USART_CR1_TE                  (1 << 3)   // Transmitter enable
#define USART_CR1_FIFOEN             (1 << 29)  // FIFO mode enable
#define USART_CR1_DEAT_Pos           21U        // Driver enable assertion time
#define USART_CR1_DEDT_Pos           16U        // Driver enable deassertion time
#define USART_CR1_RXNEIE_RXFNEIE     (1 << 5)   // RXNE/RXFIFO not empty interrupt enable
#define USART_CR1_TXFEIE             (1 << 30)  // TXFIFO empty interrupt enable
#define USART_CR1_TXEIE_TXFNFIE_Msk  (1 << 7)   // TXFIFO not full interrupt enable

#define USART_CR2_RTOEN              (1 << 23)  // Receiver timeout enable

#define USART_CR3_EIE                (1 << 0)   // Error interrupt enable
#define USART_CR3_DEM                (1 << 14)  // Driver enable mode
#define USART_CR3_DEP_Pos            15U        // Driver enable polarity selection

#define USART_ISR_FE                 (1 << 1)   // Framing error
#define USART_ISR_ORE                (1 << 3)   // Overrun error
#define USART_ISR_NE                 (1 << 2)   // Noise error
#define USART_ISR_RTOF               (1 << 11)  // Receiver timeout flag
#define USART_ISR_RXNE_RXFNE         (1 << 5)   // Read data register not empty
#define USART_ISR_TXE_TXFNF_Msk      (1 << 7)   // Transmit data register empty
#define USART_ISR_TC_Pos             (6U)
#define USART_ISR_TC_Msk             (0x1UL << USART_ISR_TC_Pos)

#define USART_ICR_RTOCF              (1 << 11)  // Receiver timeout clear flag

#define USART_RTOR_RTO_Pos           0U         // Receiver timeout value

// RCC APB/AHB Enable defines
#define RCC_APBENR2_SYSCFGEN_Pos     0U

// SysTick Control defines
#define SysTick_CTRL_CLKSOURCE_Msk   (1 << 2)
#define SysTick_CTRL_TICKINT_Msk     (1 << 1)
#define SysTick_CTRL_ENABLE_Msk      (1 << 0)

// Handle section attributes for macOS
#ifdef __APPLE__
#define __RAM_FUNC __attribute__((section("__TEXT,__text")))
#define section(x)
#else
#define __RAM_FUNC __attribute__((section(".RamFunc")))
#endif

// Basic type definitions
typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;
typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;
typedef enum {GPIO_PIN_RESET = 0, GPIO_PIN_SET} GPIO_PinState;

typedef enum {
    HAL_OK       = 0x00U,
    HAL_ERROR    = 0x01U,
    HAL_BUSY     = 0x02U,
    HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;

typedef enum {
    HAL_TICK_FREQ_10HZ         = 100U,
    HAL_TICK_FREQ_100HZ        = 10U,
    HAL_TICK_FREQ_1KHZ         = 1U,
    HAL_TICK_FREQ_DEFAULT      = HAL_TICK_FREQ_1KHZ
} HAL_TickFreqTypeDef;


// Peripheral structure definitions
typedef struct {
    __IO uint32_t CR;
    __IO uint32_t ICSCR;    // Internal Clock Source Calibration Register
    __IO uint32_t CFGR;
    __IO uint32_t PLLCFGR;
    __IO uint32_t CIR;
    __IO uint32_t AHBENR;
    __IO uint32_t APBENR1;  // APB1 peripheral clock enable register
    __IO uint32_t APB2ENR;
    __IO uint32_t BDCR;
    __IO uint32_t CSR;
    __IO uint32_t AHBRSTR;
    __IO uint32_t CFGR2;
    __IO uint32_t CFGR3;
    __IO uint32_t CR2;
    __IO uint32_t APBENR2;
    __IO uint32_t CCIPR;
    __IO uint32_t IOPENR;  // Added IO port clock enable register
} RCC_TypeDef;

typedef struct {
    __IO uint32_t MODER;
    __IO uint32_t OTYPER;
    __IO uint32_t OSPEEDR;
    __IO uint32_t PUPDR;
    __IO uint32_t IDR;
    __IO uint32_t ODR;
    __IO uint32_t BSRR;
    __IO uint32_t LCKR;
    __IO uint32_t AFR[2];
    __IO uint32_t BRR;
} GPIO_TypeDef;

typedef struct {
    __IO uint32_t IMR1;    // Interrupt mask register
    __IO uint32_t EMR1;    // Event mask register
    __IO uint32_t RTSR1;   // Rising trigger selection register
    __IO uint32_t FTSR1;   // Falling trigger selection register
    __IO uint32_t SWIER1;  // Software interrupt event register
    __IO uint32_t RPR1;    // Rising edge pending register
    __IO uint32_t FPR1;    // Falling edge pending register
    uint32_t RESERVED1[8];
    __IO uint32_t EXTICR[4]; // External interrupt configuration registers
    uint32_t RESERVED2[4];
    __IO uint32_t IMR2;    // Interrupt mask register 2
    __IO uint32_t EMR2;    // Event mask register 2
    __IO uint32_t RTSR2;   // Rising trigger selection register 2
    __IO uint32_t FTSR2;   // Falling trigger selection register 2
    __IO uint32_t SWIER2;  // Software interrupt event register 2
    __IO uint32_t RPR2;    // Rising edge pending register 2
    __IO uint32_t FPR2;    // Falling edge pending register 2
} EXTI_TypeDef;

typedef struct {
    __IO uint32_t ISR;
    __IO uint32_t IER;
    __IO uint32_t CR;
    __IO uint32_t CFGR1;
    __IO uint32_t CFGR2;
    __IO uint32_t SMPR;
    uint32_t  RESERVED1;
    uint32_t  RESERVED2;
    __IO uint32_t TR1;
    __IO uint32_t TR2;
    __IO uint32_t TR3;
    uint32_t  RESERVED3;
    __IO uint32_t CHSELR;
    __IO uint32_t DR;
    uint32_t  RESERVED4[177];
    __IO uint32_t AWD2CR;
    __IO uint32_t AWD3CR;
} ADC_TypeDef;

typedef struct {
    __IO uint32_t CCR;
} ADC_Common_TypeDef;

typedef struct {
    __IO uint32_t CCR;
    __IO uint32_t CNDTR;
    __IO uint32_t CPAR;
    __IO uint32_t CMAR;
} DMA_Channel_TypeDef;

typedef struct {
    __IO uint32_t CCR;
} DMAMUX_Channel_TypeDef;

typedef struct {
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t SMCR;
    __IO uint32_t DIER;
    __IO uint32_t SR;
    __IO uint32_t EGR;
    __IO uint32_t CCMR1;
    __IO uint32_t CCMR2;
    __IO uint32_t CCER;
    __IO uint32_t CNT;
    __IO uint32_t PSC;
    __IO uint32_t ARR;
    __IO uint32_t RCR;
    __IO uint32_t CCR1;
    __IO uint32_t CCR2;
    __IO uint32_t CCR3;
    __IO uint32_t CCR4;
    __IO uint32_t BDTR;
    __IO uint32_t DCR;
    __IO uint32_t DMAR;
} TIM_TypeDef;

typedef struct {
    __IO uint32_t CR1;        // Control register 1
    __IO uint32_t CR2;        // Control register 2
    __IO uint32_t CR3;        // Control register 3
    __IO uint32_t BRR;        // Baud rate register
    __IO uint32_t GTPR;       // Guard time and prescaler register
    __IO uint32_t RTOR;       // Receiver timeout register
    __IO uint32_t RQR;        // Request register
    __IO uint32_t ISR;        // Interrupt and status register
    __IO uint32_t ICR;        // Interrupt flag clear register
    __IO uint32_t RDR;        // Receive data register
    __IO uint32_t TDR;        // Transmit data register
} USART_TypeDef;

typedef struct {
    __IO uint32_t DR;
    __IO uint32_t IDR;
    __IO uint32_t CR;
    uint32_t RESERVED0;
    __IO uint32_t INIT;
    __IO uint32_t POL;
} CRC_TypeDef;

typedef struct {
    __IO uint32_t ACR;
    __IO uint32_t KEYR;
    __IO uint32_t OPTKEYR;
    __IO uint32_t SR;
    __IO uint32_t CR;
    __IO uint32_t ECCR;
    __IO uint32_t OPTR;
    __IO uint32_t PCROP1SR;
    __IO uint32_t PCROP1ER;
    __IO uint32_t WRP1AR;
    __IO uint32_t WRP1BR;
} FLASH_TypeDef;

typedef struct {
    __IO uint32_t CPUID;
    __IO uint32_t ICSR;
    __IO uint32_t VTOR;
    __IO uint32_t AIRCR;
    __IO uint32_t SCR;
    __IO uint32_t CCR;
    __IO uint32_t SHPR[3];  // Changed to size 3 for handlers 4-15
    __IO uint32_t SHCSR;
    __IO uint32_t CFSR;
    __IO uint32_t HFSR;
    __IO uint32_t DFSR;
    __IO uint32_t MMFAR;
    __IO uint32_t BFAR;
    __IO uint32_t AFSR;
} SCB_Type;

typedef struct {
    __IO uint32_t CTRL;
    __IO uint32_t LOAD;
    __IO uint32_t VAL;
    __IO uint32_t CALIB;
} SysTick_Type;

typedef struct {
    __IO uint32_t ISER[1];                 /*!< Interrupt Set Enable Register */
         uint32_t RESERVED0[31];
    __IO uint32_t ICER[1];                 /*!< Interrupt Clear Enable Register */
         uint32_t RESERVED1[31];
    __IO uint32_t ISPR[1];                 /*!< Interrupt Set Pending Register */
         uint32_t RESERVED2[31];
    __IO uint32_t ICPR[1];                 /*!< Interrupt Clear Pending Register */
         uint32_t RESERVED3[31];
    __IO uint32_t IPR[32];                 /*!< Interrupt Priority Register */
} NVIC_Type;

// External declarations for peripheral instances
extern RCC_TypeDef *const RCC;
extern GPIO_TypeDef *const GPIOA;
extern GPIO_TypeDef *const GPIOB;
extern GPIO_TypeDef *const GPIOC;
extern GPIO_TypeDef *const GPIOD;
extern EXTI_TypeDef *const EXTI;
extern ADC_TypeDef *const ADC1;
extern ADC_Common_TypeDef *const ADC;
extern DMA_Channel_TypeDef *const DMA1_Channel1;
extern DMAMUX_Channel_TypeDef *const DMAMUX1_Channel0;
extern TIM_TypeDef *const TIM1;
extern TIM_TypeDef *const TIM3;
extern TIM_TypeDef *const TIM14;
extern TIM_TypeDef *const TIM16;
extern USART_TypeDef *const USART1;
extern USART_TypeDef *const USART2;
extern CRC_TypeDef *const CRC;
extern FLASH_TypeDef *const FLASH;
extern SCB_Type *const SCB;
extern SysTick_Type *const SysTick;

// External variable declarations
extern uint32_t SystemCoreClock;
extern const uint8_t AHBPrescTable[16];
extern __IO uint32_t uwTick;
extern uint32_t uwTickPrio;
extern HAL_TickFreqTypeDef uwTickFreq;

// Function declarations
HAL_StatusTypeDef HAL_Init(void);
HAL_StatusTypeDef HAL_DeInit(void);
void HAL_MspInit(void);
void HAL_MspDeInit(void);
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority);
void HAL_IncTick(void);
void HAL_Delay(uint32_t Delay);
uint32_t HAL_GetTick(void);
uint32_t HAL_GetTickPrio(void);
HAL_StatusTypeDef HAL_SetTickFreq(HAL_TickFreqTypeDef Freq);
HAL_TickFreqTypeDef HAL_GetTickFreq(void);
void HAL_SuspendTick(void);
void HAL_ResumeTick(void);
uint32_t HAL_GetHalVersion(void);
uint32_t HAL_GetREVID(void);
uint32_t HAL_GetDEVID(void);
uint32_t HAL_GetUIDw0(void);
uint32_t HAL_GetUIDw1(void);
uint32_t HAL_GetUIDw2(void);

// External declaration for the interrupts enabled flag
extern volatile uint8_t g_interrupts_enabled;

// Core functions
void __enable_irq(void);
void __disable_irq(void);
static inline void __DMB(void) { }
static inline void __ISB(void) { }
static inline void __DSB(void) { }

#ifdef __cplusplus
}
#endif

#endif /* STM32G0xx_HAL_H */
