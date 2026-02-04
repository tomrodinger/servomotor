/**
 * @file current_streaming.c
 * @brief DMA-based UART current streaming implementation for SERVOMOTOR_M23
 *
 * Usage:
 *   1. Press '5' to switch debug UART to 5Mbps mode
 *   2. Reconnect terminal at 5Mbps
 *   3. Press 's' to toggle streaming on/off
 */

#include "stm32g0xx_hal.h"
#include "current_streaming.h"
#include "debug_uart.h"
#include "ADC.h"

#ifdef PRODUCT_NAME_M23

// External reference to ADC buffer
extern uint16_t ADC_buffer[DMA_ADC_BUFFER_SIZE];

// Double-buffer for ping-pong DMA
static uint8_t stream_buffer[2][STREAM_BUFFER_SIZE];
static volatile uint8_t active_buffer = 0;      // Buffer currently being transmitted by DMA
static volatile uint8_t fill_buffer = 0;        // Buffer currently being filled
static volatile uint8_t frame_index = 0;        // Current frame index within fill buffer
static volatile uint8_t streaming_enabled = 0;  // Streaming state flag
static volatile uint8_t dma_busy = 0;           // DMA transfer in progress
static volatile uint8_t high_speed_mode = 0;    // 5Mbps mode enabled flag

// DMAMUX request ID for USART2_TX on STM32G031
// From stm32g0xx_ll_dmamux.h: LL_DMAMUX_REQ_USART2_TX = 0x35 = 53
// Note: USART2_RX = 52 (0x34), USART2_TX = 53 (0x35)
#define USART2_TX_DMAREQ_ID 53

/**
 * Start DMA transfer from the specified buffer
 */
static void start_dma_transfer(uint8_t buffer_idx)
{
    // Disable DMA channel before configuration
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;

    // Clear any pending flags
    DMA1->IFCR = DMA_IFCR_CTCIF2 | DMA_IFCR_CHTIF2 | DMA_IFCR_CTEIF2 | DMA_IFCR_CGIF2;

    // Configure source address
    DMA1_Channel2->CMAR = (uint32_t)stream_buffer[buffer_idx];

    // Configure transfer count
    DMA1_Channel2->CNDTR = STREAM_BUFFER_SIZE;

    // Enable DMA channel
    DMA1_Channel2->CCR |= DMA_CCR_EN;

    dma_busy = 1;
}

void current_streaming_init(void)
{
    // Enable DMA1 clock (should already be enabled by ADC, but ensure it)
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    // Configure DMA1_Channel2 for USART2_TX
    // Peripheral address = USART2->TDR
    DMA1_Channel2->CPAR = (uint32_t)&USART2->TDR;

    // Initial memory address (will be set before each transfer)
    DMA1_Channel2->CMAR = (uint32_t)stream_buffer[0];

    // Configure DMA channel:
    // - Memory increment mode (MINC)
    // - Direction: Memory to peripheral (DIR)
    // - Transfer complete interrupt (TCIE)
    // - 8-bit transfers (MSIZE=00, PSIZE=00)
    DMA1_Channel2->CCR = DMA_CCR_MINC |      // Memory increment
                         DMA_CCR_DIR |        // Memory to peripheral
                         DMA_CCR_TCIE;        // Transfer complete interrupt

    // Configure DMAMUX1_Channel1 for DMA1_Channel2
    // DMAMUX channels are offset by 1 from DMA channels (Channel1 -> DMAMUX0, Channel2 -> DMAMUX1)
    DMAMUX1_Channel1->CCR = (USART2_TX_DMAREQ_ID << DMAMUX_CxCR_DMAREQ_ID_Pos);

    // Set interrupt priority (low - same as debug UART)
    NVIC_SetPriority(DMA1_Channel2_3_IRQn, 3);
    NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

    // Initialize state
    active_buffer = 0;
    fill_buffer = 0;
    frame_index = 0;
    streaming_enabled = 0;
    dma_busy = 0;
    high_speed_mode = 0;
}

void enable_high_speed_mode(void)
{
    if (high_speed_mode) {
        print_debug_string("Already in 5Mbps mode\n");
        return;
    }

    print_debug_string("Switching to 5Mbps. Test bytes will follow.\n");

    // Wait for message to be transmitted before switching baud rate
    // Message is ~50 chars, at 230400 baud = ~2.2ms, wait 10ms to be safe
    for (volatile int i = 0; i < 200000; i++);

    // Disable USART2 for reconfiguration
    USART2->CR1 &= ~USART_CR1_UE;
    while (USART2->CR1 & USART_CR1_UE);

    // Enable 8x oversampling (OVER8=1) to achieve higher baud rates
    // With OVER8=0 (16x): max baud = 64MHz/16 = 4Mbps (can't reach 5Mbps)
    // With OVER8=1 (8x):  max baud = 64MHz/8  = 8Mbps (can reach 5Mbps)
    USART2->CR1 |= USART_CR1_OVER8;

    // Configure BRR for ~5Mbps with OVER8=1
    // Formula: Baud = fCK / (8 × USARTDIV)
    // USARTDIV = 64MHz / (8 × 5MHz) = 1.6
    //
    // BRR encoding with OVER8=1:
    //   - Bits 15:4 = mantissa
    //   - Bit 3 = must be 0
    //   - Bits 2:0 = fraction (in 1/8ths)
    //
    // For USARTDIV = 1.6:
    //   mantissa = 1, fraction = 0.6 × 8 = 5
    //   BRR = (1 << 4) | 5 = 0x15 = 21
    //
    // Actual USARTDIV = 1 + 5/8 = 1.625
    // Actual baud = 64MHz / (8 × 1.625) = 4.92 Mbps (1.5% error)
    USART2->BRR = 0x15;

    // Re-enable USART2
    USART2->CR1 |= USART_CR1_UE;

    high_speed_mode = 1;
}

uint8_t is_high_speed_mode_enabled(void)
{
    return high_speed_mode;
}

void toggle_current_streaming(void)
{
    if (streaming_enabled) {
        // Stop streaming
        streaming_enabled = 0;

        // Wait for DMA to complete current transfer
        while (dma_busy && (DMA1_Channel2->CNDTR > 0));

        // Disable DMA channel
        DMA1_Channel2->CCR &= ~DMA_CCR_EN;
        dma_busy = 0;

        // Disable DMA for TX, back to interrupt-driven
        USART2->CR3 &= ~USART_CR3_DMAT;

        // Re-enable debug printing
        disable_or_enable_debug_printing(1);

        print_debug_string("Streaming stopped\n");
    }
    else {
        // Start streaming (no message - would interfere with frame parsing)

        // Disable debug printing (DMA will handle TX)
        disable_or_enable_debug_printing(0);

        // Disable TX interrupt (DMA will handle transmission)
        USART2->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;

        // Enable DMA for TX
        USART2->CR3 |= USART_CR3_DMAT;

        // Reset buffer state
        active_buffer = 0;
        fill_buffer = 0;
        frame_index = 0;
        dma_busy = 0;

        // Enable streaming
        streaming_enabled = 1;
    }
}

void stream_current_sample(void)
{
    if (!streaming_enabled) {
        return;
    }

    // Build frame in the fill buffer
    m23_telemetry_frame_t *frame = (m23_telemetry_frame_t *)&stream_buffer[fill_buffer][frame_index * TELEMETRY_FRAME_SIZE];

    // Fill frame data
    frame->sync = TELEMETRY_SYNC_WORD;
    frame->current_a = (int16_t)ADC_buffer[MOTOR_CURRENT_PHASE_A_CYCLE_INDEX];
    frame->current_b = (int16_t)ADC_buffer[MOTOR_CURRENT_PHASE_B_CYCLE_INDEX];
    frame->pwm_a = (int16_t)TIM1->CCR1;
    frame->pwm_b = (int16_t)TIM1->CCR2;

    frame_index++;

    // Check if buffer is full
    if (frame_index >= FRAMES_PER_BUFFER) {
        frame_index = 0;

        // If DMA is not busy, start transfer immediately
        if (!dma_busy) {
            active_buffer = fill_buffer;
            fill_buffer = 1 - fill_buffer;  // Swap to other buffer
            start_dma_transfer(active_buffer);
        }
        // else: DMA still busy, we'll overwrite this buffer (drop frames)
    }
}

uint8_t is_current_streaming_enabled(void)
{
    return streaming_enabled;
}

/**
 * DMA1 Channel 2/3 IRQ Handler
 * Called when DMA transfer completes
 */
void DMA1_Channel2_3_IRQHandler(void)
{
    // Check for Channel 2 transfer complete
    if (DMA1->ISR & DMA_ISR_TCIF2) {
        // Clear interrupt flag
        DMA1->IFCR = DMA_IFCR_CTCIF2;
        dma_busy = 0;
    }

    // Check for Channel 3 (not used, but clear if set)
    if (DMA1->ISR & DMA_ISR_TCIF3) {
        DMA1->IFCR = DMA_IFCR_CTCIF3;
    }

    // Handle errors
    if (DMA1->ISR & DMA_ISR_TEIF2) {
        DMA1->IFCR = DMA_IFCR_CTEIF2;
        dma_busy = 0;
    }
}

#endif // PRODUCT_NAME_M23
