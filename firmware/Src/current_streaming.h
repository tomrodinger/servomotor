/**
 * @file current_streaming.h
 * @brief DMA-based UART current streaming for SERVOMOTOR_M23
 *
 * Streams motor current measurements at 5Mbps via USART2 (debug UART).
 * Uses double-buffer ping-pong DMA for efficient, non-blocking transmission.
 *
 * Frame format (10 bytes, little-endian):
 *   - uint16_t sync:      0xABCD sync word
 *   - int16_t  current_a: Phase A current (ADC counts, 0-4095)
 *   - int16_t  current_b: Phase B current (ADC counts, 0-4095)
 *   - int16_t  pwm_a:     Phase A PWM duty (TIM1->CCR1, 0-1024)
 *   - int16_t  pwm_b:     Phase B PWM duty (TIM1->CCR2, 0-1024)
 *
 * One frame is sent per motor control loop iteration (~31.25 kHz).
 * Throughput: 10 bytes × 31,250 Hz = 3.125 Mbps (within 5Mbps capacity)
 */

#ifndef SRC_CURRENT_STREAMING_H_
#define SRC_CURRENT_STREAMING_H_

#include <stdint.h>

#ifdef PRODUCT_NAME_M23

// Telemetry frame sync word
#define TELEMETRY_SYNC_WORD 0xABCD

// Frame structure (10 bytes)
typedef struct __attribute__((__packed__)) {
    uint16_t sync;           // 0xABCD sync word (little-endian)
    int16_t  current_a;      // Phase A current (ADC counts)
    int16_t  current_b;      // Phase B current (ADC counts)
    int16_t  pwm_a;          // Phase A PWM duty (TIM1->CCR1)
    int16_t  pwm_b;          // Phase B PWM duty (TIM1->CCR2)
} m23_telemetry_frame_t;

// Buffer configuration
#define TELEMETRY_FRAME_SIZE      sizeof(m23_telemetry_frame_t)  // 10 bytes
#define FRAMES_PER_BUFFER         10
#define STREAM_BUFFER_SIZE        (TELEMETRY_FRAME_SIZE * FRAMES_PER_BUFFER)  // 100 bytes

/**
 * Initialize the current streaming system.
 * Configures DMA1_Channel2 for USART2_TX but does not enable streaming.
 * Must be called after debug_uart_init().
 */
void current_streaming_init(void);

/**
 * Enable high-speed (5Mbps) mode for debug UART.
 * Must be called before streaming can be used.
 * After calling this, reconnect terminal at 5Mbps.
 * Debug printing continues to work at the new baud rate.
 */
void enable_high_speed_mode(void);

/**
 * Check if high-speed mode is enabled.
 * @return 1 if 5Mbps mode is active, 0 if still at 230400 baud
 */
uint8_t is_high_speed_mode_enabled(void);

/**
 * Toggle current streaming on/off.
 * Requires high-speed mode to be enabled first (call enable_high_speed_mode()).
 * When enabled: uses DMA for TX, disables debug printing
 * When disabled: returns to interrupt-driven TX, re-enables debug printing
 */
void toggle_current_streaming(void);

/**
 * Add a current sample to the streaming buffer.
 * Called from TIM16_IRQHandler on each motor control cycle.
 * Only adds data if streaming is enabled; otherwise returns immediately.
 */
void stream_current_sample(void);

/**
 * Check if current streaming is currently enabled.
 * @return 1 if streaming is active, 0 otherwise
 */
uint8_t is_current_streaming_enabled(void);

#endif // PRODUCT_NAME_M23

#endif /* SRC_CURRENT_STREAMING_H_ */
