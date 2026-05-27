#ifndef SRC_CURRENT_CONTROL_H_
#define SRC_CURRENT_CONTROL_H_

#ifdef PRODUCT_NAME_M23

#include <stdint.h>
#include "PWM.h"

// Enable current control (comment out to revert to voltage-mode)
//#define M23_CURRENT_CONTROL

// PI controller gains — these will need experimental tuning
#define CURRENT_PI_KP       20       // proportional gain
#define CURRENT_PI_KI       3       // integral gain
#define CURRENT_PI_SHIFT    8       // right-shift for output scaling
// Limit duty cycle to 25%–75% range (±25% from midpoint) to avoid measuring
// current during switching transitions near valley/peak where ADC samples.
#define CURRENT_PI_MAX_OUTPUT (PWM_PERIOD_TIM1 >> 2)  // 256: ±25% of full PWM period

// Anti-windup: max integral = max_output << CURRENT_PI_SHIFT (so integral alone can saturate output)
#define CURRENT_PI_MAX_INTEGRAL (CURRENT_PI_MAX_OUTPUT << CURRENT_PI_SHIFT)

typedef struct {
    int32_t integral;
} current_pi_t;

/**
 * Initialize the current control module. Zeros all PI state.
 * Call once during startup.
 */
void current_control_init(void);

/**
 * Reset PI integrators to zero. Call on mode changes or when
 * MOSFETs are disabled to prevent integrator windup.
 */
void current_control_reset(void);

/**
 * Execute one step of a PI controller.
 * @param pi  Pointer to the PI controller instance (phase A or phase B)
 * @param error  Current error = reference - measured (in ADC counts)
 * @return Voltage command (signed, centered at 0, range ±CURRENT_PI_MAX_OUTPUT)
 */
int16_t current_pi_step(current_pi_t *pi, int16_t error);

/**
 * Get pointers to the PI controller instances for phase A and B.
 * Used by motor_phase_calculations() in motor_control.c.
 */
current_pi_t *current_control_get_pi_a(void);
current_pi_t *current_control_get_pi_b(void);

/**
 * Store the most recent phase A current reference (for telemetry streaming).
 * Called by motor_phase_calculations() after computing i_a_ref.
 */
void current_control_set_i_a_ref(int16_t ref);

/**
 * Store the most recent phase B current reference (for telemetry streaming).
 * Called by motor_phase_calculations() after computing i_b_ref.
 */
void current_control_set_i_b_ref(int16_t ref);

/**
 * Get the most recent phase A current reference (for telemetry streaming).
 */
int16_t current_control_get_i_a_ref(void);

/**
 * Get the most recent phase B current reference (for telemetry streaming).
 */
int16_t current_control_get_i_b_ref(void);

#endif // PRODUCT_NAME_M23

#endif /* SRC_CURRENT_CONTROL_H_ */
