#include "current_control.h"

#ifdef PRODUCT_NAME_M23

static current_pi_t pi_phase_a;
static current_pi_t pi_phase_b;
static int16_t last_i_a_ref;
static int16_t last_i_b_ref;

void current_control_init(void)
{
    pi_phase_a.integral = 0;
    pi_phase_b.integral = 0;
    last_i_a_ref = 0;
    last_i_b_ref = 0;
}

void current_control_reset(void)
{
    pi_phase_a.integral = 0;
    pi_phase_b.integral = 0;
}

int16_t current_pi_step(current_pi_t *pi, int16_t error)
{
    int32_t proportional = (int32_t)error * CURRENT_PI_KP;

    pi->integral += (int32_t)error * CURRENT_PI_KI;
    if (pi->integral > CURRENT_PI_MAX_INTEGRAL) {
        pi->integral = CURRENT_PI_MAX_INTEGRAL;
    }
    else if (pi->integral < -CURRENT_PI_MAX_INTEGRAL) {
        pi->integral = -CURRENT_PI_MAX_INTEGRAL;
    }

    int32_t output = (proportional + pi->integral) >> CURRENT_PI_SHIFT;

    if (output > CURRENT_PI_MAX_OUTPUT) {
        output = CURRENT_PI_MAX_OUTPUT;
    }
    else if (output < -CURRENT_PI_MAX_OUTPUT) {
        output = -CURRENT_PI_MAX_OUTPUT;
    }

    return (int16_t)output;
}

current_pi_t *current_control_get_pi_a(void)
{
    return &pi_phase_a;
}

current_pi_t *current_control_get_pi_b(void)
{
    return &pi_phase_b;
}

void current_control_set_i_a_ref(int16_t ref)
{
    last_i_a_ref = ref;
}

void current_control_set_i_b_ref(int16_t ref)
{
    last_i_b_ref = ref;
}

int16_t current_control_get_i_a_ref(void)
{
    return last_i_a_ref;
}

int16_t current_control_get_i_b_ref(void)
{
    return last_i_b_ref;
}

#endif // PRODUCT_NAME_M23
