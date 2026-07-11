/*
 * pid_algorithm_sim.c — headless command-line simulator for the M17 PID control loop
 * =====================================================================================
 *
 * Purpose: develop and stress-test changes to the firmware PID algorithm
 * (firmware/Src/motor_control.c PID_controller + the M17 closed-loop caller)
 * without hardware. See firmware/PID_IMPROVEMENT_PLAN.md.
 *
 * The INTEGER CONTROL PATH below is copied line-for-line from the firmware
 * (M17 semantics, PID_SHIFT_RIGHT=11, PWM_VOLTAGE_VS_COMMUTATION_POSITION_FUDGE_SHIFT=8,
 * STEP/DIR slew with 64 steps per electrical cycle and hysteresis of 1 step).
 * The plant is a continuous 2-phase stepper model:
 *
 *     tau = tau_max * (V/200) * sin(2*pi*(applied_e_angle - rotor_e_angle)/65536)
 *     J * domega/dt = tau - tau_coulomb*sgn(omega) - b*omega - tau_load
 *
 * integrated at the 31.25 kHz control tick with semi-implicit Euler. The hall
 * sensor returns the true position plus white Gaussian noise, rounded to
 * integer counts (3,276,800 counts/rev; 65,536 counts per electrical cycle).
 *
 * Algorithm variants (--alg):
 *   old    exact 0.15.3.0 behavior: floor shifts (sticky negative D bias), no anti-windup,
 *          integral clamp = 100% of full output authority
 *   step1  + rounding fixes (LPF decay rounds toward zero; output shift rounds to nearest)
 *   step2  + back-calculation anti-windup and integral clamp reduced to 25% authority
 *   step3  + D term from a 32-tick FIR difference of the error instead of
 *            per-tick difference + leaky low-pass (same DC gain: 32x)
 *
 * Output: CSV whose first 6 columns match tools/pid_debug_capture.py captures
 * (t_s,error,p_term,i_term,d_term,output) so the same plotting works, plus
 * extra columns (desired,actual,voltage) for plant ground truth.
 * A summary block of key=value metrics is printed to stdout for the test driver.
 *
 * Build:  cc -O2 -o pid_algorithm_sim pid_algorithm_sim.c -lm
 * Example:
 *   ./pid_algorithm_sim --alg old --ki 25 --move 2 --duration 8 --csv out.csv
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

/* ------------------------------------------------------------------ constants
 * These mirror the firmware values for the M17. */
#define TICK_HZ                 31250.0
#define DT                      (1.0 / TICK_HZ)
#define COUNTS_PER_REV          3276800.0
#define COUNTS_PER_ECYCLE       65536      /* 50 pole pairs */
#define COUNTS_PER_RAD          (COUNTS_PER_REV / (2.0 * M_PI))
#define PID_SHIFT_RIGHT         11
#define FUDGE_SHIFT             8          /* PWM_VOLTAGE_VS_COMMUTATION_POSITION_FUDGE_SHIFT */
#define AVG_SCALAR_SHIFT        5
#define AVG_SCALAR              (1 << AVG_SCALAR_SHIFT)   /* 32 */
#define HALL_90_DEG             16384      /* HALL_TO_POSITION_90_DEGREE_OFFSET */
#define N_COMM_STEPS            64
#define N_COMM_SUB_STEPS        1024
#define STEP_HYSTERESIS         1
#define MAX_PWM_V               200        /* default max_motor_pwm_voltage */
#define D_RING_SHIFT            5
#define D_RING_N                (1 << D_RING_SHIFT)       /* 32-tick FIR difference for step3 */

typedef enum { ALG_OLD = 0, ALG_STEP1A, ALG_STEP1B, ALG_STEP1, ALG_STEP2, ALG_STEP3 } alg_t;
static const char *ALG_NAMES[] = {"old", "step1a", "step1b", "step1", "step2", "step3"};
/* step1a = only the LPF decay fix; step1b = only the output rounding fix (isolation variants) */

/* zero-crossing-gated integral windup (analysis agent "zerocross", 2026-07-11):
 * the integral may wind past the 25% clamp toward FULL authority only while the
 * position error has kept one sign (beyond a deadband) for i_gate_holdoff ticks.
 * On a zero crossing the gate closes; mode selects the reaction:
 *   clamp  : immediately re-clamp I to the 25% rail
 *   freeze : freeze integration for holdoff ticks; I keeps its value (and while
 *            the gate is closed, |I| may not GROW past 25% but keeps any excess)
 *   bleed  : while the gate is closed, decay the above-25% excess by
 *            excess >> bleed_shift per tick (time constant ~2^bleed_shift ticks)
 * Only active for --alg step2/step3. */
typedef enum { GATE_OFF = 0, GATE_CLAMP, GATE_FREEZE, GATE_BLEED, GATE_AVG } gate_t;
static const char *GATE_NAMES[] = {"off", "clamp", "freeze", "bleed", "avg"};
/* avg: on a zero crossing, set I to the midpoint of its values at this and the
 * previous crossing (the stick-slip cycle turns symmetrically around the true
 * hold torque, so the midpoint self-tunes to it), then freeze for holdoff. */

/* ------------------------------------------------------------------ options */
typedef struct {
    alg_t alg;
    uint32_t kp, ki, kd;
    double duration_s;
    double move_rot;         /* trapezoid displacement in rotations (0 = no move) */
    double step_rot;         /* INSTANT setpoint step in rotations at move_delay (0 = none) */
    int no_slew;             /* disable the STEP/DIR 1-step-per-tick slew limit (apply angle directly) */
    int linear_torque;       /* torque proportional to output only (no quadratic V*sin shape) */
    double move_time_s;
    double move_delay_s;
    double noise_counts;     /* hall sensor white noise sigma */
    double tau_max;          /* N*m at V=200 */
    double inertia;          /* kg*m^2 */
    double coulomb;          /* kinetic (Coulomb) friction, N*m */
    double static_friction;  /* breakaway friction, N*m (>= coulomb; stick-slip) */
    double viscous;          /* N*m*s/rad */
    double load;             /* constant external torque N*m */
    uint64_t seed;
    const char *csv_path;
    int decimate;
    int quiet;
    int32_t i_deadband;      /* skip integration when |error| < this (0 = off) */
    int32_t i_freeze_above;  /* integral separation: skip integration when |error| > this (0 = off) */
    int i_clamp_shift;       /* integral clamp = full authority >> this (step2/3; default 2) */
    int i_clamp_frac16;      /* if nonzero, integral clamp = full authority * frac/16 (overrides shift) */
    int d_shift;             /* derivative filter strength: EMA over 2^d_shift ticks (default 5; 0 = no filter) */
    int floor_output;        /* ablation: use the OLD floor output shift (no half-LSB rounding) on step2+ */
    int sticky_decay;        /* ablation: use the OLD sticky (floor) LPF decay on step2+ */
    gate_t i_gate;           /* zero-crossing windup gate mode (default off) */
    int32_t i_gate_holdoff;  /* ticks of one-signed error before I may exceed 25%; also freeze window */
    int32_t i_gate_deadband; /* |error| must exceed this for a sign to register (0 = every count) */
    int i_gate_bleed_shift;  /* bleed mode: per-tick excess decay = excess >> this */
    int i_gate_base_bc;      /* back-calc clamp uses the BASE 25% limit instead of the gated limit */
} opts_t;

/* ------------------------------------------------------------------ RNG (deterministic) */
static uint64_t rng_state;
static double urand(void) {           /* xorshift64* -> (0,1) */
    rng_state ^= rng_state >> 12;
    rng_state ^= rng_state << 25;
    rng_state ^= rng_state >> 27;
    return ((rng_state * 2685821657736338717ULL) >> 11) * (1.0 / 9007199254740992.0) + 1e-16;
}
static double grand_(void) {          /* Box-Muller */
    static int have = 0; static double spare;
    if (have) { have = 0; return spare; }
    double u1 = urand(), u2 = urand();
    double r = sqrt(-2.0 * log(u1)), t = 2.0 * M_PI * u2;
    spare = r * sin(t); have = 1;
    return r * cos(t);
}

/* ------------------------------------------------------------------ controller state (integer, firmware-exact) */
static int32_t integral_term;
static int32_t previous_error;
static int32_t low_pass_filtered_error_change;
static int32_t error_ring[D_RING_N];    /* step3 */
static int32_t error_ring_idx;
static int32_t error_ring_fill;

static int32_t max_error;                /* (V << (11+8)) / kP */
static int32_t max_error_change;         /* (V << (11+8)) / kD */
static int32_t max_integral_term;        /* old/step1: V << 19; step2/3: >> 2 */
static int32_t deriv_scaled;             /* kD >> 5 */
static int32_t ki_effective;             /* kI after the overflow cap (step2+) */
static int32_t output_limit;             /* V << FUDGE_SHIFT: full authority in output counts */

/* zero-crossing gate state (all int32; ports to firmware as 3 words) */
static int32_t max_integral_full;        /* full pre-shift authority (gate-open I limit) */
static int32_t zc_prev_sign;             /* -1/0/+1: last error sign seen beyond the deadband */
static int32_t zc_same_ticks;            /* consecutive registered ticks with that sign (saturates at holdoff) */
static int32_t zc_freeze_ticks;          /* freeze mode: remaining ticks with integration paused */
static int32_t zc_last_cross_i;          /* avg mode: integral value at the previous crossing */
static int32_t i_limit_now;              /* effective integral limit this tick (base or full) */
static int64_t gate_open_ticks;          /* metric */
static int64_t gate_crossings;           /* metric */

static int64_t saturated_ticks;          /* metrics */
static int64_t overflow_violations;

/* overflow-safety bounds; must match firmware/Src/motor_control.c (see test_pid_overflow.sh) */
#define PID_ERROR_INPUT_CLAMP        (1 << 29)
#define PID_MAX_AUTHORITY_PRE_SHIFT  800000000
#define PID_MAX_ERROR_CHANGE         2000000

static void controller_init(const opts_t *o) {
    integral_term = 0;
    previous_error = 0;
    low_pass_filtered_error_change = 0;
    memset(error_ring, 0, sizeof(error_ring));
    error_ring_idx = 0;
    error_ring_fill = 0;
    saturated_ticks = 0;
    overflow_violations = 0;
    zc_prev_sign = 0;
    zc_same_ticks = 0;
    zc_freeze_ticks = 0;
    zc_last_cross_i = 0;
    gate_open_ticks = 0;
    gate_crossings = 0;

    if (o->alg >= ALG_STEP2) {
        /* hardened constants pipeline (firmware >= 0.15.3.2): 64-bit with caps */
        int64_t kp = o->kp > (uint32_t)INT32_MAX ? INT32_MAX : (int64_t)o->kp;
        int64_t kd = o->kd > (uint32_t)INT32_MAX ? INT32_MAX : (int64_t)o->kd;
        int64_t ki = o->ki > (uint32_t)INT32_MAX ? INT32_MAX : (int64_t)o->ki;
        int64_t authority = (int64_t)MAX_PWM_V << (PID_SHIFT_RIGHT + FUDGE_SHIFT);
        if (authority > PID_MAX_AUTHORITY_PRE_SHIFT) authority = PID_MAX_AUTHORITY_PRE_SHIFT;
        int64_t me = authority;
        if (kp != 0) me /= kp;
        max_error = (int32_t)me;
        max_error_change = 0;
        if (kd != 0) {
            int64_t mec = authority / kd;
            if (mec > PID_MAX_ERROR_CHANGE) mec = PID_MAX_ERROR_CHANGE;
            max_error_change = (int32_t)mec;
        }
        int64_t mi = authority;
        if (o->i_clamp_frac16 > 0) mi = (mi * o->i_clamp_frac16) >> 4;
        else                       mi >>= o->i_clamp_shift;
        max_integral_term = (int32_t)mi;
        max_integral_full = (int32_t)authority;
        /* the ki overflow cap must be computed against the LARGEST value the
         * integral can hold; with the gate enabled that is full authority */
        int64_t i_cap_for_ki = (o->i_gate != GATE_OFF) ? max_integral_full : max_integral_term;
        int64_t eff_me = me > PID_ERROR_INPUT_CLAMP ? (int64_t)PID_ERROR_INPUT_CLAMP : me;
        if (eff_me > 0) {
            int64_t ki_max = ((int64_t)INT32_MAX - i_cap_for_ki) / eff_me;
            if (ki > ki_max) ki = ki_max;
        }
        ki_effective = (int32_t)ki;
        deriv_scaled = (int32_t)(kd >> o->d_shift);
    } else {
        /* legacy constants pipeline (firmware <= 0.15.3.1) */
        max_error = (int32_t)(MAX_PWM_V << (PID_SHIFT_RIGHT + FUDGE_SHIFT));
        if (o->kp != 0) max_error /= (int32_t)o->kp;
        max_error_change = 0;
        if (o->kd != 0) max_error_change = (int32_t)(MAX_PWM_V << (PID_SHIFT_RIGHT + FUDGE_SHIFT)) / (int32_t)o->kd;
        max_integral_term = (int32_t)(MAX_PWM_V << (PID_SHIFT_RIGHT + FUDGE_SHIFT));
        ki_effective = (int32_t)o->ki;
        deriv_scaled = (int32_t)(o->kd >> AVG_SCALAR_SHIFT);
    }
    if (o->alg < ALG_STEP2) max_integral_full = max_integral_term;
    i_limit_now = max_integral_term;
    output_limit = MAX_PWM_V << FUDGE_SHIFT;             /* 51200 at V=200 */
    /* firmware >= 0.15.3.4: the anti-windup saturation point is the LARGER of the
     * voltage term and the 90-degree commutation angle (matters only for V < 64) */
    if (o->alg >= ALG_STEP2 && output_limit < HALL_90_DEG) output_limit = HALL_90_DEG;
}

/* firmware-exact PID with selectable variant; also exports the terms for logging */
static int32_t pid_tick(const opts_t *o, int64_t error_i64,
                        int32_t *p_out, int32_t *i_out, int32_t *d_out) {
    int32_t output_value;
    int32_t proportional_term;
    int32_t derivative_term;
    int32_t error;

    if (o->alg >= ALG_STEP2) {
        /* hardened entry clamp (firmware >= 0.15.3.2) */
        if (error_i64 > PID_ERROR_INPUT_CLAMP) error = PID_ERROR_INPUT_CLAMP;
        else if (error_i64 < -PID_ERROR_INPUT_CLAMP) error = -PID_ERROR_INPUT_CLAMP;
        else error = (int32_t)error_i64;
    } else {
        error = (int32_t)error_i64;   /* legacy truncation */
    }

    /* ---- derivative ---- */
    if (deriv_scaled != 0) {
        if (o->alg <= ALG_STEP2) {
            /* per-tick difference + leaky integrator (old firmware topology) */
            int32_t error_change = error - previous_error;
            if (error_change > max_error_change) error_change = max_error_change;
            else if (error_change < -max_error_change) error_change = -max_error_change;
            int scalar = 1 << o->d_shift;
            if (o->alg == ALG_OLD || o->alg == ALG_STEP1B || o->sticky_decay) {
                /* floor shift: negative values never decay (the sticky-bias bug) */
                low_pass_filtered_error_change =
                    (low_pass_filtered_error_change * (scalar - 1)) >> o->d_shift;
            } else {
                int32_t decayed = low_pass_filtered_error_change * (scalar - 1);
                if (decayed < 0) decayed += scalar - 1;   /* round toward zero */
                low_pass_filtered_error_change = decayed >> o->d_shift;
            }
            low_pass_filtered_error_change += error_change;
            derivative_term = low_pass_filtered_error_change * deriv_scaled;
        } else {
            /* step3: 32-tick FIR difference; same DC gain as leaky filter (32x per-tick change) */
            int32_t oldest = error_ring[error_ring_idx];
            error_ring[error_ring_idx] = error;
            error_ring_idx = (error_ring_idx + 1) & (D_RING_N - 1);
            int32_t diff;
            if (error_ring_fill < D_RING_N) { error_ring_fill++; diff = 0; }
            else                            { diff = error - oldest; }
            int32_t diff_limit = max_error_change << D_RING_SHIFT;   /* equivalent authority */
            if (diff > diff_limit) diff = diff_limit;
            else if (diff < -diff_limit) diff = -diff_limit;
            derivative_term = diff * deriv_scaled;
        }
    } else {
        derivative_term = 0;
    }
    previous_error = error;

    /* ---- clamp error, integrate, proportional ---- */
    if (error < -max_error) error = -max_error;
    else if (error > max_error) error = max_error;

    if (ki_effective != 0) {
        if (o->i_gate != GATE_OFF && o->alg >= ALG_STEP2) {
            /* ---- zero-crossing-gated windup ----
             * sign of error beyond the deadband; sign==0 (inside deadband) holds
             * all gate state (neither progress nor reset). */
            int32_t s = 0;
            if (error > o->i_gate_deadband) s = 1;
            else if (error < -o->i_gate_deadband) s = -1;
            int crossed = 0;
            if (s != 0) {
                if (s == zc_prev_sign) {
                    if (zc_same_ticks < o->i_gate_holdoff) zc_same_ticks++;
                } else {
                    if (zc_prev_sign != 0) crossed = 1;
                    zc_prev_sign = s;
                    zc_same_ticks = 0;
                }
            }
            int gate_open = (zc_same_ticks >= o->i_gate_holdoff);
            if (gate_open) gate_open_ticks++;
            if (crossed) gate_crossings++;
            i_limit_now = gate_open ? max_integral_full : max_integral_term;

            if (crossed) {
                if (o->i_gate == GATE_CLAMP) {
                    if (integral_term > max_integral_term) integral_term = max_integral_term;
                    else if (integral_term < -max_integral_term) integral_term = -max_integral_term;
                } else if (o->i_gate == GATE_FREEZE) {
                    zc_freeze_ticks = o->i_gate_holdoff;
                } else if (o->i_gate == GATE_AVG) {
                    /* midpoint of I at this and the previous crossing (self-tunes
                     * to the hold torque of a symmetric stick-slip cycle) */
                    int32_t tmp = integral_term;
                    integral_term = (int32_t)(((int64_t)integral_term + zc_last_cross_i) >> 1);
                    zc_last_cross_i = tmp;
                    zc_freeze_ticks = o->i_gate_holdoff;
                }
            }

            if ((o->i_gate == GATE_FREEZE || o->i_gate == GATE_AVG) && zc_freeze_ticks > 0) {
                zc_freeze_ticks--;          /* integration paused; I keeps its value */
            } else {
                /* integrate with an outward limit of i_limit_now; if |I| already
                 * exceeds the (just lowered) limit, allow only inward movement */
                int64_t ni = (int64_t)integral_term + (int64_t)error * ki_effective;
                if (ni > i_limit_now) {
                    if (ni < integral_term)      integral_term = (int32_t)ni;      /* unwinding */
                    else if (integral_term < i_limit_now) integral_term = i_limit_now;
                    /* else: hold the excess (freeze-above-limit) */
                } else if (ni < -i_limit_now) {
                    if (ni > integral_term)      integral_term = (int32_t)ni;
                    else if (integral_term > -i_limit_now) integral_term = -i_limit_now;
                } else {
                    integral_term = (int32_t)ni;
                }
            }

            if (o->i_gate == GATE_BLEED && !gate_open) {
                /* decay any above-25% excess with time constant ~2^bleed_shift ticks;
                 * linear tail of 1/tick so it always reaches the rail eventually */
                if (integral_term > max_integral_term) {
                    int32_t excess = integral_term - max_integral_term;
                    int32_t dec = excess >> o->i_gate_bleed_shift;
                    if (dec == 0) dec = 1;
                    integral_term -= dec;
                } else if (integral_term < -max_integral_term) {
                    int32_t excess = -max_integral_term - integral_term;
                    int32_t dec = excess >> o->i_gate_bleed_shift;
                    if (dec == 0) dec = 1;
                    integral_term += dec;
                }
            }
        } else {
            /* optional integration deadband: within +/-i_deadband counts of the target,
             * stop pushing -- the position is already as good as the mechanics can hold,
             * and integrating across the torque dead zone only causes perpetual hunting */
            int within_sep = (o->i_freeze_above == 0 || (error <= o->i_freeze_above && error >= -o->i_freeze_above));
            if (within_sep && (o->i_deadband == 0 || error > o->i_deadband || error < -o->i_deadband)) {
                integral_term += error * ki_effective;
                if (integral_term > max_integral_term) integral_term = max_integral_term;
                else if (integral_term < -max_integral_term) integral_term = -max_integral_term;
            }
        }
    } else {
        integral_term = 0;
    }
    proportional_term = error * (int32_t)o->kp;

    /* ---- overflow shadow check (int64 vs int32 semantics) ---- */
    {
        int64_t s64 = (int64_t)integral_term + proportional_term + derivative_term;
        if (s64 > INT32_MAX || s64 < INT32_MIN) overflow_violations++;
    }

    /* ---- sum, shift, (anti-windup) ---- */
    if (o->alg == ALG_OLD || o->alg == ALG_STEP1A || o->floor_output) {
        output_value = (integral_term + proportional_term + derivative_term) >> PID_SHIFT_RIGHT;
    } else {
        output_value = (integral_term + proportional_term + derivative_term
                        + (1 << (PID_SHIFT_RIGHT - 1))) >> PID_SHIFT_RIGHT;
    }
    if (o->alg >= ALG_STEP2) {
        int32_t sat = output_value;
        if (sat > output_limit) sat = output_limit;
        else if (sat < -output_limit) sat = -output_limit;
        if (sat != output_value) {
            /* back-calculation: bleed the excess out of the integrator (gain 1).
             * int64 multiply (not shift: shifting negatives is UB) so it cannot overflow.
             * With the zero-cross gate: clamp to the gated limit (i_limit_now), or to
             * the base 25% limit if --i-gate-base-bc was given. */
            /* gain-1 back-calc removes exactly the output excess; the additional
             * hard clamp uses FULL authority when the gate is enabled (clamping to
             * the gate-closed 25% limit would slam a legitimately wound-up I on a
             * transient D-spike saturation) */
            int32_t bc_limit = max_integral_term;
            if (o->i_gate != GATE_OFF && !o->i_gate_base_bc) bc_limit = max_integral_full;
            int64_t correction = (int64_t)(output_value - sat) * (1 << PID_SHIFT_RIGHT);
            int64_t new_i = (int64_t)integral_term - correction;
            if (new_i > bc_limit) new_i = bc_limit;
            else if (new_i < -bc_limit) new_i = -bc_limit;
            integral_term = (int32_t)new_i;
            saturated_ticks++;
            output_value = sat;
        }
    }

    *p_out = proportional_term;
    *i_out = integral_term;
    *d_out = derivative_term;
    return output_value;
}

/* ------------------------------------------------------------------ M17 caller + STEP/DIR (firmware-exact) */
static uint32_t actual_step_position;   /* 0..63 */
static int g_no_slew = 0;

static void m17_output_stage(int32_t pid_output, int32_t sensor_position_wrapped,
                             int32_t *voltage_out, uint32_t *applied_ecounts_out) {
    uint32_t commutation_position = (uint32_t)sensor_position_wrapped;  /* offset = 0 in the sim frame */
    int32_t desired_motor_pwm_voltage = pid_output;
    int32_t motor_pwm_voltage;

    if (desired_motor_pwm_voltage > HALL_90_DEG) {
        commutation_position += HALL_90_DEG;
    } else if (desired_motor_pwm_voltage < -HALL_90_DEG) {
        commutation_position -= HALL_90_DEG;
    } else {
        commutation_position += desired_motor_pwm_voltage;
    }
    desired_motor_pwm_voltage >>= FUDGE_SHIFT;    /* arithmetic shift, exactly like firmware */
    if (desired_motor_pwm_voltage >= 0) motor_pwm_voltage = desired_motor_pwm_voltage;
    else motor_pwm_voltage = -desired_motor_pwm_voltage;
    if (motor_pwm_voltage > MAX_PWM_V) motor_pwm_voltage = MAX_PWM_V;

    /* STEP/DIR slew: at most 1 step of 1024 counts per tick, hysteresis 1 step */
    if (g_no_slew) {   /* ablation: pretend the driver can apply any angle instantly */
        *voltage_out = motor_pwm_voltage;
        *applied_ecounts_out = commutation_position & (COUNTS_PER_ECYCLE - 1);
        return;
    }
    uint32_t desired_step_position = (commutation_position / N_COMM_SUB_STEPS) & (N_COMM_STEPS - 1);
    int32_t steps_to_target = (int32_t)desired_step_position - (int32_t)actual_step_position;
    if (steps_to_target > (N_COMM_STEPS >> 1)) steps_to_target -= N_COMM_STEPS;
    else if (steps_to_target < -(N_COMM_STEPS >> 1)) steps_to_target += N_COMM_STEPS;
    if (steps_to_target > STEP_HYSTERESIS) steps_to_target -= STEP_HYSTERESIS;
    else if (steps_to_target < -STEP_HYSTERESIS) steps_to_target += STEP_HYSTERESIS;
    if (steps_to_target < 0) actual_step_position = (actual_step_position - 1) & (N_COMM_STEPS - 1);
    else if (steps_to_target > 0) actual_step_position = (actual_step_position + 1) & (N_COMM_STEPS - 1);

    *voltage_out = motor_pwm_voltage;
    *applied_ecounts_out = actual_step_position * N_COMM_SUB_STEPS;   /* 0..65535 within the e-cycle */
}

/* ------------------------------------------------------------------ trapezoid trajectory (counts) */
typedef struct {
    int64_t start_tick;      /* when the move begins */
    int64_t t1, t2;          /* accel ticks, cruise ticks */
    double accel;            /* counts/tick^2 */
    double pos;              /* accumulated commanded position, counts */
    double vel;              /* counts/tick */
    int active;
} traj_t;

static void traj_start(traj_t *tr, double displacement_counts, double time_s, int64_t now_tick) {
    int64_t total = (int64_t)(time_s * TICK_HZ);
    int64_t t1 = 2124;                     /* ~= default max_velocity / max_acceleration in ticks */
    if (t1 * 2 > total) t1 = total / 2;
    int64_t t2 = total - 2 * t1;
    tr->start_tick = now_tick;
    tr->t1 = t1; tr->t2 = t2;
    tr->accel = displacement_counts / ((double)(t1 + t2) * (double)t1);
    tr->active = 1;
}

static void traj_tick(traj_t *tr, int64_t now_tick) {
    if (!tr->active) return;
    int64_t k = now_tick - tr->start_tick;
    if (k < tr->t1)                    tr->vel += tr->accel;
    else if (k < tr->t1 + tr->t2)      { /* cruise */ }
    else if (k < 2 * tr->t1 + tr->t2)  tr->vel -= tr->accel;
    else { tr->vel = 0; tr->active = 0; }
    tr->pos += tr->vel;
}

/* ------------------------------------------------------------------ metrics helpers */
typedef struct { double sum, sum2; int64_t n; } stat_t;
static void stat_add(stat_t *s, double v) { s->sum += v; s->sum2 += v * v; s->n++; }
static double stat_mean(const stat_t *s) { return s->n ? s->sum / s->n : 0.0; }
static double stat_std(const stat_t *s) {
    if (!s->n) return 0.0;
    double m = stat_mean(s);
    double v = s->sum2 / s->n - m * m;
    return v > 0 ? sqrt(v) : 0.0;
}

/* ------------------------------------------------------------------ main */
static void usage(void) {
    fprintf(stderr,
        "usage: pid_algorithm_sim [options]\n"
        "  --alg old|step1|step2|step3   algorithm variant (default old)\n"
        "  --kp N --ki N --kd N          PID constants (default 2000 5 175000)\n"
        "  --duration S                  sim length in seconds (default 5)\n"
        "  --move ROT                    trapezoid move of ROT rotations (default none)\n"
        "  --move-time S                 move duration (default 2)\n"
        "  --move-delay S                move start time (default 1)\n"
        "  --noise SIGMA                 hall noise sigma in counts (default 18)\n"
        "  --tau-max NM                  stall torque at V=200 (default 0.40)\n"
        "  --inertia KGM2                rotor+load inertia (default 5.4e-6)\n"
        "  --coulomb NM                  kinetic Coulomb friction (default 0.022)\n"
        "  --static-friction NM          breakaway friction (default 0.060; stick-slip)\n"
        "  --viscous NMS                 viscous friction (default 0.0009)\n"
        "  --load NM                     constant external torque (default 0)\n"
        "  --seed N                      RNG seed (default 12345)\n"
        "  --csv PATH                    write CSV log\n"
        "  --decimate N                  log every Nth tick (default 4)\n"
        "  --quiet                       suppress the summary banner\n"
        "  --step ROT                    INSTANT setpoint step at move-delay (vs profiled --move)\n"
        "  --no-slew                     ablation: bypass the STEP/DIR 1-step-per-tick slew limit\n"
        "  --linear-torque               ablation: torque proportional to output (no V*sin shape)\n"
        "  --d-shift N                   D filter strength: EMA over 2^N ticks (default 5; 0=off)\n"
        "  --i-deadband N                skip integration when |error| < N counts (default off)\n"
        "  --i-freeze-above N            integral separation: skip integration when |error| > N (default off)\n"
        "  --i-clamp-shift N             integral clamp = authority >> N (default 2 = 25%%)\n"
        "  --i-clamp-frac N              integral clamp = authority * N/16 (overrides shift)\n"
        "  --i-gate MODE                 zero-cross windup gate: clamp|freeze|bleed (default off)\n"
        "  --i-gate-holdoff N            one-signed ticks before I may exceed 25%% (default 3125)\n"
        "  --i-gate-deadband N           |error| must exceed N counts to register a sign (default 0)\n"
        "  --i-gate-bleed-shift N        bleed mode: excess decay = excess >> N per tick (default 11)\n"
        "  --i-gate-base-bc              back-calc clamps I to the 25%% rail even when gate open\n");
}

int main(int argc, char **argv) {
    opts_t o = {
        .alg = ALG_OLD, .kp = 2000, .ki = 5, .kd = 175000,
        .duration_s = 5.0, .move_rot = 0.0, .move_time_s = 2.0, .move_delay_s = 1.0,
        /* plant defaults calibrated 2026-07-06 against the bench M17 (stiff-shaft QC reject)
         * so that the "old" algorithm reproduces the measured baselines in
         * firmware/pid_baseline_captures/2026-07-06/: standstill error std ~33 counts,
         * d_term mean ~-85k (sticky rounding bias), kI=5 move peak error ~10k with clean
         * settling, kI=25 move -> sustained stick-slip limit cycle ~±9k at ~22 Hz. */
        .noise_counts = 33.0, .tau_max = 0.40, .inertia = 5.4e-6,
        .coulomb = 0.022, .static_friction = 0.13, .viscous = 0.0009, .load = 0.0,
        .seed = 12345, .csv_path = NULL, .decimate = 4, .quiet = 0,
        .i_deadband = 0, .i_clamp_shift = 2, .d_shift = 5,
        .i_gate = GATE_OFF, .i_gate_holdoff = 3125, .i_gate_deadband = 0,
        .i_gate_bleed_shift = 11, .i_gate_base_bc = 0,
    };

    for (int i = 1; i < argc; i++) {
        #define ARG(name) (!strcmp(argv[i], name) && i + 1 < argc)
        if (ARG("--alg")) {
            i++;
            if      (!strcmp(argv[i], "old"))    o.alg = ALG_OLD;
            else if (!strcmp(argv[i], "step1a")) o.alg = ALG_STEP1A;
            else if (!strcmp(argv[i], "step1b")) o.alg = ALG_STEP1B;
            else if (!strcmp(argv[i], "step1"))  o.alg = ALG_STEP1;
            else if (!strcmp(argv[i], "step2"))  o.alg = ALG_STEP2;
            else if (!strcmp(argv[i], "step3"))  o.alg = ALG_STEP3;
            else { usage(); return 2; }
        }
        else if (ARG("--kp")) o.kp = (uint32_t)strtoul(argv[++i], NULL, 10);
        else if (ARG("--ki")) o.ki = (uint32_t)strtoul(argv[++i], NULL, 10);
        else if (ARG("--kd")) o.kd = (uint32_t)strtoul(argv[++i], NULL, 10);
        else if (ARG("--duration")) o.duration_s = atof(argv[++i]);
        else if (ARG("--move")) o.move_rot = atof(argv[++i]);
        else if (ARG("--step")) o.step_rot = atof(argv[++i]);
        else if (!strcmp(argv[i], "--no-slew")) o.no_slew = 1;
        else if (!strcmp(argv[i], "--linear-torque")) o.linear_torque = 1;
        else if (ARG("--move-time")) o.move_time_s = atof(argv[++i]);
        else if (ARG("--move-delay")) o.move_delay_s = atof(argv[++i]);
        else if (ARG("--noise")) o.noise_counts = atof(argv[++i]);
        else if (ARG("--tau-max")) o.tau_max = atof(argv[++i]);
        else if (ARG("--inertia")) o.inertia = atof(argv[++i]);
        else if (ARG("--coulomb")) o.coulomb = atof(argv[++i]);
        else if (ARG("--static-friction")) o.static_friction = atof(argv[++i]);
        else if (ARG("--viscous")) o.viscous = atof(argv[++i]);
        else if (ARG("--load")) o.load = atof(argv[++i]);
        else if (ARG("--seed")) o.seed = strtoull(argv[++i], NULL, 10);
        else if (ARG("--csv")) o.csv_path = argv[++i];
        else if (ARG("--decimate")) o.decimate = atoi(argv[++i]);
        else if (ARG("--i-deadband")) o.i_deadband = (int32_t)atoi(argv[++i]);
        else if (ARG("--i-freeze-above")) o.i_freeze_above = (int32_t)atoi(argv[++i]);
        else if (ARG("--i-clamp-shift")) o.i_clamp_shift = atoi(argv[++i]);
        else if (ARG("--i-clamp-frac")) o.i_clamp_frac16 = atoi(argv[++i]);
        else if (ARG("--d-shift")) o.d_shift = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--floor-output")) o.floor_output = 1;
        else if (!strcmp(argv[i], "--sticky-decay")) o.sticky_decay = 1;
        else if (ARG("--i-gate")) {
            i++;
            if      (!strcmp(argv[i], "clamp"))  o.i_gate = GATE_CLAMP;
            else if (!strcmp(argv[i], "freeze")) o.i_gate = GATE_FREEZE;
            else if (!strcmp(argv[i], "bleed"))  o.i_gate = GATE_BLEED;
            else if (!strcmp(argv[i], "avg"))    o.i_gate = GATE_AVG;
            else if (!strcmp(argv[i], "off"))    o.i_gate = GATE_OFF;
            else { usage(); return 2; }
        }
        else if (ARG("--i-gate-holdoff")) o.i_gate_holdoff = (int32_t)atoi(argv[++i]);
        else if (ARG("--i-gate-deadband")) o.i_gate_deadband = (int32_t)atoi(argv[++i]);
        else if (ARG("--i-gate-bleed-shift")) o.i_gate_bleed_shift = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--i-gate-base-bc")) o.i_gate_base_bc = 1;
        else if (!strcmp(argv[i], "--quiet")) o.quiet = 1;
        else if (!strcmp(argv[i], "--help")) { usage(); return 0; }
        else { fprintf(stderr, "unknown arg: %s\n", argv[i]); usage(); return 2; }
        #undef ARG
    }

    rng_state = o.seed ? o.seed : 1;
    g_no_slew = o.no_slew;
    controller_init(&o);
    actual_step_position = 0;

    /* plant state */
    double theta = 0.0;         /* true rotor position, counts */
    double omega = 0.0;         /* counts/s */

    traj_t tr; memset(&tr, 0, sizeof(tr));
    int64_t n_ticks = (int64_t)(o.duration_s * TICK_HZ);
    int64_t move_start_tick = (int64_t)(o.move_delay_s * TICK_HZ);
    int64_t move_end_tick = move_start_tick + (int64_t)(o.move_time_s * TICK_HZ);
    int move_started = 0;

    FILE *csv = NULL;
    if (o.csv_path) {
        csv = fopen(o.csv_path, "w");
        if (!csv) { perror("csv"); return 1; }
        fprintf(csv, "# pid_algorithm_sim v1\n");
        fprintf(csv, "# alg: %s\n# set_pid: %u %u %u\n", ALG_NAMES[o.alg], o.kp, o.ki, o.kd);
        fprintf(csv, "# noise: %g, tau_max: %g, inertia: %g, coulomb: %g, static: %g, viscous: %g, load: %g, seed: %llu\n",
                o.noise_counts, o.tau_max, o.inertia, o.coulomb, o.static_friction, o.viscous, o.load,
                (unsigned long long)o.seed);
        if (o.move_rot != 0.0)
            fprintf(csv, "# move_sent_at_s: %.6f\n", o.move_delay_s);
        fprintf(csv, "t_s,error,p_term,i_term,d_term,output,desired,actual,voltage\n");
    }

    /* metrics windows */
    stat_t last1s_err = {0}, last05s_err = {0};              /* settled window at end of run */
    int64_t hold_settle_tick = -1, hold_settle_candidate = -1;   /* first |err|<1000 sustained 0.2 s */
    int32_t i_term_final = 0;
    stat_t ss_err = {0}, ss_d = {0}, ss_i = {0};             /* steady state before move (or last 60% if no move) */
    stat_t post_err = {0};                                    /* after move ends + settle grace */
    double peak_err_move = 0, peak_i = 0;
    double post_peak_err = 0;
    int64_t settle_tick = -1;                                 /* first tick after move end where |err| stays < 200 for 0.2 s */
    int64_t settle_candidate = -1;
    int32_t max_abs_error_seen = 0;
    double vel_at_linear_entry = 0.0;   /* rotor speed when |error| first drops below max_error after the stimulus */
    int linear_entry_seen = 0;
    double peak_abs_vel = 0.0;
    double overshoot_counts = 0.0;      /* most negative error after the stimulus (positive number) */
    int crossed_zero = 0;
    /* limit-cycle frequency: count zero crossings of error in the last 40% of the run */
    int64_t zc_window_start = (int64_t)(n_ticks * 0.6);
    int64_t zero_crossings = 0;
    int32_t prev_sign = 0;

    for (int64_t tick = 0; tick < n_ticks; tick++) {
        /* trajectory */
        if (o.move_rot != 0.0 && !move_started && tick >= move_start_tick) {
            traj_start(&tr, o.move_rot * COUNTS_PER_REV, o.move_time_s, tick);
            move_started = 1;
        }
        if (o.step_rot != 0.0 && !move_started && tick >= move_start_tick) {
            tr.pos += o.step_rot * COUNTS_PER_REV;   /* instant setpoint jump */
            move_started = 1;
        }
        traj_tick(&tr, tick);
        int64_t desired_i64 = (int64_t)llround(tr.pos);

        /* sensor */
        double measured = theta + o.noise_counts * grand_();
        int64_t hall_i64 = (int64_t)llround(measured);
        int32_t sensor_wrapped = (int32_t)hall_i64;

        /* controller (firmware-exact int path; callers pass the full int64 difference) */
        int32_t p_term, i_term, d_term;
        int64_t error_i64 = desired_i64 - hall_i64;
        int32_t output = pid_tick(&o, error_i64, &p_term, &i_term, &d_term);
        int32_t error32 = (int32_t)(error_i64 > PID_ERROR_INPUT_CLAMP ? PID_ERROR_INPUT_CLAMP
                          : (error_i64 < -PID_ERROR_INPUT_CLAMP ? -PID_ERROR_INPUT_CLAMP : error_i64));

        int32_t voltage; uint32_t applied_e;
        m17_output_stage(output, sensor_wrapped, &voltage, &applied_e);

        /* plant: torque from load angle between applied and true electrical angle */
        int32_t true_e = (int32_t)(((int64_t)llround(theta)) & (COUNTS_PER_ECYCLE - 1));
        int32_t delta_e = (int32_t)applied_e - true_e;
        /* wrap to [-32768, 32767] */
        if (delta_e > COUNTS_PER_ECYCLE / 2) delta_e -= COUNTS_PER_ECYCLE;
        else if (delta_e < -COUNTS_PER_ECYCLE / 2) delta_e += COUNTS_PER_ECYCLE;
        double load_angle_rad = (double)delta_e * (2.0 * M_PI / COUNTS_PER_ECYCLE);
        double tau;
        if (o.linear_torque) {
            /* ablation: torque exactly proportional to the PID output (with the same
             * full-authority cap), removing the quadratic V*sin() softness near zero */
            double out_norm = (double)output / (double)(MAX_PWM_V << FUDGE_SHIFT);
            if (out_norm > 1.0) out_norm = 1.0;
            else if (out_norm < -1.0) out_norm = -1.0;
            tau = o.tau_max * out_norm;
        } else {
            tau = o.tau_max * ((double)voltage / MAX_PWM_V) * sin(load_angle_rad);
        }

        double omega_rad = omega / COUNTS_PER_RAD;
        double tau_net;
        /* stick-slip friction: when stopped, the drive torque must exceed the (higher)
         * static friction to break away; once moving, kinetic Coulomb + viscous apply,
         * and the rotor re-sticks when velocity crosses zero with sub-Coulomb torque. */
        double tau_drive = tau - o.load;
        int stuck = (omega == 0.0) ||
                    (fabs(omega_rad) < 2e-3 && fabs(tau_drive) < o.coulomb);
        if (stuck && fabs(tau_drive) <= o.static_friction) {
            omega = 0.0;
            tau_net = 0.0;
        } else {
            double sgn = (omega_rad > 1e-9) ? 1.0 : ((omega_rad < -1e-9) ? -1.0
                          : (tau_drive > 0 ? 1.0 : -1.0));  /* just broke away: kinetic opposes drive */
            tau_net = tau_drive - o.coulomb * sgn - o.viscous * omega_rad;
        }
        double alpha = tau_net / o.inertia;                   /* rad/s^2 */
        omega += alpha * COUNTS_PER_RAD * DT;                 /* counts/s */
        theta += omega * DT;

        /* ---- metrics ---- */
        if (fabs(omega) > peak_abs_vel) peak_abs_vel = fabs(omega);
        if (move_started) {
            if (!linear_entry_seen && error32 < max_error && error32 > 0) {
                vel_at_linear_entry = omega;
                linear_entry_seen = 1;
            }
            if (error32 <= 0) crossed_zero = 1;
            if (crossed_zero && -(double)error32 > overshoot_counts) overshoot_counts = -(double)error32;
        }
        int32_t abs_err = error32 < 0 ? -error32 : error32;
        if (abs_err > max_abs_error_seen) max_abs_error_seen = abs_err;
        if (fabs((double)i_term) > peak_i) peak_i = fabs((double)i_term);
        i_term_final = i_term;

        /* settled-error windows: last 1.0 s and last 0.5 s of the run */
        if (tick >= n_ticks - (int64_t)TICK_HZ)         stat_add(&last1s_err, error32);
        if (tick >= n_ticks - (int64_t)(0.5 * TICK_HZ)) stat_add(&last05s_err, error32);
        /* hold-settle: first time |error| enters and stays below 1000 counts for 0.2 s */
        if (hold_settle_tick < 0) {
            if (abs_err < 1000) {
                if (hold_settle_candidate < 0) hold_settle_candidate = tick;
                if (tick - hold_settle_candidate >= (int64_t)(0.2 * TICK_HZ)) hold_settle_tick = hold_settle_candidate;
            } else {
                hold_settle_candidate = -1;
            }
        }

        if (o.move_rot != 0.0) {
            if (tick < move_start_tick) { stat_add(&ss_err, error32); stat_add(&ss_d, d_term); stat_add(&ss_i, i_term); }
            if (tick >= move_start_tick && tick < move_end_tick && abs_err > peak_err_move) peak_err_move = abs_err;
            if (tick >= move_end_tick + (int64_t)(0.05 * TICK_HZ)) {
                stat_add(&post_err, error32);
                if (abs_err > post_peak_err) post_peak_err = abs_err;
                /* settling detection */
                if (settle_tick < 0) {
                    if (abs_err < 200) {
                        if (settle_candidate < 0) settle_candidate = tick;
                        if (tick - settle_candidate >= (int64_t)(0.2 * TICK_HZ)) settle_tick = settle_candidate;
                    } else {
                        settle_candidate = -1;
                    }
                }
            }
        } else if (tick >= (int64_t)(n_ticks * 0.4)) {
            stat_add(&ss_err, error32); stat_add(&ss_d, d_term); stat_add(&ss_i, i_term);
        }
        if (tick >= zc_window_start) {
            /* Schmitt-style crossing detector so sensor noise near zero does not count:
             * a crossing is registered only when the error passes +/-1000 counts. */
            int32_t sign = error32 > 1000 ? 1 : (error32 < -1000 ? -1 : 0);
            if (prev_sign != 0 && sign != 0 && sign != prev_sign) zero_crossings++;
            if (sign != 0) prev_sign = sign;
        }

        if (csv && (tick % o.decimate) == 0) {
            /* log the POST-max_error-clamp error, exactly as the firmware's debug
             * struct records it, so the CSV column matches hardware captures */
            int32_t log_err = error32;
            if (log_err > max_error) log_err = max_error;
            else if (log_err < -max_error) log_err = -max_error;
            fprintf(csv, "%.6f,%d,%d,%d,%d,%d,%lld,%lld,%d\n",
                    tick * DT, log_err, p_term, i_term, d_term, output,
                    (long long)desired_i64, (long long)hall_i64, voltage);
        }
    }
    if (csv) fclose(csv);

    /* limit-cycle frequency estimate from zero crossings in the last 40% */
    double zc_secs = (n_ticks - zc_window_start) * DT;
    double osc_freq = zero_crossings / 2.0 / zc_secs;
    double post_rms = post_err.n ? sqrt(post_err.sum2 / post_err.n) : 0.0;

    printf("alg=%s kp=%u ki=%u kd=%u\n", ALG_NAMES[o.alg], o.kp, o.ki, o.kd);
    printf("ss_err_std=%.1f\n", stat_std(&ss_err));
    printf("ss_d_mean=%.0f\n", stat_mean(&ss_d));
    printf("ss_d_std=%.0f\n", stat_std(&ss_d));
    printf("ss_i_mean=%.0f\n", stat_mean(&ss_i));
    printf("peak_i=%.0f\n", peak_i);
    printf("peak_i_pct_of_full_rail=%.1f\n", 100.0 * peak_i / (double)(MAX_PWM_V << (PID_SHIFT_RIGHT + FUDGE_SHIFT)));
    printf("max_abs_error=%d\n", max_abs_error_seen);
    if (o.move_rot != 0.0) {
        printf("move_peak_err=%.0f\n", peak_err_move);
        printf("post_move_rms=%.1f\n", post_rms);
        printf("post_move_peak_err=%.0f\n", post_peak_err);
        printf("settle_ms=%.1f\n", settle_tick >= 0 ? (settle_tick - move_end_tick) * DT * 1000.0 : -1.0);
    }
    printf("osc_freq_hz=%.1f\n", osc_freq);
    printf("last1s_err_mean=%.1f\n", stat_mean(&last1s_err));
    printf("last1s_err_std=%.1f\n", stat_std(&last1s_err));
    printf("last05s_err_mean=%.1f\n", stat_mean(&last05s_err));
    printf("last05s_err_std=%.1f\n", stat_std(&last05s_err));
    printf("hold_settle_s=%.3f\n", hold_settle_tick >= 0 ? hold_settle_tick * DT : -1.0);
    printf("i_final=%d\n", i_term_final);
    if (o.i_gate != GATE_OFF) {
        printf("gate=%s holdoff=%d deadband=%d bleed_shift=%d base_bc=%d\n",
               GATE_NAMES[o.i_gate], o.i_gate_holdoff, o.i_gate_deadband,
               o.i_gate_bleed_shift, o.i_gate_base_bc);
        printf("gate_open_pct=%.1f\n", 100.0 * gate_open_ticks / (double)n_ticks);
        printf("gate_crossings=%lld\n", (long long)gate_crossings);
    }
    printf("vel_at_linear_entry_cps=%.0f\n", vel_at_linear_entry);
    printf("peak_vel_rev_s=%.2f\n", peak_abs_vel / COUNTS_PER_REV);
    printf("overshoot_counts=%.0f\n", overshoot_counts);
    printf("saturated_ticks=%lld\n", (long long)saturated_ticks);
    printf("overflow_violations=%lld\n", (long long)overflow_violations);
    if (!o.quiet) {
        fprintf(stderr, "[sim] %s done: %.1f s simulated, final pos %.0f (desired %.0f)\n",
                ALG_NAMES[o.alg], o.duration_s, theta, tr.pos);
    }
    return overflow_violations ? 3 : 0;
}
