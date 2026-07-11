/*
 * test_pid_overflow.c — adversarial integer-overflow test for the firmware PID
 * =============================================================================
 *
 * Compiled by test_pid_overflow.sh, which first extracts PID_controller() and
 * recompute_pid_parameters_and_set_pwm_voltage() VERBATIM from
 * firmware/Src/motor_control.c, so this always tests the code that ships.
 *
 * Detection is layered:
 *   1. clang UBSan (-fsanitize=signed-integer-overflow,shift,integer-divide-by-zero)
 *      reports the exact source line of any signed overflow / bad shift / div0.
 *   2. Invariant shadow checks after every tick (int64 recomputation of the
 *      output identity, integral-clamp bound, LPF-state bound).
 *
 * The driver is deliberately adversarial:
 *   - a grid of gain/voltage configurations including every boundary the math
 *     cares about (kD around the AVG_SCALAR edge 31..64, kI at the integral
 *     increment limit, V at each product's shift-overflow boundary, full u32
 *     extremes for all three gains),
 *   - hostile error waveforms (extremes, alternating extremes, ramps, triangle
 *     waves across 6 decades of amplitude, spike trains, windup pump cycles,
 *     random heavy-tailed noise),
 *   - runtime gain/voltage CHANGES mid-waveform (a stale filter state meeting
 *     new constants is a classic overflow source),
 *   - a long random soak.
 *
 * Every configuration/waveform pair also verifies, in int64, that
 *     output == clamp((I + P + D + half) >> PID_SHIFT_RIGHT)
 * still holds sample-for-sample, so "no overflow" cannot be achieved by
 * silently producing wrong values.
 *
 * Run:  ./test_pid_overflow.sh          (from c_test_programs/)
 * Exit 0 = clean. Any UBSan report or invariant failure -> exit 1.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>

/* ---- firmware environment (M17 values; overridable per product) ---- */
#ifndef PID_SHIFT_RIGHT
#define PID_SHIFT_RIGHT 11
#endif
#ifndef PWM_VOLTAGE_VS_COMMUTATION_POSITION_FUDGE_SHIFT
#define PWM_VOLTAGE_VS_COMMUTATION_POSITION_FUDGE_SHIFT 8
#endif
/* the wrapper script extracts DERIVATIVE_CONSTANT_AVERAGING_SCALAR_SHIFT and
 * INTEGRAL_TERM_AUTHORITY_SHIFT from the firmware sources into clamp_fallback.h
 * (-include'd before this file); these are last-resort fallbacks only */
#ifndef DERIVATIVE_CONSTANT_AVERAGING_SCALAR_SHIFT
#define DERIVATIVE_CONSTANT_AVERAGING_SCALAR_SHIFT 5
#endif
#define DERIVATIVE_CONSTANT_AVERAGING_SCALAR (1 << DERIVATIVE_CONSTANT_AVERAGING_SCALAR_SHIFT)
#ifndef INTEGRAL_TERM_AUTHORITY_SHIFT
#define INTEGRAL_TERM_AUTHORITY_SHIFT 2
#endif
/* commutation 90-degree offset: (N_COMMUTATION_STEPS * N_COMMUTATION_SUB_STEPS) >> 2;
 * 16384 for M17/M23 (64 x 1024). Override via EXTRA_PID_CFLAGS for other tables. */
#ifndef HALL_TO_POSITION_90_DEGREE_OFFSET
#define HALL_TO_POSITION_90_DEGREE_OFFSET 16384
#endif
#define READ_PID_DEBUG_DATA_TEST_MODE 3
#define MULTIPURPOSE_DATA_TYPE_PID_DEBUG_DATA 4
static void __disable_irq(void) {}
static void __enable_irq(void) {}

/* globals the extracted firmware code references */
static int32_t integral_term = 0;
static int32_t previous_error = 0;
static int32_t low_pass_filtered_error_change = 0;
int32_t min_PID_error = INT32_MAX;
int32_t max_PID_error = INT32_MIN;
static uint32_t pid_p = 0, pid_i = 0, pid_d = 0;
static int32_t proportional_constant_pid = 0;
static int32_t integral_constant_pid = 0;
static int32_t derivative_constant_pid_scaled_for_averaging = 0;
static int32_t max_integral_term = 0;
static int32_t max_error = 0;
static int32_t max_error_change = 0;
static int32_t pid_output_limit = 0;
static uint16_t max_motor_pwm_voltage = 200;
static uint16_t max_motor_regen_pwm_voltage = 200;
static uint8_t test_mode = 0;
static uint8_t multipurpose_data_type = 0;
static uint16_t multipurpose_data_size = 0;
static uint8_t calibration[64];
struct __attribute__((__packed__)) pid_debug_data_struct {
    int32_t error, proportional_term, integral_term, derivative_term, output_value;
};
static struct pid_debug_data_struct *pid_debug_data = (void *)&calibration;

/* ---- the extracted firmware code, verbatim (provided by the shell script) ---- */
#include "extracted_recompute.c"
#include "extracted_pid.c"

static void set_constants(uint32_t p, uint32_t i, uint32_t d, uint16_t v) {
    pid_p = p; pid_i = i; pid_d = d;
    recompute_pid_parameters_and_set_pwm_voltage(v, v);
}

static void reset_state(void) {
    integral_term = 0;
    previous_error = 0;
    low_pass_filtered_error_change = 0;
}

/* ------------------------------------------------------------------ checks */
static uint64_t g_ticks = 0, g_invariant_failures = 0;
static const char *g_context = "";

static void fail(const char *what, int64_t a, int64_t b) {
    if (g_invariant_failures < 20) {
        fprintf(stderr, "INVARIANT FAIL [%s] tick=%" PRIu64 ": %s (%" PRId64 " vs %" PRId64 ")  "
                        "gains=%u/%u/%u V=%u I=%d lpf=%d\n",
                g_context, g_ticks, what, a, b, pid_p, pid_i, pid_d,
                max_motor_pwm_voltage, integral_term, low_pass_filtered_error_change);
    }
    g_invariant_failures++;
}

/* one controller tick + full int64 shadow verification */
static void tick(int64_t error_i64) {
    /* snapshot pre-tick state for the shadow model */
    int64_t I0 = integral_term;
    int64_t prev0 = previous_error;
    int64_t lpf0 = low_pass_filtered_error_change;

    int32_t out = PID_controller(error_i64);
    g_ticks++;

    /* ---- int64 shadow model of the (hardened) firmware semantics ---- */
    int64_t e = error_i64;
    if (e > PID_ERROR_INPUT_CLAMP) e = PID_ERROR_INPUT_CLAMP;
    else if (e < -PID_ERROR_INPUT_CLAMP) e = -PID_ERROR_INPUT_CLAMP;
#ifdef PRODUCT_NAME_M2
    e >>= 3;   /* mirror the M2-only error prescale in PID_controller */
#endif

    int64_t d_term = 0;
    int64_t lpf = lpf0;
    if (derivative_constant_pid_scaled_for_averaging != 0) {
        int64_t chg = e - prev0;
        if (chg > max_error_change) chg = max_error_change;
        else if (chg < -max_error_change) chg = -max_error_change;
        int64_t dec = lpf * (DERIVATIVE_CONSTANT_AVERAGING_SCALAR - 1);
        if (dec < 0) dec += DERIVATIVE_CONSTANT_AVERAGING_SCALAR - 1;
        lpf = (dec >> DERIVATIVE_CONSTANT_AVERAGING_SCALAR_SHIFT) + chg;
        d_term = lpf * derivative_constant_pid_scaled_for_averaging;
    }

    int64_t ec = e;
    if (ec > max_error) ec = max_error;
    else if (ec < -max_error) ec = -max_error;
    int64_t I = I0;
    if (integral_constant_pid != 0) {
        I += ec * integral_constant_pid;
        if (I > max_integral_term) I = max_integral_term;
        else if (I < -max_integral_term) I = -max_integral_term;
    } else {
        I = 0;
    }
    int64_t P = ec * proportional_constant_pid;
    int64_t sum = I + P + d_term + (1 << (PID_SHIFT_RIGHT - 1));
    int64_t expected = sum >> PID_SHIFT_RIGHT;
    if (expected > pid_output_limit) {
        int64_t ci = I - ((expected - pid_output_limit) * (1 << PID_SHIFT_RIGHT));
        if (ci < -max_integral_term) ci = -max_integral_term;
        I = ci;
        expected = pid_output_limit;
    } else if (expected < -pid_output_limit) {
        int64_t ci = I - ((expected + pid_output_limit) * (1 << PID_SHIFT_RIGHT));
        if (ci > max_integral_term) ci = max_integral_term;
        I = ci;
        expected = -pid_output_limit;
    }

    /* ---- compare ---- */
    if ((int64_t)out != expected) fail("output identity", out, expected);
    if ((int64_t)integral_term != I) fail("integral state", integral_term, I);
    if ((int64_t)low_pass_filtered_error_change != lpf) fail("lpf state", low_pass_filtered_error_change, lpf);
    /* absolute bounds that make later-tick int32 arithmetic provably safe */
    int64_t lpf_bound = 32LL * (max_error_change ? max_error_change : 0);
    if (llabs((int64_t)low_pass_filtered_error_change) > lpf_bound)
        fail("lpf bound", low_pass_filtered_error_change, lpf_bound);
    if (llabs((int64_t)integral_term) > max_integral_term)
        fail("integral bound", integral_term, max_integral_term);
}

/* ------------------------------------------------------------------ waveforms */
static uint64_t rng = 0x243F6A8885A308D3ULL;
static uint64_t xr(void) { rng ^= rng >> 12; rng ^= rng << 25; rng ^= rng >> 27; return rng * 2685821657736338717ULL; }
static int64_t rr(int64_t lo, int64_t hi) { return lo + (int64_t)(xr() % (uint64_t)(hi - lo + 1)); }

#define WAVE_TICKS 20000

static void wave_const(int64_t v)               { for (int k = 0; k < WAVE_TICKS; k++) tick(v); }
static void wave_alternate(int64_t v)           { for (int k = 0; k < WAVE_TICKS; k++) tick(k & 1 ? v : -v); }
static void wave_ramp(int64_t step)             { int64_t e = 0; for (int k = 0; k < WAVE_TICKS; k++) { e += step; tick(e); } }
static void wave_triangle(int64_t amp, int per) {
    int64_t e = 0, dir = 1;
    for (int k = 0; k < WAVE_TICKS; k++) {
        e += dir * (2 * amp / per);
        if (e > amp) { e = amp; dir = -1; } else if (e < -amp) { e = -amp; dir = 1; }
        tick(e);
    }
}
static void wave_spikes(int64_t v, int gap)     { for (int k = 0; k < WAVE_TICKS; k++) tick((k % gap) == 0 ? ((k / gap) & 1 ? v : -v) : 0); }
static void wave_pump(int64_t v, int hold)      { /* windup pump: sustain, reverse, sustain... */
    for (int k = 0; k < WAVE_TICKS; k++) tick(((k / hold) & 1) ? -v : v);
}
static void wave_random(int64_t amp)            { for (int k = 0; k < WAVE_TICKS; k++) tick(rr(-amp, amp)); }

typedef struct { const char *name; void (*run)(void); } wave_t;
static void w_zero(void)      { wave_const(0); }
static void w_maxpos(void)    { wave_const(INT64_MAX / 4); }
static void w_maxneg(void)    { wave_const(INT64_MIN / 4); }
static void w_i32edge(void)   { wave_alternate((int64_t)INT32_MAX); }
static void w_alt_huge(void)  { wave_alternate(1LL << 40); }
static void w_alt_small(void) { wave_alternate(1); }
static void w_ramp_slow(void) { wave_ramp(3); }
static void w_ramp_fast(void) { wave_ramp(1000000); }
static void w_tri_1e3(void)   { wave_triangle(1000, 50); }
static void w_tri_1e5(void)   { wave_triangle(100000, 200); }
static void w_tri_1e7(void)   { wave_triangle(10000000, 500); }
static void w_tri_1e9(void)   { wave_triangle(1000000000, 100); }
static void w_spikes(void)    { wave_spikes(1LL << 35, 7); }
static void w_pump_fast(void) { wave_pump(1LL << 33, 40); }
static void w_pump_slow(void) { wave_pump(200000, 5000); }
static void w_rand_small(void){ wave_random(2000); }
static void w_rand_big(void)  { wave_random(1LL << 36); }
static const wave_t WAVES[] = {
    {"zero", w_zero}, {"const_max", w_maxpos}, {"const_min", w_maxneg},
    {"alt_i32max", w_i32edge}, {"alt_2^40", w_alt_huge}, {"alt_1", w_alt_small},
    {"ramp_slow", w_ramp_slow}, {"ramp_fast", w_ramp_fast},
    {"tri_1e3", w_tri_1e3}, {"tri_1e5", w_tri_1e5}, {"tri_1e7", w_tri_1e7}, {"tri_1e9", w_tri_1e9},
    {"spikes", w_spikes}, {"pump_fast", w_pump_fast}, {"pump_slow", w_pump_slow},
    {"rand_small", w_rand_small}, {"rand_big", w_rand_big},
};
#define N_WAVES (sizeof(WAVES) / sizeof(WAVES[0]))

/* ------------------------------------------------------------------ main */
static const uint32_t KP_VALS[] = {0, 1, 2, 3, 31, 32, 33, 1000, 2000, 20000, 65535,
                                   1000000, 2147483647u, 2147483648u, 4294967295u};
static const uint32_t KI_VALS[] = {0, 1, 5, 25, 200, 40000, 40459, 40460, 65535,
                                   1000000, 2147483647u, 2147483648u, 4294967295u};
static const uint32_t KD_VALS[] = {0, 1, 31, 32, 33, 40, 48, 49, 52, 63, 64, 175000, 350000,
                                   1000000, 100000000, 2147483647u, 2147483648u, 4294967295u};
static const uint16_t V_VALS[]  = {0, 1, 31, 32, 100, 200, 400, 511, 512, 1820, 1821, 4095, 4096, 65535};

int main(int argc, char **argv) {
    int quick = (argc > 1 && !strcmp(argv[1], "--quick"));
    char ctx[256];

    /* -------- phase 1: boundary gain grid x all waveforms (sampled) -------- */
    fprintf(stderr, "[phase 1] boundary gain grid x waveforms...\n");
    uint64_t n_cfg = 0;
    for (unsigned a = 0; a < sizeof(KP_VALS) / 4; a++)
    for (unsigned b = 0; b < sizeof(KI_VALS) / 4; b++)
    for (unsigned c = 0; c < sizeof(KD_VALS) / 4; c++) {
        /* full cross of gains, but sample voltages + waveforms to keep runtime sane:
         * every config gets 3 pseudo-randomly chosen voltages and 5 waveforms,
         * chosen deterministically from the config index. */
        for (int vi = 0; vi < (quick ? 1 : 3); vi++) {
            uint16_t v = V_VALS[(a * 7 + b * 5 + c * 3 + vi * 11) % (sizeof(V_VALS) / 2)];
            set_constants(KP_VALS[a], KI_VALS[b], KD_VALS[c], v);
            for (int wi = 0; wi < (quick ? 2 : 5); wi++) {
                const wave_t *w = &WAVES[(a + b * 3 + c * 7 + vi * 13 + wi * 5) % N_WAVES];
                snprintf(ctx, sizeof(ctx), "grid kp=%u ki=%u kd=%u V=%u wave=%s",
                         KP_VALS[a], KI_VALS[b], KD_VALS[c], v, w->name);
                g_context = ctx;
                reset_state();
                w->run();
                n_cfg++;
            }
        }
    }
    fprintf(stderr, "[phase 1] %" PRIu64 " config/waveform pairs done\n", n_cfg);

    /* -------- phase 2: the known-nasty corners, exhaustively -------- */
    fprintf(stderr, "[phase 2] targeted corners...\n");
    /* 2a: small-kD LPF decay corner across all voltages and pump waveforms */
    for (unsigned c = 0; c < sizeof(KD_VALS) / 4; c++) {
        for (unsigned vi = 0; vi < sizeof(V_VALS) / 2; vi++) {
            set_constants(2000, 5, KD_VALS[c], V_VALS[vi]);
            snprintf(ctx, sizeof(ctx), "lpf-corner kd=%u V=%u", KD_VALS[c], V_VALS[vi]);
            g_context = ctx;
            reset_state();
            wave_alternate(1LL << 33);   /* max error_change every tick */
            wave_pump(1LL << 33, 3);
        }
    }
    /* 2b: integral increment corner: max error with each kI, tiny kP (max_error large) */
    for (unsigned b = 0; b < sizeof(KI_VALS) / 4; b++) {
        for (unsigned a = 0; a < sizeof(KP_VALS) / 4; a++) {
            set_constants(KP_VALS[a], KI_VALS[b], 175000, 200);
            snprintf(ctx, sizeof(ctx), "I-corner kp=%u ki=%u", KP_VALS[a], KI_VALS[b]);
            g_context = ctx;
            reset_state();
            wave_pump(1LL << 33, 200);
        }
    }

    /* -------- phase 3: runtime gain/voltage changes with hot state -------- */
    fprintf(stderr, "[phase 3] mid-flight constant changes (stale-state hazards)...\n");
    g_context = "gain-change fuzz";
    rng = 0x9E3779B97F4A7C15ULL;
    int n_rounds = quick ? 2000 : 20000;
    reset_state();
    set_constants(2000, 5, 175000, 200);
    for (int round = 0; round < n_rounds; round++) {
        /* run a short hostile burst, then change constants WITHOUT resetting state */
        int64_t amp = 1LL << rr(0, 40);
        for (int k = 0; k < 60; k++) tick((k & 1 ? amp : -amp));
        uint32_t p = KP_VALS[xr() % (sizeof(KP_VALS) / 4)];
        uint32_t i = KI_VALS[xr() % (sizeof(KI_VALS) / 4)];
        uint32_t d = KD_VALS[xr() % (sizeof(KD_VALS) / 4)];
        uint16_t v = V_VALS[xr() % (sizeof(V_VALS) / 2)];
        snprintf(ctx, sizeof(ctx), "gain-change round=%d -> kp=%u ki=%u kd=%u V=%u", round, p, i, d, v);
        g_context = ctx;
        set_constants(p, i, d, v);
        for (int k = 0; k < 60; k++) tick(rr(-(1LL << 38), 1LL << 38));
    }

    /* -------- phase 4: long random soak at plausible settings -------- */
    fprintf(stderr, "[phase 4] random soak...\n");
    g_context = "soak";
    set_constants(2000, 25, 350000, 200);
    reset_state();
    int64_t e = 0;
    uint64_t soak = quick ? 1000000 : 10000000;
    for (uint64_t k = 0; k < soak; k++) {
        e += rr(-3000, 3000);                       /* random walk */
        if ((xr() & 0xFFF) == 0) e += rr(-(1LL << 34), 1LL << 34);  /* rare huge jumps */
        tick(e);
    }

    fprintf(stderr, "\n%" PRIu64 " total ticks, %" PRIu64 " invariant failures\n",
            g_ticks, g_invariant_failures);
    if (g_invariant_failures) { printf("FAILED\n"); return 1; }
    printf("PASSED (invariants; UBSan verdict comes from the wrapper script)\n");
    return 0;
}
