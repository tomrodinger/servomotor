#!/bin/bash
# Differential test: the REAL firmware PID_controller (extracted verbatim from
# firmware/Src/motor_control.c at build time) vs the simulator's step2 model
# (c_test_programs/pid_algorithm_sim.c). Feeds both identical random error
# sequences with periodic random gain changes and asserts bitwise-identical
# outputs and integrator state. Exits 0 on PASS.
set -e
cd "$(dirname "$0")"
# PID_SRC (absolute path) overrides the source under test -- used by mutation testing
# to point the harness at a deliberately-broken copy and assert the test FAILS.
SRC="${PID_SRC:-../firmware/Src/motor_control.c}"
WORK="${TMPDIR:-/tmp}/pid_diff_$$"
mkdir -p "$WORK"
trap 'rm -rf "$WORK"' EXIT

# Extract the verbatim firmware functions (from the marker line to the closing brace
# at column 0). If the extraction comes up empty the source layout changed - fail loudly.
extract() {  # $1 = pattern of the function's first line, $2 = output file
    awk "/$1/{found=1} found{print} found && /^}/{exit}" "$SRC" > "$2"
    [ -s "$2" ] || { echo "EXTRACTION FAILED for $1"; exit 1; }
}
extract '^void recompute_pid_parameters_and_set_pwm_voltage' "$WORK/recompute.c"
extract '^int32_t PID_controller\(' "$WORK/pid.c"
# pull the overflow-safety and clamp-policy #defines out of the firmware so both sides
# use the shipped values (a firmware change to INTEGRAL_TERM_AUTHORITY_SHIFT etc. must
# fail this test until the reference model is updated, not be silently masked by stubs)
grep -h "^#define PID_ERROR_INPUT_CLAMP\|^#define PID_MAX_\|^#define INTEGRAL_TERM_AUTHORITY_SHIFT" "$SRC" > "$WORK/pid_defines.h"
grep -h "^#define DERIVATIVE_CONSTANT_AVERAGING_SCALAR_SHIFT" "$(dirname "$SRC")/motor_control.h" >> "$WORK/pid_defines.h" 2>/dev/null || true
[ -s "$WORK/pid_defines.h" ] || { echo "EXTRACTION FAILED for PID_ overflow defines"; exit 1; }
grep -q "INTEGRAL_TERM_AUTHORITY_SHIFT" "$WORK/pid_defines.h" || { echo "EXTRACTION FAILED for INTEGRAL_TERM_AUTHORITY_SHIFT"; exit 1; }

cat > "$WORK/harness.c" <<'EOF'
/* Differential harness: firmware-extracted PID vs simulator step2 model. */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

/* ---- firmware environment stubs / constants (M17 values) ---- */
#include "pid_defines.h"   /* PID_ERROR_INPUT_CLAMP, INTEGRAL_TERM_AUTHORITY_SHIFT,
                            * DERIVATIVE_CONSTANT_AVERAGING_SCALAR_SHIFT -- extracted
                            * from the firmware sources so this harness cannot mask a
                            * firmware clamp-policy change with a stale stub value */
#define PID_SHIFT_RIGHT 11
#define PWM_VOLTAGE_VS_COMMUTATION_POSITION_FUDGE_SHIFT 8
#define DERIVATIVE_CONSTANT_AVERAGING_SCALAR (1 << DERIVATIVE_CONSTANT_AVERAGING_SCALAR_SHIFT)
#define HALL_TO_POSITION_90_DEGREE_OFFSET 16384   /* (64*1024)>>2, M17 commutation table */
#define READ_PID_DEBUG_DATA_TEST_MODE 3
#define MULTIPURPOSE_DATA_TYPE_PID_DEBUG_DATA 4
static void __disable_irq(void) {}
static void __enable_irq(void) {}

/* globals the extracted code references */
static int32_t integral_term = 0;
static int32_t previous_error = 0;
static int32_t low_pass_filtered_error_change = 0;
int32_t min_PID_error = 2147483647;
int32_t max_PID_error = -2147483648;
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
static struct pid_debug_data_struct *pid_debug_data = (void*)&calibration;

/* ---- the extracted firmware code, verbatim ---- */
#include "recompute.c"
#include "pid.c"

void fw_set_pid_constants(uint32_t p, uint32_t i, uint32_t d, uint16_t v) {
    pid_p = p; pid_i = i; pid_d = d;
    recompute_pid_parameters_and_set_pwm_voltage(v, v);
}

/* ---- independent model: hardened step2 semantics (kept in sync with
 * c_test_programs/pid_algorithm_sim.c pid_tick / controller_init and the
 * firmware overflow hardening of 0.15.3.2) ---- */
static int32_t s_integral, s_prev_err, s_lpf;
static int32_t s_max_error, s_max_err_chg, s_max_integral, s_deriv_scaled, s_out_limit;
static int32_t s_kp, s_ki;

static void sim_set_constants(uint32_t p, uint32_t i, uint32_t d, uint16_t v) {
    int64_t p64 = p > (uint32_t)INT32_MAX ? INT32_MAX : (int64_t)p;
    int64_t i64 = i > (uint32_t)INT32_MAX ? INT32_MAX : (int64_t)i;
    int64_t d64 = d > (uint32_t)INT32_MAX ? INT32_MAX : (int64_t)d;
    int64_t authority = (int64_t)v << (11 + 8);
    if (authority > PID_MAX_AUTHORITY_PRE_SHIFT) authority = PID_MAX_AUTHORITY_PRE_SHIFT;
    int64_t me = authority;
    if (p64) me /= p64;
    s_max_error = (int32_t)me;
    s_max_err_chg = 0;
    if (d64) {
        int64_t mec = authority / d64;
        if (mec > PID_MAX_ERROR_CHANGE) mec = PID_MAX_ERROR_CHANGE;
        s_max_err_chg = (int32_t)mec;
    }
    s_max_integral = (int32_t)(authority >> INTEGRAL_TERM_AUTHORITY_SHIFT);
    int64_t ki = i64;
    int64_t eff_me = me > PID_ERROR_INPUT_CLAMP ? (int64_t)PID_ERROR_INPUT_CLAMP : me;
    if (eff_me > 0) {
        int64_t ki_max = ((int64_t)INT32_MAX - s_max_integral) / eff_me;
        if (ki > ki_max) ki = ki_max;
    }
    s_ki = (int32_t)ki;
    s_kp = (int32_t)p64;
    s_deriv_scaled = (int32_t)(d64 >> DERIVATIVE_CONSTANT_AVERAGING_SCALAR_SHIFT);
    s_out_limit = (int32_t)((int64_t)v << 8);
    if (s_out_limit < HALL_TO_POSITION_90_DEGREE_OFFSET) s_out_limit = HALL_TO_POSITION_90_DEGREE_OFFSET;
    /* constants change resets the derivative FILTER state and re-bounds the integral;
     * previous_error is deliberately kept (see firmware comment) */
    s_lpf = 0;
    if (s_integral > s_max_integral) s_integral = s_max_integral;
    else if (s_integral < -s_max_integral) s_integral = -s_max_integral;
}

static int32_t sim_pid(int64_t error_i64) {
    int32_t error;
    if (error_i64 > PID_ERROR_INPUT_CLAMP) error = PID_ERROR_INPUT_CLAMP;
    else if (error_i64 < -PID_ERROR_INPUT_CLAMP) error = -PID_ERROR_INPUT_CLAMP;
    else error = (int32_t)error_i64;
    int32_t d_term;
    if (s_deriv_scaled != 0) {
        int32_t chg = error - s_prev_err;
        if (chg > s_max_err_chg) chg = s_max_err_chg;
        else if (chg < -s_max_err_chg) chg = -s_max_err_chg;
        int32_t dec = s_lpf * (DERIVATIVE_CONSTANT_AVERAGING_SCALAR - 1);
        if (dec < 0) dec += DERIVATIVE_CONSTANT_AVERAGING_SCALAR - 1;
        s_lpf = (dec >> DERIVATIVE_CONSTANT_AVERAGING_SCALAR_SHIFT) + chg;
        d_term = s_lpf * s_deriv_scaled;
    } else d_term = 0;
    s_prev_err = error;
    if (error < -s_max_error) error = -s_max_error;
    else if (error > s_max_error) error = s_max_error;
    if (s_ki != 0) {
        s_integral += error * s_ki;
        if (s_integral > s_max_integral) s_integral = s_max_integral;
        else if (s_integral < -s_max_integral) s_integral = -s_max_integral;
    } else s_integral = 0;
    int32_t p_term = error * s_kp;
    int32_t out = (s_integral + p_term + d_term + (1 << 10)) >> 11;
    if (out > s_out_limit) {
        int64_t ci = (int64_t)s_integral - ((int64_t)(out - s_out_limit) * (1 << 11));
        if (ci < -s_max_integral) ci = -s_max_integral;
        s_integral = (int32_t)ci;
        out = s_out_limit;
    } else if (out < -s_out_limit) {
        int64_t ci = (int64_t)s_integral - ((int64_t)(out + s_out_limit) * (1 << 11));
        if (ci > s_max_integral) ci = s_max_integral;
        s_integral = (int32_t)ci;
        out = -s_out_limit;
    }
    return out;
}

/* ---- driver ---- */
static uint64_t rng = 987654321;
static uint64_t xr(void) { rng ^= rng >> 12; rng ^= rng << 25; rng ^= rng >> 27; return rng * 2685821657736338717ULL; }

int main(void) {
    uint64_t mismatches = 0, n = 0;
    const struct { uint32_t p, i, d; uint16_t v; } gains[] = {
        {2000, 5, 175000, 200}, {2000, 25, 350000, 200}, {2000, 0, 175000, 200}, {2000, 5, 0, 200},
        {500, 100, 1000000, 400}, {20000, 500, 2000000, 65535}, {1, 1, 1, 1}, {0, 0, 0, 0},
        {65535, 1000, 4000000, 4095},
        /* overflow-cap territory: u32 extremes, small-kD LPF corner, kI increment cap */
        {4294967295u, 4294967295u, 4294967295u, 65535},
        {1, 2147483648u, 40, 200}, {2000, 40460, 45, 1820}, {1, 4294967295u, 33, 65535},
    };
    for (int g = 0; g < (int)(sizeof(gains) / sizeof(gains[0])); g++) {
        fw_set_pid_constants(gains[g].p, gains[g].i, gains[g].d, gains[g].v);
        sim_set_constants(gains[g].p, gains[g].i, gains[g].d, gains[g].v);
        /* reset both states fully */
        integral_term = 0; previous_error = 0; low_pass_filtered_error_change = 0;
        s_integral = 0; s_prev_err = 0; s_lpf = 0;
        for (int k = 0; k < 500000; k++) {
            /* halfway through, change the constants WITHOUT a full state reset, so the
             * firmware's constants-change state handling is compared too */
            if (k == 250000) {
                int gg = (g + 5) % (int)(sizeof(gains) / sizeof(gains[0]));
                fw_set_pid_constants(gains[gg].p, gains[gg].i, gains[gg].d, gains[gg].v);
                sim_set_constants(gains[gg].p, gains[gg].i, gains[gg].d, gains[gg].v);
            }
            /* mix of small errors, mid-range noise, and huge int64 spikes */
            int64_t e;
            uint32_t r = (uint32_t)(xr() >> 40);
            if      (r % 7 == 0) e = (int64_t)(xr() % 2000001) - 1000000;
            else if (r % 7 == 1) e = (int64_t)(xr() % 200001) - 100000;
            else if (r % 7 == 2) e = (int64_t)(xr() >> 20) - (1LL << 43);
            else                 e = (int64_t)(xr() % 2001) - 1000;
            int32_t a = PID_controller(e);
            int32_t b = sim_pid(e);
            n++;
            if (a != b || integral_term != s_integral ||
                low_pass_filtered_error_change != s_lpf) {
                if (mismatches < 5)
                    printf("MISMATCH gains=%u/%u/%u/%u k=%d e=%lld fw=%d sim=%d fwI=%d simI=%d fwLPF=%d simLPF=%d\n",
                           gains[g].p, gains[g].i, gains[g].d, gains[g].v, k, (long long)e, a, b,
                           integral_term, s_integral, low_pass_filtered_error_change, s_lpf);
                mismatches++;
            }
        }
    }
    printf("%llu samples, %llu mismatches\n", (unsigned long long)n, (unsigned long long)mismatches);
    printf(mismatches ? "FAILED\n" : "PASSED\n");
    return mismatches != 0;
}
EOF

cc -O2 -Wall -I "$WORK" -o "$WORK/pid_diff" "$WORK/harness.c" -lm
"$WORK/pid_diff"
