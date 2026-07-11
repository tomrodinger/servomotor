#!/bin/bash
# Permanent integer-overflow test for the firmware PID control loop.
# ===================================================================
# Extracts PID_controller() and recompute_pid_parameters_and_set_pwm_voltage()
# VERBATIM from firmware/Src/motor_control.c, compiles them together with the
# adversarial driver test_pid_overflow.c under clang UBSan, and fails if ANY
# signed-integer overflow, invalid shift, or divide-by-zero occurs anywhere in
# the extracted firmware code, or if the int64 shadow model disagrees with the
# int32 firmware arithmetic on any tick.
#
# By default the driver runs once per PRODUCT PARAMETER SET (the PID is shared
# code): M17 (PID_SHIFT_RIGHT=11), M23 (14), and M1/M2 (18, with the M2-only
# 'error >>= 3' prescale compiled in), so all products' arithmetic is proven.
#
# Run this whenever PID_controller or its constants change:
#     cd c_test_programs && ./test_pid_overflow.sh          (~4-6 min, all products)
#     ./test_pid_overflow.sh --quick                        (~1 min smoke, all products)
#
# A meta-test first proves the sanitizer actually detects a known overflow, so
# a silently-misconfigured toolchain cannot produce a false PASS.
set -e
cd "$(dirname "$0")"
# PID_SRC (absolute path) overrides the source under test -- used by mutation testing
# and product-parameter studies to point the harness at a modified copy.
# EXTRA_PID_CFLAGS adds compiler flags; when set, it REPLACES the built-in product
# parameter loop (single run with your flags), e.g. -DPID_SHIFT_RIGHT=14.
SRC="${PID_SRC:-../firmware/Src/motor_control.c}"
SRC_HDR="$(dirname "$SRC")/motor_control.h"
WORK="${TMPDIR:-/tmp}/pid_overflow_$$"
mkdir -p "$WORK"
trap 'rm -rf "$WORK"' EXIT

SAN_FLAGS="-fsanitize=signed-integer-overflow,shift,integer-divide-by-zero"

# ---------------------------------------------------------------- meta-test
cat > "$WORK/meta.c" <<'EOF'
#include <stdio.h>
#include <stdint.h>
int main(void) {
    volatile int32_t a = 2000000000, b = 2000000000;
    volatile int32_t c = a + b;              /* must trigger UBSan */
    printf("%d\n", c);
    return 0;
}
EOF
cc -O1 $SAN_FLAGS -o "$WORK/meta" "$WORK/meta.c"
if ! "$WORK/meta" 2>&1 | grep -q "runtime error"; then
    echo "META-TEST FAILED: UBSan did not detect a known int32 overflow -- toolchain problem."
    exit 2
fi
echo "meta-test: sanitizer detects a known overflow -- OK"

# ---------------------------------------------------------------- extraction
extract() {  # $1 = pattern of the function's first line, $2 = output file
    awk "/$1/{found=1} found{print} found && /^}/{exit}" "$SRC" > "$2"
    [ -s "$2" ] || { echo "EXTRACTION FAILED for $1 -- did the function signature change?"; exit 2; }
}
extract '^void recompute_pid_parameters_and_set_pwm_voltage' "$WORK/extracted_recompute.c"
extract '^int32_t PID_controller\(' "$WORK/extracted_pid.c"
echo "extracted $(wc -l < "$WORK/extracted_pid.c") + $(wc -l < "$WORK/extracted_recompute.c") lines of firmware code"

# Pull the PID overflow-safety #defines out of the firmware sources so the extracted
# functions compile with the exact values that ship (the driver's own defines are
# #ifndef fallbacks). DERIVATIVE_CONSTANT_AVERAGING_SCALAR_SHIFT lives in the header.
grep -h "^#define PID_\|^#define INTEGRAL_TERM_AUTHORITY_SHIFT" "$SRC" > "$WORK/clamp_fallback.h" || true
if [ -f "$SRC_HDR" ]; then
    grep -h "^#define DERIVATIVE_CONSTANT_AVERAGING_SCALAR_SHIFT" "$SRC_HDR" >> "$WORK/clamp_fallback.h" || true
fi
if ! grep -q "PID_ERROR_INPUT_CLAMP" "$WORK/clamp_fallback.h"; then
    echo "#define PID_ERROR_INPUT_CLAMP ((int64_t)INT32_MAX)" >> "$WORK/clamp_fallback.h"
fi

# ---------------------------------------------------------------- build & run (per product)
run_one() {  # $1 = label, $2 = extra cflags; remaining script args passed to the driver
    local label="$1" cflags="$2"; shift 2
    echo ""
    echo "================ product parameters: $label ================"
    cc -O1 -Wall $SAN_FLAGS $cflags -include "$WORK/clamp_fallback.h" -I "$WORK" \
       -o "$WORK/overflow_test" test_pid_overflow.c
    set +e
    "$WORK/overflow_test" "$@" > "$WORK/stdout.log" 2> "$WORK/stderr.log"
    local rc=$?
    set -e
    local hits
    hits=$(grep -c "runtime error" "$WORK/stderr.log" || true)
    grep -v "runtime error" "$WORK/stderr.log" | tail -8
    cat "$WORK/stdout.log"
    if [ "$hits" -gt 0 ]; then
        echo "UBSan detected $hits overflow/shift/div0 sites in the firmware PID code [$label]:"
        grep "runtime error" "$WORK/stderr.log" | sort -u | head -20
        return 1
    fi
    [ "$rc" -eq 0 ] || { echo "invariant mismatches [$label] -- see above"; return 1; }
    return 0
}

FAILED=0
if [ -n "${EXTRA_PID_CFLAGS:-}" ]; then
    run_one "custom (EXTRA_PID_CFLAGS)" "$EXTRA_PID_CFLAGS" "$@" || FAILED=1
else
    run_one "M17 (shift 11)" "-DPID_SHIFT_RIGHT=11" "$@" || FAILED=1
    run_one "M23 (shift 14)" "-DPID_SHIFT_RIGHT=14" "$@" || FAILED=1
    run_one "M1/M2 (shift 18, M2 error prescale)" "-DPID_SHIFT_RIGHT=18 -DPRODUCT_NAME_M2" "$@" || FAILED=1
fi

echo ""
if [ "$FAILED" -ne 0 ]; then
    echo "OVERALL: FAILED"
    exit 1
fi
echo "OVERALL: PASSED -- no signed overflow, no bad shifts, no div0, shadow model exact on every tick (all parameter sets)"
