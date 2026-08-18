#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_current_and_pid_settings.cpp
//
// THE TWO CONTROLLER-TUNING SETTINGS NOBODY OWNS:
//   cmd 28  Set maximum motor current  (motorCurrent, regenerationCurrent)
//   cmd 43  Set PID constants          (kP, kI, kD)
//
// These are the only settings on the device that reach into the current-control
// and feedback maths rather than into the planner, and they are the only two
// that are applied ATOMICALLY WITH INTERRUPTS DISABLED while the machine may be
// moving (motor_control.c:2319 `__disable_irq()` ... :2344 `__enable_irq()`).
// Both funnel into the same function -- `recompute_pid_parameters_and_set_pwm_voltage()`
// (motor_control.c:2258) -- which rebuilds the controller's max-error window,
// integral clamp, derivative-change cap and output limit from scratch and then
// re-clamps the LIVE integrator state to the new bounds (motor_control.c:2338).
// That is a rewrite of the control law underneath a move in flight. If it were
// not exactly right, the visible symptom would be a move that quietly ends up
// somewhere other than where it was told to go.
//
// WHAT THIS MODULE ASSERTS
//   1. The enforced acceptance envelope is exactly the DOCUMENTED one. The
//      ceiling is a per-product compile-time constant, MAX_PWM_VOLTAGE_AT_ZERO_VELOCITY
//      (motor_control.c:54 M1/M2 = 300, :57 M17 = 390, :60 M23 = PWM_PERIOD_TIM1 = 1024),
//      and the check is strictly greater-than (motor_control.c:3513, :3516), so
//      the ceiling VALUE ITSELF is legal and one past it is fatal error 23,
//      ERROR_MAX_PWM_VOLTAGE_TOO_HIGH. The ceiling is not assumed here: it is
//      MEASURED by binary search and then compared against what the API
//      documentation promises for the product the device says it is.
//   2. The regeneration argument is INDEPENDENT of the motor-current argument.
//      It has its own check against the same ceiling (motor_control.c:3516) and
//      no ordering relationship between the two is enforced -- regen may exceed
//      the motor current, either may be zero while the other is at the ceiling.
//      The API docs warn that regen "is still range-checked ... do not treat
//      this parameter as ignorable"; that warning is what section 4 verifies.
//   3. setPidConstants accepts the whole u32 domain, INCLUDING ZEROS. Zero is
//      the interesting case: `new_max_error /= p64` (motor_control.c:2277-2279),
//      `authority / d64` (:2296-2297) and the ki cap (:2313) are all divisions by
//      a user-supplied gain, each guarded by an explicit non-zero test.
//      HONEST LIMIT ON WHAT THAT PROVES: integer division by zero does NOT trap
//      on either MCU here -- these are int64 divisions, so they go through the
//      compiler's __aeabi_ldivmod helper, which returns 0 rather than faulting.
//      The firmware says so itself about the same hazard in the planner
//      ("would be by zero, which does NOT trap on this core", motor_control.c:457).
//      So a regression that dropped one of those guards would be SILENT, and no
//      black-box test can see it. What is asserted below is therefore only that
//      zero gains are ACCEPTED and leave the machine working -- not that the
//      guards are still present.
//   4. These are IMMEDIATE-effect settings (main.c:630-641 and main.c:976-991 --
//      no add_to_queue() on either path), so they can be changed mid-move
//      without disturbing the move. The proof is differential: an undisturbed
//      reference move is measured first, then the identical move is repeated
//      while the settings are hammered in flight, and the two must land in the
//      same place to within the suite's 2-count determinism tolerance.
//   5. Setting them QUEUES NOTHING. At idle the depth is identical before and
//      after a burst. Mid-move the assertion is the race-free half of that
//      claim -- the depth never GROWS -- because the ISR may legitimately retire
//      a segment while the burst is in flight, and "identical" would then fail
//      for a reason that has nothing to do with the settings.
//   6. Neither survives a reset, and a rejection does not widen the ceiling.
//
// HOW THIS IS DISTINCT FROM WHAT ALREADY EXISTS
//   * tm_pid_dynamics owns the PID-ERROR WINDOW (cmd 39) in CLOSED LOOP, with
//     the MOSFETs on: it is about whether the controller controls. This module
//     never enters closed loop and asserts nothing about torque or tracking.
//   * tm_settings touches cmd 28 in passing -- four in-range values and 65535 --
//     but never measures where the boundary actually is, never probes the regen
//     argument separately, and never tests the settings against motion.
//   * tm_setting_timing shows cmd 28 and cmd 43 are ACCEPTED mid-move; it does
//     not check that the move is UNHARMED, which is the interesting half.
//   * tm_reset_completeness cripples every setting at once; the current/PID
//     specific part of that claim is sharpened here.
//   * tm_command_state_matrix records cmd 43 as "accepted in every state" as one
//     cell of a 36-cell table, with no range or interference content.
//
// SAFETY. The MOSFETs stay OFF for the entire module: `enableMosfets()` and
// `goToClosedLoop()` are never called, and the output stage is explicitly
// disabled at entry. Every position read is `getPositionRaw()`, the COMMANDED
// position, which the planner advances whether or not the output stage is
// energised -- so nothing turns, on any motor, at any point, and no current
// flows however the current limit is set. Because no torque is produced, this
// module deliberately asserts ACCEPTANCE and NON-INTERFERENCE only, never
// physical effect. The largest move is half a rotation (sections 8 and 9 use a
// quarter and an eighth), well inside even the default two-rotation deviation
// window, so error 45 cannot be tripped even where the ceilings are left at
// their factory values. Fatal error 23 is raised on purpose several
// times and is cleared by a reset like any other. No broadcasts, no alias
// writes, no test modes: safe on the shared 35-motor rack.
// ---------------------------------------------------------------------------

static const uint8_t ERR_MAX_PWM_VOLTAGE_TOO_HIGH = 23;

// Above every product's ceiling (the largest is M23's 1024), so it is the
// known-rejected end of the binary search below.
static const uint16_t SEARCH_HI = 2048;

// Currents that are legal on EVERY product (the smallest ceiling is 300), used
// wherever the point is something other than the boundary itself.
static const uint16_t SAFE_CURRENTS[] = { 0, 50, 100, 200, 150 };
static const int N_SAFE_CURRENTS = 5;

// PID gain triples used for the in-flight hammering and the queue burst. All are
// well inside int32, so they exercise the ordinary path rather than the u32 ->
// int32 caps; the caps themselves are covered by section 6's combos table.
static const uint32_t PID_SET[][3] = {
    {0, 0, 0},
    {2000, 5, 175000},
    {1, 1, 1},
    {100000, 0, 0},
    {0, 0, 1000000},
};
static const int N_PID_SET = 5;

// Set when a Set-maximum-motor-current rejection came back with something other
// than error 23. The binary search would silently return the wrong ceiling if a
// comms failure were counted as a rejection, so it is tracked and asserted.
static bool g_unexpectedRejectionCode = false;
static uint8_t g_firstUnexpectedCode = 0;

// ---------------------------------------------------------------------------
// Send one Set-maximum-motor-current WITHOUT resetting first, and report the
// resulting error code (0 = accepted).
//
// The firmware answers the offending command itself from inside fatal_error()'s
// spin loop, so a rejection usually shows up as the library's immediate error;
// but if it does not, the latched code is still in the status word. Both routes
// are checked, exactly as tm_settings does, because relying on only one of them
// produced an ambiguous result there.
// ---------------------------------------------------------------------------
static uint8_t trySetCurrent(Servomotor* m, uint16_t motorCur, uint16_t regen) {
    m->setMaximumMotorCurrentRaw(motorCur, regen);
    int imm = m->getError();
    if (imm != 0) return (uint8_t)imm;
    return tfhFatal(m);
}

// ---------------------------------------------------------------------------
// The same probe, but from a freshly reset machine.
//
// This matters: once a rejection has latched a fatal error, EVERY subsequent
// command answers with that same code (campaign finding F2), so a probe made
// after an earlier rejection would report error 23 no matter what it asked for
// and the search would converge on nonsense. Resetting first makes each probe
// an independent measurement.
// ---------------------------------------------------------------------------
static uint8_t probeCurrent(Servomotor* m, uint16_t motorCur, uint16_t regen) {
    tfhReset(m);
    uint8_t code = trySetCurrent(m, motorCur, regen);
    if (code != 0 && code != ERR_MAX_PWM_VOLTAGE_TOO_HIGH) {
        if (!g_unexpectedRejectionCode) g_firstUnexpectedCode = code;
        g_unexpectedRejectionCode = true;
    }
    return code;
}

// ---------------------------------------------------------------------------
// Binary search for the largest ACCEPTED value of one argument, holding the
// other at 0 (always legal) so the two checks cannot be confused for each other.
// Invariant: `lo` is known accepted, `hi` is known rejected.
// ---------------------------------------------------------------------------
static uint16_t findCeiling(Servomotor* m, bool regenArg, uint16_t lo, uint16_t hi, int* probes) {
    while ((uint16_t)(hi - lo) > 1) {
        uint16_t mid = (uint16_t)(lo + (uint16_t)((hi - lo) / 2));
        uint8_t code = regenArg ? probeCurrent(m, 0, mid) : probeCurrent(m, mid, 0);
        (*probes)++;
        if (code == 0) lo = mid; else hi = mid;
    }
    return lo;
}

// ---------------------------------------------------------------------------
// Run a trapezoid move of a fixed size and report the COMMANDED displacement.
//
//   mode 0: undisturbed reference (polls the queue only)
//   mode 1: hammer Set-maximum-motor-current in flight
//   mode 2: hammer Set-PID-constants in flight
//   mode 3: hammer both, alternating
//
// Every mode runs the identical loop and the identical move, so the ONLY
// difference between the reference and a disturbed run is the setting commands.
// The move is never waited out by a computed duration -- the loop exits on an
// empty queue and tfhDrain() confirms it -- because a fixed wait that is even
// slightly short reads a PARTIAL move and looks exactly like interference.
// ---------------------------------------------------------------------------
static int64_t moveUnderDisturbance(Servomotor* m, int mode, int* nSent, uint8_t* fatal) {
    const int32_t DIST = (int32_t)(TFH_CPR / 2);       // half a rotation
    const uint32_t TICKS = 2 * (uint32_t)TFH_TPS;      // over two seconds

    tfhFreshOpen(m);
    m->disableMosfets();
    delay(100);

    int64_t before = m->getPositionRaw();
    if (m->getError() != 0) { *fatal = 0xFB; *nSent = 0; return 0; }

    m->trapezoidMoveRaw(DIST, TICKS);
    if (m->getError() != 0) { *fatal = 0xFD; *nSent = 0; return 0; }

    int sent = 0;
    uint32_t deadline = millis() + 12000;
    while (millis() < deadline) {
        uint8_t q = m->getNQueuedItems();
        if (m->getError() != 0) break;
        if (q == 0) break;
        if (mode == 1 || (mode == 3 && (sent & 1) == 0)) {
            uint16_t c = SAFE_CURRENTS[sent % N_SAFE_CURRENTS];
            uint16_t r = SAFE_CURRENTS[(sent + 2) % N_SAFE_CURRENTS];
            m->setMaximumMotorCurrentRaw(c, r);
            if (m->getError() != 0) break;
            sent++;
        }
        else if (mode == 2 || mode == 3) {
            const uint32_t* g = PID_SET[sent % N_PID_SET];
            m->setPidConstants(g[0], g[1], g[2]);
            if (m->getError() != 0) break;
            sent++;
        }
        delay(20);
    }
    *nSent = sent;

    bool drained = tfhDrain(m, 8000);
    *fatal = tfhFatal(m);
    if (!drained && *fatal == 0) *fatal = 0xFC;
    if (*fatal != 0) return 0;
    return m->getPositionRaw() - before;
}

void tm_current_and_pid_settings(void) {
    Serial.println("tm_current_and_pid_settings: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // File-scope state must be re-initialised at entry: the host harness can run
    // modules back-to-back inside one process.
    g_unexpectedRejectionCode = false;
    g_firstUnexpectedCode = 0;

    tfhReset(m);
    m->disableMosfets();          // belt and braces; a reset already leaves them off
    delay(200);

    // =====================================================================
    // 1. WHICH PRODUCT IS THIS? The current ceiling and the factory PID gains
    //    are per-product compile-time constants, so the documented value to
    //    check against depends on the answer. Asking the device rather than
    //    hard-coding it means this module is correct on an M1, M2, M17 or M23
    //    and says so out loud when it meets something it does not know.
    // =====================================================================
    printf("\n--- 1. the device identifies the product whose limits apply ---\n");
    uint16_t documentedCeiling = 0;
    const char* productName = "unknown";
    {
        getProductInfoResponse info = m->getProductInfo();
        bool infoOk = (m->getError() == 0);
        char code[9];
        memcpy(code, info.productCode, 8);
        code[8] = '\0';
        // "M1" and "M2" are distinguished from "M17"/"M23" by whether the third
        // character is a digit -- the same test tm_actuator_edges uses.
        bool thirdIsDigit = (code[2] >= '0' && code[2] <= '9');
        if (infoOk && code[0] == 'M' && code[1] == '1' && !thirdIsDigit) {
            productName = "M1";  documentedCeiling = 300;   // motor_control.c:54
        }
        else if (infoOk && code[0] == 'M' && code[1] == '2' && !thirdIsDigit) {
            productName = "M2";  documentedCeiling = 300;   // motor_control.c:54
        }
        else if (infoOk && code[0] == 'M' && code[1] == '1' && code[2] == '7') {
            productName = "M17"; documentedCeiling = 390;   // motor_control.c:57
        }
        else if (infoOk && code[0] == 'M' && code[1] == '2' && code[2] == '3') {
            productName = "M23"; documentedCeiling = 1024;  // motor_control.c:60, PWM_PERIOD_TIM1
        }
        printf("   productCode=\"%s\" -> product %s, documented current ceiling %u\n",
               code, productName, (unsigned)documentedCeiling);
        TEST_RESULT("the device reports which product it is", infoOk);
        TEST_RESULT("the product is one whose documented current ceiling is known",
                    documentedCeiling != 0);
    }
    if (documentedCeiling == 0) documentedCeiling = 300;   // most conservative fallback
    tfhReset(m);

    // =====================================================================
    // 2. THE ENFORCED CEILING IS THE DOCUMENTED CEILING, ON BOTH ARGUMENTS.
    //    Measured, not assumed. A binary search over 0..2048 finds the largest
    //    value the firmware actually accepts for each argument in turn, with
    //    the other argument held at zero so the two range checks
    //    (motor_control.c:3513 and :3516) cannot be mistaken for one another.
    //
    //    Why this is worth the resets it costs: the ceiling is the difference
    //    between a working current limit and a machine that faults and disables
    //    its output stage the moment a configuration file is copied from one
    //    product to another. If the constant ever moves, the docs and this test
    //    disagree immediately instead of silently.
    // =====================================================================
    printf("\n--- 2. the enforced ceiling equals the documented ceiling ---\n");
    uint16_t measuredCeiling = documentedCeiling;
    {
        uint8_t zeroCode = probeCurrent(m, 0, 0);
        printf("   (0,0) -> code %u\n", (unsigned)zeroCode);
        TEST_RESULT("zero is an accepted maximum motor current", zeroCode == 0);

        uint8_t hiCode = probeCurrent(m, SEARCH_HI, 0);
        printf("   (%u,0) -> code %u\n", (unsigned)SEARCH_HI, (unsigned)hiCode);
        TEST_RESULT("a value above every product ceiling is refused with error 23",
                    hiCode == ERR_MAX_PWM_VOLTAGE_TOO_HIGH);

        int probes = 0;
        uint16_t curCeil = (zeroCode == 0 && hiCode == ERR_MAX_PWM_VOLTAGE_TOO_HIGH)
                               ? findCeiling(m, false, 0, SEARCH_HI, &probes) : 0;
        uint16_t regCeil = (zeroCode == 0 && hiCode == ERR_MAX_PWM_VOLTAGE_TOO_HIGH)
                               ? findCeiling(m, true, 0, SEARCH_HI, &probes) : 0;
        printf("   measured motor-current ceiling %u, regeneration ceiling %u"
               " (documented %u for %s), %d probes\n",
               (unsigned)curCeil, (unsigned)regCeil,
               (unsigned)documentedCeiling, productName, probes);
        if (curCeil != 0) measuredCeiling = curCeil;

        TEST_RESULT("the motor-current argument stops accepting at the documented ceiling",
                    curCeil == documentedCeiling);
        TEST_RESULT("the regeneration-current argument stops accepting at the same ceiling",
                    regCeil == documentedCeiling && regCeil == curCeil);
        printf("   unexpected rejection code seen: %d (first was %u)\n",
               (int)g_unexpectedRejectionCode, (unsigned)g_firstUnexpectedCode);
        TEST_RESULT("every refusal during the search was error 23 and not a comms failure",
                    !g_unexpectedRejectionCode);
    }
    tfhReset(m);

    // =====================================================================
    // 3. THE WHOLE DOCUMENTED RANGE IS ACCEPTED, INCLUDING BOTH ENDS. The
    //    ceiling value ITSELF must be legal: the firmware test is `>` and not
    //    `>=` (motor_control.c:3513), so a user who sets exactly the documented
    //    maximum -- the obvious thing to do -- must not be faulted. Zero must
    //    also be legal: it is the natural "no torque" setting and it drives
    //    `authority` to 0, which is the degenerate end of all the derived-limit
    //    arithmetic in recompute_pid_parameters_and_set_pwm_voltage().
    // =====================================================================
    printf("\n--- 3. every documented in-range value is accepted ---\n");
    {
        const uint16_t vals[] = {
            0, 1, 2, 10, 50, 100,
            (uint16_t)(measuredCeiling / 4), (uint16_t)(measuredCeiling / 2),
            (uint16_t)(measuredCeiling - 1), measuredCeiling
        };
        const int nVals = (int)(sizeof(vals) / sizeof(vals[0]));

        bool allCurOk = true, ceilingItselfOk = false;
        for (int i = 0; i < nVals; i++) {
            uint8_t code = trySetCurrent(m, vals[i], 100);
            if (code != 0) { allCurOk = false; printf("   motorCurrent=%u REFUSED code %u\n",
                                                     (unsigned)vals[i], (unsigned)code); tfhReset(m); }
            if (vals[i] == measuredCeiling && code == 0) ceilingItselfOk = true;
        }
        printf("   motor-current sweep over %d values: allAccepted=%d\n", nVals, (int)allCurOk);
        TEST_RESULT("every in-range motor current from zero to the ceiling is accepted", allCurOk);
        TEST_RESULT("the ceiling value itself is legal (the check is strictly greater-than)",
                    ceilingItselfOk);

        bool allRegOk = true;
        for (int i = 0; i < nVals; i++) {
            uint8_t code = trySetCurrent(m, 100, vals[i]);
            if (code != 0) { allRegOk = false; printf("   regen=%u REFUSED code %u\n",
                                                     (unsigned)vals[i], (unsigned)code); tfhReset(m); }
        }
        printf("   regeneration sweep over %d values: allAccepted=%d\n", nVals, (int)allRegOk);
        TEST_RESULT("every in-range regeneration current from zero to the ceiling is accepted",
                    allRegOk);
    }
    tfhReset(m);

    // =====================================================================
    // 4. BEYOND THE CEILING IS REFUSED, NOT CLAMPED -- ON EITHER ARGUMENT
    //    INDEPENDENTLY. The firmware's limits are pass/fail checks throughout,
    //    and this one is no exception: it does not quietly reduce an
    //    over-ambitious current to the maximum, it raises fatal error 23 and
    //    disables the output stage until a reset. The pairings below are what
    //    make the two arguments provably independent -- an illegal value on
    //    EITHER one is refused even when the other is perfectly legal, which is
    //    exactly the case the API docs warn about ("do not treat this parameter
    //    as ignorable").
    // =====================================================================
    printf("\n--- 4. one past the ceiling is refused on either argument ---\n");
    {
        uint16_t over = (uint16_t)(measuredCeiling + 1);

        uint8_t c1 = probeCurrent(m, over, 0);
        printf("   (%u, 0)      -> code %u\n", (unsigned)over, (unsigned)c1);
        TEST_RESULT("one past the ceiling as motor current raises error 23", c1 == ERR_MAX_PWM_VOLTAGE_TOO_HIGH);

        uint8_t c2 = probeCurrent(m, 0, over);
        printf("   (0, %u)      -> code %u\n", (unsigned)over, (unsigned)c2);
        TEST_RESULT("one past the ceiling as regeneration current raises error 23",
                    c2 == ERR_MAX_PWM_VOLTAGE_TOO_HIGH);

        uint8_t c3 = probeCurrent(m, over, 100);
        printf("   (%u, 100)    -> code %u\n", (unsigned)over, (unsigned)c3);
        TEST_RESULT("an illegal motor current is refused even when the regeneration current is legal",
                    c3 == ERR_MAX_PWM_VOLTAGE_TOO_HIGH);

        uint8_t c4 = probeCurrent(m, 100, over);
        printf("   (100, %u)    -> code %u\n", (unsigned)over, (unsigned)c4);
        TEST_RESULT("an illegal regeneration current is refused even when the motor current is legal",
                    c4 == ERR_MAX_PWM_VOLTAGE_TOO_HIGH);

        // Every one of those left a latched fault; a reset must clear it, and a
        // legal value must be accepted again straight afterwards.
        tfhReset(m);
        uint8_t after = trySetCurrent(m, 100, 100);
        printf("   after reset, (100,100) -> code %u, fatal %u\n",
               (unsigned)after, (unsigned)tfhFatal(m));
        TEST_RESULT("a reset clears the refusal and a legal value is accepted immediately after",
                    after == 0);
    }
    tfhReset(m);

    // =====================================================================
    // 5. THE TWO ARGUMENTS ARE NOT COUPLED. Nothing in the firmware relates
    //    them -- they are two independent range checks feeding two independent
    //    stored values (motor_control.c:2327-2328) -- so every legal
    //    combination must be accepted regardless of their relative sizes. A
    //    future "regen must not exceed the motor current" rule would be a
    //    silent API break for anyone using regenerative braking, and this
    //    section is what would catch it.
    // =====================================================================
    printf("\n--- 5. the regeneration argument is independent of the motor current ---\n");
    {
        uint16_t ceilV = measuredCeiling;
        // Reset after any refusal so the four combinations stay INDEPENDENT
        // measurements: a latched fault would make every later probe answer with
        // the first one's error code (campaign finding F2) and turn one failure
        // into four.
        uint8_t a = trySetCurrent(m, 10, ceilV);        // regen far ABOVE the motor current
        if (a != 0) tfhReset(m);
        uint8_t b = trySetCurrent(m, ceilV, 0);         // regen zero, motor current at the ceiling
        if (b != 0) tfhReset(m);
        uint8_t c = trySetCurrent(m, 0, ceilV);         // motor current zero, regen at the ceiling
        if (c != 0) tfhReset(m);
        uint8_t d = trySetCurrent(m, ceilV, ceilV);     // both at the ceiling
        if (d != 0) tfhReset(m);
        printf("   (10,%u)=%u  (%u,0)=%u  (0,%u)=%u  (%u,%u)=%u\n",
               (unsigned)ceilV, (unsigned)a, (unsigned)ceilV, (unsigned)b,
               (unsigned)ceilV, (unsigned)c, (unsigned)ceilV, (unsigned)ceilV, (unsigned)d);
        TEST_RESULT("the regeneration current may be set far above the motor current", a == 0);
        TEST_RESULT("the motor current may sit at the ceiling with the regeneration current at zero",
                    b == 0);
        TEST_RESULT("the motor current may sit at zero with the regeneration current at the ceiling",
                    c == 0);
        TEST_RESULT("both arguments may sit at the ceiling together", d == 0);
    }
    tfhReset(m);

    // =====================================================================
    // 6. THE PID CONSTANTS ACCEPT THE WHOLE u32 DOMAIN, INCLUDING ZEROS.
    //    Zero is the case that matters. Three separate divisions inside
    //    recompute_pid_parameters_and_set_pwm_voltage() use a user-supplied
    //    gain as the divisor -- `new_max_error /= p64` (motor_control.c:2278),
    //    `authority / d64` (:2297) and the ki cap's `/ effective_max_error`
    //    (:2314) -- and each is guarded by its own explicit non-zero test. Zero
    //    gains are a legitimate configuration (P-only, PD-only, feedforward-only
    //    tuning), so those guards are on the normal path, not an exotic one.
    //
    //    WHAT THIS CAN AND CANNOT SEE. It can see that the whole domain is
    //    accepted and leaves the machine usable. It CANNOT see whether the
    //    non-zero guards are still there: these are int64 divisions, resolved by
    //    __aeabi_ldivmod on both MCUs, and dividing by zero there returns 0
    //    instead of faulting (the firmware makes the same point about the
    //    planner's divide at motor_control.c:457). A dropped guard would be
    //    silent from the bus. The assertion names below say "accepted", not
    //    "no divide-by-zero", so that this test is never mistaken for a guard
    //    that it is not.
    //
    //    The far end is the same story: the command takes u32 but the working
    //    constants are int32, so values above INT32_MAX are capped rather than
    //    allowed to wrap negative (motor_control.c:2267-2269) -- a wrap there
    //    would turn the feedback POSITIVE, which is a runaway, not a bad tune.
    //    The capping is likewise not observable from outside with the output
    //    stage off, so only ACCEPTANCE is asserted. With the MOSFETs off none of
    //    these gains can do anything, which is precisely why the whole domain
    //    can be swept safely here.
    // =====================================================================
    printf("\n--- 6. the PID constants accept zeros and extremes ---\n");
    {
        const uint32_t U32MAX = 4294967295u;
        const uint32_t I32MAX = 2147483647u;
        const uint32_t combos[][3] = {
            {0, 0, 0},                      // all three zero: every division guard at once
            {2000, 5, 175000},              // the compiled-in M17 defaults
            {500, 1, 1000},                 // the compiled-in M23 defaults
            {5000, 1, 5000},                // the compiled-in M1 defaults
            {20000, 1, 100000},             // the compiled-in M2 defaults
            {1, 0, 0},                      // P only
            {0, 1, 0},                      // I only  (kP == 0 makes max_error degenerate)
            {0, 0, 1},                      // D only  (below the 5-bit internal shift: no D action)
            {0, 0, 31},                     // still below the shift; documented as a no-op
            {I32MAX, 0, 0},
            {0, I32MAX, 0},
            {0, 0, I32MAX},
            {I32MAX, I32MAX, I32MAX},
            {U32MAX, U32MAX, U32MAX},       // must be capped, never wrapped negative
        };
        const int nCombos = (int)(sizeof(combos) / sizeof(combos[0]));

        bool allOk = true, zerosOk = false, u32Ok = false, i32Ok = false, isolatedOk = true;
        for (int i = 0; i < nCombos; i++) {
            m->setPidConstants(combos[i][0], combos[i][1], combos[i][2]);
            int imm = m->getError();
            uint8_t fat = (imm == 0) ? tfhFatal(m) : (uint8_t)imm;
            if (fat != 0) {
                allOk = false;
                printf("   kP=%u kI=%u kD=%u REFUSED (err %d, fatal %u)\n",
                       (unsigned)combos[i][0], (unsigned)combos[i][1],
                       (unsigned)combos[i][2], imm, (unsigned)fat);
                tfhReset(m);
            }
            if (i == 0)  zerosOk = (fat == 0);
            if (i >= 5 && i <= 8 && fat != 0) isolatedOk = false;
            if (i == 12) i32Ok = (fat == 0);
            if (i == 13) u32Ok = (fat == 0);
        }
        printf("   %d PID combinations tried: allAccepted=%d\n", nCombos, (int)allOk);
        TEST_RESULT("all three PID constants can be set to zero and the command is accepted", zerosOk);
        TEST_RESULT("a single nonzero gain with the other two at zero is accepted", isolatedOk);
        TEST_RESULT("INT32_MAX on all three PID constants is accepted", i32Ok);
        TEST_RESULT("UINT32_MAX on all three PID constants is accepted (the capping itself is "
                    "not observable with the output stage off)", u32Ok);
        TEST_RESULT("no PID constant combination in the sweep raises a fatal error", allOk);

        // The setter must still work after the extremes: a stuck or corrupted
        // constant would show up as the next call failing.
        m->setPidConstants(2000, 5, 175000);
        TEST_RESULT("the defaults can be restored after sweeping the extremes",
                    m->getError() == 0 && tfhFatal(m) == 0);
    }
    tfhReset(m);

    // =====================================================================
    // 7. CHANGING THEM MID-MOVE DOES NOT DISTURB THE MOVE. This is the whole
    //    point of the module. Both settings are applied with interrupts
    //    disabled while the motor-control ISR is running the trajectory, and
    //    the application also re-clamps the live integrator (motor_control.c:2338)
    //    and zeroes the derivative filter (:2337). If any of that leaked into
    //    the planner, the move would land somewhere else.
    //
    //    The test is DIFFERENTIAL: the same half-rotation move is run four
    //    times through the identical polling loop, once with no setting
    //    commands and three times with them hammered in flight, and the
    //    commanded displacements are compared. Comparing against an undisturbed
    //    reference measured on the same motor in the same session is far
    //    stronger than comparing against a computed prediction, because it
    //    cancels every constant offset the planner has.
    //
    //    Tolerance is 2 counts out of 1,638,400 -- the same tolerance
    //    tm_determinism established for repeated identical moves.
    // =====================================================================
    printf("\n--- 7. settings changed in flight do not move the endpoint ---\n");
    {
        const int64_t TOL = 2;
        int sent0 = 0, sent1 = 0, sent2 = 0, sent3 = 0;
        uint8_t f0 = 0, f1 = 0, f2 = 0, f3 = 0;

        int64_t ref  = moveUnderDisturbance(m, 0, &sent0, &f0);
        int64_t dCur = moveUnderDisturbance(m, 1, &sent1, &f1);
        int64_t dPid = moveUnderDisturbance(m, 2, &sent2, &f2);
        int64_t dBoth = moveUnderDisturbance(m, 3, &sent3, &f3);

        printf("   reference        : moved %lld (fatal %u)\n", (long long)ref, (unsigned)f0);
        printf("   current hammered : moved %lld after %d commands (fatal %u), delta %lld\n",
               (long long)dCur, sent1, (unsigned)f1, (long long)(dCur - ref));
        printf("   PID hammered     : moved %lld after %d commands (fatal %u), delta %lld\n",
               (long long)dPid, sent2, (unsigned)f2, (long long)(dPid - ref));
        printf("   both hammered    : moved %lld after %d commands (fatal %u), delta %lld\n",
               (long long)dBoth, sent3, (unsigned)f3, (long long)(dBoth - ref));

        TEST_RESULT("the undisturbed reference move completes and delivers its half rotation",
                    f0 == 0 && llabs((long long)(ref - TFH_CPR / 2)) <= 8);

        TEST_RESULT("changing the maximum motor current in flight does not fault the machine",
                    f1 == 0 && sent1 >= 10);
        TEST_RESULT("a move whose current limit was rewritten in flight lands on the reference endpoint",
                    f1 == 0 && llabs((long long)(dCur - ref)) <= TOL);

        TEST_RESULT("changing the PID constants in flight does not fault the machine",
                    f2 == 0 && sent2 >= 10);
        TEST_RESULT("a move whose PID gains were rewritten in flight lands on the reference endpoint",
                    f2 == 0 && llabs((long long)(dPid - ref)) <= TOL);

        TEST_RESULT("changing both settings alternately in flight does not fault the machine",
                    f3 == 0 && sent3 >= 10);
        TEST_RESULT("a move with both settings rewritten in flight lands on the reference endpoint",
                    f3 == 0 && llabs((long long)(dBoth - ref)) <= TOL);
    }
    tfhReset(m);

    // =====================================================================
    // 8. NEITHER SETTING COSTS A QUEUE SLOT. Both handlers reply immediately
    //    and never call add_to_queue() (main.c:630-641, main.c:976-991), so the
    //    depth must be untouched. This is the property that makes it safe to
    //    retune from a supervisory loop while a motion program is running: if
    //    either setting consumed a slot, a plan sized to the 32-item queue
    //    would start failing with ERROR_QUEUE_IS_FULL depending on how often
    //    the operator adjusted a gain.
    //
    //    The mid-move reading uses tm_queue_accounting's technique -- a
    //    deliberately SLOW acceleration limit so each queued segment lasts about
    //    a second -- otherwise the depth read races the ISR draining the queue
    //    and a perfectly innocent decrement looks like a measurement.
    // =====================================================================
    printf("\n--- 8. setting them queues nothing ---\n");
    {
        tfhFreshOpen(m);
        uint8_t d0 = m->getNQueuedItems();
        for (int i = 0; i < 16; i++) m->setMaximumMotorCurrentRaw(SAFE_CURRENTS[i % N_SAFE_CURRENTS], 100);
        uint8_t d1 = m->getNQueuedItems();
        for (int i = 0; i < 16; i++) m->setPidConstants(PID_SET[i % N_PID_SET][0],
                                                        PID_SET[i % N_PID_SET][1],
                                                        PID_SET[i % N_PID_SET][2]);
        uint8_t d2 = m->getNQueuedItems();
        printf("   idle depths: before %u, after 16 current sets %u, after 16 PID sets %u\n",
               (unsigned)d0, (unsigned)d1, (unsigned)d2);
        TEST_RESULT("sixteen of each setting command at idle leave the queue empty",
                    d0 == 0 && d1 == 0 && d2 == 0);

        // Mid-move: one ordinary trapezoid costs 3 slots and, with these slow
        // limits, its first ramp alone lasts about a second.
        tfhReset(m);
        m->setMaximumVelocity(1.0f);
        m->setMaximumAcceleration(1.0f);
        m->trapezoidMoveRaw((int32_t)(TFH_CPR / 4), 4 * (uint32_t)TFH_TPS);
        bool queued = (m->getError() == 0);
        uint8_t qBefore = m->getNQueuedItems();
        for (int i = 0; i < 16; i++) {
            m->setMaximumMotorCurrentRaw(SAFE_CURRENTS[i % N_SAFE_CURRENTS], 100);
            m->setPidConstants(PID_SET[i % N_PID_SET][0], PID_SET[i % N_PID_SET][1],
                               PID_SET[i % N_PID_SET][2]);
        }
        uint8_t qAfter = m->getNQueuedItems();
        printf("   mid-move depths: before %u, after 32 setting commands %u (queued=%d)\n",
               (unsigned)qBefore, (unsigned)qAfter, (int)queued);
        // The claim is "these settings do not CONSUME OR ADD a slot", and the
        // only race-free way to state it mid-move is that the depth never grew.
        // Insisting on qAfter == 3 would additionally require that the ISR
        // retired nothing during the burst -- true with these limits (the first
        // ramp is a full second and the burst is tens of milliseconds) but it is
        // a timing assumption, not the property under test, and it would fail on
        // a slower transport for a reason that has nothing to do with cmd 28 or
        // cmd 43. qBefore == 3 is safe to assert exactly: it is read immediately
        // after queueing, and tm_queue_accounting establishes 3 slots under
        // these identical limits.
        TEST_RESULT("thirty-two setting commands mid-move never increase the queue depth",
                    queued && qBefore == 3 && qAfter <= qBefore);

        bool finished = tfhDrain(m, 20000) && (tfhFatal(m) == 0);
        TEST_RESULT("the move that was in the queue during the burst still completes cleanly",
                    finished);
    }
    tfhReset(m);

    // =====================================================================
    // 9. A RESET RESTORES THEM, AND A REFUSAL DOES NOT MOVE THE GOALPOSTS.
    //    Neither setting is written to non-volatile memory (main.c calls only
    //    the RAM setters; the boot path re-applies DEFAULT_MAX_MOTOR_PWM_VOLTAGE
    //    from motor_control.h:121-140 and the per-product PID defaults via
    //    main.c:1438), so a reset returns the machine to the factory tuning.
    //
    //    HONEST LIMITATION, stated because it changes what may be claimed:
    //    neither setting has a getter, and with the output stage off neither has
    //    any observable physical effect, so "restored" cannot be read back
    //    directly here. What IS observable, and what is asserted, is that the
    //    reset restores the factory ACCEPTANCE ENVELOPE and leaves no residue:
    //    the ceiling is exactly where it was before a refusal (a rejected
    //    attempt neither widens nor narrows it), the full documented range is
    //    accepted again, and a machine whose current and gains were driven to
    //    zero comes back able to run an ordinary move. tm_reset_completeness
    //    owns the general "a reset restores everything" proof; this is the
    //    current/PID-specific part of it.
    // =====================================================================
    printf("\n--- 9. a reset restores the factory envelope, a refusal does not shift it ---\n");
    {
        uint16_t over = (uint16_t)(measuredCeiling + 1);

        uint8_t first = probeCurrent(m, over, 0);
        uint8_t second = probeCurrent(m, over, 0);
        printf("   %u refused twice across a reset: codes %u, %u\n",
               (unsigned)over, (unsigned)first, (unsigned)second);
        TEST_RESULT("a value one past the ceiling is refused just the same after a reset",
                    first == ERR_MAX_PWM_VOLTAGE_TOO_HIGH &&
                    second == ERR_MAX_PWM_VOLTAGE_TOO_HIGH);

        tfhReset(m);
        uint8_t atCeiling = trySetCurrent(m, measuredCeiling, measuredCeiling);
        printf("   after a reset, the ceiling pair (%u,%u) -> code %u\n",
               (unsigned)measuredCeiling, (unsigned)measuredCeiling, (unsigned)atCeiling);
        TEST_RESULT("the ceiling is still accepted after a refusal and a reset, so the range did not narrow",
                    atCeiling == 0);

        // Drive both settings to their least useful values, reset, and check the
        // machine is ordinary again. Repeated, because a residue that only shows
        // up on the second cycle is exactly the kind of bug a single pass misses.
        bool cyclesOk = true;
        for (int cycle = 0; cycle < 3 && cyclesOk; cycle++) {
            tfhFreshOpen(m);
            m->setMaximumMotorCurrentRaw(0, 0);
            m->setPidConstants(0, 0, 0);
            if (m->getError() != 0 || tfhFatal(m) != 0) {
                cyclesOk = false;
                printf("   cycle %d: crippling the settings itself failed\n", cycle);
                break;
            }
            tfhFreshOpen(m);                       // the reset under test
            if (tfhFatal(m) != 0) {
                cyclesOk = false;
                printf("   cycle %d: a fatal error survived the reset\n", cycle);
                break;
            }
            bool ok = false;
            int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 8), (uint32_t)TFH_TPS, &ok);
            if (!ok || llabs((long long)(moved - TFH_CPR / 8)) > 16) {
                cyclesOk = false;
                printf("   cycle %d: post-reset move delivered %lld of %lld (ok=%d)\n",
                       cycle, (long long)moved, (long long)(TFH_CPR / 8), (int)ok);
                break;
            }
        }
        TEST_RESULT("three cripple-and-reset cycles each leave a fully working machine", cyclesOk);

        // And the whole documented range is accepted once more.
        tfhReset(m);
        bool rangeBack = true;
        const uint16_t recheck[] = { 0, 1, 100, (uint16_t)(measuredCeiling / 2), measuredCeiling };
        for (int i = 0; i < 5; i++) {
            if (trySetCurrent(m, recheck[i], recheck[i]) != 0) { rangeBack = false; tfhReset(m); }
        }
        printf("   post-reset re-sweep of the documented range: allAccepted=%d\n", (int)rangeBack);
        TEST_RESULT("the full documented current range is accepted again after a reset", rangeBack);
    }

    tfhReset(m);
    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
}
