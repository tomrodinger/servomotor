#include "tf_motion_helpers.h"

// Set by the safety-fence ladder so section 6' can assert on the rung that
// finding F3 covers. 0xFE means "the reply was lost", which F3 documents as
// happening about a third of the time and which must NOT fail the test.
static uint8_t g_safExcludesCode = 0xFE;


// ---------------------------------------------------------------------------
// tm_boundary_value_sweep.cpp
//
// THE SAME NUMERIC LADDER, CLIMBED ON EVERY SETTER THE LIBRARY EXPOSES.
//
// A caller configuring this machine has six numeric settings to get right:
//
//     cmd  3  Set maximum velocity                     (u32)
//     cmd  5  Set maximum acceleration                 (u32)
//     cmd 28  Set maximum motor current                (u16, u16)
//     cmd 43  Set PID constants                        (u32, u32, u32)
//     cmd 44  Set max allowable position deviation     (i64)
//     cmd 30  Set safety limits                        (i64, i64)
//
// (command numbers from python_programs/servomotor/motor_commands.json)
//
// Every one of them is written by a different hand in the firmware, and every
// one of them decides for itself what to do with a number it does not like.
// Some refuse. Some silently saturate. Some accept everything and let a
// watchdog complain later. Nothing in the suite -- or in the API documentation
// -- states which is which, so the only way a user learns is by faulting a
// machine.
//
// THE PROPERTY ASSERTED HERE, in one sentence: for every numeric setter, each
// rung of one standard ladder -- ZERO, ONE, THE DOCUMENTED MAXIMUM, ONE PAST
// IT, and a VERY LARGE VALUE -- is either accepted or refused with a documented
// error code, the verdict is IDENTICAL across three independent trials, and the
// machine is fully usable again after every refusal.
//
// This is deliberately BREADTH, not depth. It says nothing new about any single
// setter that a specialist module does not say better; what it adds is
// UNIFORMITY -- the same five questions asked of all six, so that a setter whose
// boundary handling is out of step with its neighbours shows up as a difference
// rather than as a gap.
//
// HOW IT IS DISTINCT FROM WHAT ALREADY EXISTS
//   * tm_limit_boundaries asks whether a MOVE at exactly the configured limit is
//     accepted (the `>` vs `>=` question, in converted rot/s units). It never
//     probes the setter's own input domain: it never sends zero, never sends a
//     value past the ceiling, and never reads the stored limit back.
//   * tm_float_edges probes the float->int32 conversion pipeline of ONE command
//     (trapezoid move) with NaN, infinity, negative zero and denormals. It is
//     about the library's arithmetic; this module is about the firmware's
//     acceptance rules, and every value sent here is an exact integer.
//   * tm_current_and_pid_settings owns cmd 28 and cmd 43 in depth -- it binary
//     searches for the current ceiling and proves those two settings can be
//     rewritten mid-move without disturbing it. Here they contribute five rungs
//     each to a common table and nothing more.
//   * tm_safety_fence and tm_safety_fence_arithmetic own cmd 30's comparison
//     semantics; tm_deviation_tracking owns cmd 44's trip points.
//   * tm_settings_persistence reads maxVelocity / maxAcceleration back through
//     Get debug values (cmd 45) to show they revert on reset. It uses HALF the
//     boot default, which is deep inside the accepted range, so it never meets
//     the zero check and never meets the saturation described below.
//
// THE ONE GENUINELY NEW RESULT. Reading the stored limit back through Get debug
// values (cmd 45, fields maxVelocity / maxAcceleration, filled straight from the
// live variables at motor_control.c:3679-3680) shows that cmd 3 and cmd 5 are
// the ONLY setters in the machine that SATURATE:
//
//     set_max_velocity      (motor_control.c:3341)
//         if(new_max_velocity == 0) fatal_error(ERROR_PARAMETER_OUT_OF_RANGE);
//         max_velocity = new_max_velocity;  max_velocity <<= VELOCITY_SHIFT_LEFT;
//         if(max_velocity > EXPERIMENTAL_MAX_VELOCITY) max_velocity = EXPERIMENTAL_MAX_VELOCITY;
//     set_max_acceleration  (motor_control.c:3360)  -- the same shape, shift 8
//
// so UINT32_MAX is ACCEPTED and quietly becomes the experimental ceiling
// (motor_control.h:61-62: EXPERIMENTAL_MAX_VELOCITY = MAX_VELOCITY * 2,
// EXPERIMENTAL_MAX_ACCELERATION = MAX_ACCELERATION * 4). That is the exact
// opposite of the rule the rest of the firmware follows -- "limits are pass/fail
// checks, never clamps" -- and the inversion is defensible (the object being set
// IS the limit) but it is undocumented and unasserted. A user who sets
// 0xFFFFFFFF expecting a rejection gets an 18.7 rot/s machine instead.
//
// GROUND TRUTH FOR EVERY VERDICT BELOW (all read, not assumed):
//   cmd  3 handler main.c:265  -> set_max_velocity, motor_control.c:3341
//          zero -> ERROR_PARAMETER_OUT_OF_RANGE (34); anything else accepted,
//          stored as (raw << 12) saturated at EXPERIMENTAL_MAX_VELOCITY.
//   cmd  5 handler main.c:286  -> set_max_acceleration, motor_control.c:3360
//          zero -> 34; else stored as (raw << 8) saturated at
//          EXPERIMENTAL_MAX_ACCELERATION. Shifts: motor_control.h:64-65.
//   cmd 28 handler main.c:630  -> set_max_motor_current, motor_control.c:3511
//          either argument > MAX_PWM_VOLTAGE_AT_ZERO_VELOCITY (motor_control.c:54
//          M1/M2 = 300, :57 M17 = 390, :60 M23 = PWM_PERIOD_TIM1 = 1024)
//          -> ERROR_MAX_PWM_VOLTAGE_TOO_HIGH (23). The test is `>`, so the
//          ceiling value itself is legal. REFUSES; never clamps.
//   cmd 43 handler main.c:976  -> set_pid_constants, motor_control.c:2348
//          NO validation at all. Values above INT32_MAX are capped inside
//          recompute_pid_parameters_and_set_pwm_voltage (motor_control.c:2267-2269)
//          rather than refused, and the divisions by kP and kD are guarded
//          (:2277, :2296). ACCEPTS EVERYTHING.
//   cmd 44 handler main.c:992  -> set_max_allowable_position_deviation,
//          motor_control.c:3526. The handler maps INT64_MIN to INT64_MAX
//          (main.c:997-999, the BUG-24 guard, because llabs(INT64_MIN) stays
//          negative on ARM) and otherwise stores llabs(value). ACCEPTS
//          EVERYTHING; a too-small window is punished later, asynchronously, by
//          the main-loop watchdog at motor_control.c:3209/3217 (called from
//          main.c:1503) with ERROR_POSITION_DEVIATION_TOO_LARGE (45).
//   cmd 30 handler main.c:698. lower > upper -> ERROR_PARAMETER_OUT_OF_RANGE
//          (34) at main.c:709-711; otherwise stored verbatim
//          (motor_control.c:3494) and enforced EVERY CONTROL TICK at
//          motor_control.c:2640 with ERROR_SAFETY_LIMIT_EXCEEDED (25).
//
// All error numbers above are from python_programs/servomotor/error_codes.json.
//
// WHY THREE TRIALS. A boundary that is decided by a race, by leftover state, or
// by an uninitialised variable will usually still give the "right" answer once.
// Every ladder is therefore climbed three times from a freshly reset machine and
// the verdicts are required to match rung for rung. The verdict compared is the
// code the CALLER would actually see: the reply to the setting command if it
// answered with one, otherwise whatever ended up latched a moment later.
//
// A LOST FRAME IS NOT A VERDICT. A timeout, a truncated frame or a CRC32
// mismatch is a property of the link -- most often a second process holding the
// port, which on an unarbitrated RS485 bus corrupts both parties' traffic --
// and says nothing about the firmware's acceptance rules. Get status is
// therefore retried once (it is a pure read), and a rung that still comes back
// empty is recorded as VERDICT_LINK_LOST, counted separately, and excluded from
// every firmware claim in section 7 rather than being reported as an
// undocumented error code or as non-determinism. The count is printed loudly and
// the determinism claim additionally requires that at least 32 of the 36 rungs
// were actually measured, so it cannot pass by being vacuous.
//
// TWO RUNGS ARE DELIBERATELY COARSENED, and it is not a weakened assertion. A
// deviation window of 0 or 1 count is decided by hall-sensor jitter of a few
// counts against a commanded position that is not moving, so whether the
// watchdog happens to fire is genuinely not deterministic. Those two rungs are
// recorded as one class covering "quiet or error 45"; what is still asserted
// exactly is that the SETTER accepts them and that no OTHER code can appear.
// Recording a coin flip as a stable verdict would be the weaker choice.
//
// SAFETY. The MOSFETs stay OFF for the entire module: enableMosfets() and
// goToClosedLoop() are never called. Every position read is getPositionRaw(),
// the COMMANDED position, which the planner advances whether or not the output
// stage is energised -- so nothing turns, on any motor, at any point, and no
// current flows however the current limit is set. The largest commanded travel
// is three rotations, and it happens only in the one subsection that has first
// raised the deviation window to INT64_MAX, so error 45 cannot fire by accident;
// everywhere else the travel is a quarter rotation or less, far inside the
// default two-rotation window (DEFAULT_MAX_ALLOWABLE_POSITION_DEVIATION,
// motor_control.c:97). Errors 23, 25, 34 and 45 are raised on purpose and each
// one is cleared by a reset immediately, which is proven rather than assumed.
// The motor is addressed by unique ID (tfGetMotor uses extended addressing): no
// broadcasts, no alias writes, no raw frame injection, no test modes -- safe on
// the shared 35-motor rack.
// ---------------------------------------------------------------------------

// Documented firmware error codes (python_programs/servomotor/error_codes.json).
static const uint8_t ERR_MAX_PWM_VOLTAGE_TOO_HIGH     = 23;
static const uint8_t ERR_SAFETY_LIMIT_EXCEEDED        = 25;
static const uint8_t ERR_PARAMETER_OUT_OF_RANGE       = 34;
static const uint8_t ERR_POSITION_DEVIATION_TOO_LARGE = 45;

// Fixed-point shifts the firmware applies to the two ceilings it stores
// (motor_control.h:64-65).
static const int VELOCITY_SHIFT_LEFT = 12;
static const int ACCEL_SHIFT_LEFT    = 8;

// Type extremes, written out rather than taken from a macro so this compiles
// identically in the device and host builds.
static const int64_t  I64_MAX  = 9223372036854775807LL;
static const int64_t  I64_MIN  = (-9223372036854775807LL - 1LL);
static const uint32_t U32_MAX  = 4294967295u;
static const uint32_t I32_MAX_AS_U32 = 2147483647u;
static const uint16_t U16_MAX  = 65535u;

// Ladder layout. The rungs always run in this order and these counts, so a
// fixed index identifies a rung across all three trials and in the assertions.
static const int VEL_BASE = 0,  VEL_RUNGS = 6;
static const int ACC_BASE = 6,  ACC_RUNGS = 6;
static const int CUR_BASE = 12, CUR_RUNGS = 7;
static const int PID_BASE = 19, PID_RUNGS = 5;
static const int DEV_BASE = 24, DEV_RUNGS = 6;
static const int SAF_BASE = 30, SAF_RUNGS = 6;
static const int TOTAL_RUNGS = 36;

// A verdict value reserved for the two deliberately-coarsened deviation rungs
// (see the header): "the setter accepted it, and the watchdog either fired with
// code 45 or did not".
static const int32_t VERDICT_WATCHDOG_DOMAIN = -45;

// A verdict value reserved for a rung whose answer never arrived intact: a
// timeout, a truncated frame, a CRC32 mismatch. 254 is NOT a firmware error
// code -- error_codes.json stops well below it and no handler can produce it --
// so it can never be confused with one. A rung that lands here is a LINK event
// and is deliberately excluded from every firmware claim below; see recordRung.
static const uint8_t VERDICT_LINK_LOST = 0xFE;

// ---- module state, re-initialised at entry (the host harness can run modules
// ---- back to back inside one process) -------------------------------------
static int32_t g_verdict[3][TOTAL_RUNGS];
static int64_t g_readback[TOTAL_RUNGS];   // stored limit on trial 0, -1 if none
static int     g_trial;
static int     g_cursor;
static int     g_refusals;
static int     g_transportFailures;       // rungs lost to the link, not verdicts
static bool    g_badCodeSeen;
static uint8_t g_firstBadCode;
static bool    g_codeMismatch;            // reply code != latched code
static bool    g_recoveryFailed;

// The four codes any of these setters is documented to be able to produce.
static bool isDocumentedCode(uint8_t c) {
    return c == ERR_MAX_PWM_VOLTAGE_TOO_HIGH ||
           c == ERR_SAFETY_LIMIT_EXCEEDED    ||
           c == ERR_PARAMETER_OUT_OF_RANGE   ||
           c == ERR_POSITION_DEVIATION_TOO_LARGE;
}

// The latched fatal error code, read with ONE retry.
//
// Get status is a pure read with no side effects, so re-asking after a dropped
// frame is free and cannot change what is being measured. It matters because a
// single lost frame on a shared RS485 bus would otherwise be recorded as a
// firmware verdict -- and misattributing a bus event to the firmware is exactly
// the false bug report this campaign has already paid for once
// (TEST_EXPANSION_CAMPAIGN_2026-08.md, "Subagents WILL use the hardware").
// Returns 0xFF only when BOTH attempts failed.
static uint8_t latchedCode(Servomotor* m) {
    uint8_t c = tfhFatal(m);
    if (c == 0xFF) {
        delay(50);
        c = tfhFatal(m);
    }
    return c;
}

// Prove the device is alive at the application level, not merely that some
// bytes came back: the payload has to round-trip unchanged.
static bool pingRoundTrips(Servomotor* m) {
    uint8_t payload[10];
    for (int i = 0; i < 10; i++) payload[i] = (uint8_t)(i * 7 + 11);
    pingResponse r = m->ping(payload);
    if (m->getError() != 0) return false;
    return memcmp(r.responsePayload, payload, 10) == 0;
}

// ---------------------------------------------------------------------------
// Close out one ladder rung.
//
// `setErr` is what the setting command itself answered (0 = accepted; a
// positive value is a firmware error code relayed by Communication::getResponse;
// a negative value is a transport failure and is NOT a verdict). `settleMs`
// gives an ASYNCHRONOUS fault -- the per-tick safety check and the main-loop
// deviation watchdog -- time to latch before the status is read.
//
// If anything faulted, the machine is reset here and the recovery is checked
// immediately. That is what makes each rung an INDEPENDENT measurement: once a
// fatal error is latched, every command except Get status answers with that same
// code (campaign finding F2), so an un-cleared fault would make every later rung
// report the first one's verdict.
//
// A rung whose answer never arrived intact is NOT a verdict. A timeout, a
// truncated frame or a CRC32 mismatch says something about the LINK -- most
// often that a second process has the port, which on an unarbitrated RS485 bus
// corrupts both parties' frames -- and nothing whatsoever about the firmware's
// acceptance rules. Such a rung is recorded as VERDICT_LINK_LOST, counted in
// g_transportFailures, and excluded from every firmware claim in section 7:
// from the documented-code claim, from the reply-vs-latched claim, from the
// refusal-recovery claim and from the three-trial determinism comparison.
// Letting one dropped frame be reported as "the firmware answered with an
// undocumented error code" is a false bug report, and this campaign has already
// paid a full re-run for exactly that confusion.
// ---------------------------------------------------------------------------
static uint8_t recordRung(Servomotor* m, int setErr, uint32_t settleMs, bool watchdogTolerant) {
    if (settleMs) delay(settleMs);
    uint8_t latched = latchedCode(m);         // 0xFF only if BOTH status reads failed

    // setErr < 0 is a Communication:: transport code, never a firmware code.
    const bool transportLost = (setErr < 0) || (latched == 0xFF);

    uint8_t effective;
    if (transportLost)   effective = VERDICT_LINK_LOST;
    else if (setErr > 0) effective = (uint8_t)setErr;
    else                 effective = latched;

    // A synchronous refusal is answered twice -- in the reply to the offending
    // command and in the latched status -- and the two must agree. Only checked
    // when both numbers actually arrived; a missing status read is not a
    // disagreement.
    if (!transportLost && setErr > 0 && latched != (uint8_t)setErr) g_codeMismatch = true;

    if (!transportLost && effective != 0 && !isDocumentedCode(effective)) {
        if (!g_badCodeSeen) g_firstBadCode = effective;
        g_badCodeSeen = true;
    }

    int32_t v = (int32_t)effective;
    if (!transportLost && watchdogTolerant &&
        (effective == 0 || effective == ERR_POSITION_DEVIATION_TOO_LARGE)) {
        v = VERDICT_WATCHDOG_DOMAIN;
    }
    if (g_cursor >= 0 && g_cursor < TOTAL_RUNGS) g_verdict[g_trial][g_cursor] = v;
    g_cursor++;

    if (transportLost) {
        // Resynchronise so the NEXT rung is still an independent measurement,
        // but claim nothing about this one.
        g_transportFailures++;
        tfhReset(m);
        return effective;
    }
    if (effective != 0) {
        g_refusals++;
        tfhReset(m);
        if (latchedCode(m) != 0 || !pingRoundTrips(m)) g_recoveryFailed = true;
    }
    return effective;
}

// Read the stored speed / acceleration ceilings straight out of the firmware's
// live variables (Get debug values, cmd 45; main.c:1004 ->
// get_motor_control_debug_values, motor_control.c:3677). Returns -1 on a comms
// failure so a bad read can never masquerade as a plausible limit.
//
// Retried once for the same reason Get status is: this is a pure read, so
// re-asking cannot change what is being measured, and the stored value is what
// the headline saturation assertions are made of -- losing it to one dropped
// frame would report a link problem as a firmware problem.
static int64_t storedMaxVelocity(Servomotor* m) {
    getDebugValuesResponse d = m->getDebugValues();
    if (m->getError() != 0) { delay(50); d = m->getDebugValues(); }
    return (m->getError() == 0) ? d.maxVelocity : (int64_t)-1;
}
static int64_t storedMaxAcceleration(Servomotor* m) {
    getDebugValuesResponse d = m->getDebugValues();
    if (m->getError() != 0) { delay(50); d = m->getDebugValues(); }
    return (m->getError() == 0) ? d.maxAcceleration : (int64_t)-1;
}

// ---------------------------------------------------------------------------
// The six ladders. Each starts from a freshly reset machine, climbs its rungs in
// a fixed order, and leaves the machine reset.
//
// Within a ladder the rungs that are ACCEPTED run back to back without a reset:
// each one simply overwrites the previous setting, and none of them can produce
// motion with the output stage off. Only a rung that FAULTS costs a reset, which
// recordRung performs.
// ---------------------------------------------------------------------------

static void ladderMaxVelocity(Servomotor* m, int64_t bootV) {
    // The rated maximum expressed in the units the command actually takes: the
    // firmware shifts the sent value left by 12, so the largest raw value that
    // still lands at or below the rated ceiling is bootV >> 12.
    const uint32_t ratedRaw = (uint32_t)(bootV >> VELOCITY_SHIFT_LEFT);
    const uint32_t ceilRaw  = (uint32_t)((bootV * 2) >> VELOCITY_SHIFT_LEFT);
    const uint32_t rungs[VEL_RUNGS] = {
        0u,             // zero
        1u,             // one
        ratedRaw,       // the documented (rated) maximum
        ratedRaw + 1u,  // one past it -- legal, the rated value is a default
        ceilRaw + 1u,   // one past the experimental ceiling -- saturates
        U32_MAX         // a very large value -- saturates to the same place
    };

    tfhReset(m);
    for (int i = 0; i < VEL_RUNGS; i++) {
        m->setMaximumVelocityRaw(rungs[i]);
        int e = m->getError();
        int64_t rb = (e == 0) ? storedMaxVelocity(m) : (int64_t)-1;
        if (g_trial == 0) g_readback[VEL_BASE + i] = rb;
        uint8_t eff = recordRung(m, e, 0, false);
        if (g_trial == 0) {
            printf("   set max velocity %10lu -> code %2u, stored %lld\n",
                   (unsigned long)rungs[i], (unsigned)eff, (long long)rb);
        }
    }
    // No trailing reset: every ladder OPENS with one, so the next ladder's
    // independence is already guaranteed and a second reset here would only cost
    // 1.5 s per ladder per trial.
}

static void ladderMaxAcceleration(Servomotor* m, int64_t bootA) {
    const uint32_t ratedRaw = (uint32_t)(bootA >> ACCEL_SHIFT_LEFT);
    const uint32_t ceilRaw  = (uint32_t)((bootA * 4) >> ACCEL_SHIFT_LEFT);
    const uint32_t rungs[ACC_RUNGS] = {
        0u, 1u, ratedRaw, ratedRaw + 1u, ceilRaw + 1u, U32_MAX
    };

    tfhReset(m);
    for (int i = 0; i < ACC_RUNGS; i++) {
        m->setMaximumAccelerationRaw(rungs[i]);
        int e = m->getError();
        int64_t rb = (e == 0) ? storedMaxAcceleration(m) : (int64_t)-1;
        if (g_trial == 0) g_readback[ACC_BASE + i] = rb;
        uint8_t eff = recordRung(m, e, 0, false);
        if (g_trial == 0) {
            printf("   set max acceleration %10lu -> code %2u, stored %lld\n",
                   (unsigned long)rungs[i], (unsigned)eff, (long long)rb);
        }
    }
}

static void ladderMotorCurrent(Servomotor* m, uint16_t ceilCur) {
    // Five rungs on the motor-current argument with the regeneration argument
    // held at zero, then the top two rungs mirrored onto the regeneration
    // argument. Holding the other argument at zero (always legal) keeps the two
    // independent range checks (motor_control.c:3513 and :3516) from being
    // mistaken for one another.
    const uint16_t cur[CUR_RUNGS] = { 0u, 1u, ceilCur, (uint16_t)(ceilCur + 1u), U16_MAX, 0u, 0u };
    const uint16_t reg[CUR_RUNGS] = { 0u, 0u, 0u,      0u,                       0u,      ceilCur, (uint16_t)(ceilCur + 1u) };

    tfhReset(m);
    for (int i = 0; i < CUR_RUNGS; i++) {
        m->setMaximumMotorCurrentRaw(cur[i], reg[i]);
        int e = m->getError();
        if (g_trial == 0) g_readback[CUR_BASE + i] = -1;   // no readback exists
        uint8_t eff = recordRung(m, e, 0, false);
        if (g_trial == 0) {
            printf("   set max motor current (%5lu, %5lu) -> code %2u\n",
                   (unsigned long)cur[i], (unsigned long)reg[i], (unsigned)eff);
        }
    }
}

static void ladderPidConstants(Servomotor* m) {
    // The "documented maximum" for a gain is INT32_MAX: above that the firmware
    // caps rather than refuses (motor_control.c:2267-2269), because letting a
    // u32 wrap into the int32 working constant would turn the feedback POSITIVE.
    const uint32_t gains[PID_RUNGS] = {
        0u, 1u, I32_MAX_AS_U32, I32_MAX_AS_U32 + 1u, U32_MAX
    };

    tfhReset(m);
    for (int i = 0; i < PID_RUNGS; i++) {
        m->setPidConstants(gains[i], gains[i], gains[i]);
        int e = m->getError();
        if (g_trial == 0) g_readback[PID_BASE + i] = -1;   // no readback exists
        uint8_t eff = recordRung(m, e, 0, false);
        if (g_trial == 0) {
            printf("   set PID constants (%lu, %lu, %lu) -> code %2u\n",
                   (unsigned long)gains[i], (unsigned long)gains[i],
                   (unsigned long)gains[i], (unsigned)eff);
        }
    }
}

static void ladderDeviationWindow(Servomotor* m) {
    // The default window is two shaft rotations (motor_control.c:97). INT64_MIN
    // is the "one past the maximum" of a two's-complement i64 and is the rung
    // that exercises the BUG-24 guard at main.c:997-999.
    const int64_t windows[DEV_RUNGS] = {
        0,                    // zero -- any deviation at all is too much
        1,                    // one count
        TFH_CPR * 2,          // the factory default, two rotations
        I64_MAX,              // the documented maximum
        I64_MIN,              // one past it, in two's complement
        -(TFH_CPR * 2)        // a large negative value -- the absolute-value contract
    };
    // Rungs 0 and 1 are decided by hall jitter, not by the setter; see header.
    const bool tolerant[DEV_RUNGS] = { true, true, false, false, false, false };

    tfhReset(m);
    for (int i = 0; i < DEV_RUNGS; i++) {
        m->setMaxAllowablePositionDeviationRaw(windows[i]);
        int e = m->getError();
        if (g_trial == 0) g_readback[DEV_BASE + i] = -1;   // no readback exists
        // 250 ms is far longer than one pass of the main loop that hosts the
        // watchdog (main.c:1503), so a window that is going to trip has tripped.
        uint8_t eff = recordRung(m, e, 250, tolerant[i]);
        if (g_trial == 0) {
            printf("   set deviation window %20lld -> code %2u\n",
                   (long long)windows[i], (unsigned)eff);
        }
    }
}

static void ladderSafetyLimits(Servomotor* m) {
    tfhReset(m);
    // Nothing has moved since the reset, so this is the origin; the fences below
    // are built around it so that "contains the current position" is exact.
    int64_t pos = m->getPositionRaw();
    if (m->getError() != 0) pos = 0;

    const int64_t lower[SAF_RUNGS] = { pos,     pos - 1, I64_MIN, pos + 1, pos + 1, I64_MAX };
    const int64_t upper[SAF_RUNGS] = { pos,     pos + 1, I64_MAX, pos + 2, pos,     I64_MIN };

    for (int i = 0; i < SAF_RUNGS; i++) {
        m->setSafetyLimitsRaw(lower[i], upper[i]);
        int e = m->getError();
        if (g_trial == 0) g_readback[SAF_BASE + i] = -1;   // no readback exists
        // The fence is re-checked every control tick (motor_control.c:2640), so
        // a fence that excludes the current position faults within 32 us -- but
        // possibly a hair BEFORE the command's own reply is transmitted, which
        // is why the verdict is the effective code rather than the reply alone.
        uint8_t eff = recordRung(m, e, 250, false);
        if (i == 3) g_safExcludesCode = eff;   // the excludes-current-position rung
        if (g_trial == 0) {
            printf("   set safety limits (%lld, %lld) -> code %2u\n",
                   (long long)lower[i], (long long)upper[i], (unsigned)eff);
        }
    }
    tfhReset(m);
}

void tm_boundary_value_sweep(void) {
    Serial.println("tm_boundary_value_sweep: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // File-scope state must be re-initialised at entry.
    for (int t = 0; t < 3; t++)
        for (int i = 0; i < TOTAL_RUNGS; i++) g_verdict[t][i] = 0x7FFFFFFF;
    for (int i = 0; i < TOTAL_RUNGS; i++) g_readback[i] = -1;
    g_trial = 0; g_cursor = 0; g_refusals = 0; g_transportFailures = 0;
    g_badCodeSeen = false; g_firstBadCode = 0;
    g_codeMismatch = false; g_recoveryFailed = false;

    // =====================================================================
    // 0. PREFLIGHT. Two of the six setters can be read back, and the values
    //    read here are what every later expectation is derived from, so they
    //    are measured on a freshly reset machine before anything is changed.
    //    Deriving the ladder from the device rather than hard-coding M17's
    //    numbers keeps the module correct on an M1, M2, M17 or M23 -- and the
    //    documented RELATIONSHIPS between those numbers become assertions of
    //    their own further down.
    // =====================================================================
    printf("\n--- 0. preflight: the machine and the two readable ceilings ---\n");
    int64_t bootV = 0, bootA = 0;
    uint16_t ceilCur = 0;
    const char* productName = "unknown";
    {
        tfhReset(m);
        uint8_t startFatal = latchedCode(m);
        TEST_RESULT("the machine starts with no latched fatal error", startFatal == 0);

        bootV = storedMaxVelocity(m);
        bootA = storedMaxAcceleration(m);
        printf("   boot maxVelocity = %lld, boot maxAcceleration = %lld\n",
               (long long)bootV, (long long)bootA);
        TEST_RESULT("the stored speed and acceleration ceilings are readable and positive",
                    bootV > 0 && bootA > 0);

        // The current ceiling is a per-product compile-time constant with no
        // readback, so the device is asked which product it is.
        getProductInfoResponse info = m->getProductInfo();
        bool infoOk = (m->getError() == 0);
        char code[9];
        memcpy(code, info.productCode, 8);
        code[8] = '\0';
        bool thirdIsDigit = (code[2] >= '0' && code[2] <= '9');
        if (infoOk && code[0] == 'M' && code[1] == '1' && !thirdIsDigit) {
            productName = "M1";  ceilCur = 300;   // motor_control.c:54
        } else if (infoOk && code[0] == 'M' && code[1] == '2' && !thirdIsDigit) {
            productName = "M2";  ceilCur = 300;   // motor_control.c:54
        } else if (infoOk && code[0] == 'M' && code[1] == '1' && code[2] == '7') {
            productName = "M17"; ceilCur = 390;   // motor_control.c:57
        } else if (infoOk && code[0] == 'M' && code[1] == '2' && code[2] == '3') {
            productName = "M23"; ceilCur = 1024;  // motor_control.c:60, PWM_PERIOD_TIM1
        }
        printf("   productCode=\"%s\" -> %s, documented current ceiling %lu\n",
               code, productName, (unsigned long)ceilCur);
        TEST_RESULT("the product is one whose documented current ceiling is known",
                    infoOk && ceilCur != 0);
        if (ceilCur == 0) ceilCur = 300;          // most conservative fallback
        if (bootV <= 0) bootV = 4203359652212LL;  // M17 MAX_VELOCITY, so the sweep still runs
        if (bootA <= 0) bootA = 172938225691LL;   // M17 MAX_ACCELERATION
    }

    // =====================================================================
    // 1..6. CLIMB EVERY LADDER, THREE TIMES. Trial 0 prints and is the trial
    //    the value assertions are read from; trials 1 and 2 exist only to be
    //    compared against it. Each trial starts from a reset machine inside
    //    each ladder, so the trials share no state whatsoever.
    // =====================================================================
    for (g_trial = 0; g_trial < 3; g_trial++) {
        g_cursor = 0;
        if (g_trial == 0) {
            printf("\n--- 1. cmd 3, Set maximum velocity ---\n");
            ladderMaxVelocity(m, bootV);
            printf("\n--- 2. cmd 5, Set maximum acceleration ---\n");
            ladderMaxAcceleration(m, bootA);
            printf("\n--- 3. cmd 28, Set maximum motor current ---\n");
            ladderMotorCurrent(m, ceilCur);
            printf("\n--- 4. cmd 43, Set PID constants ---\n");
            ladderPidConstants(m);
            printf("\n--- 5. cmd 44, Set max allowable position deviation ---\n");
            ladderDeviationWindow(m);
            printf("\n--- 6. cmd 30, Set safety limits ---\n");
            ladderSafetyLimits(m);
        } else {
            printf("\n--- repeat trial %d of the whole sweep ---\n", g_trial + 1);
            ladderMaxVelocity(m, bootV);
            ladderMaxAcceleration(m, bootA);
            ladderMotorCurrent(m, ceilCur);
            ladderPidConstants(m);
            ladderDeviationWindow(m);
            ladderSafetyLimits(m);
        }
        printf("   trial %d completed %d rungs (%d refusals so far)\n",
               g_trial + 1, g_cursor, g_refusals);
    }
    g_trial = 0;   // later probes record nothing; keep the index in range

    // =====================================================================
    // 1'. THE SPEED CEILING, RUNG BY RUNG. The stored value is the whole
    //     story here: below the experimental ceiling the setter is exactly
    //     linear (stored == raw << 12), and above it every input collapses
    //     onto one number. Asserting the stored value rather than merely
    //     "accepted" is what distinguishes a saturating setter from one that
    //     honoured an absurd request.
    // =====================================================================
    printf("\n--- 1'. what cmd 3 did with each rung ---\n");
    {
        const uint32_t ratedRaw = (uint32_t)(bootV >> VELOCITY_SHIFT_LEFT);
        const int64_t  ceilV    = bootV * 2;   // EXPERIMENTAL_MAX_VELOCITY, motor_control.h:61
        printf("   rated %lld, experimental ceiling %lld, ratedRaw %lu\n",
               (long long)bootV, (long long)ceilV, (unsigned long)ratedRaw);

        TEST_RESULT("a maximum velocity of zero is refused with ERROR_PARAMETER_OUT_OF_RANGE (34)",
                    g_verdict[0][VEL_BASE + 0] == (int32_t)ERR_PARAMETER_OUT_OF_RANGE);
        TEST_RESULT("a maximum velocity of one is accepted and stored as exactly one, shifted left by 12",
                    g_verdict[0][VEL_BASE + 1] == 0 &&
                    g_readback[VEL_BASE + 1] == ((int64_t)1 << VELOCITY_SHIFT_LEFT));
        TEST_RESULT("the rated maximum velocity is accepted and stored exactly as sent",
                    g_verdict[0][VEL_BASE + 2] == 0 &&
                    g_readback[VEL_BASE + 2] == ((int64_t)ratedRaw << VELOCITY_SHIFT_LEFT));
        TEST_RESULT("one past the rated maximum velocity is accepted -- the rated value is a "
                    "default, not a ceiling",
                    g_verdict[0][VEL_BASE + 3] == 0 &&
                    g_readback[VEL_BASE + 3] == ((int64_t)(ratedRaw + 1u) << VELOCITY_SHIFT_LEFT));
        TEST_RESULT("past the experimental ceiling the speed limit SATURATES instead of being "
                    "refused, and UINT32_MAX lands on the very same stored value",
                    g_verdict[0][VEL_BASE + 4] == 0 && g_verdict[0][VEL_BASE + 5] == 0 &&
                    g_readback[VEL_BASE + 4] == ceilV && g_readback[VEL_BASE + 5] == ceilV);
        TEST_RESULT("the value the speed limit saturates to is exactly twice the boot default and "
                    "strictly above the rated maximum",
                    g_readback[VEL_BASE + 5] == bootV * 2 &&
                    g_readback[VEL_BASE + 5] > g_readback[VEL_BASE + 2]);
    }

    // =====================================================================
    // 2'. THE ACCELERATION CEILING, RUNG BY RUNG. The identical shape with a
    //     different shift and a different multiplier, which is the point: two
    //     setters written independently must behave the same way at their
    //     boundaries or a caller cannot reason about either.
    // =====================================================================
    printf("\n--- 2'. what cmd 5 did with each rung ---\n");
    {
        const uint32_t ratedRaw = (uint32_t)(bootA >> ACCEL_SHIFT_LEFT);
        const int64_t  ceilA    = bootA * 4;   // EXPERIMENTAL_MAX_ACCELERATION, motor_control.h:62
        printf("   rated %lld, experimental ceiling %lld, ratedRaw %lu\n",
               (long long)bootA, (long long)ceilA, (unsigned long)ratedRaw);

        TEST_RESULT("a maximum acceleration of zero is refused with ERROR_PARAMETER_OUT_OF_RANGE (34)",
                    g_verdict[0][ACC_BASE + 0] == (int32_t)ERR_PARAMETER_OUT_OF_RANGE);
        TEST_RESULT("a maximum acceleration of one is accepted and stored as exactly one, "
                    "shifted left by 8",
                    g_verdict[0][ACC_BASE + 1] == 0 &&
                    g_readback[ACC_BASE + 1] == ((int64_t)1 << ACCEL_SHIFT_LEFT));
        TEST_RESULT("the rated maximum acceleration is accepted and stored exactly as sent",
                    g_verdict[0][ACC_BASE + 2] == 0 &&
                    g_readback[ACC_BASE + 2] == ((int64_t)ratedRaw << ACCEL_SHIFT_LEFT));
        TEST_RESULT("one past the rated maximum acceleration is accepted",
                    g_verdict[0][ACC_BASE + 3] == 0 &&
                    g_readback[ACC_BASE + 3] == ((int64_t)(ratedRaw + 1u) << ACCEL_SHIFT_LEFT));
        TEST_RESULT("past the experimental ceiling the acceleration limit SATURATES instead of "
                    "being refused, and UINT32_MAX lands on the very same stored value",
                    g_verdict[0][ACC_BASE + 4] == 0 && g_verdict[0][ACC_BASE + 5] == 0 &&
                    g_readback[ACC_BASE + 4] == ceilA && g_readback[ACC_BASE + 5] == ceilA);
        TEST_RESULT("the value the acceleration limit saturates to is exactly four times the boot "
                    "default and strictly above the rated maximum",
                    g_readback[ACC_BASE + 5] == bootA * 4 &&
                    g_readback[ACC_BASE + 5] > g_readback[ACC_BASE + 2]);
    }

    // =====================================================================
    // 3'. THE CURRENT LIMIT. The first setter in the sweep that REFUSES, and
    //     the contrast with sections 1' and 2' is the module's headline: the
    //     same shape of request -- "a number far beyond anything sensible" --
    //     is silently accepted by two setters and faulted by this one.
    // =====================================================================
    printf("\n--- 3'. what cmd 28 did with each rung ---\n");
    {
        TEST_RESULT("zero and one are both accepted as a maximum motor current",
                    g_verdict[0][CUR_BASE + 0] == 0 && g_verdict[0][CUR_BASE + 1] == 0);
        TEST_RESULT("the documented current ceiling itself is accepted -- the firmware test is "
                    "strictly greater-than",
                    g_verdict[0][CUR_BASE + 2] == 0);
        TEST_RESULT("one past the current ceiling is refused with "
                    "ERROR_MAX_PWM_VOLTAGE_TOO_HIGH (23), never clamped",
                    g_verdict[0][CUR_BASE + 3] == (int32_t)ERR_MAX_PWM_VOLTAGE_TOO_HIGH);
        TEST_RESULT("the largest representable current (65535) is refused with the same code",
                    g_verdict[0][CUR_BASE + 4] == (int32_t)ERR_MAX_PWM_VOLTAGE_TOO_HIGH);
        TEST_RESULT("the regeneration argument follows the identical ladder: the ceiling is "
                    "accepted and one past it is refused with 23",
                    g_verdict[0][CUR_BASE + 5] == 0 &&
                    g_verdict[0][CUR_BASE + 6] == (int32_t)ERR_MAX_PWM_VOLTAGE_TOO_HIGH);
    }

    // =====================================================================
    // 4'. THE PID GAINS. The third distinct policy in six setters: this one
    //     validates nothing at all. Worth stating explicitly, because "the
    //     command was accepted" is the ONLY feedback a caller gets, and here it
    //     carries no information about whether the value survived intact.
    // =====================================================================
    printf("\n--- 4'. what cmd 43 did with each rung ---\n");
    {
        bool allAccepted = true;
        for (int i = 0; i < PID_RUNGS; i++)
            if (g_verdict[0][PID_BASE + i] != 0) allAccepted = false;
        TEST_RESULT("every rung of the PID ladder is accepted, including all-zero gains -- this "
                    "setter refuses nothing",
                    allAccepted);
        TEST_RESULT("gains past INT32_MAX are accepted rather than refused (they are capped "
                    "internally, which is not observable from the bus)",
                    g_verdict[0][PID_BASE + 3] == 0 && g_verdict[0][PID_BASE + 4] == 0);

        // With the gains still at their most extreme the planner must be
        // untouched: the PID constants belong to the closed loop, and with the
        // output stage off they cannot reach anything.
        tfhFreshOpen(m);
        m->setPidConstants(U32_MAX, U32_MAX, U32_MAX);
        bool setOk = (m->getError() == 0);
        bool ok = false;
        int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 8), tfhTicksFor(TFH_CPR / 8), &ok);
        printf("   with all gains at UINT32_MAX an eighth of a rotation moved %lld of %lld\n",
               (long long)moved, (long long)(TFH_CPR / 8));
        TEST_RESULT("an ordinary move still runs correctly with all three gains at UINT32_MAX",
                    setOk && ok && llabs((long long)(moved - TFH_CPR / 8)) <= 16);
        tfhReset(m);
    }

    // =====================================================================
    // 5'. THE DEVIATION WINDOW. The fourth policy: accept everything, and let
    //     an asynchronous watchdog deal with the consequences. The two
    //     behavioural checks here are the ones the ladder cannot make, because
    //     the setter has no readback: that a NEGATIVE window is interpreted as
    //     its magnitude, and that INT64_MIN saturates instead of poisoning
    //     every tick.
    // =====================================================================
    printf("\n--- 5'. what cmd 44 did with each rung ---\n");
    {
        bool allAccepted = true;
        for (int i = 0; i < DEV_RUNGS; i++) {
            int32_t v = g_verdict[0][DEV_BASE + i];
            // The two coarsened rungs count as accepted-by-the-setter too: the
            // command answered, and only the watchdog had anything to say.
            if (v != 0 && v != VERDICT_WATCHDOG_DOMAIN) allAccepted = false;
        }
        TEST_RESULT("every rung of the deviation ladder is accepted by the command itself, "
                    "including zero and INT64_MIN -- this setter refuses nothing",
                    allAccepted);
        TEST_RESULT("the only fault a tiny deviation window can produce is "
                    "ERROR_POSITION_DEVIATION_TOO_LARGE (45)",
                    (g_verdict[0][DEV_BASE + 0] == 0 ||
                     g_verdict[0][DEV_BASE + 0] == VERDICT_WATCHDOG_DOMAIN) &&
                    (g_verdict[0][DEV_BASE + 1] == 0 ||
                     g_verdict[0][DEV_BASE + 1] == VERDICT_WATCHDOG_DOMAIN));

        // A negative window must behave as its absolute value. If the sign were
        // taken literally the comparison llabs(deviation) > window would be true
        // on the very first tick and even a tiny move would fault; if the
        // magnitude were ignored altogether nothing would ever fault. Requiring
        // BOTH halves -- inside completes, outside trips -- pins it down.
        tfhReset(m);
        m->setMaximumVelocity(15.0f);
        m->setMaximumAcceleration(5000.0f);
        m->setMaxAllowablePositionDeviationRaw(-(int64_t)TFH_CPR);   // one rotation, negative
        bool negSetOk = (m->getError() == 0);
        bool insideOk = false;
        int64_t insideMoved = tfhMove(m, (int32_t)(TFH_CPR / 4),
                                      tfhTicksFor(TFH_CPR / 4), &insideOk);
        uint8_t insideFatal = latchedCode(m);

        tfhReset(m);
        m->setMaximumVelocity(15.0f);
        m->setMaximumAcceleration(5000.0f);
        m->setMaxAllowablePositionDeviationRaw(-(int64_t)TFH_CPR);
        m->trapezoidMoveRaw((int32_t)(TFH_CPR * 3 / 2), tfhTicksFor(TFH_CPR * 3 / 2));
        int outsideQueueErr = m->getError();
        // POLL for the trip; do not wait a fixed duration for a move. The move
        // lasts about 0.63 s and crosses the one-rotation window roughly two
        // thirds of the way through, so this normally resolves in well under a
        // second. The 5 s ceiling exists only so that a firmware which does NOT
        // trip ends the test with outsideFatal == 0 -- a real, reported failure
        // -- instead of hanging. tfhDrain cannot be used here: the move is meant
        // to fault, and a faulted device refuses Get n queued items (finding F2).
        uint8_t outsideFatal = 0;
        for (uint32_t t0 = millis(); (millis() - t0) < 5000; ) {
            uint8_t f = tfhFatal(m);
            if (f != 0 && f != 0xFF) { outsideFatal = f; break; }
            delay(50);
        }
        printf("   window -1 rot: quarter rotation moved %lld (fatal %u); "
               "one and a half rotations -> fatal %u (queue err %d)\n",
               (long long)insideMoved, (unsigned)insideFatal,
               (unsigned)outsideFatal, outsideQueueErr);
        TEST_RESULT("a negative deviation window behaves as its absolute value: a move inside it "
                    "completes and a move outside it trips error 45",
                    negSetOk && insideOk && insideFatal == 0 &&
                    llabs((long long)(insideMoved - TFH_CPR / 4)) <= 64 &&
                    outsideFatal == ERR_POSITION_DEVIATION_TOO_LARGE);

        // INT64_MIN is the value llabs() cannot represent. The firmware maps it
        // to INT64_MAX (main.c:997-999); had it not, the stored window would be
        // negative and the machine would fault on the first tick. Three
        // rotations is chosen because it is comfortably OUTSIDE the two-rotation
        // default, so a machine that had quietly kept the default would fail
        // this too.
        tfhReset(m);
        m->setMaximumVelocity(15.0f);
        m->setMaximumAcceleration(5000.0f);
        m->setMaxAllowablePositionDeviationRaw(I64_MIN);
        bool minSetOk = (m->getError() == 0);
        bool bigOk = false;
        int64_t bigMoved = tfhMove(m, (int32_t)(TFH_CPR * 3), tfhTicksFor(TFH_CPR * 3), &bigOk);
        printf("   window INT64_MIN: three rotations moved %lld of %lld (ok=%d)\n",
               (long long)bigMoved, (long long)(TFH_CPR * 3), (int)bigOk);
        TEST_RESULT("INT64_MIN saturates to INT64_MAX instead of faulting every tick: a "
                    "three-rotation move completes untouched",
                    minSetOk && bigOk &&
                    llabs((long long)(bigMoved - TFH_CPR * 3)) <= 256);
        tfhReset(m);
    }

    // =====================================================================
    // 6'. THE SAFETY FENCE. The fifth policy, and the only setter that can be
    //     refused for the RELATIONSHIP between its two arguments rather than
    //     for either value on its own -- plus the only one whose accepted value
    //     can fault the machine on the very next control tick.
    // =====================================================================
    printf("\n--- 6'. what cmd 30 did with each rung ---\n");
    {
        TEST_RESULT("a zero-width fence exactly on the current position is accepted and quiet -- "
                    "the parameter check is strictly greater-than",
                    g_verdict[0][SAF_BASE + 0] == 0);
        TEST_RESULT("a fence one count either side of the current position is accepted and quiet, "
                    "and so is the widest possible fence",
                    g_verdict[0][SAF_BASE + 1] == 0 && g_verdict[0][SAF_BASE + 2] == 0);
        // FINDING F3: this rung's REPLY is lost about a third of the time.
        // The fence goes live at main.c:712 and the reply is only sent at :715,
        // while the control ISR re-checks the fence every tick
        // (motor_control.c:2640) and calls fatal_error(), which never returns.
        // Measured: 10 of 30 trials lost the reply; the control (a fence that
        // includes the current position) lost 0 of 30. The FENCE itself is
        // reliable -- code 25 was raised in 29 of 30 trials.
        //
        // So the assertion is the part that is deterministic: the fault fires
        // with the documented code. Whether the command's own reply survives is
        // recorded and printed, not asserted, because asserting it would make
        // this test fail a third of the time on correct firmware.
        TEST_RESULT("a fence that EXCLUDES the current position faults with "
                    "ERROR_SAFETY_LIMIT_EXCEEDED (25), whether or not its own "
                    "reply survives the race (finding F3)",
                    g_safExcludesCode == 25 || g_safExcludesCode == 0xFE);
        TEST_RESULT("limits inverted by a single count are refused with "
                    "ERROR_PARAMETER_OUT_OF_RANGE (34)",
                    g_verdict[0][SAF_BASE + 4] == (int32_t)ERR_PARAMETER_OUT_OF_RANGE);
        TEST_RESULT("the maximally inverted pair (INT64_MAX, INT64_MIN) gives the identical code",
                    g_verdict[0][SAF_BASE + 5] == (int32_t)ERR_PARAMETER_OUT_OF_RANGE);
    }

    // =====================================================================
    // 7. THE CROSS-CUTTING CLAIMS. Everything above is about one setter at a
    //    time; these five are about the sweep as a whole, and they are the
    //    reason for climbing every ladder three times.
    // =====================================================================
    printf("\n--- 7. the sweep as a whole ---\n");
    {
        // A rung is comparable only if all three trials produced a real verdict.
        // A rung poisoned by a lost frame is counted and reported, never
        // compared: a dropped frame is a bus event and would otherwise show up
        // as "the firmware is non-deterministic", which is the single most
        // expensive kind of false report this suite can make. The comparable
        // count is asserted too, so the claim cannot pass by being vacuous.
        int mismatches = 0, firstMismatch = -1, comparable = 0, lostRungs = 0;
        for (int i = 0; i < TOTAL_RUNGS; i++) {
            if (g_verdict[0][i] == (int32_t)VERDICT_LINK_LOST ||
                g_verdict[1][i] == (int32_t)VERDICT_LINK_LOST ||
                g_verdict[2][i] == (int32_t)VERDICT_LINK_LOST) {
                lostRungs++;
                continue;
            }
            comparable++;
            if (g_verdict[0][i] != g_verdict[1][i] || g_verdict[0][i] != g_verdict[2][i]) {
                mismatches++;
                if (firstMismatch < 0) firstMismatch = i;
            }
        }
        if (mismatches) {
            printf("   %d rung(s) disagreed; first is rung %d: %ld / %ld / %ld\n",
                   mismatches, firstMismatch,
                   (long)g_verdict[0][firstMismatch], (long)g_verdict[1][firstMismatch],
                   (long)g_verdict[2][firstMismatch]);
        }
        if (lostRungs || g_transportFailures) {
            printf("   WARNING: %d rung(s) lost to the link (%d transport failures). "
                   "Check for a second process on the port BEFORE suspecting the firmware.\n",
                   lostRungs, g_transportFailures);
        }
        printf("   %d rungs x 3 trials, %d comparable, %d refusals provoked and recovered\n",
               TOTAL_RUNGS, comparable, g_refusals);
        TEST_RESULT("every rung measured in all three trials gave the identical verdict, and at "
                    "least thirty-two of the thirty-six rungs were measured",
                    mismatches == 0 && comparable >= TOTAL_RUNGS - 4);

        printf("   undocumented code seen: %d (first was %u); reply/latched mismatch: %d\n",
               (int)g_badCodeSeen, (unsigned)g_firstBadCode, (int)g_codeMismatch);
        TEST_RESULT("every refusal used one of the four documented codes (23, 25, 34, 45) and "
                    "nothing else",
                    !g_badCodeSeen);
        TEST_RESULT("a refusal reports the same code twice: in the reply to the offending command "
                    "and in the latched status",
                    !g_codeMismatch);
        TEST_RESULT("every refusal was recoverable: a reset cleared the fault and the device "
                    "round-tripped a ping, every time",
                    g_refusals > 0 && !g_recoveryFailed);

        // The headline contrast, stated as one assertion so that a firmware
        // change in EITHER direction -- making the ceilings refuse, or making
        // the current limit clamp -- is flagged rather than absorbed.
        bool ceilingsSaturate = (g_verdict[0][VEL_BASE + 5] == 0 && g_verdict[0][ACC_BASE + 5] == 0 &&
                                 g_readback[VEL_BASE + 5] == bootV * 2 &&
                                 g_readback[ACC_BASE + 5] == bootA * 4);
        bool othersDoNot = (g_verdict[0][CUR_BASE + 4] == (int32_t)ERR_MAX_PWM_VOLTAGE_TOO_HIGH &&
                            g_verdict[0][PID_BASE + 4] == 0 &&
                            g_verdict[0][SAF_BASE + 5] == (int32_t)ERR_PARAMETER_OUT_OF_RANGE);
        TEST_RESULT("only the speed and acceleration ceilings saturate an over-large value; the "
                    "current limit refuses it, the safety fence refuses it, and the PID gains "
                    "keep it",
                    ceilingsSaturate && othersDoNot);
    }

    // Leave a clean machine, and prove it is not merely quiet but working.
    tfhFreshOpen(m);
    bool finalOk = false;
    int64_t finalMoved = tfhMove(m, (int32_t)(TFH_CPR / 4), tfhTicksFor(TFH_CPR / 4), &finalOk);
    printf("   closing move: %lld of %lld\n",
           (long long)finalMoved, (long long)(TFH_CPR / 4));
    tfhReset(m);
    TEST_RESULT("after all thirty-six rungs the machine still runs an ordinary move and is left "
                "clean",
                finalOk && llabs((long long)(finalMoved - TFH_CPR / 4)) <= 32 &&
                latchedCode(m) == 0);
}
