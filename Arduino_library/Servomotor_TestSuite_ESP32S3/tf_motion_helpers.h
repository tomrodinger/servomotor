#pragma once
#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tf_motion_helpers.h
//
// Shared setup for motion-related test modules. Every helper here exists
// because getting it wrong produced a FALSE BUG REPORT during the August 2026
// test-expansion campaign. Read the comments before deviating from them.
//
// Modules written before this header have their own local copies; that is fine
// and they are not worth churning. New modules should include this instead.
// ---------------------------------------------------------------------------

static const int64_t TFH_CPR = 3276800;    // encoder counts per shaft rotation
static const int32_t TFH_TPS = 31250;      // control ticks per second

// ---------------------------------------------------------------------------
// Wait for the motion queue to empty.
//
// NEVER wait a computed duration instead. A fixed wait that is even slightly
// short reads a PARTIAL move and reports a distance shorter than commanded,
// which is indistinguishable from a planner bug. That exact mistake -- a 4 s
// cap against a 6.4 s move, reporting 626 of 1000 counts -- was the campaign's
// first "finding" and cost a debugging cycle to disprove.
// ---------------------------------------------------------------------------
static bool tfhDrain(Servomotor* m, uint32_t budgetMs) {
    uint32_t deadline = millis() + budgetMs;
    while (millis() < deadline) {
        uint8_t q = m->getNQueuedItems();
        if (m->getError() != 0) return false;
        if (q == 0) { delay(150); return true; }
        delay(20);
    }
    return false;
}

// The latched fatal error code, or 0xFF if the status read itself failed.
static uint8_t tfhFatal(Servomotor* m) {
    getStatusResponse st = m->getStatus();
    return (m->getError() == 0) ? st.fatalErrorCode : 0xFF;
}

// ---------------------------------------------------------------------------
// Reset to a known machine and re-establish the unit settings.
//
// The 1500 ms wait is not padding: a reset drops through the bootloader window,
// and a command that lands inside it pins the device in the bootloader
// (status 0x0001) until another reset. Draining the RX buffer afterwards
// removes any late bytes from before the reset, which would otherwise be
// mis-parsed as the head of the next response.
// ---------------------------------------------------------------------------
static void tfhReset(Servomotor* m) {
    m->systemReset();
    delay(1500);
    for (uint32_t t0 = millis(); Serial1.available() && (millis() - t0) < 500; ) Serial1.read();
    m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    m->setTimeUnit(TimeUnit::TIMESTEPS);
    m->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    m->setAccelerationUnit(AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED);
}

// ---------------------------------------------------------------------------
// Raise the ceilings that otherwise become the thing under test.
//
// TWO traps, both of which produced false failures during the campaign:
//
//  1. MAX ALLOWABLE POSITION DEVIATION defaults to TWO ROTATIONS
//     (DEFAULT_MAX_ALLOWABLE_POSITION_DEVIATION, motor_control.c:97). With the
//     MOSFETs off the shaft never follows the commanded position, so the
//     deviation grows with every count commanded and ANY move past about two
//     rotations raises fatal error 45. Every MOSFETs-off test that commands
//     more than two turns must raise this first.
//
//  2. MAX VELOCITY caps how far a move can get in a given time, and
//     acceleration segments ACCUMULATE velocity across a queued sequence. A
//     ceiling chosen for the typical case will refuse the worst case, and the
//     test then measures the ceiling rather than its subject. The firmware
//     clamps at EXPERIMENTAL_MAX_VELOCITY = 2 x MAX_VELOCITY (motor_control.h:61),
//     about 18.7 rot/s, so 15 rot/s is the practical working ceiling.
//
// Neither of these makes the machine less safe with the output stage off: the
// deviation limit is a diagnostic about a shaft that was never going to move,
// and a speed ceiling cannot cause motion. A reset restores both defaults.
// ---------------------------------------------------------------------------
static void tfhOpenCeilings(Servomotor* m) {
    m->setMaximumVelocity(15.0f);
    m->setMaximumAcceleration(5000.0f);
    m->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    m->setMaxAllowablePositionDeviation(100000.0f);
    m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
}

// Reset and open the ceilings in one call: the usual starting point.
static void tfhFreshOpen(Servomotor* m) {
    tfhReset(m);
    tfhOpenCeilings(m);
}

// ---------------------------------------------------------------------------
// Ticks needed to cover `counts` at a sustainable speed, with ramp headroom.
// Asking for a shorter duration than this is asking to be REFUSED: the
// firmware's limits are pass/fail checks, never clamps -- it does not stretch
// an over-ambitious move to fit, it raises fatal error 15, 16 or 28.
// ---------------------------------------------------------------------------
static uint32_t tfhTicksFor(int64_t counts) {
    double rotations = (double)(counts < 0 ? -counts : counts) / (double)TFH_CPR;
    double seconds = rotations / 12.0 + 0.5;
    return (uint32_t)(seconds * (double)TFH_TPS);
}

// Perform one trapezoid move and return the commanded-position delta.
// `ok` reports whether it was accepted, completed, and left no fatal error.
static int64_t tfhMove(Servomotor* m, int32_t counts, uint32_t ticks, bool* ok) {
    int64_t before = m->getPositionRaw();
    m->trapezoidMoveRaw(counts, ticks);
    if (m->getError() != 0) { *ok = false; return 0; }
    uint32_t budget = (uint32_t)((uint64_t)ticks * 1000ULL / TFH_TPS) + 10000;
    if (!tfhDrain(m, budget)) { *ok = false; return 0; }
    if (tfhFatal(m) != 0) { *ok = false; return 0; }
    *ok = true;
    return m->getPositionRaw() - before;
}
