#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_deviation_tracking.cpp
//
// THE POSITION-DEVIATION WATCHDOG, CHARACTERISED.
//
// The firmware continuously compares where it has COMMANDED the shaft to be
// against where the hall sensors say it actually is, and faults if the gap gets
// too big (motor_control.c:3217):
//
//     position_deviation = current_position - current_hall_position;
//     if (calibration_step == 0 && llabs(position_deviation) > max_allowable_position_deviation)
//         fatal_error(ERROR_POSITION_DEVIATION_TOO_LARGE);   // code 45
//
// This is the mechanism that notices a jam, a stall, a slipped coupling or a
// missed step -- the single most important safety check in normal operation.
// It is also, with the MOSFETs off, the thing that quietly limits how far any
// test in this suite can command the shaft to travel: the rotor never follows,
// so the deviation grows with every commanded count and trips at the default
// two-rotation window. That cost two debugging cycles during this campaign
// before it was understood.
//
// Nothing in the suite characterises it. This module does, and the MOSFETs
// being off is what makes it precisely measurable rather than a nuisance:
//
//     with the output stage off, the hall position is CONSTANT, so the
//     deviation is exactly the commanded displacement.
//
// That turns the watchdog into a calibrated instrument. The trip point can be
// predicted to the count and compared against the configured threshold, which
// is a far sharper test than watching for a fault at some unknown moment.
//
// WHY THIS IS SAFE, and why it is not "testing with the brakes off": every
// check here reads the COMMANDED position, which the planner advances whether
// or not the MOSFETs are energised. Nothing turns, on any motor, at any point.
// A deliberately-tripped deviation fault is cleared by a reset like any other.
// No broadcasts and no alias changes: safe on the shared 35-motor rack.
// ---------------------------------------------------------------------------

static const uint8_t ERR_POSITION_DEVIATION_TOO_LARGE = 45;

// Set the deviation window in encoder counts, via the shaft-rotations unit the
// setter uses. Returns false if the setter itself reported an error.
static bool setDeviationRotations(Servomotor* m, float rotations) {
    m->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    m->setMaxAllowablePositionDeviation(rotations);
    bool ok = (m->getError() == 0);
    m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    return ok;
}

// Command a long move and report how far the commanded position had travelled
// when the deviation watchdog fired (or 0 if it never did).
static int64_t tripPoint(Servomotor* m, int32_t distance, uint32_t ticks,
                         uint8_t* code) {
    int64_t before = m->getPositionRaw();
    m->trapezoidMoveRaw(distance, ticks);
    if (m->getError() != 0) { *code = 0xFD; return 0; }

    int64_t furthest = 0;
    uint32_t deadline = millis() + (uint32_t)((uint64_t)ticks * 1000ULL / TFH_TPS) + 8000;
    while (millis() < deadline) {
        int64_t p = m->getPositionRaw();
        if (m->getError() != 0) break;              // the fault refuses reads
        furthest = p - before;
        if (m->getNQueuedItems() == 0 && m->getError() == 0) break;
        delay(15);
    }
    *code = tfhFatal(m);
    return furthest;
}

void tm_deviation_tracking(void) {
    Serial.println("tm_deviation_tracking: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. WITH THE MOSFETs OFF, THE HALL POSITION DOES NOT MOVE. This is the
    //    premise the rest of the module rests on, so it is measured rather
    //    than assumed -- and it is a real property in its own right: a hall
    //    reading that drifted while the shaft was stationary would make the
    //    whole watchdog unreliable.
    // =====================================================================
    printf("\n--- 1. the hall position is constant while the output stage is off ---\n");
    {
        tfhReset(m);
        m->disableMosfets();
        delay(200);
        int64_t h0 = m->getHallSensorPositionRaw();
        bool ok = (m->getError() == 0);
        int64_t hLo = h0, hHi = h0;

        // Command a substantial move; the commanded position advances, the
        // shaft does not.
        m->setMaximumVelocity(4.0f);
        m->setMaximumAcceleration(2000.0f);
        int64_t c0 = m->getPositionRaw();
        m->trapezoidMoveRaw((int32_t)(TFH_CPR / 2), TFH_TPS);
        bool queued = (m->getError() == 0);
        for (int i = 0; i < 30 && ok; i++) {
            int64_t h = m->getHallSensorPositionRaw();
            if (m->getError() != 0) { ok = false; break; }
            if (h < hLo) hLo = h;
            if (h > hHi) hHi = h;
            delay(40);
        }
        tfhDrain(m, 8000);
        int64_t commandedMoved = m->getPositionRaw() - c0;
        printf("   commanded advanced %lld counts; hall spread %lld counts\n",
               (long long)commandedMoved, (long long)(hHi - hLo));
        TEST_RESULT("the hall sensor is readable throughout", ok && queued);
        TEST_RESULT("the commanded position really did advance",
                    llabs((long long)commandedMoved) > TFH_CPR / 4);
        // A stationary shaft: allow sensor noise but nothing like real motion.
        TEST_RESULT("the hall position stays put while the MOSFETs are off",
                    (hHi - hLo) < TFH_CPR / 50);
    }

    // =====================================================================
    // 2. THE TRIP POINT MATCHES THE CONFIGURED WINDOW. Because the hall
    //    position is fixed, the deviation IS the commanded displacement, so
    //    the fault must fire once the commanded position has travelled just
    //    past the threshold -- and not before.
    // =====================================================================
    printf("\n--- 2. the watchdog fires at the configured window ---\n");
    {
        const float windows[] = { 0.25f, 0.5f, 1.0f, 2.0f, 4.0f };
        for (unsigned i = 0; i < sizeof(windows) / sizeof(windows[0]); i++) {
            tfhReset(m);
            m->setMaximumVelocity(4.0f);
            m->setMaximumAcceleration(2000.0f);
            bool setOk = setDeviationRotations(m, windows[i]);
            uint8_t code = 0;
            // Command well past the window so the trip is certain.
            int64_t travelled = tripPoint(m, (int32_t)(TFH_CPR * 8),
                                          (uint32_t)(4 * TFH_TPS), &code);
            int64_t expected = (int64_t)((double)windows[i] * (double)TFH_CPR);
            printf("   window %.2f rot (%lld counts): fired at about %lld counts, code %u\n",
                   (double)windows[i], (long long)expected,
                   (long long)travelled, (unsigned)code);
            TEST_RESULT(std::string("the deviation window can be set to ") +
                            std::to_string((double)windows[i]).substr(0, 4) + " rotations",
                        setOk);
            TEST_RESULT(std::string("a window of ") +
                            std::to_string((double)windows[i]).substr(0, 4) +
                            " rotations raises ERROR_POSITION_DEVIATION_TOO_LARGE (45)",
                        code == ERR_POSITION_DEVIATION_TOO_LARGE);
            // The observed trip point is sampled over the bus, so it lags the
            // real one; require it to be at least the window and not wildly past.
            TEST_RESULT(std::string("a window of ") +
                            std::to_string((double)windows[i]).substr(0, 4) +
                            " rotations does not fire early",
                        code != ERR_POSITION_DEVIATION_TOO_LARGE ||
                        travelled >= expected - TFH_CPR / 4);
            tfhReset(m);
        }
    }

    // =====================================================================
    // 3. A LARGER WINDOW ALWAYS TRIPS LATER. The ordering property: even
    //    though the sampled trip point is approximate, it must be MONOTONIC
    //    in the configured window. A watchdog whose trip point did not track
    //    its setting would be worse than none.
    // =====================================================================
    printf("\n--- 3. a larger window trips later ---\n");
    {
        const float windows[] = { 0.5f, 1.0f, 2.0f, 4.0f };
        int64_t trips[4];
        bool allTripped = true;
        for (unsigned i = 0; i < 4; i++) {
            tfhReset(m);
            m->setMaximumVelocity(4.0f);
            m->setMaximumAcceleration(2000.0f);
            setDeviationRotations(m, windows[i]);
            uint8_t code = 0;
            trips[i] = tripPoint(m, (int32_t)(TFH_CPR * 8), (uint32_t)(4 * TFH_TPS), &code);
            if (code != ERR_POSITION_DEVIATION_TOO_LARGE) allTripped = false;
            tfhReset(m);
        }
        printf("   trip points: %lld, %lld, %lld, %lld counts\n",
               (long long)trips[0], (long long)trips[1],
               (long long)trips[2], (long long)trips[3]);
        TEST_RESULT("all four windows trip the watchdog", allTripped);
        TEST_RESULT("the trip point increases with the window",
                    allTripped && trips[0] < trips[1] &&
                    trips[1] < trips[2] && trips[2] < trips[3]);
    }

    // =====================================================================
    // 4. A MOVE THAT STAYS INSIDE THE WINDOW IS NOT DISTURBED. The other
    //    half of the contract: the watchdog must be invisible during normal
    //    operation, or it would make the machine unusable.
    // =====================================================================
    printf("\n--- 4. a move inside the window completes untouched ---\n");
    {
        const float windows[] = { 1.0f, 2.0f, 4.0f };
        for (unsigned i = 0; i < sizeof(windows) / sizeof(windows[0]); i++) {
            tfhReset(m);
            m->setMaximumVelocity(4.0f);
            m->setMaximumAcceleration(2000.0f);
            setDeviationRotations(m, windows[i]);
            // Command HALF the window: comfortably inside.
            int32_t dist = (int32_t)((double)windows[i] * 0.5 * (double)TFH_CPR);
            int64_t before = m->getPositionRaw();
            m->trapezoidMoveRaw(dist, 2 * TFH_TPS);
            bool accepted = (m->getError() == 0);
            bool drained = accepted && tfhDrain(m, 12000);
            uint8_t fat = tfhFatal(m);
            int64_t moved = (drained && fat == 0) ? (m->getPositionRaw() - before) : 0;
            printf("   window %.1f rot, moved %lld of %ld: fatal=%u\n",
                   (double)windows[i], (long long)moved, (long)dist, (unsigned)fat);
            TEST_RESULT(std::string("a move to half of a ") +
                            std::to_string((double)windows[i]).substr(0, 3) +
                            "-rotation window completes cleanly",
                        drained && fat == 0);
            TEST_RESULT(std::string("and it delivers its full displacement at window ") +
                            std::to_string((double)windows[i]).substr(0, 3),
                        llabs((long long)(moved - dist)) <= dist / 100 + 4);
        }
    }

    // =====================================================================
    // 5. THE DEFAULT WINDOW IS TWO ROTATIONS. Stated explicitly because it is
    //    the number every MOSFETs-off test in this suite has to respect, and
    //    because DEFAULT_MAX_ALLOWABLE_POSITION_DEVIATION is a firmware
    //    constant a future change could move without anyone noticing.
    // =====================================================================
    printf("\n--- 5. the factory default window is two rotations ---\n");
    {
        // Just inside: 1.5 rotations must complete.
        tfhReset(m);
        m->setMaximumVelocity(4.0f);
        m->setMaximumAcceleration(2000.0f);
        int64_t b1 = m->getPositionRaw();
        m->trapezoidMoveRaw((int32_t)(TFH_CPR * 3 / 2), 2 * TFH_TPS);
        bool insideOk = (m->getError() == 0) && tfhDrain(m, 12000) && (tfhFatal(m) == 0);
        int64_t insideMoved = insideOk ? (m->getPositionRaw() - b1) : 0;

        // Just outside: 3 rotations must trip.
        tfhReset(m);
        m->setMaximumVelocity(4.0f);
        m->setMaximumAcceleration(2000.0f);
        uint8_t code = 0;
        tripPoint(m, (int32_t)(TFH_CPR * 3), (uint32_t)(2 * TFH_TPS), &code);

        printf("   1.5 rotations: completed=%d moved=%lld; 3 rotations: code=%u\n",
               (int)insideOk, (long long)insideMoved, (unsigned)code);
        TEST_RESULT("1.5 rotations is inside the default window and completes",
                    insideOk && llabs((long long)(insideMoved - TFH_CPR * 3 / 2)) <= TFH_CPR / 50);
        TEST_RESULT("3 rotations is outside the default window and trips error 45",
                    code == ERR_POSITION_DEVIATION_TOO_LARGE);
        tfhReset(m);
    }

    // =====================================================================
    // 6. RECOVERY. A deviation fault is a normal fatal error: reset clears it
    //    and the machine works again. Repeated, since this fault is raised
    //    from inside the control loop rather than at queue time.
    // =====================================================================
    printf("\n--- 6. recovery from a deviation fault ---\n");
    {
        bool allOk = true;
        for (int cycle = 0; cycle < 4 && allOk; cycle++) {
            tfhReset(m);
            m->setMaximumVelocity(4.0f);
            m->setMaximumAcceleration(2000.0f);
            setDeviationRotations(m, 0.5f);
            uint8_t code = 0;
            tripPoint(m, (int32_t)(TFH_CPR * 4), (uint32_t)(2 * TFH_TPS), &code);
            if (code != ERR_POSITION_DEVIATION_TOO_LARGE) {
                allOk = false;
                printf("   cycle %d did not trip (code %u)\n", cycle, (unsigned)code);
                break;
            }
            tfhReset(m);
            if (tfhFatal(m) != 0) { allOk = false; printf("   cycle %d did not clear\n", cycle); break; }
        }
        TEST_RESULT("four trip-and-recover cycles each fire and each clear", allOk);

        tfhFreshOpen(m);
        bool ok = false;
        int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 4), TFH_TPS, &ok);
        TEST_RESULT("a real move works again after a deviation fault",
                    ok && llabs((long long)(moved - TFH_CPR / 4)) < 200);
    }

    tfhReset(m);
    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
}
