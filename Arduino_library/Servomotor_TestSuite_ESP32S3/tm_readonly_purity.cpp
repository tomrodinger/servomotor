#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_readonly_purity.cpp
//
// READING SOMETHING MUST NOT CHANGE IT.
//
// The suite has a module for almost every individual getter, each checking that
// the value it returns is sane. None of them checks the property that makes a
// getter safe to call at all:
//
//     A read-only command must have NO observable effect. It must not move the
//     machine, not touch the queue, not raise an error, not disturb a move that
//     is already running, and not change what the next read returns.
//
// This matters more here than in most systems. Every one of these commands is a
// packet on a shared half-duplex bus, handled by the same main loop that runs
// the motion planner, on a Cortex-M0+ with no cache and no MMU to isolate
// anything. A getter that took a lock, reset a latch, or nudged a shared
// accumulator would be invisible to a test that only checked its return value --
// and would corrupt whatever motion happened to be running when a monitoring
// tool polled it.
//
// It is also the property that supervisory software depends on absolutely: a
// dashboard polling temperature and position at 10 Hz must not perturb the
// machine it is watching.
//
// Three things are checked for each command:
//   * calling it repeatedly on an IDLE machine changes nothing observable
//   * calling it DURING a move does not change where that move ends up
//   * calling it does not latch an error, however many times it is called
//
// SAFETY: MOSFETs stay OFF. Only reads and one small move; the COMMANDED
// position is what is measured, and the planner advances it regardless of the
// output stage, so nothing turns. No broadcasts and no alias changes: safe on
// the shared 35-motor rack.
// ---------------------------------------------------------------------------

// Every read-only command the library exposes that is safe to call on a shared
// rack. Deliberately excluded: detectDevices (bus-wide), getCommunicationStatistics
// (has a reset-counter argument, so it is NOT pure by design), captureHallSensorData
// (returns bulk data and takes time), readMultipurposeBuffer (contents depend on
// what last wrote to it).
enum { N_READERS = 10 };

static const char* readerName(int i) {
    switch (i) {
        case 0: return "get position";
        case 1: return "get status";
        case 2: return "get n queued items";
        case 3: return "get supply voltage";
        case 4: return "get temperature";
        case 5: return "get firmware version";
        case 6: return "get product info";
        case 7: return "get hall sensor position";
        case 8: return "get comprehensive position";
        case 9: return "ping";
    }
    return "?";
}

// Invoke reader i. Returns false if the call itself reported an error.
static bool callReader(Servomotor* m, int i) {
    switch (i) {
        case 0: m->getPositionRaw(); break;
        case 1: m->getStatus(); break;
        case 2: m->getNQueuedItems(); break;
        case 3: m->getSupplyVoltageRaw(); break;
        case 4: m->getTemperatureRaw(); break;
        case 5: m->getFirmwareVersion(); break;
        case 6: m->getProductInfo(); break;
        case 7: m->getHallSensorPositionRaw(); break;
        case 8: m->getComprehensivePositionRaw(); break;
        case 9: { uint8_t p[10]; for (int k = 0; k < 10; k++) p[k] = (uint8_t)(k * 7 + 1);
                  m->ping(p); break; }
    }
    return m->getError() == 0;
}

void tm_readonly_purity(void) {
    Serial.println("tm_readonly_purity: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. ON AN IDLE MACHINE, READING CHANGES NOTHING. Each command is called
    //    twenty times and the machine's observable state is compared before
    //    and after: position, queue depth, fatal code.
    // =====================================================================
    printf("\n--- 1. repeated reads do not disturb an idle machine ---\n");
    {
        for (int i = 0; i < N_READERS; i++) {
            tfhReset(m);
            int64_t pos0 = m->getPositionRaw();
            uint8_t q0 = m->getNQueuedItems();
            uint8_t f0 = tfhFatal(m);

            bool allSucceeded = true;
            for (int r = 0; r < 20; r++) {
                if (!callReader(m, i)) { allSucceeded = false; break; }
            }

            int64_t pos1 = m->getPositionRaw();
            uint8_t q1 = m->getNQueuedItems();
            uint8_t f1 = tfhFatal(m);

            bool unchanged = (pos0 == pos1) && (q0 == q1) && (f0 == f1) && (f1 == 0);
            if (!unchanged || !allSucceeded) {
                printf("   %-28s: ok=%d pos %lld->%lld q %u->%u fatal %u->%u\n",
                       readerName(i), (int)allSucceeded,
                       (long long)pos0, (long long)pos1,
                       (unsigned)q0, (unsigned)q1, (unsigned)f0, (unsigned)f1);
            }
            TEST_RESULT(std::string(readerName(i)) + " can be called twenty times in a row",
                        allSucceeded);
            TEST_RESULT(std::string(readerName(i)) + " leaves the idle machine unchanged",
                        unchanged);
        }
    }

    // =====================================================================
    // 2. READING DURING A MOVE DOES NOT CHANGE WHERE IT LANDS. The real test
    //    of purity: run the SAME move twice, once undisturbed and once while
    //    hammering the bus with the command under test, and require the same
    //    endpoint. A getter that perturbed the planner would show up as a
    //    difference here and nowhere else.
    // =====================================================================
    printf("\n--- 2. reading during a move does not change its outcome ---\n");
    {
        const int32_t dist = (int32_t)(TFH_CPR / 2);
        const uint32_t dur = 2 * TFH_TPS;

        // The undisturbed reference, measured a few times so the comparison is
        // against a known-stable baseline rather than a single sample.
        int64_t reference = 0;
        bool refOk = true;
        {
            int64_t lo = 0, hi = 0;
            for (int r = 0; r < 3; r++) {
                tfhFreshOpen(m);
                bool ok = false;
                int64_t moved = tfhMove(m, dist, dur, &ok);
                if (!ok) { refOk = false; break; }
                if (r == 0) { lo = hi = moved; }
                else { if (moved < lo) lo = moved; if (moved > hi) hi = moved; }
                reference = moved;
            }
            printf("   undisturbed reference: %lld counts (spread %lld)\n",
                   (long long)reference, (long long)(hi - lo));
            TEST_RESULT("the undisturbed reference move is repeatable",
                        refOk && (hi - lo) <= 2);
        }

        for (int i = 0; i < N_READERS; i++) {
            tfhFreshOpen(m);
            int64_t before = m->getPositionRaw();
            m->trapezoidMoveRaw(dist, dur);
            bool queued = (m->getError() == 0);

            // Hammer the bus with this one command for the whole move.
            int calls = 0; bool allSucceeded = true;
            uint32_t deadline = millis() + 2200;
            while (millis() < deadline) {
                if (!callReader(m, i)) { allSucceeded = false; break; }
                calls++;
            }
            bool drained = queued && tfhDrain(m, 10000);
            uint8_t fat = tfhFatal(m);
            int64_t moved = (drained && fat == 0) ? (m->getPositionRaw() - before) : 0;
            bool same = drained && (fat == 0) &&
                        (llabs((long long)(moved - reference)) <= 4);
            if (!same || !allSucceeded) {
                printf("   %-28s: %d calls, ok=%d fatal=%u moved %lld vs reference %lld\n",
                       readerName(i), calls, (int)allSucceeded,
                       (unsigned)fat, (long long)moved, (long long)reference);
            }
            TEST_RESULT(std::string(readerName(i)) +
                            " can be polled continuously during a move", allSucceeded);
            TEST_RESULT(std::string(readerName(i)) +
                            " does not change where the move lands", same);
        }
    }

    // =====================================================================
    // 3. ALL OF THEM AT ONCE, DURING A MOVE. Individually pure is not the
    //    same as jointly harmless: interleaving different command types is
    //    what a real monitoring tool does, and it exercises the packet
    //    handling in a way no single-command loop can.
    // =====================================================================
    printf("\n--- 3. all readers interleaved during a move ---\n");
    {
        const int32_t dist = (int32_t)(TFH_CPR / 2);
        const uint32_t dur = 2 * TFH_TPS;

        tfhFreshOpen(m);
        bool refOk = false;
        int64_t reference = tfhMove(m, dist, dur, &refOk);

        tfhFreshOpen(m);
        int64_t before = m->getPositionRaw();
        m->trapezoidMoveRaw(dist, dur);
        bool queued = (m->getError() == 0);
        int calls = 0; bool allSucceeded = true;
        uint32_t deadline = millis() + 2200;
        while (millis() < deadline) {
            if (!callReader(m, calls % N_READERS)) { allSucceeded = false; break; }
            calls++;
        }
        bool drained = queued && tfhDrain(m, 10000);
        uint8_t fat = tfhFatal(m);
        int64_t moved = (drained && fat == 0) ? (m->getPositionRaw() - before) : 0;
        printf("   %d interleaved calls during the move: fatal=%u moved %lld vs reference %lld\n",
               calls, (unsigned)fat, (long long)moved, (long long)reference);
        TEST_RESULT("every interleaved read succeeds during a move",
                    allSucceeded && calls > 20);
        TEST_RESULT("the move completes cleanly under continuous mixed polling",
                    refOk && drained && fat == 0);
        TEST_RESULT("and it lands where the undisturbed move did",
                    refOk && llabs((long long)(moved - reference)) <= 4);
    }

    // =====================================================================
    // 4. READS DO NOT CONSUME QUEUE SLOTS. A getter that accidentally queued
    //    something would be catastrophic and almost invisible: the queue would
    //    fill during monitoring and a later move would be refused for no
    //    apparent reason.
    // =====================================================================
    printf("\n--- 4. reads never occupy queue slots ---\n");
    {
        tfhFreshOpen(m);
        m->setMaximumVelocity(1.0f);
        m->setMaximumAcceleration(1.0f);          // long segments: depth is stable
        m->trapezoidMoveRaw((int32_t)(TFH_CPR / 200), 8 * TFH_TPS);
        uint8_t depthBefore = m->getNQueuedItems();

        bool allSucceeded = true;
        for (int r = 0; r < 40 && allSucceeded; r++) {
            allSucceeded = callReader(m, r % N_READERS);
        }
        uint8_t depthAfter = m->getNQueuedItems();
        printf("   queue depth %u -> 40 mixed reads -> %u\n",
               (unsigned)depthBefore, (unsigned)depthAfter);
        TEST_RESULT("all forty reads succeed with a move queued", allSucceeded);
        TEST_RESULT("the queue depth is unchanged by reading",
                    depthBefore > 0 && depthAfter == depthBefore);
        m->emergencyStop();
        delay(300);
    }

    // =====================================================================
    // 5. READS ARE STABLE ACROSS EACH OTHER. Calling a different command in
    //    between must not change what a getter returns. This catches a shared
    //    response buffer being clobbered -- the exact shape of the
    //    variable-length response bugs the Arduino library had.
    // =====================================================================
    printf("\n--- 5. one reader does not corrupt another's answer ---\n");
    {
        tfhReset(m);
        getProductInfoResponse pi0 = m->getProductInfo();
        bool ok0 = (m->getError() == 0);
        getFirmwareVersionResponse fv0 = m->getFirmwareVersion();
        bool ok1 = (m->getError() == 0);

        bool stable = ok0 && ok1;
        for (int r = 0; r < 15 && stable; r++) {
            // Interleave every other reader between the two identity reads.
            for (int i = 0; i < N_READERS; i++) {
                if (!callReader(m, i)) { stable = false; break; }
            }
            getProductInfoResponse pi = m->getProductInfo();
            if (m->getError() != 0 || pi.uniqueId != pi0.uniqueId) {
                stable = false;
                printf("   unique ID changed after interleaving: %016llX -> %016llX\n",
                       (unsigned long long)pi0.uniqueId, (unsigned long long)pi.uniqueId);
                break;
            }
            getFirmwareVersionResponse fv = m->getFirmwareVersion();
            if (m->getError() != 0 ||
                fv.firmwareVersion.major != fv0.firmwareVersion.major ||
                fv.firmwareVersion.minor != fv0.firmwareVersion.minor ||
                fv.firmwareVersion.patch != fv0.firmwareVersion.patch ||
                fv.firmwareVersion.dev   != fv0.firmwareVersion.dev) {
                stable = false;
                printf("   firmware version changed after interleaving\n");
                break;
            }
        }
        TEST_RESULT("identity reads stay identical across 15 rounds of interleaved reads",
                    stable);
    }

    // =====================================================================
    // 6. Still healthy afterwards.
    // =====================================================================
    printf("\n--- 6. still healthy afterwards ---\n");
    {
        tfhFreshOpen(m);
        bool ok = false;
        int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 4), TFH_TPS, &ok);
        TEST_RESULT("an ordinary move still works after the purity sweep",
                    ok && llabs((long long)(moved - TFH_CPR / 4)) < 200);
    }

    tfhReset(m);
}
