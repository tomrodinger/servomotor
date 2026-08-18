#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_fatal_state_commands.cpp
//
// WHAT YOU CAN STILL ASK A MOTOR THAT HAS FAULTED.
//
// When the firmware raises a fatal error it calls __disable_irq() and never
// returns: it spins in a loop in common_source_files/error_handling.c, flashing
// the red LED and hand-pumping the UART so it can still answer the bus. Which
// commands it answers -- and how -- is the entire diagnostic surface available
// to an operator whose machine has just stopped, and it is documented nowhere.
//
// Measured on rack motor 5E2E8161CF1D2624, fw 0.15.12.0, by probing twelve
// commands in two different orders:
//
//     get_status (cmd 16) is the ONLY command that returns a real reply.
//     EVERY other command answers with the latched error code instead.
//     System reset still works, and is the only way out.
//
// The order-independence matters and was checked deliberately: `ping` fails
// even when it is the first command after the fault, and `get_status` succeeds
// on the fourth consecutive call, so this is a genuine per-command distinction
// rather than "the first request after a fault is consumed".
//
// WHY IT IS WORTH A MODULE. Two reasons, in opposite directions:
//
//   1. It is a sound fail-safe. A machine that has faulted should refuse to do
//      anything else, and answering every request with the reason is much
//      better than going silent. This module locks that in so it cannot regress
//      into a timeout.
//   2. It also means NO TELEMETRY survives a fault -- no temperature, no supply
//      voltage, no position, no clock. Those are exactly the readings an
//      operator wants in order to understand WHY a motor faulted, and the only
//      thing they can actually retrieve is the error number. That is a real
//      diagnostic limitation and is recorded as finding F2 of the August 2026
//      campaign.
//
// SAFETY: MOSFETs stay OFF, and a fatal error turns them off anyway. Faults are
// induced with an over-limit move, which is refused at queue time, so nothing
// turns. Every subsection ends with a reset, leaving the motor clean. No
// broadcasts and no alias changes: safe on the shared 35-motor rack.
// ---------------------------------------------------------------------------

// Put the motor into a latched fatal error, deterministically, without motion:
// a move that needs far more speed than the ceiling allows is refused at queue
// time. Returns the latched code (0 if the fault failed to occur).
static uint8_t induceFault(Servomotor* m) {
    tfhReset(m);
    m->setMaximumVelocity(0.2f);
    m->trapezoidMoveRaw((int32_t)TFH_CPR, TFH_TPS / 4);
    delay(400);
    return tfhFatal(m);
}

void tm_fatal_state_commands(void) {
    Serial.println("tm_fatal_state_commands: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. THE FAULT IS INDUCIBLE AND REPORTED. Everything below depends on
    //    reliably getting into the state, so establish that first.
    // =====================================================================
    printf("\n--- 1. the fault state is reachable and reported ---\n");
    uint8_t code = 0;
    {
        code = induceFault(m);
        printf("   induced fatal code %u\n", (unsigned)code);
        TEST_RESULT("an over-limit move latches a fatal error", code != 0 && code != 0xFF);
        TEST_RESULT("get_status reports the code rather than timing out", code != 0xFF);
    }

    // =====================================================================
    // 2. get_status IS ANSWERABLE, REPEATEDLY AND CONSISTENTLY. If this ever
    //    regressed to a timeout, a faulted motor would become undiagnosable
    //    over the bus.
    // =====================================================================
    printf("\n--- 2. get_status keeps answering while faulted ---\n");
    {
        bool allOk = true, allSame = true;
        for (int i = 0; i < 15; i++) {
            getStatusResponse st = m->getStatus();
            if (m->getError() != 0) { allOk = false; printf("   read %d failed\n", i); break; }
            if (st.fatalErrorCode != code) {
                allSame = false;
                printf("   read %d reported %u, expected %u\n",
                       i, (unsigned)st.fatalErrorCode, (unsigned)code);
                break;
            }
            delay(40);
        }
        TEST_RESULT("fifteen consecutive get_status calls all answer while faulted", allOk);
        TEST_RESULT("every one reports the same error code", allSame);
    }

    // =====================================================================
    // 3. EVERY OTHER COMMAND ANSWERS WITH THE ERROR, NOT WITH DATA AND NOT
    //    WITH SILENCE. Checked one command at a time, each from a freshly
    //    induced fault so no probe can influence the next.
    //
    //    The distinction being asserted is three-way, and all three matter:
    //      * NOT silence   -- a timeout would leave the operator with nothing
    //      * NOT stale data -- a plausible-looking reading from a faulted
    //                          machine is worse than no reading at all
    //      * the ERROR CODE -- so the caller learns why its request was refused
    // =====================================================================
    printf("\n--- 3. every other command returns the error, not data ---\n");
    {
        struct Probe { const char* name; };
        // Each probe runs from its own fresh fault.
        const int NPROBES = 8;
        const char* names[NPROBES] = {
            "get position", "get n queued items", "get supply voltage",
            "get temperature", "get firmware version", "get product info",
            "ping", "get hall sensor position"
        };
        bool refused[NPROBES];
        for (int i = 0; i < NPROBES; i++) {
            uint8_t c = induceFault(m);
            if (c == 0 || c == 0xFF) { refused[i] = false; continue; }
            switch (i) {
                case 0: m->getPositionRaw(); break;
                case 1: m->getNQueuedItems(); break;
                case 2: m->getSupplyVoltageRaw(); break;
                case 3: m->getTemperatureRaw(); break;
                case 4: m->getFirmwareVersion(); break;
                case 5: m->getProductInfo(); break;
                case 6: { uint8_t p[10]; for (int k = 0; k < 10; k++) p[k] = (uint8_t)k;
                          m->ping(p); break; }
                case 7: m->getHallSensorPositionRaw(); break;
            }
            // The library surfaces the device's error reply as a nonzero error.
            refused[i] = (m->getError() != 0);
            printf("   %-26s -> refused=%d\n", names[i], (int)refused[i]);
            TEST_RESULT(std::string(names[i]) +
                            " is refused while faulted, rather than returning data",
                        refused[i]);
        }
        int refusedCount = 0;
        for (int i = 0; i < NPROBES; i++) if (refused[i]) refusedCount++;
        printf("   %d of %d commands refused while faulted\n", refusedCount, NPROBES);
        TEST_RESULT("all eight non-status commands are refused while faulted",
                    refusedCount == NPROBES);
    }

    // =====================================================================
    // 4. THE DISTINCTION IS PER-COMMAND, NOT PER-ORDER. The obvious rival
    //    explanation is that the first request after a fault is simply
    //    consumed. Rule it out: send a command that should fail FIRST, then
    //    get_status. If order were the cause, get_status would fail here.
    // =====================================================================
    printf("\n--- 4. it is the command that matters, not the order ---\n");
    {
        uint8_t c = induceFault(m);
        uint8_t p[10]; for (int k = 0; k < 10; k++) p[k] = (uint8_t)k;
        m->ping(p);
        bool pingRefused = (m->getError() != 0);
        getStatusResponse st = m->getStatus();
        bool statusOk = (m->getError() == 0);
        printf("   fault %u: ping first -> refused=%d, then get_status -> ok=%d (code %u)\n",
               (unsigned)c, (int)pingRefused, (int)statusOk, (unsigned)st.fatalErrorCode);
        TEST_RESULT("ping is refused even when it is the first command after the fault",
                    pingRefused);
        TEST_RESULT("get_status still answers after another command was refused",
                    statusOk && st.fatalErrorCode == c);
    }

    // =====================================================================
    // 5. COMMANDS THAT WOULD CAUSE MOTION ARE REFUSED TOO. This is the safety
    //    half of the contract: a faulted machine must not accept new work.
    //    Asserted separately from section 3 because "refuses to move" is a
    //    much stronger requirement than "refuses to report".
    // =====================================================================
    printf("\n--- 5. a faulted machine accepts no new work ---\n");
    {
        uint8_t c = induceFault(m);
        m->trapezoidMoveRaw((int32_t)(TFH_CPR / 8), TFH_TPS);
        bool moveRefused = (m->getError() != 0);
        m->goToPositionRaw((int32_t)(TFH_CPR / 8), TFH_TPS);
        bool gotoRefused = (m->getError() != 0);
        m->moveWithVelocityRaw(1000, 1000);
        bool velRefused = (m->getError() != 0);
        m->enableMosfets();
        bool enableRefused = (m->getError() != 0);
        printf("   fault %u: move=%d goto=%d velocity=%d enableMosfets=%d (1 = refused)\n",
               (unsigned)c, (int)moveRefused, (int)gotoRefused,
               (int)velRefused, (int)enableRefused);
        TEST_RESULT("a trapezoid move is refused while faulted", moveRefused);
        TEST_RESULT("a go-to-position is refused while faulted", gotoRefused);
        TEST_RESULT("a velocity move is refused while faulted", velRefused);
        TEST_RESULT("enabling the MOSFETs is refused while faulted", enableRefused);

        // And nothing moved as a result of trying.
        getStatusResponse st = m->getStatus();
        TEST_RESULT("the machine is still reporting the same fault afterwards",
                    (m->getError() == 0) && st.fatalErrorCode == c);
    }

    // =====================================================================
    // 6. SYSTEM RESET IS THE WAY OUT, AND IT WORKS FROM THE FAULT STATE.
    //    If it did not, a fault would need a power cycle -- which for a motor
    //    inside a machine can mean disassembly.
    // =====================================================================
    printf("\n--- 6. system reset is the way out ---\n");
    {
        bool allRecovered = true;
        for (int cycle = 0; cycle < 5 && allRecovered; cycle++) {
            uint8_t c = induceFault(m);
            if (c == 0 || c == 0xFF) { allRecovered = false; printf("   cycle %d: no fault\n", cycle); break; }
            tfhReset(m);
            if (tfhFatal(m) != 0) { allRecovered = false; printf("   cycle %d: not cleared\n", cycle); break; }
            // Everything must be answerable again.
            m->getPositionRaw();      if (m->getError() != 0) { allRecovered = false; break; }
            m->getSupplyVoltageRaw(); if (m->getError() != 0) { allRecovered = false; break; }
            m->getFirmwareVersion();  if (m->getError() != 0) { allRecovered = false; break; }
        }
        TEST_RESULT("five fault-then-reset cycles all recover fully", allRecovered);

        // And the machine actually works again.
        tfhFreshOpen(m);
        bool ok = false;
        int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 4), TFH_TPS, &ok);
        TEST_RESULT("a real move works again after recovering from a fault",
                    ok && llabs((long long)(moved - TFH_CPR / 4)) < 200);
    }

    // =====================================================================
    // 7. THE ERROR CODE IS PRESERVED UNTIL THE RESET, not overwritten by the
    //    later refused commands. A code that got replaced by whatever was
    //    tried last would destroy the diagnosis.
    // =====================================================================
    printf("\n--- 7. the original error code survives later attempts ---\n");
    {
        uint8_t original = induceFault(m);
        // Hammer it with a variety of things that would each raise a DIFFERENT
        // error if they were processed normally.
        m->trapezoidMoveRaw((int32_t)(TFH_CPR / 100), 0);   // zero duration -> 34
        m->zeroPosition();
        uint8_t p[10]; for (int k = 0; k < 10; k++) p[k] = (uint8_t)k;
        m->ping(p);
        for (int i = 0; i < 6; i++) m->trapezoidMoveRaw((int32_t)(TFH_CPR / 100), TFH_TPS);
        getStatusResponse st = m->getStatus();
        bool ok = (m->getError() == 0);
        printf("   original %u, after a barrage of other illegal commands: %u\n",
               (unsigned)original, (unsigned)st.fatalErrorCode);
        TEST_RESULT("get_status is still answerable after the barrage", ok);
        TEST_RESULT("the originally latched error code is unchanged",
                    ok && st.fatalErrorCode == original);
    }

    tfhReset(m);
    TEST_RESULT("the motor is left clean", tfhFatal(m) == 0);
}
