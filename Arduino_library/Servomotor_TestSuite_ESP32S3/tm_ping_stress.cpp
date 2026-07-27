#include "tf_framework.h"

// Ping robustness stress test (cmd 31).
//
// Mirrors the intent of python_programs/test_ping.py: the Ping command echoes a
// fixed 10-byte payload back unchanged. Here we hammer it with 100 pings using a
// wide spread of byte patterns and require EVERY echo to match exactly (memcmp)
// with no communication error. We also report the round-trip min/max in ms.
//
// Ping is a pure request/response with no motion and no motor state change, so
// this module never enables the MOSFETs. It is addressed by unique ID via
// tfGetMotor(), so it is collision-safe on the 39-motor rack (no broadcast).
//
// Payload patterns (cycled across the 100 pings):
//   0: all 0x00
//   1: all 0xFF
//   2: 0x55 / 0xAA alternating
//   3: incrementing bytes derived from the iteration index
//   4: pseudorandom from a deterministic seeded LCG (repeatable run-to-run)

static const int PING_COUNT = 100;

// Deterministic 32-bit LCG (glibc constants) so the "random" payloads are the
// same on every run and on both host and device.
static uint32_t lcg_state;
static uint8_t nextRandomByte() {
    lcg_state = lcg_state * 1103515245u + 12345u;
    return (uint8_t)((lcg_state >> 16) & 0xFF);
}

// Fill a 10-byte payload for ping iteration i using pattern (i % 5).
static void buildPayload(int i, uint8_t out[10]) {
    int pattern = i % 5;
    switch (pattern) {
        case 0:
            memset(out, 0x00, 10);
            break;
        case 1:
            memset(out, 0xFF, 10);
            break;
        case 2:
            for (int b = 0; b < 10; b++) out[b] = (b & 1) ? 0xAA : 0x55;
            break;
        case 3:
            for (int b = 0; b < 10; b++) out[b] = (uint8_t)(i + b);
            break;
        default:  // case 4: pseudorandom
            for (int b = 0; b < 10; b++) out[b] = nextRandomByte();
            break;
    }
}

void tm_ping_stress(void) {
    Serial.println("test_ard_ping_stress: BEGIN\n");

    // Re-initialize mutable file-scope state (host runs modules back-to-back).
    lcg_state = 0xC0FFEEu;

    Servomotor* motor = tfGetMotor();

    // ---- Clean known state ----
    motor->systemReset();
    delay(1500);  // reset; motor does not respond during reset

    // Ping is unit-independent, but set units explicitly for a defined state.
    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);

    int echoed = 0;         // pings whose echo matched exactly with no comm error
    int mismatches = 0;     // echo differed from what was sent
    int commErrors = 0;     // motor reported a communication/protocol error
    unsigned long minMs = 0xFFFFFFFFul;
    unsigned long maxMs = 0;

    for (int i = 0; i < PING_COUNT; i++) {
        uint8_t sent[10];
        buildPayload(i, sent);

        unsigned long t0 = millis();
        pingResponse resp = motor->ping(sent);
        unsigned long dt = millis() - t0;

        if (dt < minMs) minMs = dt;
        if (dt > maxMs) maxMs = dt;

        int err = motor->getError();
        bool match = (memcmp(resp.responsePayload, sent, 10) == 0);

        if (err != 0) {
            commErrors++;
            printf("Ping %d: comm error code %d\n", i, err);
        } else if (!match) {
            mismatches++;
            printf("Ping %d: echo MISMATCH\n  sent: ", i);
            for (int b = 0; b < 10; b++) printf("%02X ", sent[b]);
            printf("\n  recv: ");
            for (int b = 0; b < 10; b++) printf("%02X ", resp.responsePayload[b]);
            printf("\n");
        } else {
            echoed++;
        }
    }

    printf("\nPing stress: %d/%d echoed exactly (%d mismatches, %d comm errors)\n",
           echoed, PING_COUNT, mismatches, commErrors);
    if (maxMs >= minMs && minMs != 0xFFFFFFFFul) {
        printf("Round-trip: min %lu ms, max %lu ms\n",
               (unsigned long)minMs, (unsigned long)maxMs);
    }

    TEST_RESULT("All 100 Pings Echoed Exactly", echoed == PING_COUNT);
    TEST_RESULT("No Ping Echo Mismatches", mismatches == 0);
    TEST_RESULT("No Ping Communication Errors", commErrors == 0);

    // ---- Clean up: no motion happened; leave the device reset. ----
    motor->systemReset();
    delay(1500);
}
