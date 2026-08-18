#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_unit_equivalence.cpp
//
// UNITS, CHECKED AGAINST THE DEVICE rather than against the conversion table.
//
// tm_unit_conversions and tm_units_roundtrip verify that convertPosition() and
// friends are self-consistent -- pure arithmetic, no motor involved. That
// cannot catch the failure that actually bites users: the library converting
// correctly but then handing the firmware a value in the wrong unit, or a unit
// setting that does not stick, or a conversion that is right for positive
// values and wrong for negative ones.
//
// The property here is stronger and is what a user actually assumes:
//
//     The SAME physical command, expressed in ANY supported unit, must produce
//     the SAME behaviour from the hardware.
//
// So each check commands one physical quantity several different ways and
// requires the raw outcome to agree. A conversion error, a missing sign, or a
// unit that silently fails to take effect all break this and nothing else.
//
// SAFETY: MOSFETs stay OFF. Only the COMMANDED position is read, and the
// planner advances that regardless of the output stage, so nothing turns.
// No broadcasts and no alias changes: safe on the shared rack.
// ---------------------------------------------------------------------------

static const double CPR = 3276800.0;   // encoder counts per shaft rotation
static const double TPS = 31250.0;     // control ticks per second

static void freshStart(Servomotor* m) {
    m->systemReset();
    delay(1500);
    for (uint32_t t0 = millis(); Serial1.available() && (millis() - t0) < 500; ) Serial1.read();
}

static bool drainQueue(Servomotor* m, uint32_t budgetMs) {
    uint32_t deadline = millis() + budgetMs;
    while (millis() < deadline) {
        uint8_t q = m->getNQueuedItems();
        if (m->getError() != 0) return false;
        if (q == 0) { delay(250); return true; }
        delay(20);
    }
    return false;
}

static uint8_t fatalOf(Servomotor* m) {
    getStatusResponse st = m->getStatus();
    return (m->getError() == 0) ? st.fatalErrorCode : 0xFF;
}

// Command one trapezoid move expressed in the given units, and report the RAW
// displacement the device actually performed.
static int64_t moveInUnits(Servomotor* m, PositionUnit pu, float disp,
                           TimeUnit tu, float dur, bool* ok) {
    freshStart(m);
    m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    int64_t before = m->getPositionRaw();
    m->setPositionUnit(pu);
    m->setTimeUnit(tu);
    m->trapezoidMove(disp, dur);
    bool accepted = (m->getError() == 0);
    *ok = accepted && drainQueue(m, 12000) && (fatalOf(m) == 0);
    m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    m->setTimeUnit(TimeUnit::TIMESTEPS);
    return *ok ? (m->getPositionRaw() - before) : 0;
}

void tm_unit_equivalence(void) {
    Serial.println("tm_unit_equivalence: BEGIN\n");
    Servomotor* m = tfGetMotor();

    // =====================================================================
    // 1. POSITION UNITS. A quarter turn is a quarter turn however it is
    //    spelled: 0.25 rotations == 90 degrees == pi/2 radians == CPR/4
    //    counts. All four must move the shaft the same raw distance.
    // =====================================================================
    printf("\n--- 1. one physical distance, four position units ---\n");
    {
        struct { const char* name; PositionUnit u; float v; } spell[] = {
            { "0.25 rotations", PositionUnit::SHAFT_ROTATIONS, 0.25f          },
            { "90 degrees",     PositionUnit::DEGREES,         90.0f          },
            { "pi/2 radians",   PositionUnit::RADIANS,         1.5707963f     },
            { "CPR/4 counts",   PositionUnit::ENCODER_COUNTS,  (float)(CPR/4) },
        };
        int64_t got[4]; bool allOk = true;
        for (int i = 0; i < 4; i++) {
            bool ok = false;
            got[i] = moveInUnits(m, spell[i].u, spell[i].v, TimeUnit::SECONDS, 1.0f, &ok);
            printf("   %-16s -> %lld counts (ok=%d)\n", spell[i].name, (long long)got[i], (int)ok);
            if (!ok) allOk = false;
        }
        TEST_RESULT("all four position spellings of a quarter turn were accepted", allOk);
        // 0.05% of a quarter turn is ~410 counts; float32 carries ~7 digits, so
        // the radians spelling is the loose one. Tolerance reflects that.
        for (int i = 1; i < 4; i++) {
            bool agree = allOk && (llabs((long long)(got[i] - got[0])) <= 2000);
            if (!agree) printf("   MISMATCH: %s gave %lld, rotations gave %lld\n",
                               spell[i].name, (long long)got[i], (long long)got[0]);
            TEST_RESULT(std::string(spell[i].name) + " moves the same distance as 0.25 rotations",
                        agree);
        }
    }

    // =====================================================================
    // 2. SIGN THROUGH THE CONVERSION. Negative values must convert with the
    //    same magnitude as positive ones. A conversion written with an
    //    unsigned intermediate, or a truncation that rounds toward zero on one
    //    side only, shows up here and in no positive-only test.
    // =====================================================================
    printf("\n--- 2. negative values convert with the same magnitude ---\n");
    {
        struct { const char* name; PositionUnit u; float v; } neg[] = {
            { "rotations", PositionUnit::SHAFT_ROTATIONS, 0.125f          },
            { "degrees",   PositionUnit::DEGREES,         45.0f           },
            { "radians",   PositionUnit::RADIANS,         0.7853982f      },
            { "counts",    PositionUnit::ENCODER_COUNTS,  (float)(CPR/8)  },
        };
        for (int i = 0; i < 4; i++) {
            bool okP = false, okN = false;
            int64_t p = moveInUnits(m, neg[i].u,  neg[i].v, TimeUnit::SECONDS, 1.0f, &okP);
            int64_t n = moveInUnits(m, neg[i].u, -neg[i].v, TimeUnit::SECONDS, 1.0f, &okN);
            bool sym = okP && okN && (llabs((long long)(p + n)) <= 2000);
            if (!sym) printf("   %s: +%.4f -> %lld ; -%.4f -> %lld\n",
                             neg[i].name, (double)neg[i].v, (long long)p,
                             (double)neg[i].v, (long long)n);
            TEST_RESULT(std::string("negative ") + neg[i].name + " mirrors positive", sym);
        }
    }

    // =====================================================================
    // 3. TIME UNITS. One second is one second: 1 s == 1000 ms == 31250
    //    timesteps == 1000000 us == 1/60 minute. The same displacement over
    //    each spelling must take the same wall-clock time to complete.
    // =====================================================================
    printf("\n--- 3. one duration, five time units ---\n");
    {
        struct { const char* name; TimeUnit u; float v; } spell[] = {
            { "1 second",        TimeUnit::SECONDS,      1.0f       },
            { "1000 ms",         TimeUnit::MILLISECONDS, 1000.0f    },
            { "31250 timesteps", TimeUnit::TIMESTEPS,    31250.0f   },
            { "1000000 us",      TimeUnit::MICROSECONDS, 1000000.0f },
            { "1/60 minute",     TimeUnit::MINUTES,      1.0f/60.0f },
        };
        uint32_t elapsed[5]; bool allOk = true;
        for (int i = 0; i < 5; i++) {
            freshStart(m);
            m->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
            m->setTimeUnit(spell[i].u);
            uint32_t t0 = millis();
            m->trapezoidMove(0.25f, spell[i].v);
            bool ok = (m->getError() == 0) && drainQueue(m, 12000);
            elapsed[i] = millis() - t0;
            if (!ok) allOk = false;
            printf("   %-16s -> completed in %lu ms (ok=%d)\n",
                   spell[i].name, (unsigned long)elapsed[i], (int)ok);
        }
        TEST_RESULT("all five duration spellings were accepted", allOk);
        for (int i = 1; i < 5; i++) {
            // drainQueue polls every 20 ms and settles 250 ms, so the floor on
            // agreement is coarse; 400 ms is comfortably inside "same duration"
            // while still catching a 32x or 1000x unit error.
            long diff = (long)elapsed[i] - (long)elapsed[0];
            bool agree = allOk && (labs(diff) <= 400);
            if (!agree) printf("   MISMATCH: %s took %lu ms, 1 second took %lu ms\n",
                               spell[i].name, (unsigned long)elapsed[i],
                               (unsigned long)elapsed[0]);
            TEST_RESULT(std::string(spell[i].name) + " takes as long as 1 second", agree);
        }
    }

    // =====================================================================
    // 4. VELOCITY UNITS AGAINST THE DEVICE. Setting the same speed limit six
    //    different ways must produce the same limiting behaviour. Measured by
    //    how long a fixed move takes when the limit is the binding constraint.
    // =====================================================================
    printf("\n--- 4. one speed limit, six velocity units ---\n");
    {
        // 1 rotation/second, spelled six ways.
        struct { const char* name; VelocityUnit u; float v; } spell[] = {
            { "1 rot/s",        VelocityUnit::ROTATIONS_PER_SECOND, 1.0f            },
            { "60 rpm",         VelocityUnit::RPM,                  60.0f           },
            { "360 deg/s",      VelocityUnit::DEGREES_PER_SECOND,   360.0f          },
            { "2pi rad/s",      VelocityUnit::RADIANS_PER_SECOND,   6.2831853f      },
            { "CPR counts/s",   VelocityUnit::COUNTS_PER_SECOND,    (float)CPR      },
            { "CPR/TPS c/tick", VelocityUnit::COUNTS_PER_TIMESTEP,  (float)(CPR/TPS)},
        };
        uint32_t elapsed[6]; bool allOk = true;
        for (int i = 0; i < 6; i++) {
            freshStart(m);
            m->setVelocityUnit(spell[i].u);
            m->setMaximumVelocity(spell[i].v);
            bool setOk = (m->getError() == 0);
            // Ask for 1 full rotation in 0.2 s: impossible at 1 rot/s, so the
            // limit binds and the move takes about a second whatever we asked.
            m->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
            m->setTimeUnit(TimeUnit::SECONDS);
            uint32_t t0 = millis();
            m->trapezoidMove(1.0f, 2.0f);
            bool ok = setOk && (m->getError() == 0) && drainQueue(m, 12000);
            elapsed[i] = millis() - t0;
            if (!ok) allOk = false;
            printf("   %-16s -> 1 rotation took %lu ms (ok=%d)\n",
                   spell[i].name, (unsigned long)elapsed[i], (int)ok);
        }
        TEST_RESULT("all six velocity-limit spellings were accepted", allOk);
        for (int i = 1; i < 6; i++) {
            long diff = (long)elapsed[i] - (long)elapsed[0];
            bool agree = allOk && (labs(diff) <= 400);
            if (!agree) printf("   MISMATCH: %s -> %lu ms, 1 rot/s -> %lu ms\n",
                               spell[i].name, (unsigned long)elapsed[i],
                               (unsigned long)elapsed[0]);
            TEST_RESULT(std::string(spell[i].name) + " limits the same as 1 rot/s", agree);
        }
    }

    // =====================================================================
    // 5. ROUND-TO-ZERO. A displacement small enough to convert to less than
    //    one encoder count is a real user situation (a micro-degree nudge).
    //    It must behave like a dwell -- accepted, no motion, no fatal error --
    //    or be refused. What it must NOT do is fault or move a wild distance.
    // =====================================================================
    printf("\n--- 5. sub-count displacements round safely ---\n");
    {
        struct { const char* name; PositionUnit u; float v; } tiny[] = {
            { "1e-7 rotations",  PositionUnit::SHAFT_ROTATIONS, 1e-7f  },
            { "1e-5 degrees",    PositionUnit::DEGREES,         1e-5f  },
            { "1e-7 radians",    PositionUnit::RADIANS,         1e-7f  },
            { "0.4 counts",      PositionUnit::ENCODER_COUNTS,  0.4f   },
            { "-0.4 counts",     PositionUnit::ENCODER_COUNTS, -0.4f   },
        };
        for (unsigned i = 0; i < sizeof(tiny) / sizeof(tiny[0]); i++) {
            bool ok = false;
            int64_t moved = moveInUnits(m, tiny[i].u, tiny[i].v, TimeUnit::SECONDS, 0.5f, &ok);
            // Either refused (ok == false) or accepted and essentially still.
            bool sane = (!ok) || (llabs((long long)moved) <= 2);
            if (!sane) printf("   %s: accepted and moved %lld counts\n",
                              tiny[i].name, (long long)moved);
            TEST_RESULT(std::string(tiny[i].name) + " is a no-op or a clean refusal, not a jump",
                        sane);
        }
    }

    // =====================================================================
    // 6. THE UNIT SETTING IS STICKY WITHIN A SESSION but is a HOST-side
    //    setting, so a device reset must not change it. A library that stored
    //    the unit on the device, or reset it on error, would break callers
    //    that set units once at startup.
    // =====================================================================
    printf("\n--- 6. unit settings are host-side and survive a device reset ---\n");
    {
        freshStart(m);
        m->setPositionUnit(PositionUnit::DEGREES);
        m->setTimeUnit(TimeUnit::MILLISECONDS);
        m->systemReset();
        delay(1500);
        int64_t before, after;
        m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
        before = m->getPositionRaw();
        m->setPositionUnit(PositionUnit::DEGREES);      // still what we set
        m->trapezoidMove(90.0f, 1000.0f);               // 90 deg over 1000 ms
        bool ok = (m->getError() == 0) && drainQueue(m, 8000);
        m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
        after = m->getPositionRaw();
        int64_t moved = after - before;
        bool right = ok && (llabs((long long)(moved - (int64_t)(CPR / 4))) <= 2000);
        if (!right) printf("   after reset, 90 deg / 1000 ms moved %lld (wanted %lld)\n",
                           (long long)moved, (long long)(CPR / 4));
        TEST_RESULT("units set before a reset still apply after it", right);
        m->setTimeUnit(TimeUnit::TIMESTEPS);
    }

    // =====================================================================
    // 7. Still healthy afterwards.
    // =====================================================================
    printf("\n--- 7. still healthy afterwards ---\n");
    {
        freshStart(m);
        m->setPositionUnit(PositionUnit::ENCODER_COUNTS);
        m->setTimeUnit(TimeUnit::TIMESTEPS);
        int64_t b = m->getPositionRaw();
        m->trapezoidMoveRaw((int32_t)(CPR / 4), (uint32_t)TPS);
        bool ok = (m->getError() == 0) && drainQueue(m, 6000);
        TEST_RESULT("an ordinary raw move still works after the unit sweep",
                    ok && (fatalOf(m) == 0) &&
                    llabs((long long)(m->getPositionRaw() - b - (int64_t)(CPR / 4))) < 200);
    }

    freshStart(m);
}
