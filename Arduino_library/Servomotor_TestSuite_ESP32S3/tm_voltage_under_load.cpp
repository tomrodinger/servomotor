#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_voltage_under_load.cpp
//
// Arduino-library on-device test for "Get supply voltage" (cmd 38), exercised
// BOTH at rest and UNDER LOAD. Extends the intent of
// python_programs/test_get_supply_voltage.py with a motion phase.
//
// The point: the supply-voltage read must stay sane while the motor is
// actually spinning and drawing current. A collapsing supply (undersized PSU,
// bad wiring) or a decode corruption of the u16 reply under bus load would show
// up here as an out-of-range reading or a large deviation from the rest
// baseline.
//
// SPEC:
//   * Baseline supply voltage at rest is a plausible lab-supply value
//     (10..30 V), and the volts vs millivolts unit conversions agree.
//   * During a 1.5 rot/s constant-velocity move, five mid-motion samples each
//     stay in (10..30 V) and within 2.5 V of the rest baseline.
//
// Cmd 38 returns u16 decivolts (value/10 = volts); getSupplyVoltage() applies
// the selected VoltageUnit, getSupplyVoltageRaw() returns raw decivolts.
//
// SAFETY: the only motion is one constant-velocity move; it is ALWAYS followed
// (queued immediately) by a zero-velocity item so the queue never empties at
// nonzero velocity (avoids async fatal error 18). Displacement is bounded:
// 1.5 rot/s for ~3 s (well under the ~8.6 rot/s ceiling). One motor on the bus,
// addressed by unique ID, so all reads are collision-safe.
//
// One device on the bus, addressed by unique-ID (extended addressing); all
// methods are called WITHOUT a uniqueId argument.
// ---------------------------------------------------------------------------

static const float   MIN_VOLTS            = 10.0f;
static const float   MAX_VOLTS            = 30.0f;
static const float   UNIT_TOLERANCE_VOLTS = 0.2f;   // |volts vs millivolts/1000|
static const float   LOAD_DEVIATION_VOLTS = 2.5f;   // max drop/rise vs baseline under load
static const float   MOVE_VELOCITY_RPS    = 1.5f;   // rotations/second
static const float   MOVE_DURATION_S      = 3.0f;   // constant-velocity duration
static const int     N_LOAD_SAMPLES       = 5;
static const int     LOAD_SAMPLE_GAP_MS   = 400;    // 5 samples => ~2 s, inside the 3 s move

// Wait until the motion queue is empty, then settle.
static void waitForIdle(Servomotor* motor) {
    for (int i = 0; i < 1000; i++) {
        uint8_t n = motor->getNQueuedItems();
        if (n == 0) break;
        delay(10);
    }
    delay(150);
}

void tm_voltage_under_load(void) {
    Serial.println("test_ard_voltage_under_load: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // ---- Clean, known starting state ----
    motor->systemReset();
    delay(1500);  // reset; device does not respond during reset
    delay(200);   // let the supply ADC settle after reset

    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    motor->setVoltageUnit(VoltageUnit::VOLTS);

    // =======================================================================
    // PART A: baseline at rest
    // =======================================================================
    printf("\n--- Baseline supply voltage at rest ---\n");
    float baselineV = motor->getSupplyVoltage();
    checkMotorError(*motor, "getSupplyVoltage baseline");
    printf("Baseline supply voltage: %.2f V\n", baselineV);
    TEST_RESULT("Baseline supply voltage in plausible range (10..30 V)",
                baselineV >= MIN_VOLTS && baselineV <= MAX_VOLTS);

    // Volts vs millivolts unit conversion must agree for the same instant.
    motor->setVoltageUnit(VoltageUnit::MILLIVOLTS);
    float baselineMv = motor->getSupplyVoltage();
    checkMotorError(*motor, "getSupplyVoltage baseline mV");
    motor->setVoltageUnit(VoltageUnit::VOLTS);  // restore
    float mvAsVolts = baselineMv / 1000.0f;
    float unitDiff = std::fabs(mvAsVolts - baselineV);
    printf("Baseline in millivolts: %.1f mV -> %.3f V (diff vs volts = %.4f V)\n",
           baselineMv, mvAsVolts, unitDiff);
    TEST_RESULT("Baseline volts and millivolts reads agree",
                unitDiff <= UNIT_TOLERANCE_VOLTS);

    // =======================================================================
    // PART B: under load (1.5 rot/s constant-velocity move)
    // =======================================================================
    printf("\n--- Supply voltage under load (%.1f rot/s) ---\n", MOVE_VELOCITY_RPS);

    motor->enableMosfets();
    checkMotorError(*motor, "enableMosfets");
    delay(300);  // let the commutation-alignment transient settle

    // Queue the constant-velocity move, IMMEDIATELY followed by a zero-velocity
    // item so the queue never empties at nonzero velocity (no async fatal 18).
    motor->moveWithVelocity(MOVE_VELOCITY_RPS, MOVE_DURATION_S);
    checkMotorError(*motor, "moveWithVelocity load");
    motor->moveWithVelocity(0.0f, 0.1f);  // guaranteed zero-velocity tail
    checkMotorError(*motor, "moveWithVelocity stop");

    // Sample voltage mid-motion. All samples happen inside the 3 s move window.
    bool allInRange = true;
    bool allNearBaseline = true;
    float worstDeviation = 0.0f;
    for (int i = 0; i < N_LOAD_SAMPLES; i++) {
        delay(LOAD_SAMPLE_GAP_MS);
        float v = motor->getSupplyVoltage();
        checkMotorError(*motor, "getSupplyVoltage under load");
        float dev = std::fabs(v - baselineV);
        if (dev > worstDeviation) worstDeviation = dev;
        bool inRange = (v >= MIN_VOLTS && v <= MAX_VOLTS);
        bool nearBase = (dev <= LOAD_DEVIATION_VOLTS);
        if (!inRange) allInRange = false;
        if (!nearBase) allNearBaseline = false;
        printf("  load sample %d: %.2f V (dev from baseline = %.2f V) %s%s\n",
               i + 1, v, dev,
               inRange ? "" : "[OUT OF RANGE] ",
               nearBase ? "" : "[FAR FROM BASELINE]");
    }
    printf("Worst deviation from baseline under load: %.2f V (limit %.2f V)\n",
           worstDeviation, LOAD_DEVIATION_VOLTS);

    TEST_RESULT("All under-load samples in plausible range (10..30 V)", allInRange);
    TEST_RESULT("All under-load samples within 2.5 V of rest baseline",
                allNearBaseline);

    // Let the move finish (it ends at zero velocity by design of the queued tail).
    waitForIdle(motor);

    // ---- Clean up: motor at rest, MOSFETs disabled, reset. ----
    motor->disableMosfets();
    checkMotorError(*motor, "disableMosfets");
    motor->systemReset();
    delay(1500);
}
