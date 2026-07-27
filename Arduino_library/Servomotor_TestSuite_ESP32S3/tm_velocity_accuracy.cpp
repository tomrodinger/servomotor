#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_velocity_accuracy.cpp
//
// Physical, end-to-end validation of the velocity unit-conversion chain
// (cmd 30 Move with velocity). Rather than trust the commanded value, this
// module MEASURES the actual shaft rate during a constant-velocity coast and
// compares it to what was commanded:
//
//   * enable MOSFETs + goToClosedLoop (closed-loop is required for the motor
//     to actually track a commanded velocity accurately)
//   * command a constant 1.0 rot/s move for 2 s, with a zero-velocity
//     terminator queued immediately behind it (MOTION SAFETY: a velocity move
//     whose queue runs dry at non-zero velocity raises async fatal 18)
//   * DURING the coast, read getPositionRaw() (int64 raw counts, does not
//     disturb the selected unit) at two millis()-timestamped points >= 800 ms
//     apart, and compute measured rot/s = delta-counts / COUNTS / delta-t
//   * assert the measured rate is within 15% of the commanded 1.0 rot/s
//   * repeat at 2.0 rot/s with a +/-20% band
//
// Mirrors python_programs/test_move_with_velocity.py (rotations_per_second leg)
// but validates the RATE physically instead of only the final position.
//
// One motor on the bus, addressed by unique-ID (extended addressing) via
// tfGetMotor(), so every method is collision-safe on the 39-motor rack.
// Motion is gentle: 1.0 and 2.0 rot/s (<= 3 rot/s), continuous spin.
// ---------------------------------------------------------------------------

static const int64_t COUNTS_PER_ROTATION = COUNTS_PER_REVOLUTION;  // 3276800
static const int     NORMAL_RESET_DELAY_MS = 1500;

// Wait until the motion queue is empty (all queued moves consumed), then settle.
static void waitForIdle(Servomotor* motor) {
    for (int i = 0; i < 1000; i++) {           // up to ~10 s
        uint8_t n = motor->getNQueuedItems();
        if (n == 0) break;
        delay(10);
    }
    delay(200);  // small settle margin
}

// Run one constant-velocity leg, measuring the actual shaft rate during coast.
// Queues the velocity move plus its mandatory zero terminator, samples the raw
// encoder position twice > 800 ms apart, and returns measured rot/s. Drains the
// queue before returning so the motor is at rest. Returns false on a delta-t
// that is too small to trust (should not happen; guards the division).
static bool measureVelocityRotPerSec(Servomotor* motor, float commandedRotPerSec,
                                     float* measuredOut) {
    *measuredOut = 0.0f;

    // Queue the 2 s constant-velocity move, then IMMEDIATELY queue the zero
    // terminator so the queue never drains at non-zero velocity (fatal 18).
    motor->moveWithVelocity(commandedRotPerSec, 2.0f);
    checkMotorError(*motor, "moveWithVelocity run");
    motor->moveWithVelocity(0.0f, 0.1f);
    checkMotorError(*motor, "moveWithVelocity terminator");

    // Let the shaft reach steady coast, then take the two timestamped samples.
    // 350 + 1000 ms of sampling stays comfortably inside the 2 s coast window.
    delay(350);
    int64_t p1 = motor->getPositionRaw();
    unsigned long ts1 = millis();
    checkMotorError(*motor, "getPositionRaw sample1");

    delay(1000);
    int64_t p2 = motor->getPositionRaw();
    unsigned long ts2 = millis();
    checkMotorError(*motor, "getPositionRaw sample2");

    // Drain the terminator so the motor comes to rest before we return.
    waitForIdle(motor);

    double dt = (double)(ts2 - ts1) / 1000.0;
    if (dt < 0.8) {
        printf("  delta-t too small (%.3f s) -- cannot measure\n", dt);
        return false;
    }
    double measured = ((double)(p2 - p1) / (double)COUNTS_PER_ROTATION) / dt;
    *measuredOut = (float)measured;
    printf("  commanded %.2f rot/s: p1=%lld p2=%lld dt=%.3fs -> measured %.4f rot/s\n",
           commandedRotPerSec, (long long)p1, (long long)p2, dt, measured);
    return true;
}

void tm_velocity_accuracy(void) {
    Serial.println("test_ard_velocity_accuracy: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // ---- Clean, known starting state ----
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);  // no response during reset; don't check errors

    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);

    // ---- Enable + go to closed loop (needed for accurate velocity tracking) ----
    motor->enableMosfets();
    checkMotorError(*motor, "enableMosfets");
    delay(300);  // let the commutation-alignment transient settle

    motor->goToClosedLoop();
    bool closedLoopOk = (motor->getError() == 0);
    checkMotorError(*motor, "goToClosedLoop");
    delay(500);  // settle into closed-loop control before commanding motion
    TEST_RESULT("Entered closed loop without error", closedLoopOk);

    // =====================================================================
    // Leg 1: commanded 1.0 rot/s, measured must be within 15%.
    // =====================================================================
    printf("\n=== Leg 1: constant 1.0 rot/s ===\n");
    float measured1 = 0.0f;
    bool ok1 = measureVelocityRotPerSec(motor, 1.0f, &measured1);
    TEST_RESULT("1.0 rot/s: velocity measurement obtained", ok1);
    TEST_RESULT("Measured velocity within 15% of commanded 1.0 rot/s",
                ok1 && (std::fabs(measured1 - 1.0f) <= 0.15f));

    delay(300);  // brief rest between legs

    // =====================================================================
    // Leg 2: commanded 2.0 rot/s, measured must be within 20%.
    // =====================================================================
    printf("\n=== Leg 2: constant 2.0 rot/s ===\n");
    float measured2 = 0.0f;
    bool ok2 = measureVelocityRotPerSec(motor, 2.0f, &measured2);
    TEST_RESULT("2.0 rot/s: velocity measurement obtained", ok2);
    TEST_RESULT("Measured velocity within 20% of commanded 2.0 rot/s",
                ok2 && (std::fabs(measured2 - 2.0f) <= 0.40f));

    // The two legs together prove the conversion scales linearly: doubling the
    // command roughly doubles the measured rate.
    TEST_RESULT("2.0 rot/s leg is faster than 1.0 rot/s leg (monotonic)",
                ok1 && ok2 && (measured2 > measured1 * 1.4f));

    // ---- Clean up: motor is already at rest; disable and reset. ----
    printf("\nCleaning up...\n");
    motor->disableMosfets();
    checkMotorError(*motor, "disableMosfets");
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);
}
