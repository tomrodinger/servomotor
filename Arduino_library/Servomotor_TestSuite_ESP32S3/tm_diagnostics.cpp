#include "tf_framework.h"

// ============================================================================
// tm_diagnostics
//
// Arduino-library mirror of three Python per-command tests:
//   * test_control_hall_sensor_statistics.py  (cmd 32 Control hall sensor statistics)
//   * test_get_hall_sensor_statistics.py      (cmd 33 Get hall sensor statistics)
//   * test_get_debug_values.py                (cmd 45 Get debug values)
//
// Commands covered: 32, 33, 45.
//
// Statistics gathering (cmd 32) is OFF by default after reset. control=1 resets
// the accumulators AND enables gathering; control=0 freezes; any other value is
// a no-op. Cmd 33 returns a fixed 10-field snapshot (3 max, 3 min, 3 sum, count)
// with min <= max and sum/n in [min, max]. Cmd 45 returns a fixed 30-field debug
// snapshot; at idle currentVelocity/nTimeSteps are 0 and it comes alive under
// motion.
//
// All response structs here are fixed-size (getHallSensorStatisticsResponse,
// getDebugValuesResponse), so there are NO known-bug expected-failures in this
// group.
// ============================================================================

static const unsigned long RESET_DELAY_MS       = 1500;
static const unsigned long SAMPLE_WINDOW_MS      = 300;  // ~32 kHz ISR -> ~9600 samples
static const unsigned long FREEZE_WINDOW_MS      = 200;
static const unsigned long ENABLE_SETTLE_MS      = 300;
static const unsigned long CLOSED_LOOP_TIMEOUT_MS = 6000;

static const uint32_t MIN_EXPECTED_SAMPLES = 100;   // loose floor: proves "growing > 0"
static const uint32_t HALL_ADC_SUM_MAX     = 4 * 4095; // 4 oversampled 12-bit ADC reads = 16380
static const int32_t  MEASURED_VELOCITY_NOISE_BAND = 200;
static const uint16_t CLOSED_LOOP_BIT      = 1 << 2;
static const int32_t  COUNTS_PER_ROTATION  = 3276800;

// Helper: fetch measurementCount, and assert the getter itself succeeded.
static uint32_t measurementCount(Servomotor* motor, const char* label) {
    getHallSensorStatisticsResponse s = motor->getHallSensorStatistics();
    if (motor->getError() != 0) {
        printf("  [%s] getHallSensorStatistics error=%d\n", label, motor->getError());
    }
    return s.measurementCount;
}

void tm_diagnostics(void) {
    Serial.println("tm_diagnostics: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // ------------------------------------------------------------------------
    // Clean state.
    // ------------------------------------------------------------------------
    motor->systemReset();
    delay(RESET_DELAY_MS);
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    motor->setTimeUnit(TimeUnit::SECONDS);

    char buf[160];

    // ========================================================================
    // CMD 32 — Control hall sensor statistics (control semantics)
    // ========================================================================
    Serial.println("\n--- CMD 32: Control hall sensor statistics ---");

    // 1) OFF by default after reset: count must NOT advance.
    uint32_t n_before = measurementCount(motor, "off-default before");
    delay(FREEZE_WINDOW_MS);
    uint32_t n_after = measurementCount(motor, "off-default after");
    snprintf(buf, sizeof(buf), "off-by-default count: %u -> %u (delta=%d)\n",
             n_before, n_after, (int)(n_after - n_before));
    Serial.print(buf);
    TEST_RESULT("Cmd32 stats OFF by default after reset", n_after == n_before);
    TEST_RESULT("Cmd32 no error reading stats after reset", motor->getError() == 0);

    // 2) control=1 (reset & enable): count must grow within the sample window.
    motor->controlHallSensorStatistics(1);
    TEST_RESULT("Cmd32 control=1 returns without error", motor->getError() == 0);
    delay(SAMPLE_WINDOW_MS);
    uint32_t n_running = measurementCount(motor, "after enable");
    snprintf(buf, sizeof(buf), "count after enable window: %u\n", n_running);
    Serial.print(buf);
    TEST_RESULT("Cmd32 control=1 starts gathering (count grows)",
                n_running >= MIN_EXPECTED_SAMPLES);

    // 3) control=0 (disable): count must freeze.
    motor->controlHallSensorStatistics(0);
    TEST_RESULT("Cmd32 control=0 returns without error", motor->getError() == 0);
    uint32_t n_frozen_a = measurementCount(motor, "frozen a");
    delay(FREEZE_WINDOW_MS);
    uint32_t n_frozen_b = measurementCount(motor, "frozen b");
    snprintf(buf, sizeof(buf), "count while disabled: %u -> %u\n", n_frozen_a, n_frozen_b);
    Serial.print(buf);
    TEST_RESULT("Cmd32 control=0 freezes gathering (count static)",
                n_frozen_b == n_frozen_a);

    // 4) no-op value (5) while disabled: state unchanged, count still frozen.
    motor->controlHallSensorStatistics(5);
    TEST_RESULT("Cmd32 no-op value (5) returns without error", motor->getError() == 0);
    delay(FREEZE_WINDOW_MS);
    uint32_t n_after_noop = measurementCount(motor, "after no-op");
    snprintf(buf, sizeof(buf), "count after no-op: %u\n", n_after_noop);
    Serial.print(buf);
    TEST_RESULT("Cmd32 no-op value leaves state alone (count stays frozen)",
                n_after_noop == n_frozen_b);

    // 5) control=1 again: reset drops count BELOW the previous frozen value, then grows.
    motor->controlHallSensorStatistics(1);
    uint32_t n_just_after_reset = measurementCount(motor, "just after re-enable");
    delay(SAMPLE_WINDOW_MS);
    uint32_t n_grown = measurementCount(motor, "grown after re-enable");
    snprintf(buf, sizeof(buf), "count after re-enable: %u -> %u (previous frozen was %u)\n",
             n_just_after_reset, n_grown, n_frozen_b);
    Serial.print(buf);
    TEST_RESULT("Cmd32 control=1 resets accumulators (count < previous frozen)",
                n_just_after_reset < n_frozen_b);
    TEST_RESULT("Cmd32 count grows again after re-enable",
                n_grown > n_just_after_reset);

    // No fatal error anywhere in the cmd 32 sequence.
    getStatusResponse st32 = motor->getStatus();
    snprintf(buf, sizeof(buf), "cmd32 status flags=0x%04X fatalError=%u\n",
             st32.statusFlags, st32.fatalErrorCode);
    Serial.print(buf);
    TEST_RESULT("Cmd32 no fatal error after sequence", st32.fatalErrorCode == 0);

    // ========================================================================
    // CMD 33 — Get hall sensor statistics (reply shape & contents)
    // ========================================================================
    Serial.println("\n--- CMD 33: Get hall sensor statistics ---");

    // Fresh reset so we can observe the boot-zero state.
    motor->systemReset();
    delay(RESET_DELAY_MS);

    // Post-reset (gathering OFF, never enabled this session): everything zero.
    getHallSensorStatisticsResponse s0 = motor->getHallSensorStatistics();
    TEST_RESULT("Cmd33 read after reset returns no error", motor->getError() == 0);
    snprintf(buf, sizeof(buf),
             "post-reset max=[%u,%u,%u] min=[%u,%u,%u] n=%u\n",
             s0.maxHall1, s0.maxHall2, s0.maxHall3,
             s0.minHall1, s0.minHall2, s0.minHall3, s0.measurementCount);
    Serial.print(buf);
    bool boot_zero = (s0.maxHall1 == 0 && s0.maxHall2 == 0 && s0.maxHall3 == 0 &&
                      s0.minHall1 == 0 && s0.minHall2 == 0 && s0.minHall3 == 0 &&
                      s0.sumHall1 == 0 && s0.sumHall2 == 0 && s0.sumHall3 == 0 &&
                      s0.measurementCount == 0);
    TEST_RESULT("Cmd33 post-reset stats are boot-zero (min=0 too, not 65535)", boot_zero);

    // Enable & sample -> a real observation.
    motor->controlHallSensorStatistics(1);
    delay(SAMPLE_WINDOW_MS);
    getHallSensorStatisticsResponse s1 = motor->getHallSensorStatistics();
    TEST_RESULT("Cmd33 running read returns no error", motor->getError() == 0);
    uint16_t min1[3] = {s1.minHall1, s1.minHall2, s1.minHall3};
    uint16_t max1[3] = {s1.maxHall1, s1.maxHall2, s1.maxHall3};
    uint64_t sum1[3] = {s1.sumHall1, s1.sumHall2, s1.sumHall3};
    uint32_t n1 = s1.measurementCount;
    snprintf(buf, sizeof(buf), "running1 min=[%u,%u,%u] max=[%u,%u,%u] n=%u\n",
             min1[0], min1[1], min1[2], max1[0], max1[1], max1[2], n1);
    Serial.print(buf);
    TEST_RESULT("Cmd33 measurementCount non-trivial after enable",
                n1 >= MIN_EXPECTED_SAMPLES);

    bool envelope_ok = true;
    bool avg_ok = true;
    for (int i = 0; i < 3; i++) {
        if (!(min1[i] <= max1[i] && max1[i] <= HALL_ADC_SUM_MAX)) envelope_ok = false;
        if (n1 > 0) {
            double avg = (double)sum1[i] / (double)n1;
            if (!(avg >= (double)min1[i] - 0.5 && avg <= (double)max1[i] + 0.5)) avg_ok = false;
        }
    }
    TEST_RESULT("Cmd33 each channel: 0 <= min <= max <= 16380", envelope_ok);
    TEST_RESULT("Cmd33 each channel: sum/n within [min, max]", avg_ok);

    // Second read, no reset: n grows, min non-increasing, max non-decreasing, sum non-decreasing.
    delay(FREEZE_WINDOW_MS);
    getHallSensorStatisticsResponse s2 = motor->getHallSensorStatistics();
    uint16_t min2[3] = {s2.minHall1, s2.minHall2, s2.minHall3};
    uint16_t max2[3] = {s2.maxHall1, s2.maxHall2, s2.maxHall3};
    uint64_t sum2[3] = {s2.sumHall1, s2.sumHall2, s2.sumHall3};
    uint32_t n2 = s2.measurementCount;
    snprintf(buf, sizeof(buf), "running2 min=[%u,%u,%u] max=[%u,%u,%u] n=%u\n",
             min2[0], min2[1], min2[2], max2[0], max2[1], max2[2], n2);
    Serial.print(buf);
    TEST_RESULT("Cmd33 measurementCount strictly grows across reads", n2 > n1);
    bool monotonic = true;
    for (int i = 0; i < 3; i++) {
        if (min2[i] > min1[i]) monotonic = false;  // min must not go up
        if (max2[i] < max1[i]) monotonic = false;  // max must not go down
        if (sum2[i] < sum1[i]) monotonic = false;  // sum must not go down
    }
    TEST_RESULT("Cmd33 accumulators monotonic (min down, max up, sum up)", monotonic);

    // control=1 (reset & enable): next read reflects fresh accumulators.
    motor->controlHallSensorStatistics(1);
    delay(SAMPLE_WINDOW_MS);
    getHallSensorStatisticsResponse s3 = motor->getHallSensorStatistics();
    uint16_t min3[3] = {s3.minHall1, s3.minHall2, s3.minHall3};
    uint16_t max3[3] = {s3.maxHall1, s3.maxHall2, s3.maxHall3};
    uint32_t n3 = s3.measurementCount;
    snprintf(buf, sizeof(buf), "post-reset-run min=[%u,%u,%u] max=[%u,%u,%u] n=%u (prev n=%u)\n",
             min3[0], min3[1], min3[2], max3[0], max3[1], max3[2], n3, n2);
    Serial.print(buf);
    TEST_RESULT("Cmd33 control=1 reset drops measurementCount below prior", n3 < n2);
    bool fresh_ok = true;
    for (int i = 0; i < 3; i++) {
        // min != 0 proves the 65535 seed was applied (and a real sample landed),
        // max > 0 proves a sample was recorded, min <= max is invariant.
        if (min3[i] == 0) fresh_ok = false;
        if (max3[i] == 0) fresh_ok = false;
        if (min3[i] > max3[i]) fresh_ok = false;
    }
    TEST_RESULT("Cmd33 fresh accumulators: min!=0, max>0, min<=max", fresh_ok);

    // Leave stats disabled and reset before the debug-values section.
    motor->controlHallSensorStatistics(0);
    motor->systemReset();
    delay(RESET_DELAY_MS);
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    motor->setTimeUnit(TimeUnit::SECONDS);

    // ========================================================================
    // CMD 45 — Get debug values (30-field reply, idle + under motion)
    // ========================================================================
    Serial.println("\n--- CMD 45: Get debug values ---");

    // Idle read.
    getDebugValuesResponse d0 = motor->getDebugValues();
    TEST_RESULT("Cmd45 idle read returns no error", motor->getError() == 0);
    snprintf(buf, sizeof(buf),
             "idle currentVelocity=%lld measuredVelocity=%d nTimeSteps=%u "
             "hall=[%u,%u,%u] phasesReversed=%u\n",
             (long long)d0.currentVelocity, (int)d0.measuredVelocity, d0.nTimeSteps,
             d0.hallSensor1Voltage, d0.hallSensor2Voltage, d0.hallSensor3Voltage,
             d0.motorPhasesReversed);
    Serial.print(buf);
    TEST_RESULT("Cmd45 idle currentVelocity == 0", d0.currentVelocity == 0);
    TEST_RESULT("Cmd45 idle |measuredVelocity| within noise band",
                std::llabs((long long)d0.measuredVelocity) <= MEASURED_VELOCITY_NOISE_BAND);
    TEST_RESULT("Cmd45 idle nTimeSteps == 0 (queue empty)", d0.nTimeSteps == 0);
    TEST_RESULT("Cmd45 motorPhasesReversed in {0,1}", d0.motorPhasesReversed <= 1);
    bool hall_v_ok =
        d0.hallSensor1Voltage <= HALL_ADC_SUM_MAX &&
        d0.hallSensor2Voltage <= HALL_ADC_SUM_MAX &&
        d0.hallSensor3Voltage <= HALL_ADC_SUM_MAX;
    TEST_RESULT("Cmd45 hall sensor voltages within [0, 16380]", hall_v_ok);

    // Second read: settings fields (max_acceleration, max_velocity) must be stable.
    getDebugValuesResponse d1 = motor->getDebugValues();
    TEST_RESULT("Cmd45 second read returns no error", motor->getError() == 0);
    TEST_RESULT("Cmd45 maxAcceleration stable on quiet device",
                d0.maxAcceleration == d1.maxAcceleration);
    TEST_RESULT("Cmd45 maxVelocity stable on quiet device",
                d0.maxVelocity == d1.maxVelocity);

    // Liveness under motion: enter closed loop, queue a small move, watch fields come alive.
    motor->enableMosfets();
    delay(ENABLE_SETTLE_MS);
    motor->goToClosedLoop();

    // Wait for the closed-loop status bit.
    bool closed_loop = false;
    unsigned long waited = 0;
    while (waited < CLOSED_LOOP_TIMEOUT_MS) {
        getStatusResponse st = motor->getStatus();
        if (st.fatalErrorCode == 0 && (st.statusFlags & CLOSED_LOOP_BIT)) {
            closed_loop = true;
            break;
        }
        delay(100);
        waited += 100;
    }
    TEST_RESULT("Cmd45 closed-loop engaged (motor calibrated)", closed_loop);

    bool saw_motion = false;
    bool saw_n_time_steps = false;
    if (closed_loop) {
        motor->zeroPosition();
        float target = (float)(COUNTS_PER_ROTATION / 8);  // 1/8 rotation, gentle & small
        motor->trapezoidMove(target, 1.0f);               // ends at rest by construction

        unsigned long move_waited = 0;
        while (move_waited < 1500) {
            getDebugValuesResponse dm = motor->getDebugValues();
            if (dm.currentVelocity != 0) saw_motion = true;
            if (dm.nTimeSteps > 0) saw_n_time_steps = true;
            if (motor->getNQueuedItems() == 0) break;
            delay(20);
            move_waited += 20;
        }
        // Drain any remaining queue.
        unsigned long drain = 0;
        while (motor->getNQueuedItems() > 0 && drain < 3000) {
            delay(10);
            drain += 10;
        }
        delay(300);
    }
    TEST_RESULT("Cmd45 currentVelocity went non-zero during move", saw_motion);
    TEST_RESULT("Cmd45 nTimeSteps went above 0 while move queued", saw_n_time_steps);

    // After the move drains, currentVelocity and nTimeSteps return to 0.
    getDebugValuesResponse d_end = motor->getDebugValues();
    snprintf(buf, sizeof(buf), "after-move currentVelocity=%lld nTimeSteps=%u measuredVelocity=%d\n",
             (long long)d_end.currentVelocity, d_end.nTimeSteps, (int)d_end.measuredVelocity);
    Serial.print(buf);
    if (closed_loop) {
        TEST_RESULT("Cmd45 currentVelocity back to 0 after move", d_end.currentVelocity == 0);
        TEST_RESULT("Cmd45 nTimeSteps back to 0 after move", d_end.nTimeSteps == 0);
        TEST_RESULT("Cmd45 measuredVelocity back within noise band after move",
                    std::llabs((long long)d_end.measuredVelocity) <= MEASURED_VELOCITY_NOISE_BAND);
    }

    // ------------------------------------------------------------------------
    // Return to zero and clean up (motion safety: trapezoid ends at rest).
    // ------------------------------------------------------------------------
    if (closed_loop) {
        motor->trapezoidMove(0.0f, 1.0f);
        unsigned long drain = 0;
        while (motor->getNQueuedItems() > 0 && drain < 3000) {
            delay(10);
            drain += 10;
        }
    }
    motor->disableMosfets();
    motor->systemReset();
    delay(RESET_DELAY_MS);
}
