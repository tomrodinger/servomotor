#include "tf_framework.h"

// Per-command telemetry test for the Arduino library. Mirrors the Python tests:
//   test_get_position.py            (cmd 34 Get position)
//   test_get_hall_sensor_position.py(cmd 15 Get hall sensor position)
//   test_get_supply_voltage.py      (cmd 38 Get supply voltage)
//   test_get_n_queued_items.py      (cmd 11 Get n queued items)
//
// The motor is addressed by unique ID via tfGetMotor(),
// so command methods are called WITHOUT a uniqueId argument.

static const int64_t COUNTS_PER_ROTATION = COUNTS_PER_REVOLUTION;  // 3,276,800

void tm_telemetry(void) {
    Servomotor* motor = tfGetMotor();
    char buf[160];

    // Clean, known state. Don't check errors right after reset (motor doesn't reply).
    motor->systemReset();
    delay(1500);

    // ========================================================================
    // SECTION A — cmd 34 Get position
    // Read ~0 after zero; same value consistent across units; tracks a move.
    // ========================================================================
    printf("\n=== Section A: Get position (cmd 34) ===\n");
    motor->setTimeUnit(TimeUnit::SECONDS);

    motor->enableMosfets();
    checkMotorError(*motor, "enableMosfets");
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition");
    delay(100);

    // Read the same position in 4 units and confirm they agree (in counts).
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    float pos_counts = motor->getPosition();
    checkMotorError(*motor, "getPosition(counts)");
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    float pos_rot = motor->getPosition();
    motor->setPositionUnit(PositionUnit::DEGREES);
    float pos_deg = motor->getPosition();
    motor->setPositionUnit(PositionUnit::RADIANS);
    float pos_rad = motor->getPosition();

    float rot_as_counts = pos_rot * (float)COUNTS_PER_ROTATION;
    float deg_as_counts = (pos_deg / 360.0f) * (float)COUNTS_PER_ROTATION;
    float rad_as_counts = (pos_rad / (2.0f * (float)M_PI)) * (float)COUNTS_PER_ROTATION;
    float spread_zero = fmaxf(fmaxf(fabsf(pos_counts - rot_as_counts), fabsf(pos_counts - deg_as_counts)),
                              fabsf(pos_counts - rad_as_counts));
    snprintf(buf, sizeof(buf), "  at zero: counts=%.1f rot=%.6f deg=%.4f rad=%.4f (unit spread=%.1f)\n",
             pos_counts, pos_rot, pos_deg, pos_rad, spread_zero);
    printf("%s", buf);

    // Desired position lands essentially exactly on target; tolerance tight.
    TEST_RESULT("Position at zero is ~0 (cmd 34)", fabsf(pos_counts) < 50.0f);
    TEST_RESULT("Position units consistent at zero (cmd 34)", spread_zero < 2.0f);

    // Move a quarter turn and confirm the reported position tracks the target.
    int64_t target_counts = COUNTS_PER_ROTATION / 4;
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    printf("  Moving to %lld counts (1/4 rotation)...\n", (long long)target_counts);
    motor->trapezoidMove((float)target_counts, 1.0f);
    checkMotorError(*motor, "trapezoidMove(target)");
    while (motor->getNQueuedItems() > 0) {
        delay(10);
    }
    delay(150);

    float pos_after = motor->getPosition();
    checkMotorError(*motor, "getPosition(after move)");
    snprintf(buf, sizeof(buf), "  after 1/4-turn move: counts=%.1f (target=%lld)\n",
             pos_after, (long long)target_counts);
    printf("%s", buf);
    TEST_RESULT("Position tracks 1/4-turn target (cmd 34)",
                fabsf(pos_after - (float)target_counts) < 50.0f);

    // getPositionRaw should agree with the float read (both in counts here).
    int64_t pos_raw = motor->getPositionRaw();
    checkMotorError(*motor, "getPositionRaw");
    printf("  raw position = %lld counts\n", (long long)pos_raw);
    TEST_RESULT("getPositionRaw agrees with getPosition (cmd 34)",
                llabs(pos_raw - (int64_t)llroundf(pos_after)) < 50);

    // Return to zero (trapezoid move ends at rest — motion-safe).
    motor->trapezoidMove(0.0f, 1.0f);
    checkMotorError(*motor, "trapezoidMove(return)");
    while (motor->getNQueuedItems() > 0) {
        delay(10);
    }
    delay(100);

    // ========================================================================
    // SECTION B — cmd 15 Get hall sensor position
    // Read ~0 after zero; units consistent; hall moves and tracks desired.
    // ========================================================================
    printf("\n=== Section B: Get hall sensor position (cmd 15) ===\n");
    const int64_t ZERO_TOL = 500;            // hall jitter near rest
    const int64_t TRACK_TOL = 20000;         // hall vs desired after open-loop 1/8 turn
    const int64_t MIN_MOTION = COUNTS_PER_ROTATION / 20;
    const int64_t UNIT_SPREAD_TOL = 1000;

    // Motor is already reset earlier; re-establish a clean open-loop zero here.
    motor->systemReset();
    delay(1500);
    motor->enableMosfets();
    checkMotorError(*motor, "enableMosfets(B)");
    delay(300);  // commutation-alignment settle before zeroing
    motor->zeroPosition();
    checkMotorError(*motor, "zeroPosition(B)");
    delay(100);

    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    float hall_counts = motor->getHallSensorPosition();
    checkMotorError(*motor, "getHallSensorPosition(counts)");
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    float hall_rot = motor->getHallSensorPosition();
    motor->setPositionUnit(PositionUnit::DEGREES);
    float hall_deg = motor->getHallSensorPosition();
    motor->setPositionUnit(PositionUnit::RADIANS);
    float hall_rad = motor->getHallSensorPosition();
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);

    float hrot_c = hall_rot * (float)COUNTS_PER_ROTATION;
    float hdeg_c = (hall_deg / 360.0f) * (float)COUNTS_PER_ROTATION;
    float hrad_c = (hall_rad / (2.0f * (float)M_PI)) * (float)COUNTS_PER_ROTATION;
    float hspread = fmaxf(fmaxf(fabsf(hall_counts - hrot_c), fabsf(hall_counts - hdeg_c)),
                          fabsf(hall_counts - hrad_c));
    snprintf(buf, sizeof(buf), "  hall at zero: counts=%.1f rot=%.6f deg=%.4f rad=%.4f (spread=%.1f)\n",
             hall_counts, hall_rot, hall_deg, hall_rad, hspread);
    printf("%s", buf);

    TEST_RESULT("Hall position at zero is ~0 (cmd 15)",
                fabsf(hall_counts) < (float)ZERO_TOL);
    TEST_RESULT("Hall position units consistent at zero (cmd 15)",
                hspread < (float)UNIT_SPREAD_TOL);

    int64_t hall_at_zero = (int64_t)llroundf(hall_counts);

    // Open-loop move 1/8 turn; hall must move substantially and track desired.
    int64_t target_hall = COUNTS_PER_ROTATION / 8;
    printf("  Open-loop move to %lld counts (1/8 rotation)...\n", (long long)target_hall);
    motor->trapezoidMove((float)target_hall, 1.0f);
    checkMotorError(*motor, "trapezoidMove(hall target)");
    while (motor->getNQueuedItems() > 0) {
        delay(10);
    }
    delay(300);  // physical settle

    float desired = motor->getPosition();
    checkMotorError(*motor, "getPosition(hall section)");
    float hall = motor->getHallSensorPosition();
    checkMotorError(*motor, "getHallSensorPosition(after move)");
    int64_t tracking_err = llabs((int64_t)llroundf(hall) - (int64_t)llroundf(desired));
    int64_t motion = llabs((int64_t)llroundf(hall) - hall_at_zero);
    snprintf(buf, sizeof(buf), "  desired=%.1f hall=%.1f (track err=%lld, motion=%lld)\n",
             desired, hall, (long long)tracking_err, (long long)motion);
    printf("%s", buf);

    TEST_RESULT("Hall position moved substantially (cmd 15)", motion >= MIN_MOTION);
    TEST_RESULT("Hall tracks desired after 1/8-turn move (cmd 15)", tracking_err <= TRACK_TOL);

    // getHallSensorPositionRaw should agree with the float counts read.
    int64_t hall_raw = motor->getHallSensorPositionRaw();
    checkMotorError(*motor, "getHallSensorPositionRaw");
    printf("  hall raw = %lld counts\n", (long long)hall_raw);
    TEST_RESULT("getHallSensorPositionRaw ~ getHallSensorPosition (cmd 15)",
                llabs(hall_raw - (int64_t)llroundf(hall)) < ZERO_TOL);

    // Return to zero (ends at rest — motion-safe).
    motor->trapezoidMove(0.0f, 1.0f);
    checkMotorError(*motor, "trapezoidMove(hall return)");
    while (motor->getNQueuedItems() > 0) {
        delay(10);
    }
    delay(100);
    motor->disableMosfets();
    checkMotorError(*motor, "disableMosfets(B)");

    // ========================================================================
    // SECTION C — cmd 38 Get supply voltage
    // Plausible range; V and mV agree; stable across a burst of reads.
    // ========================================================================
    printf("\n=== Section C: Get supply voltage (cmd 38) ===\n");
    const float MIN_VOLTS = 6.0f;
    const float MAX_VOLTS = 40.0f;
    const float UNIT_TOL_V = 0.2f;
    const float STABILITY_BAND_V = 1.0f;
    const int N_STABILITY = 10;

    motor->systemReset();
    delay(1500);
    delay(200);  // supply ADC settle

    motor->setVoltageUnit(VoltageUnit::VOLTS);
    float v = motor->getSupplyVoltage();
    checkMotorError(*motor, "getSupplyVoltage(volts)");
    snprintf(buf, sizeof(buf), "  supply voltage = %.3f V\n", v);
    printf("%s", buf);
    TEST_RESULT("Supply voltage in plausible range 6..40 V (cmd 38)",
                v >= MIN_VOLTS && v <= MAX_VOLTS);

    motor->setVoltageUnit(VoltageUnit::MILLIVOLTS);
    float v_mv = motor->getSupplyVoltage();
    checkMotorError(*motor, "getSupplyVoltage(millivolts)");
    motor->setVoltageUnit(VoltageUnit::VOLTS);  // restore
    float v_from_mv = v_mv / 1000.0f;
    float vdiff = fabsf(v_from_mv - v);
    snprintf(buf, sizeof(buf), "  in millivolts = %.1f mV -> %.4f V (diff = %.4f V)\n",
             v_mv, v_from_mv, vdiff);
    printf("%s", buf);
    TEST_RESULT("Supply voltage V and mV agree (cmd 38)", vdiff <= UNIT_TOL_V);

    float vlo = v, vhi = v;
    for (int i = 0; i < N_STABILITY; i++) {
        float s = motor->getSupplyVoltage();
        checkMotorError(*motor, "getSupplyVoltage(stability)");
        if (s < vlo) vlo = s;
        if (s > vhi) vhi = s;
    }
    float vspread = vhi - vlo;
    snprintf(buf, sizeof(buf), "  stability: min=%.3f V max=%.3f V spread=%.4f V\n",
             vlo, vhi, vspread);
    printf("%s", buf);
    TEST_RESULT("Supply voltage stable across reads (cmd 38)", vspread <= STABILITY_BAND_V);

    // ========================================================================
    // SECTION D — cmd 11 Get n queued items
    // 0 at rest; rises when moves queued (never > 32); drains back to 0.
    // ========================================================================
    printf("\n=== Section D: Get n queued items (cmd 11) ===\n");
    const int MAX_QUEUE = 32;
    const int N_MOVES = 5;
    const float MOVE_TIME_S = 0.4f;
    const float SMALL_MOVE_COUNTS = 20000.0f;

    motor->systemReset();
    delay(1500);
    motor->enableMosfets();
    checkMotorError(*motor, "enableMosfets(D)");
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    motor->setTimeUnit(TimeUnit::SECONDS);

    uint8_t q_rest = motor->getNQueuedItems();
    checkMotorError(*motor, "getNQueuedItems(rest)");
    printf("  queue at rest: %u\n", (unsigned)q_rest);
    TEST_RESULT("Queue empty at rest (cmd 11)", q_rest == 0);

    printf("  Queueing %d moves of %.1fs each (alternating direction)...\n", N_MOVES, MOVE_TIME_S);
    for (int i = 0; i < N_MOVES; i++) {
        float disp = (i % 2 == 0) ? SMALL_MOVE_COUNTS : -SMALL_MOVE_COUNTS;
        motor->trapezoidMove(disp, MOVE_TIME_S);
        checkMotorError(*motor, "trapezoidMove(queue)");
    }

    uint8_t q_full = motor->getNQueuedItems();
    checkMotorError(*motor, "getNQueuedItems(full)");
    printf("  queue right after queueing %d moves: %u\n", N_MOVES, (unsigned)q_full);
    // Trapezoid moves expand into multiple slots; just assert populated and within cap.
    TEST_RESULT("Queue never exceeds firmware max 32 (cmd 11)", q_full <= MAX_QUEUE);
    TEST_RESULT("Queue populated after queueing moves (cmd 11)", q_full >= (uint8_t)(N_MOVES - 1));

    printf("  Waiting for the queue to drain...\n");
    unsigned long budget_ms = (unsigned long)(N_MOVES * MOVE_TIME_S * 1000.0f) + 3000;
    unsigned long waited = 0;
    while (waited < budget_ms) {
        if (motor->getNQueuedItems() == 0) break;
        delay(20);
        waited += 20;
    }
    uint8_t q_drained = motor->getNQueuedItems();
    checkMotorError(*motor, "getNQueuedItems(drained)");
    printf("  queue after draining: %u\n", (unsigned)q_drained);
    TEST_RESULT("Queue drains back to 0 (cmd 11)", q_drained == 0);

    // Clean up: motor already at rest (all trapezoid moves end at rest).
    motor->disableMosfets();
    checkMotorError(*motor, "disableMosfets(D)");
    motor->systemReset();
    delay(1500);
}
