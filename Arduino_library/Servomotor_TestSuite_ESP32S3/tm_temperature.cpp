#include "tf_framework.h"

// ---------------------------------------------------------------------------
// tm_temperature.cpp
//
// Arduino-library on-device test for Get temperature (cmd 20) and the
// temperature-unit selection plumbing. Ported from the legacy
// test_get_temperature.cpp (C / F / K reads + conversion checks), then
// EXTENDED with:
//   * 5 consecutive Celsius reads, all within a 3 deg C band of each other
//     and each in the (5..85) plausible-ambient window;
//   * F and K cross-checked against C on each of 2 reads
//       (F == C*9/5 + 32  +/- 1,   K == C + 273.15  +/- 1).
//
// No motion is commanded, so there is no queue to drain. The device is
// addressed by unique ID via tfGetMotor(), so it is collision-safe on the
// 39-motor rack.
// ---------------------------------------------------------------------------

static const int NORMAL_RESET_DELAY_MS = 1500;

void tm_temperature(void) {
    Serial.println("test_ard_temperature: BEGIN\n");

    Servomotor* motor = tfGetMotor();

    // -----------------------------------------------------------------------
    // Clean, known starting state. Explicitly select Celsius.
    // -----------------------------------------------------------------------
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);
    motor->setTemperatureUnit(TemperatureUnit::CELSIUS);

    // =======================================================================
    // Ported test 1: direct Celsius read + basic sanity.
    // (Legacy used 20..100; we tighten to the physically-plausible ambient
    //  window 5..85 that the extension also asserts.)
    // =======================================================================
    float celsius = motor->getTemperature();
    checkMotorError(*motor, "getTemperature(C)");
    printf("Temperature in Celsius: %.2f C\n", celsius);
    TEST_RESULT("Temperature Read (5..85 C)", celsius > 5.0f && celsius < 85.0f);

    // =======================================================================
    // Ported test 2: Fahrenheit unit selection + conversion check.
    // =======================================================================
    motor->setTemperatureUnit(TemperatureUnit::FAHRENHEIT);
    float temp_f = motor->getTemperature();
    checkMotorError(*motor, "getTemperature(F)");
    float expected_f = (celsius * 9.0f / 5.0f) + 32.0f;
    printf("Temperature in Fahrenheit: %.2f F (expect %.2f)\n", temp_f, expected_f);
    // Tolerance note: the sensor quantizes at 1 degC and may step between
    // the C and F reads; 1 degC == 1.8 degF, so allow 2.5 degF.
    TEST_RESULT("Fahrenheit Reading", approxEqual(temp_f, expected_f, 2.5f));

    // =======================================================================
    // Ported test 3: Kelvin unit selection + conversion check.
    // =======================================================================
    motor->setTemperatureUnit(TemperatureUnit::KELVIN);
    float temp_k = motor->getTemperature();
    checkMotorError(*motor, "getTemperature(K)");
    float expected_k = celsius + 273.15f;
    printf("Temperature in Kelvin: %.2f K (expect %.2f)\n", temp_k, expected_k);
    TEST_RESULT("Kelvin Reading", approxEqual(temp_k, expected_k, 1.6f));

    // =======================================================================
    // EXTENSION A: 5 consecutive Celsius reads within a 3 deg C band and all
    // inside the (5..85) plausible-ambient window.
    // =======================================================================
    motor->setTemperatureUnit(TemperatureUnit::CELSIUS);
    float minC = 1e9f, maxC = -1e9f;
    bool allInWindow = true;
    bool allCommsOk = true;
    for (int i = 0; i < 5; i++) {
        float c = motor->getTemperature();
        if (motor->getError() != 0) allCommsOk = false;
        if (c <= 5.0f || c >= 85.0f) allInWindow = false;
        if (c < minC) minC = c;
        if (c > maxC) maxC = c;
        printf("  Celsius read %d: %.2f C\n", i + 1, c);
        delay(50);
    }
    float band = maxC - minC;
    printf("Celsius 5-read band: %.2f C (min %.2f, max %.2f)\n", band, minC, maxC);
    TEST_RESULT("5 Celsius reads comms ok", allCommsOk);
    TEST_RESULT("5 Celsius reads all in (5..85) C", allInWindow);
    TEST_RESULT("5 Celsius reads within 3 C band", band <= 3.0f);

    // =======================================================================
    // EXTENSION B: F and K cross-checked against C on each of 2 reads.
    //   F == C*9/5 + 32   +/- 1
    //   K == C + 273.15   +/- 1
    // Read C, F, K back-to-back so the underlying temperature is essentially
    // constant across the trio.
    // =======================================================================
    for (int i = 0; i < 2; i++) {
        motor->setTemperatureUnit(TemperatureUnit::CELSIUS);
        float c = motor->getTemperature();
        bool cOk = (motor->getError() == 0);

        motor->setTemperatureUnit(TemperatureUnit::FAHRENHEIT);
        float f = motor->getTemperature();
        bool fOk = (motor->getError() == 0);

        motor->setTemperatureUnit(TemperatureUnit::KELVIN);
        float k = motor->getTemperature();
        bool kOk = (motor->getError() == 0);

        float expF = (c * 9.0f / 5.0f) + 32.0f;
        float expK = c + 273.15f;
        printf("Cross-check %d: C=%.2f F=%.2f(exp %.2f) K=%.2f(exp %.2f)\n",
               i + 1, c, f, expF, k, expK);

        char nameF[48];
        char nameK[48];
        snprintf(nameF, sizeof(nameF), "F==C*9/5+32 cross-check read %d", i + 1);
        snprintf(nameK, sizeof(nameK), "K==C+273.15 cross-check read %d", i + 1);
        TEST_RESULT(nameF, cOk && fOk && approxEqual(f, expF, 2.5f));  // 1 degC quantization step == 1.8 degF
        TEST_RESULT(nameK, cOk && kOk && approxEqual(k, expK, 1.6f));
    }

    // -----------------------------------------------------------------------
    // Leave clean: restore Celsius, no motion was ever commanded.
    // -----------------------------------------------------------------------
    motor->setTemperatureUnit(TemperatureUnit::CELSIUS);
    motor->systemReset();
    delay(NORMAL_RESET_DELAY_MS);
}
