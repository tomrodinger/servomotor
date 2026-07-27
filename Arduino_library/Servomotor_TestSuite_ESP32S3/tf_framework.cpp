#include "tf_framework.h"

std::vector<TestResult> TestRunner::results;

void TestRunner::reset() { results.clear(); }

void TestRunner::addResult(const std::string& name, bool passed) {
    results.push_back({name, passed});
}

int TestRunner::passCount() {
    int n = 0;
    for (const auto& r : results) if (r.passed) n++;
    return n;
}

int TestRunner::failCount() {
    int n = 0;
    for (const auto& r : results) if (!r.passed) n++;
    return n;
}

void TestRunner::printResults() {
    tf_printf("\nTest Results Summary\n==================\n");
    size_t maxLen = 0;
    for (const auto& r : results) maxLen = (r.name.length() > maxLen) ? r.name.length() : maxLen;
    for (const auto& r : results) {
        tf_printf("%-*s  %s\n", (int)maxLen, r.name.c_str(), r.passed ? "PASS" : "FAIL");
    }
    tf_printf("Assertions: %d passed, %d failed\n", passCount(), failCount());
}

int tf_printf(const char* fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
#if defined(ARDUINO)
    Serial.print(buf);
#else
    fputs(buf, stdout);
    fflush(stdout);
#endif
    return n;
}

void checkMotorError(Servomotor& motor, const std::string& commandName) {
    int e = motor.getError();
    if (e != 0) {
        tf_printf("ERROR in %s: error code %d\n", commandName.c_str(), e);
        TestRunner::addResult(std::string("No error after ") + commandName, false);
    }
}
