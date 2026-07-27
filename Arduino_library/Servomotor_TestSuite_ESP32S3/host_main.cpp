// Host harness: runs the SAME tm_* test modules on the Mac through
// ArduinoEmulator (real USB-RS485 serial port), no flashing needed.
// Build with ./build_host_suite.sh ; run:
//   ./host_suite <serial_port> <16-hex-unique-id> [all | <index> | <name>]
#if !defined(ARDUINO)

#include "tf_framework.h"
#include "test_registry.h"
#include <cstdlib>
#include <cstring>

// Globals the emulator header declares (ArduinoEmulator.cpp is NOT linked —
// it contains its own main()).
ConsoleSerial Serial;
HardwareSerial Serial1;
char g_motorAlias = 'X';
uint64_t g_uniqueId = 0;
bool g_useUniqueId = true;

static Servomotor* g_m = nullptr;

Servomotor* tfGetMotor() {
    if (!g_m) {
        g_m = new Servomotor(255, Serial1);
        g_m->useUniqueId(g_uniqueId);
    }
    return g_m;
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        fprintf(stderr,
                "Usage: %s <serial_port> <16-hex-unique-id> [all | <index> | <name>]\n",
                argv[0]);
        return 2;
    }
    Serial1 = HardwareSerial(argv[1]);
    if (!Serial1.begin(230400)) {
        fprintf(stderr, "Failed to open %s\n", argv[1]);
        return 2;
    }
    g_uniqueId = strtoull(argv[2], nullptr, 16);

    const char* sel = (argc >= 4) ? argv[3] : "all";
    int only = -1;
    if (strcmp(sel, "all") != 0) {
        char* end = nullptr;
        long v = strtol(sel, &end, 10);
        if (end && *end == '\0') only = (int)v;
        else {
            for (int i = 0; i < TF_N_TESTS; i++)
                if (strcmp(TF_TESTS[i].name, sel) == 0) only = i;
        }
        if (only < 0 || only >= TF_N_TESTS) {
            fprintf(stderr, "Unknown test '%s'\n", sel);
            return 2;
        }
    }

    int testsFailed = 0, testsRun = 0;
    for (int i = 0; i < TF_N_TESTS; i++) {
        if (only >= 0 && i != only) continue;
        tf_printf("\nTEST_START %d %s\n", i, TF_TESTS[i].name);
        TestRunner::reset();
        TF_TESTS[i].fn();
        int p = TestRunner::passCount(), f = TestRunner::failCount();
        TestRunner::printResults();
        bool pass = (f == 0 && p > 0);
        tf_printf("TEST_DONE %d %s status=%s pass=%d fail=%d %s\n", i,
                  TF_TESTS[i].name, pass ? "PASS" : "FAIL", p, f,
                  (!pass && TF_TESTS[i].knownBugs[0]) ? TF_TESTS[i].knownBugs : "");
        testsRun++;
        if (!pass) testsFailed++;
    }
    tf_printf("\nHOST SUITE DONE: %d/%d tests passed\n", testsRun - testsFailed, testsRun);
    return testsFailed ? 1 : 0;
}

#endif  // !ARDUINO
