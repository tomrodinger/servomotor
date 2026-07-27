// ---------------------------------------------------------------------------
// Servomotor_TestSuite_ESP32S3 — on-device Arduino-library test suite harness.
//
// All test modules (tm_*.cpp) compile into THIS one firmware. The harness runs
// ONE test per boot and resets the ESP32-S3 between tests; run progress and
// pass/fail statistics live in NVS flash, so they survive both the software
// resets and a power cycle. Detailed logs also stream over USB CDC so the Mac
// (run_suite_mac.py) can collect them as the suite runs.
//
// Serial CLI (USB CDC, 230400, newline-terminated) when no run is active:
//   help              this text
//   list              list tests (index, name, known-bug annotation)
//   status            show run state, motor uid, firmware build id
//   results           re-print the last run's results table from NVS
//   run               start a full sequential run (test 0..N-1, reset between)
//   run <n>           run a single test by index, then stop
//   detect            re-detect the motor on RS485 and store its unique ID
//   uid <16-hex>      set the motor unique ID manually
//   stop              (also honored at boot while a run is active) abort run
//   restart           reboot the ESP32
//
// Crash/hang safety: before a test starts, its index is committed to NVS
// ("started"). A per-test esp_timer watchdog abort()s the chip if the test
// exceeds its registry timeout. On the next boot the harness sees the test
// started-but-never-finished, records it as CRASH, and continues with the
// next test. A stale run from a different firmware build is refused (build id
// is stored at run start).
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <Preferences.h>
#include "esp_system.h"
#include "esp_timer.h"

#include "tf_framework.h"   // note: defines `printf` -> tf_printf; keep system
#include "test_registry.h"  // includes ABOVE this line, and never Serial.printf

#define RS485_TXD 4
#define RS485_RXD 5
#define USB_BAUD  230400
#define MAX_TESTS 64        // NVS blob headroom (>= TF_N_TESTS)

static const uint8_t IDX_NONE = 0xFF;

// ---- persisted run state -------------------------------------------------
static Preferences prefs;
struct RunState {
    uint8_t  active;             // 1 = a run is in progress
    uint8_t  idx;                // next test index to run (sequence mode)
    uint8_t  started;            // test committed as started but not finished
    uint8_t  only;               // IDX_NONE = full sequence, else single test
    uint64_t uid;                // motor unique ID
    uint32_t buildId;            // firmware build id the run belongs to
    uint8_t  st[MAX_TESTS];      // per-test status: 0 NOTRUN 1 PASS 2 FAIL 3 CRASH
    uint16_t pa[MAX_TESTS];      // per-test passed assertion count
    uint16_t fa[MAX_TESTS];      // per-test failed assertion count
};
static RunState rs;

static uint32_t fwBuildId() {
    const char* s = __DATE__ " " __TIME__;
    uint32_t h = 5381;
    while (*s) h = h * 33 ^ (uint8_t)(*s++);
    return h;
}

static void loadState() {
    memset(&rs, 0, sizeof(rs));
    rs.started = IDX_NONE;
    rs.only    = IDX_NONE;
    rs.active  = prefs.getUChar("active", 0);
    rs.idx     = prefs.getUChar("idx", 0);
    rs.started = prefs.getUChar("started", IDX_NONE);
    rs.only    = prefs.getUChar("only", IDX_NONE);
    rs.uid     = prefs.getULong64("uid", 0);
    rs.buildId = prefs.getULong("buildid", 0);
    prefs.getBytes("st", rs.st, sizeof(rs.st));
    prefs.getBytes("pa", rs.pa, sizeof(rs.pa));
    prefs.getBytes("fa", rs.fa, sizeof(rs.fa));
}

static void saveState() {
    prefs.putUChar("active", rs.active);
    prefs.putUChar("idx", rs.idx);
    prefs.putUChar("started", rs.started);
    prefs.putUChar("only", rs.only);
    prefs.putULong64("uid", rs.uid);
    prefs.putULong("buildid", rs.buildId);
    prefs.putBytes("st", rs.st, sizeof(rs.st));
    prefs.putBytes("pa", rs.pa, sizeof(rs.pa));
    prefs.putBytes("fa", rs.fa, sizeof(rs.fa));
}

// ---- per-test hang watchdog ----------------------------------------------
static esp_timer_handle_t g_tmo = nullptr;
static void tmoCallback(void*) { abort(); }  // panic-reboot; NVS marks CRASH

static void armTimeout(uint32_t sec) {
    esp_timer_create_args_t args = {};
    args.callback = &tmoCallback;
    args.name = "test_tmo";
    if (esp_timer_create(&args, &g_tmo) == ESP_OK) {
        esp_timer_start_once(g_tmo, (uint64_t)sec * 1000000ULL);
    }
}

static void disarmTimeout() {
    if (g_tmo) {
        esp_timer_stop(g_tmo);
        esp_timer_delete(g_tmo);
        g_tmo = nullptr;
    }
}

// ---- motor under test -----------------------------------------------------
static Servomotor* g_motor = nullptr;

Servomotor* tfGetMotor() {   // declared in tf_framework.h
    if (!g_motor) {
        g_motor = new Servomotor(255, Serial1, RS485_RXD, RS485_TXD);
        g_motor->openSerialPort();
        g_motor->useUniqueId(rs.uid);
    }
    return g_motor;
}

// Broadcast-reset the bus and collect the first device's unique ID.
static uint64_t detectMotorUid() {
    Servomotor det(255, Serial1, RS485_RXD, RS485_TXD);
    det.openSerialPort();
    uint64_t uid = 0;
    for (int cycle = 0; cycle < 3 && uid == 0; cycle++) {
        det.systemReset();
        delay(1500);
        Serial1.flush();
        while (Serial1.available()) Serial1.read();
        detectDevicesResponse r = det.detectDevices();
        while (det.getError() == COMMUNICATION_SUCCESS) {
            tf_printf("  detected uniqueId=%016llX alias=%u\n",
                      (unsigned long long)r.uniqueId, (unsigned)r.alias);
            if (uid == 0) uid = r.uniqueId;
            r = det.detectDevicesGetAnotherResponse();
        }
    }
    return uid;
}

// ---- reporting ------------------------------------------------------------
static const char* statusStr(uint8_t s) {
    switch (s) {
        case 1: return "PASS";
        case 2: return "FAIL";
        case 3: return "CRASH";
        default: return "NOTRUN";
    }
}

static void printSuiteResults() {
    int passed = 0, failed = 0, crashed = 0, unexpected = 0;
    tf_printf("\n==== SUITE RESULTS ====\n");
    tf_printf("%-3s %-18s %-7s %5s %5s  %s\n", "idx", "name", "status", "pass", "fail", "note");
    for (int i = 0; i < TF_N_TESTS; i++) {
        const char* note = "";
        if (rs.st[i] == 2 && TF_TESTS[i].knownBugs[0]) note = "EXPECTED-FAIL (known bugs)";
        tf_printf("%-3d %-18s %-7s %5u %5u  %s%s%s\n", i, TF_TESTS[i].name,
                  statusStr(rs.st[i]), rs.pa[i], rs.fa[i],
                  note, TF_TESTS[i].knownBugs[0] ? " " : "", TF_TESTS[i].knownBugs);
        if (rs.st[i] == 1) passed++;
        else if (rs.st[i] == 2) { failed++; if (!TF_TESTS[i].knownBugs[0]) unexpected++; }
        else if (rs.st[i] == 3) { crashed++; unexpected++; }
    }
    tf_printf("SUITE_SUMMARY tests=%d passed=%d failed=%d crashed=%d unexpected=%d\n",
              TF_N_TESTS, passed, failed, crashed, unexpected);
}

static void finalize() {
    rs.active = 0;
    saveState();
    printSuiteResults();
    tf_printf("SUITE COMPLETE\n");
}

// ---- run flow -------------------------------------------------------------
static void runOne(int i) {
    tf_printf("\nTEST_START %d/%d %s (timeout %us, motor uid=%016llX)\n",
              i, TF_N_TESTS - 1, TF_TESTS[i].name, TF_TESTS[i].timeoutSec,
              (unsigned long long)rs.uid);
    rs.started = (uint8_t)i;
    saveState();

    TestRunner::reset();
    armTimeout(TF_TESTS[i].timeoutSec);
    TF_TESTS[i].fn();
    disarmTimeout();

    int p = TestRunner::passCount(), f = TestRunner::failCount();
    TestRunner::printResults();
    rs.st[i] = (f == 0 && p > 0) ? 1 : 2;
    rs.pa[i] = (uint16_t)p;
    rs.fa[i] = (uint16_t)f;
    rs.started = IDX_NONE;
    if (rs.only == IDX_NONE) rs.idx = (uint8_t)(i + 1);
    saveState();
    tf_printf("TEST_DONE %d %s status=%s pass=%d fail=%d\n",
              i, TF_TESTS[i].name, statusStr(rs.st[i]), p, f);
}

static void startRun(uint8_t only) {
    memset(rs.st, 0, sizeof(rs.st));
    memset(rs.pa, 0, sizeof(rs.pa));
    memset(rs.fa, 0, sizeof(rs.fa));
    rs.active  = 1;
    rs.idx     = (only == IDX_NONE) ? 0 : only;
    rs.only    = only;
    rs.started = IDX_NONE;
    rs.buildId = fwBuildId();

    bool needMotor = (only == IDX_NONE) ? true : TF_TESTS[only].needsMotor;
    if (needMotor && rs.uid == 0) {
        tf_printf("No motor uid stored; detecting...\n");
        rs.uid = detectMotorUid();
        if (rs.uid == 0) {
            tf_printf("ERROR: no motor detected on RS485. Run 'detect' or 'uid <hex>'.\n");
            rs.active = 0;
            saveState();
            return;
        }
    }
    saveState();
    tf_printf("RUN STARTING: %s (%d tests), motor uid=%016llX\n",
              (only == IDX_NONE) ? "full sequence" : TF_TESTS[only].name,
              (only == IDX_NONE) ? TF_N_TESTS : 1, (unsigned long long)rs.uid);
    Serial.flush();
    delay(300);
    ESP.restart();
}

// ---- CLI ------------------------------------------------------------------
static void printHelp() {
    tf_printf("Commands: help | list | status | results | run | run <n> | "
              "detect | uid <16-hex> | stop | restart\n");
}

static void handleCommand(String line) {
    line.trim();
    if (line.length() == 0) return;
    if (line == "help") { printHelp(); }
    else if (line == "list") {
        for (int i = 0; i < TF_N_TESTS; i++)
            tf_printf("%2d  %-18s %s%s\n", i, TF_TESTS[i].name,
                      TF_TESTS[i].needsMotor ? "" : "(no motor needed) ",
                      TF_TESTS[i].knownBugs);
    }
    else if (line == "status") {
        tf_printf("active=%u idx=%u only=%u started=%u uid=%016llX buildid=%08X n_tests=%d\n",
                  rs.active, rs.idx, rs.only, rs.started,
                  (unsigned long long)rs.uid, (unsigned)fwBuildId(), TF_N_TESTS);
    }
    else if (line == "results") { printSuiteResults(); }
    else if (line == "run") { startRun(IDX_NONE); }
    else if (line.startsWith("run ")) {
        int n = line.substring(4).toInt();
        if (n >= 0 && n < TF_N_TESTS) startRun((uint8_t)n);
        else tf_printf("Bad test index (0..%d)\n", TF_N_TESTS - 1);
    }
    else if (line == "detect") {
        uint64_t uid = detectMotorUid();
        if (uid) { rs.uid = uid; saveState(); }
        tf_printf(uid ? "Motor uid stored: %016llX\n" : "No motor detected.\n",
                  (unsigned long long)uid);
    }
    else if (line.startsWith("uid ")) {
        uint64_t uid = strtoull(line.substring(4).c_str(), nullptr, 16);
        rs.uid = uid;
        saveState();
        tf_printf("Motor uid stored: %016llX\n", (unsigned long long)uid);
    }
    else if (line == "stop") {
        rs.active = 0;
        saveState();
        tf_printf("Run stopped.\n");
    }
    else if (line == "restart") { Serial.flush(); delay(100); ESP.restart(); }
    else { tf_printf("Unknown command: %s\n", line.c_str()); printHelp(); }
}

// Brief window where a pending 'stop' line can abort an active run at boot.
static bool stopRequestedAtBoot(uint32_t windowMs) {
    uint32_t t0 = millis();
    String buf;
    while (millis() - t0 < windowMs) {
        while (Serial.available()) {
            char c = (char)Serial.read();
            if (c == '\n' || c == '\r') {
                buf.trim();
                if (buf == "stop") return true;
                buf = "";
            } else buf += c;
        }
        delay(10);
    }
    return false;
}

static const char* resetReasonStr() {
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:  return "POWERON";
        case ESP_RST_SW:       return "SW_RESTART";
        case ESP_RST_PANIC:    return "PANIC";
        case ESP_RST_INT_WDT:  return "INT_WDT";
        case ESP_RST_TASK_WDT: return "TASK_WDT";
        case ESP_RST_WDT:      return "WDT";
        case ESP_RST_BROWNOUT: return "BROWNOUT";
        case ESP_RST_USB:      return "USB";
        default:               return "OTHER";
    }
}

// ---- Arduino entry points -------------------------------------------------
void setup() {
    Serial.begin(USB_BAUD);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 4000) delay(50);
    delay(300);

    Serial1.setRxBufferSize(4096);
    Serial1.begin(230400, SERIAL_8N1, RS485_RXD, RS485_TXD);

    prefs.begin("ardsuite", false);
    loadState();

    tf_printf("\n=== Servomotor Arduino-library ON-DEVICE test suite ===\n");
    tf_printf("build=%08X reset=%s tests=%d active=%u idx=%u started=%u\n",
              (unsigned)fwBuildId(), resetReasonStr(), TF_N_TESTS,
              rs.active, rs.idx, rs.started);

    if (rs.active && rs.buildId != fwBuildId()) {
        tf_printf("Firmware changed since the run started; aborting stale run.\n");
        rs.active = 0;
        rs.started = IDX_NONE;
        saveState();
    }

    if (rs.active && rs.started != IDX_NONE) {
        // Previous boot's test never finished -> it crashed or timed out.
        uint8_t c = rs.started;
        tf_printf("TEST_CRASH %d %s (did not finish before reboot)\n", c, TF_TESTS[c].name);
        rs.st[c] = 3;
        rs.started = IDX_NONE;
        if (rs.only == IDX_NONE) rs.idx = (uint8_t)(c + 1);
        else { finalize(); return; }   // single-test mode: done (as CRASH)
        saveState();
    }

    if (rs.active && stopRequestedAtBoot(300)) {
        rs.active = 0;
        saveState();
        tf_printf("Run stopped by request.\n");
    }

    if (rs.active) {
        int next = (rs.only == IDX_NONE) ? rs.idx : rs.only;
        if (rs.only == IDX_NONE && next >= TF_N_TESTS) {
            finalize();               // sequence exhausted
        } else {
            runOne(next);
            if (rs.only != IDX_NONE || rs.idx >= TF_N_TESTS) finalize();
            tf_printf("Rebooting for next step...\n");
            Serial.flush();
            delay(300);
            ESP.restart();
        }
        return;                        // (only reached after finalize())
    }

    tf_printf("IDLE — type 'help' for commands.\n");
}

void loop() {
    static String buf;
    static uint32_t lastBeat = 0;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (buf.length()) handleCommand(buf);
            buf = "";
        } else if (buf.length() < 120) buf += c;
    }
    if (millis() - lastBeat > 15000) {
        lastBeat = millis();
        tf_printf("IDLE — type 'help' (last run: use 'results')\n");
    }
    delay(20);
}
