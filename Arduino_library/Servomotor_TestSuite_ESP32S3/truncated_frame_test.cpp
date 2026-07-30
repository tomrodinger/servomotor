// ---------------------------------------------------------------------------
// truncated_frame_test.cpp
//
// Transport-layer regression test for Communication::getResponse().
//
// WHAT IT TESTS
//   A reply that STARTS but never finishes. The library reads a valid first
//   size byte, then waits for the rest of the packet, which never comes. The
//   correct behaviour is to give up after the ~1 second receive budget and
//   return COMMUNICATION_ERROR_TIMEOUT (-1). The historical defect was an
//   infinite wait: the sketch froze with no timeout and no error.
//
//   Mechanism of the defect, for whoever reads this next:
//     getResponse() gives each receiveBytes() call the REMAINING share of one
//     1000 ms budget, as `TIMEOUT_MS - (millis() - startTime)`. millis()
//     returns an unsigned type, so once elapsed exceeds TIMEOUT_MS that
//     expression wraps and arrives in receiveBytes' `int32_t timeout_ms` as a
//     NEGATIVE number. The wait loop then compares
//     `millis() - startTime > timeout_ms` — uint32_t against int32_t — so the
//     negative value converts back to ~4.29e9 and the condition can never be
//     true. The loop spins forever.
//
//     It is reachable on the error path: a truncated reply burns the whole
//     budget reading the response character, then jumps to
//     `flush_read_remaining_bytes_and_return_error:`, which calls
//     receiveBytes AGAIN with a now-negative remaining budget.
//
// WHY IT IS A STANDALONE BINARY AND NOT A tm_* MODULE
//   It needs NO motor. It drives Communication directly against a pseudo-
//   terminal whose far end is a scripted fake device, so the truncated frame
//   is produced deterministically and on demand. A real motor cannot be asked
//   to truncate a reply on cue. Run it in CI; it needs no hardware.
//
// USAGE
//   Driven by run_truncated_frame_test.py, which creates the PTY, launches
//   this binary against the slave side, and plays the fake device on the
//   master side. Do not run it by hand — it expects a scripted peer.
//
//     ./truncated_frame_test <port> <case>
//       case = truncated   : peer sends one valid size byte then goes silent
//       case = silent      : peer sends nothing at all (the already-working path)
//       case = resync      : truncated frame, then a well-formed reply must
//                            still be readable afterwards
//       case = two_rounds  : same shape as resync; the runner decides how much
//                            of the first frame to send (used for the
//                            partially-truncated case, where several bytes of
//                            a longer declared packet do arrive)
//
//   Exit codes: 0 = expected behaviour, 1 = wrong error code / bad data,
//               42 = WATCHDOG FIRED (the call never returned = the defect).
// ---------------------------------------------------------------------------

// Host-only: arduino-cli compiles every .cpp in the sketch folder into the
// device build, where ArduinoEmulator.h would collide with the real
// HardwareSerial. Same guard host_main.cpp uses.
#if !defined(ARDUINO)

#include "../ArduinoEmulator.h"
#include "../Communication.h"

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unistd.h>

// The whole point of the test is that the code under test may never return, so
// the watchdog cannot live inside it. SIGALRM + _exit is async-signal-safe.
static void onWatchdog(int) {
    const char msg[] = "WATCHDOG: the call never returned - the receive path is hung\n";
    ssize_t ignored = ::write(STDERR_FILENO, msg, sizeof(msg) - 1);
    (void)ignored;
    _exit(42);
}

// A generous multiple of the library's own 1000 ms budget. If a single
// getResponse() has not returned in this long, it is not slow, it is stuck.
static const unsigned WATCHDOG_SECONDS = 8;

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: %s <port> <truncated|silent|resync>\n", argv[0]);
        return 2;
    }
    const std::string port = argv[1];
    const std::string testCase = argv[2];

    signal(SIGALRM, onWatchdog);

    HardwareSerial serialPort(port);
    // Open the port explicitly. Communication::openSerialPort() is wrapped in
    // #ifdef ARDUINO, so on the host build it is a no-op — and an unopened
    // emulator port silently swallows writes and reports nothing available,
    // which would make every case "time out" for entirely the wrong reason.
    // host_main.cpp does the same thing.
    if (!serialPort.begin(230400)) {
        fprintf(stderr, "failed to open %s\n", port.c_str());
        return 2;
    }
    Communication comm(serialPort, 230400);
    comm.openSerialPort();
    // The fake peer speaks raw frames; CRC is irrelevant to the receive-path
    // defect and leaving it on would only change the expected byte counts.
    comm.disableCRC32();

    uint8_t buffer[64];
    uint16_t receivedSize = 0;

    if (testCase == "truncated" || testCase == "silent") {
        // Ask for anything; the scripted peer decides what (if any) reply to send.
        comm.sendCommand(88 /* alias */, 16 /* Get status */, nullptr, 0);

        alarm(WATCHDOG_SECONDS);
        unsigned long t0 = millis();
        int16_t err = comm.getResponse(buffer, sizeof(buffer), receivedSize);
        unsigned long elapsed = millis() - t0;
        alarm(0);

        printf("case=%s err=%d elapsed_ms=%lu\n", testCase.c_str(), (int)err, elapsed);

        if (err != COMMUNICATION_ERROR_TIMEOUT) {
            printf("FAIL: expected COMMUNICATION_ERROR_TIMEOUT (%d), got %d\n",
                   COMMUNICATION_ERROR_TIMEOUT, (int)err);
            return 1;
        }
        // The budget is for the whole reply, so a truncated frame must not cost
        // meaningfully more than one budget's worth.
        if (elapsed > 3000) {
            printf("FAIL: timed out but took %lu ms, expected roughly one 1000 ms budget\n",
                   elapsed);
            return 1;
        }
        printf("PASS: timed out correctly in %lu ms\n", elapsed);
        return 0;
    }

    if (testCase == "resync" || testCase == "two_rounds") {
        // Round 1: truncated frame. Must time out rather than hang.
        comm.sendCommand(88, 16, nullptr, 0);
        alarm(WATCHDOG_SECONDS);
        int16_t err1 = comm.getResponse(buffer, sizeof(buffer), receivedSize);
        alarm(0);
        printf("resync round1 err=%d\n", (int)err1);
        if (err1 != COMMUNICATION_ERROR_TIMEOUT) {
            printf("FAIL: round 1 expected timeout (%d), got %d\n",
                   COMMUNICATION_ERROR_TIMEOUT, (int)err1);
            return 1;
        }

        // Round 2: the peer now sends a well-formed reply. Whether this
        // succeeds depends on the caller having drained the stale bytes first,
        // which is the documented recovery. The runner drains between rounds.
        comm.sendCommand(88, 16, nullptr, 0);
        alarm(WATCHDOG_SECONDS);
        receivedSize = 0;
        int16_t err2 = comm.getResponse(buffer, sizeof(buffer), receivedSize);
        alarm(0);
        printf("resync round2 err=%d receivedSize=%u\n", (int)err2, (unsigned)receivedSize);
        if (err2 != 0) {
            printf("FAIL: round 2 expected success, got %d\n", (int)err2);
            return 1;
        }
        if (receivedSize != 3) {
            printf("FAIL: round 2 expected a 3-byte Get status payload, got %u\n",
                   (unsigned)receivedSize);
            return 1;
        }
        printf("PASS: recovered and read a clean reply after a truncated frame\n");
        return 0;
    }

    fprintf(stderr, "unknown case '%s'\n", testCase.c_str());
    return 2;
}

#endif  // !ARDUINO
