// ---------------------------------------------------------------------------
// ESP32S3_RS485_Bridge
//
// Turns the ESP32-S3 into a TRANSPARENT USB <-> RS485 adapter, so that every
// host-side tool in this repository can talk to a motor that is wired to the
// ESP32's RS485 bus instead of to a USB-RS485 dongle.
//
// WHY THIS EXISTS
//   The M17 bench motor (unique ID 99856389A2B46555) is wired to this board's
//   RS485 bus and has no other path to the Mac. That made it unreachable by
//   upgrade_firmware.py, by run_all_tests.py, and by host_suite -- it could only
//   be driven by whatever happened to be compiled into the on-device test suite.
//   It was consequently stranded on firmware 0.15.9.0, three planner bugs
//   behind the rest of the fleet.
//
//   The obvious fix would be to write a firmware-upgrader sketch. This is
//   better: a dumb byte pipe means the UPGRADE PROTOCOL STAYS IN THE PYTHON
//   THAT IS ALREADY TESTED (upgrade_firmware.py), instead of being reimplemented
//   in C++ where a mistake could brick the motor by writing a malformed page.
//   The same pipe then also gives every other host tool access to this motor for
//   free, which is worth far more than the upgrader alone.
//
// WHAT IT DOES
//   Everything arriving on USB goes out on RS485. Everything arriving on RS485
//   goes back up USB. Nothing else. In particular it NEVER writes anything of
//   its own onto either link -- no banner, no prompt, no debug print -- because
//   a single stray byte would be parsed by the host as the start of a reply and
//   desynchronise the protocol.
//
// PROTOCOL NOTES THAT MATTER HERE
//   * The link is 230400 baud, 8N1. USB CDC has no real baud rate, so the host
//     can deliver a 2067-byte firmware page far faster than the wire can carry
//     it (~90 ms). Serial1's TX buffer is therefore enlarged, and write() is
//     allowed to block: blocking is CORRECT, it simply applies backpressure.
//   * The RS485 transceiver on this board is auto-direction (the test suite
//     drives Serial1 with no DE/RE pin), so there is no direction line to
//     manage and no turnaround delay to insert.
//   * Firmware upgrade is a BROADCAST to address 255. Any motor on this bus
//     with a matching model code and compatibility code will be programmed.
//     That is the intended behaviour and is why the bench motor, being alone on
//     this bus, can be upgraded safely while the 35-motor rack -- which is on a
//     completely separate adapter -- is untouched.
//
// USAGE
//   arduino-cli compile --upload -p /dev/cu.usbmodem2101 \
//     --fqbn esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc .
//   then point any host tool at /dev/cu.usbmodem2101, e.g.
//     python3 upgrade_firmware.py -p /dev/cu.usbmodem2101 <file>.firmware
//     ./host_suite /dev/cu.usbmodem2101 99856389A2B46555 <module>
//
//   To go back to running tests ON the ESP32 itself, reflash the test suite
//   with Servomotor_TestSuite_ESP32S3/flash_suite.sh.
// ---------------------------------------------------------------------------

#define RS485_TXD 4
#define RS485_RXD 5
#define RS485_BAUD 230400

// A firmware page is 2067 bytes on the wire. Buffers comfortably larger than
// that keep a whole page in flight without the host ever stalling mid-packet,
// which matters because upgrade_firmware.py paces itself with fixed sleeps
// rather than waiting for an acknowledgement.
#define RS485_RX_BUFFER 8192
#define RS485_TX_BUFFER 16384

// The USB side needs a big buffer too, and this is the one that actually bit.
// USB CDC delivers a 2067-byte firmware page in a few milliseconds, while the
// 230400-baud wire needs ~90 ms to carry it. If the sketch is inside a blocking
// Serial1.write() when the next USB packet arrives, the CDC receive buffer --
// only 256 bytes by default -- overflows and those bytes are GONE. The symptom
// is brutal to diagnose: small packets work perfectly, large ones vanish with
// no error anywhere, because the motor simply never sees a well-formed frame.
#define USB_RX_BUFFER 16384

// Moving bytes in blocks rather than one at a time keeps the USB and UART
// drivers working efficiently; 512 is well under either buffer.
#define CHUNK 1024

static uint8_t buf[CHUNK];

void setup() {
    // USB CDC. The baud argument is ignored for CDC but must be present.
    // The receive buffer MUST be enlarged before begin(): see USB_RX_BUFFER.
    Serial.setRxBufferSize(USB_RX_BUFFER);
    Serial.begin(230400);

    Serial1.setRxBufferSize(RS485_RX_BUFFER);
    #if defined(ARDUINO_ARCH_ESP32)
    // Available on ESP32 core 2.0.6+. Harmless to call before begin().
    Serial1.setTxBufferSize(RS485_TX_BUFFER);
    #endif
    Serial1.begin(RS485_BAUD, SERIAL_8N1, RS485_RXD, RS485_TXD);

    // Deliberately NO banner. Anything printed here would be indistinguishable
    // from a motor reply to the host and would break the very first command.
}

void loop() {
    bool moved = false;

    // STRICTLY NON-BLOCKING IN BOTH DIRECTIONS.
    //
    // The obvious implementation uses Serial.readBytes(), and that is a trap.
    // readBytes() inherits Stream's one-second timeout: if the driver reports N
    // bytes available but hands back fewer, it BLOCKS waiting for the rest. For
    // a whole second this loop is stuck in the USB->RS485 direction and cannot
    // move the motor's reply back up to the host, so the host times out on a
    // command the motor answered perfectly. It is rare, load-dependent, and
    // looks exactly like an intermittent firmware fault.
    //
    // read() never blocks when available() > 0, so the pump below can stall in
    // neither direction. Bounding each pass also guarantees the other direction
    // is serviced promptly even under a sustained flood.
    int budget = CHUNK;
    while (budget-- > 0 && Serial.available() > 0) {
        int c = Serial.read();
        if (c < 0) break;
        Serial1.write((uint8_t)c);
        moved = true;
    }

    budget = CHUNK;
    while (budget-- > 0 && Serial1.available() > 0) {
        int c = Serial1.read();
        if (c < 0) break;
        Serial.write((uint8_t)c);
        moved = true;
    }

    // YIELD WHEN IDLE. Not an optimisation, a correctness fix, and it cost a
    // full 111-module run to find. Without it loop() spins continuously and
    // starves the FreeRTOS idle task; short exchanges are fine but sustained
    // traffic eventually trips the task watchdog and resets the ESP32. The host
    // then sees a read timeout, which the test framework reports as fatal code
    // 255 ("the status read itself failed") -- indistinguishable from a
    // firmware fault under load unless you know to suspect the bridge.
    if (!moved) {
        delay(1);
    }
}
