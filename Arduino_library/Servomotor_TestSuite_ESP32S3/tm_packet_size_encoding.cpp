#include "tf_motion_helpers.h"

// ---------------------------------------------------------------------------
// tm_packet_size_encoding.cpp
//
// THE SIZE FIELD OF THE RS485 FRAME, EXERCISED ACROSS ITS ONE DISCONTINUITY.
//
// Every reply the motor sends begins with a length. The protocol spends only
// SEVEN bits on it and reserves the eighth, so the encoding has a seam in it:
//
//     encode_first_byte(n) = (n << 1) | 1        RS485.h:35
//     decode_first_byte(b) = b >> 1              RS485.h:40
//
// The low bit is always 1, which is what lets a receiver recognise the head of
// a frame in a stream of noise (a byte with the low bit clear can never be a
// size). That costs a bit, so a single size byte can only express 0..127 -- and
// the value 127 is not available either, because (127 << 1) | 1 == 255, which
// is FIRST_BYTE_EXTENDED_SIZE (RS485.h:12), the escape that means "the real
// size is the next two bytes, little-endian". So:
//
//     total frame <= 126   ->  one size byte,   b = (total << 1) | 1
//     total frame == 127   ->  UNREPRESENTABLE in the short form (collides)
//     total frame >= 127   ->  three size bytes, 0xFF followed by a uint16
//
// A length field with a hole in it is exactly the kind of thing that works for
// years and then breaks the day someone adds a field to a response struct. And
// the two ends of the link implement the rule in different languages, from
// different source trees: the firmware writes it (RS485.c:594 for the short
// form, RS485.c:631 for the extended one) and the Arduino library reads it
// (Communication.cpp:331-364). Nothing in the suite has ever checked that they
// agree at the seam.
//
// THE PROPERTY THIS MODULE ASSERTS:
//
//     For a reply of any length -- comfortably inside the short form, exactly
//     at the reserved value 127, and past it into the extended form -- the
//     number of payload bytes the receiver decodes is exactly the number the
//     sender put in.
//
// HOW A LENGTH IS OBSERVED WITHOUT SNIFFING THE WIRE. tm_raw_protocol.cpp
// already hand-builds frames and injects them with Serial1.write(); this module
// deliberately does the opposite and stays entirely inside the library, because
// the thing worth testing is whether the SHIPPING decoder agrees with the
// shipping encoder. Three library-level observations make the decoded length
// visible:
//
//   1. `captureHallSensorData` and `readMultipurposeBuffer` hand back an
//      `actualSize` out-parameter, which is `bytesLeftToReadWithoutTheCRC32`
//      (Communication.cpp:454) -- a number computed from the SIZE BYTES alone.
//   2. Every fixed-length getter refuses a reply whose decoded payload is not
//      exactly sizeof(its response struct), setting the error to
//      COMMUNICATION_ERROR_DATA_WRONG_SIZE (-2). So a getter that merely
//      SUCCEEDS has proved its length decoded to the byte.
//   3. The CRC32 the receiver checks is computed over the SIZE BYTES too
//      (Communication.cpp:464 folds sizeBytes[0..sizeByteCount-1] in first,
//      matching RS485.c:637). A reply that validates therefore proves both ends
//      agreed on how many size bytes there were, not merely on the total.
//
// WHAT MAKES THE SEAM REACHABLE. Nearly every reply is a fixed-size struct, and
// the biggest of them (Get debug values, cmd 45) is 119 bytes on the wire -- so
// the whole fixed command set lives below the seam and can never cross it. The
// one command whose reply length is chosen by the caller is Capture hall sensor
// data (cmd 7): its payload is n_points x channels x 2 (main.c:335-336) and it
// is streamed out with the ALWAYS-extended header (main.c:340 ->
// RS485.c:622-643). One extra point adds exactly two payload bytes, so adding
// points one at a time walks the declared frame size across 127 in steps of
// two -- which is the finest resolution this instrument has, and is why the
// seam itself is hit by choosing the point count that lands on it exactly.
//
// HOW THIS DIFFERS FROM tm_hall_capture.cpp, which also counts capture bytes:
// that module is about cmd 7's PARAMETER VALIDATION -- its three error codes,
// its bitmask rules, its payload ceiling -- and its byte counts are all small
// and far from any encoding boundary. Here cmd 7 is not the subject at all; it
// is the instrument, the only available way to dial a reply to a chosen wire
// length. Every size in this file is picked for where it lands relative to 127,
// 255 and 256, and sections 1, 2, 3, 7 and 8 do not involve cmd 7 at all.
//
// SAFETY. Nothing here commands motion except one small confidence move at the
// end, read through the COMMANDED position, which the planner advances whether
// or not the output stage is energised -- so nothing turns. enableMosfets() and
// goToClosedLoop() are never called. Capturing hall data drives nothing: the
// sensors are passive. Every capture uses time_steps_per_sample = 32 (about one
// millisecond per point), which is far slower than the 230400-baud link can
// carry a 2-byte point, so ERROR_CAPTURE_OVERFLOW (21) is never provoked. No
// broadcasts, no alias changes, no test modes: safe on the shared 35-motor rack.
// ---------------------------------------------------------------------------

// --- the protocol's own numbers, from the firmware and the library ----------
static const uint16_t EXT_MARKER_DECODED = 127;  // Communication.h:25 / RS485.h:13
static const uint8_t  EXT_MARKER_ENCODED = 255;  // RS485.h:12 FIRST_BYTE_EXTENDED_SIZE
static const uint16_t SHORT_FORM_OVERHEAD = 3;   // 1 size byte + response char + error code
static const uint16_t EXT_FORM_OVERHEAD   = 5;   // 3 size bytes + response char + error code
static const uint16_t CRC32_BYTES         = 4;

// Library error codes we expect to see by name (Communication.h:11-20).
static const int ERR_BUFFER_TOO_SMALL = -5;      // COMMUNICATION_ERROR_BUFFER_TOO_SMALL

// Capture parameters that are held constant everywhere below, so that the only
// thing varying between probes is the reply LENGTH. capture_type 1, one channel
// (bitmask 1 -> 2 bytes per point), one sample summed, division factor 1.
static const uint8_t  CAP_TYPE   = 1;
static const uint8_t  CAP_MASK   = 1;     // a single channel: 2 bytes per point
static const uint16_t CAP_STEPS  = 32;    // ~1 ms per point: far slower than the link
static const uint16_t CAP_SUM    = 1;
static const uint16_t CAP_DIV    = 1;

// A CEILING ON THE POINT COUNT, for anyone extending this module. At 32 time
// steps per sample a capture takes n_points x 32 / 31250 seconds to stream, and
// the library gives the WHOLE response one budget of TIMEOUT_MS = 1000 ms
// (Communication.cpp:6, spent from the first size byte to the last CRC byte).
// So the reply must stay under about 950 points at this sample rate or a
// perfectly healthy motor times out. The largest capture below is 180 points
// (~185 ms), which is comfortable; do not raise it towards the ceiling without
// also lowering CAP_STEPS, and note that lowering CAP_STEPS runs into
// ERROR_CAPTURE_OVERFLOW (21) from the other side.
static const uint32_t CAP_MAX_SAFE_POINTS = 900;

// Total wire length of a reply carried by the SHORT form: one size byte, the
// response character, the error code, the payload, and the CRC32 if enabled
// (RS485.c:588-594).
static uint16_t shortFormTotal(uint16_t payload, bool crc) {
    return (uint16_t)(SHORT_FORM_OVERHEAD + payload + (crc ? CRC32_BYTES : 0));
}

// Total wire length of a reply carried by the EXTENDED form: the 0xFF marker,
// the 16-bit size, the response character, the error code, the payload, and the
// CRC32 if enabled (RS485.c:627-637).
static uint16_t extFormTotal(uint16_t payload, bool crc) {
    return (uint16_t)(EXT_FORM_OVERHEAD + payload + (crc ? CRC32_BYTES : 0));
}

// Drain leftover RX bytes, bounded by wall clock so a quiet line cannot hang
// the slot. Used after a deliberately-failed read, so that no stray byte can be
// mistaken for the head of the next reply.
static void pseDrainRx(uint32_t maxMs) {
    for (uint32_t t0 = millis(); Serial1.available() && (millis() - t0) < maxMs; ) {
        Serial1.read();
    }
}

// One capture. Returns the byte count the library decoded from the size field;
// `err` receives the library's error code (0 on success) and `fatal` the
// device's latched fatal error. `bufSize` is deliberately a parameter: passing
// a buffer that is too small is how section 5 reads the decoded length back out
// of a reply whose contents were thrown away.
static uint16_t capture(Servomotor* m, uint32_t nPoints, uint8_t* buf,
                        uint16_t bufSize, int* err, uint8_t* fatal) {
    uint16_t actual = 0;
    // Self-explaining failure rather than a mysterious timeout: see the note on
    // CAP_MAX_SAFE_POINTS above.
    if (nPoints > CAP_MAX_SAFE_POINTS) {
        printf("   WARNING: %lu points at %u time steps per sample will outlast the "
               "library's %d ms response budget; this is a TEST BUG, not a motor fault\n",
               (unsigned long)nPoints, (unsigned)CAP_STEPS, 1000);
    }
    m->captureHallSensorData(CAP_TYPE, nPoints, CAP_MASK, CAP_STEPS, CAP_SUM,
                             CAP_DIV, buf, bufSize, &actual);
    *err = m->getError();
    pseDrainRx(50);
    *fatal = tfhFatal(m);
    return actual;
}

// Switch the whole link to no-CRC framing. Order matters: the firmware is told
// first -- the REQUEST still carries a CRC32, because the firmware has not
// switched yet and still wants one -- and only then does the library stop
// adding them. Doing it the other way round desynchronises the link and
// everything times out. Recipe proven in tm_crc32_matrix.cpp.
//
// Note what the firmware actually does (main.c, CRC32_CONTROL_COMMAND): it
// calls rs485_set_crc32_enable() BEFORE transmitting the acknowledgement, so
// the REPLY already comes back unprotected. getResponse copes with that because
// it decides whether to expect a CRC32 from the response CHARACTER in the frame
// (253 vs 252), not from the library's own setting. The consequence that
// matters is for recovery: if that reply were ever lost, the firmware would be
// in no-CRC framing while the library still believed CRC32 was on. See
// restoreFraming() below, which is built to survive exactly that.
static bool crcOff(Servomotor* m) {
    m->crc32Control(0);                       // cmd 46, CRC32 control
    if (m->getError() != 0) return false;
    m->disableCRC32();
    return true;
}

// Back to the default framing. A system reset restores the FIRMWARE side (it
// boots with CRC32 on), so the library has to be put back in step with it.
//
// THREE resets, deliberately: a reset framed the way the device is not
// expecting does not land, and on a shared 35-motor rack a device left in
// no-CRC framing would break every module that ran afterwards. The two
// mis-framings are NOT symmetric, which is what makes the ordering below the
// safe one -- both directions traced through RS485.c:206-262:
//
//   * host omits the CRC32 while the firmware wants one: packet_size (3) is
//     smaller than bytes_parsed + command + crc32_size (7), so
//     rs485_get_next_packet() counts a decode error and DISCARDS the packet.
//     Harmless: nothing is latched.
//   * host sends a CRC32 while the firmware does not want one: the four CRC
//     bytes are counted as payload, so payload_size is 4 and
//     check_payload_size_is_zero() raises fatal error ERROR_COMMAND_SIZE_WRONG
//     (main.c). That one costs a latched fault and must be recoverable from.
//
// The device can only be in one of two states here, and this sequence covers
// both without ever needing the caller to know which:
//
//   * the switch took effect (library off, firmware off): reset #1 lands and
//     the firmware comes back up on CRC32; reset #2 is then the harmless
//     discarded kind; reset #3 lands.
//   * the switch did not take effect at the library because the reply was lost
//     (library on, firmware off): reset #1 is the harmful kind and faults the
//     device -- but reset #2, sent after the library is forced to no-CRC,
//     lands, and a reset clears the fault; reset #3 lands.
//
// Either way the firmware finishes on CRC32 with the library in step, and the
// LAST packet sent is always a correctly-framed one.
static void restoreFraming(Servomotor* m) {
    tfhReset(m);                              // in whatever framing is in force
    m->disableCRC32();
    tfhReset(m);                              // the other possibility
    m->enableCRC32();                         // the firmware boots with CRC32 on
    tfhReset(m);                              // now provably in step
}

void tm_packet_size_encoding(void) {
    Serial.println("tm_packet_size_encoding: BEGIN\n");
    Servomotor* m = tfGetMotor();

    static uint8_t buf[1024];

    // =====================================================================
    // 1. THE ENCODING RULE ITSELF, AND WHY 127 IS RESERVED.
    //
    //    These are the shared helpers every frame in both directions passes
    //    through (Communication.h:40-50). They are three-line inline functions
    //    that nothing tests, and they encode a rule with a hole in it. Checking
    //    them costs nothing and pins the constants: if a future change widened
    //    the size field or moved the escape value, the assertions below would
    //    be the first thing to notice.
    // =====================================================================
    printf("\n--- 1. the size-byte encoding rule ---\n");
    {
        bool roundTrips = true, lsbAlwaysSet = true;
        int firstBadRoundTrip = -1;
        for (int n = 1; n <= 126; n++) {
            uint8_t enc = encodeFirstByte((uint8_t)n);
            if ((enc & 0x01) == 0) lsbAlwaysSet = false;
            if (decodeFirstByte(enc) != (uint8_t)n) {
                roundTrips = false;
                if (firstBadRoundTrip < 0) firstBadRoundTrip = n;
            }
        }
        printf("   sizes 1..126 round-trip: %d (first failure %d)\n",
               (int)roundTrips, firstBadRoundTrip);
        TEST_RESULT("every frame length from 1 to 126 survives encode-then-decode",
                    roundTrips);
        TEST_RESULT("every encoded size byte has its low bit set, so it is "
                    "distinguishable from mid-frame noise",
                    lsbAlwaysSet);

        // The collision that creates the reserved value. 127 cannot be a short
        // form length because its encoding IS the extended-size escape byte.
        uint8_t enc127 = encodeFirstByte((uint8_t)EXT_MARKER_DECODED);
        printf("   encodeFirstByte(127) = %u; the extended-size marker is %u\n",
               (unsigned)enc127, (unsigned)EXT_MARKER_ENCODED);
        TEST_RESULT("a frame length of 127 would encode to the very byte that "
                    "means 'extended size', so 127 is reserved",
                    enc127 == EXT_MARKER_ENCODED &&
                    decodeFirstByte(EXT_MARKER_ENCODED) == EXT_MARKER_DECODED);

        // And past the reservation the single byte has simply run out: the
        // shift overflows the byte, so 128 does not merely collide, it vanishes.
        uint8_t enc128 = encodeFirstByte((uint8_t)128);
        printf("   encodeFirstByte(128) = %u, which decodes back to %u\n",
               (unsigned)enc128, (unsigned)decodeFirstByte(enc128));
        TEST_RESULT("a frame length of 128 cannot be expressed in one size byte "
                    "at all, so the extended form is not optional",
                    decodeFirstByte(enc128) != 128);
    }
    tfhReset(m);

    // =====================================================================
    // 2. EVERY FIXED-LENGTH REPLY DECODES TO ITS EXACT DECLARED LENGTH, AND
    //    THE WHOLE FIXED COMMAND SET LIVES BELOW THE SEAM.
    //
    //    Each of these getters compares the decoded payload length against
    //    sizeof(its response struct) and fails with
    //    COMMUNICATION_ERROR_DATA_WRONG_SIZE (-2) if they differ (the pattern
    //    at Servomotor.cpp:708-714 and repeated for every fixed getter). So a
    //    getter that returns cleanly has proved its length decoded to the byte,
    //    and -- since CRC32 is on by default and covers the size bytes -- that
    //    both ends agreed on how many size bytes there were.
    //
    //    The lengths are printed as a table because the interesting number is
    //    the HEADROOM: the largest fixed reply in the protocol is Get debug
    //    values, and it is much closer to the reserved 127 than anyone would
    //    guess. Adding two more int32 fields to it would silently break the
    //    protocol, and this section is where that would be caught.
    // =====================================================================
    printf("\n--- 2. fixed-length replies, and how close they get to 127 ---\n");
    {
        tfhReset(m);
        uint16_t largestTotal = 0;
        const char* largestName = "";

        struct { const char* name; uint16_t payload; bool ok; } probes[7];
        int n = 0;

        // 1-byte payload: Get n queued items (cmd 11).
        m->getNQueuedItems();
        probes[n].name = "Get n queued items (cmd 11)";
        probes[n].payload = (uint16_t)sizeof(getNQueuedItemsResponse);
        probes[n].ok = (m->getError() == 0);
        n++;

        // 2-byte payload: Get supply voltage (cmd 38).
        m->getSupplyVoltageRaw();
        probes[n].name = "Get supply voltage (cmd 38)";
        probes[n].payload = (uint16_t)sizeof(getSupplyVoltageResponse);
        probes[n].ok = (m->getError() == 0);
        n++;

        // 3-byte payload: Get status (cmd 16).
        m->getStatus();
        probes[n].name = "Get status (cmd 16)";
        probes[n].payload = (uint16_t)sizeof(getStatusResponse);
        probes[n].ok = (m->getError() == 0);
        n++;

        // 8-byte payload: Get position (cmd 34).
        m->getPositionRaw();
        probes[n].name = "Get position (cmd 34)";
        probes[n].payload = (uint16_t)sizeof(getPositionResponse);
        probes[n].ok = (m->getError() == 0);
        n++;

        // 10-byte payload: Ping (cmd 31) -- the only fixed reply whose contents
        // we also know in advance, so check the echo as well as the length.
        {
            uint8_t out[10];
            for (int i = 0; i < 10; i++) out[i] = (uint8_t)(0x5A ^ (i * 31));
            pingResponse pr = m->ping(out);
            bool echoed = (m->getError() == 0) &&
                          (memcmp(pr.responsePayload, out, 10) == 0);
            probes[n].name = "Ping (cmd 31)";
            probes[n].payload = (uint16_t)sizeof(pingResponse);
            probes[n].ok = echoed;
            n++;
        }

        // 28-byte payload: Get product info (cmd 22).
        m->getProductInfo();
        probes[n].name = "Get product info (cmd 22)";
        probes[n].payload = (uint16_t)sizeof(getProductInfoResponse);
        probes[n].ok = (m->getError() == 0);
        n++;

        // 112-byte payload: Get debug values (cmd 45) -- the largest fixed
        // reply the firmware sends (main.c:1004-1115).
        m->getDebugValues();
        probes[n].name = "Get debug values (cmd 45)";
        probes[n].payload = (uint16_t)sizeof(getDebugValuesResponse);
        probes[n].ok = (m->getError() == 0);
        n++;

        bool noneReachesTheSeam = true;
        bool noneEncodesToTheMarker = true;
        for (int i = 0; i < n; i++) {
            uint16_t total = shortFormTotal(probes[i].payload, true);
            uint8_t enc = (uint8_t)((total << 1) | 1);
            printf("   %-30s payload %3u -> frame %3u -> size byte %3u\n",
                   probes[i].name, (unsigned)probes[i].payload,
                   (unsigned)total, (unsigned)enc);
            if (total >= EXT_MARKER_DECODED) noneReachesTheSeam = false;
            if (enc == EXT_MARKER_ENCODED) noneEncodesToTheMarker = false;
            if (total > largestTotal) { largestTotal = total; largestName = probes[i].name; }
            TEST_RESULT(std::string(probes[i].name) +
                            " decodes to exactly " +
                            std::to_string((unsigned)probes[i].payload) + " payload bytes",
                        probes[i].ok);
        }

        printf("   largest fixed reply: %s at %u bytes; short-form ceiling is 126, "
               "so the headroom is %d bytes\n",
               largestName, (unsigned)largestTotal, (int)126 - (int)largestTotal);
        // Scoped honestly: this observes the seven replies probed above, not
        // every fixed reply in the protocol. That is still the right guard,
        // because Get debug values (112 bytes) is by a wide margin the largest
        // response struct in Servomotor.h -- the runners-up are Get hall sensor
        // statistics at 40 and Get product info at 28 -- so if anything is ever
        // going to reach the seam it is this one, and it is in the list.
        TEST_RESULT("none of the fixed-length replies probed here reaches the "
                    "reserved frame length of 127",
                    noneReachesTheSeam && noneEncodesToTheMarker);
        TEST_RESULT("the largest fixed reply is Get debug values at 119 bytes, "
                    "leaving only 7 bytes of short-form headroom",
                    largestTotal == 119);
    }
    tfhReset(m);

    // =====================================================================
    // 3. A VARIABLE-LENGTH REPLY IN THE SHORT FORM: GET PRODUCT DESCRIPTION.
    //
    //    Despite the name, cmd 24's reply is a fixed 11 bytes: the firmware
    //    sends sizeof(PRODUCT_DESCRIPTION) where PRODUCT_DESCRIPTION is the
    //    literal "Servomotor" (main.c:60, main.c:586-590), so the terminating
    //    NUL travels on the wire and the frame is 3 + 11 + 4 = 18 bytes.
    //
    //    It is worth its own section because the library reports TWO different
    //    numbers for it, and both are correct:
    //      * on success, actualSize is strlen() of the received text -- 10;
    //      * on the buffer-too-small path, actualSize is the DECODED WIRE
    //        LENGTH -- 11 (Servomotor.cpp:1008-1012).
    //    That second number is a direct readout of the size field, and it is
    //    the measuring technique the boundary sections below depend on, so it
    //    is calibrated here against a reply whose length is known from source.
    // =====================================================================
    printf("\n--- 3. Get product description: 10 characters, 11 bytes on the wire ---\n");
    {
        tfhReset(m);
        char text[64];
        uint16_t chars = 0;
        memset(text, 0, sizeof(text));
        m->getProductDescription(text, sizeof(text), &chars);
        bool readOk = (m->getError() == 0);
        printf("   text=\"%s\" characters=%u -> frame would be %u bytes\n",
               readOk ? text : "<failed>", (unsigned)chars,
               (unsigned)shortFormTotal(11, true));
        TEST_RESULT("the product description reads back as 10 characters of text",
                    readOk && chars == 10 && strlen(text) == 10);

        // Now ask for it into a one-byte buffer. The payload cannot fit, so the
        // library reports how many bytes the size field said were coming.
        char tiny[1];
        uint16_t wireLen = 0;
        tiny[0] = 'x';
        m->getProductDescription(tiny, sizeof(tiny), &wireLen);
        int err = m->getError();
        pseDrainRx(50);
        printf("   into a 1-byte buffer: err=%d wire length=%u (expect %d and 11)\n",
               err, (unsigned)wireLen, ERR_BUFFER_TOO_SMALL);
        TEST_RESULT("read into a buffer too small for it, the library reports the "
                    "true wire length of 11 bytes, one more than the text",
                    err == ERR_BUFFER_TOO_SMALL && wireLen == 11);
    }
    tfhReset(m);

    // =====================================================================
    // 4. WALKING THE DECLARED FRAME LENGTH ACROSS 127.
    //
    //    This is the point of the module. A capture's declared frame length is
    //    5 header bytes + 2 bytes per point + 4 CRC bytes (RS485.c:632-636), so
    //    each extra point moves it by exactly 2 and the sizes below step
    //    123, 125, 127, 129, 131, 133 -- two under the seam, the seam itself,
    //    and three over it.
    //
    //    The middle one is the interesting case, and it is subtler than "just
    //    below and just above". A capture always uses the EXTENDED form, so at
    //    59 points the frame's first byte is 0xFF and its 16-bit size field
    //    holds the value 127 -- the reserved short-form length appearing as an
    //    ordinary number one layer down. A decoder that confused the escape
    //    value with a size, or re-applied the short-form rule to the extended
    //    field, would fail here and nowhere else.
    //
    //    Captures run back to back without a reset between them: proven safe by
    //    tm_hall_capture section 7 (ten consecutive captures, identical sizes).
    // =====================================================================
    printf("\n--- 4. the declared frame length stepped across the reserved 127 ---\n");
    {
        tfhReset(m);
        const uint32_t pointCounts[] = { 57, 58, 59, 60, 61, 62 };
        bool allExact = true;
        bool seamSeen = false;
        for (unsigned i = 0; i < sizeof(pointCounts) / sizeof(pointCounts[0]); i++) {
            uint32_t nPoints = pointCounts[i];
            uint16_t wantPayload = (uint16_t)(nPoints * 2u);
            uint16_t wantFrame = extFormTotal(wantPayload, true);
            int err = 0; uint8_t fatal = 0;
            memset(buf, 0, 512);
            uint16_t got = capture(m, nPoints, buf, sizeof(buf), &err, &fatal);
            bool exact = (err == 0) && (fatal == 0) && (got == wantPayload);
            if (!exact) allExact = false;
            if (wantFrame == EXT_MARKER_DECODED) seamSeen = true;
            printf("   %lu points -> frame %3u bytes: decoded %3u payload bytes "
                   "(want %3u) err=%d fatal=%u\n",
                   (unsigned long)nPoints, (unsigned)wantFrame, (unsigned)got,
                   (unsigned)wantPayload, err, (unsigned)fatal);
            TEST_RESULT(std::string("a reply declaring a frame length of ") +
                            std::to_string((unsigned)wantFrame) +
                            " decodes to exactly " +
                            std::to_string((unsigned)wantPayload) + " payload bytes",
                        exact);
        }
        TEST_RESULT("the whole run across the boundary is exact, including the "
                    "reply whose declared length is the reserved value 127",
                    allExact && seamSeen);
    }
    tfhReset(m);

    // =====================================================================
    // 5. THE LENGTH COMES FROM THE SIZE FIELD, NOT FROM COUNTING THE DATA.
    //
    //    Section 4 could in principle be satisfied by a receiver that ignored
    //    the size field and simply read until the line went quiet. This section
    //    rules that out: the payload is thrown away as it arrives (the caller's
    //    buffer is one byte), so the only thing the reported number can come
    //    from is the decoded size field itself (Communication.cpp:444-451).
    //
    //    It is also the harshest recovery test in the module. The library has
    //    to abandon a frame mid-payload, discard the remainder, and leave the
    //    link usable -- straddling the boundary, where the number of size bytes
    //    to skip is itself in question.
    // =====================================================================
    printf("\n--- 5. the reported length is read from the size field ---\n");
    {
        tfhReset(m);
        const uint32_t pointCounts[] = { 58, 59, 60 };   // frames 125, 127, 129
        for (unsigned i = 0; i < sizeof(pointCounts) / sizeof(pointCounts[0]); i++) {
            uint32_t nPoints = pointCounts[i];
            uint16_t wantPayload = (uint16_t)(nPoints * 2u);
            uint16_t wantFrame = extFormTotal(wantPayload, true);
            int err = 0; uint8_t fatal = 0;
            uint8_t tiny[1];
            tiny[0] = 0;
            uint16_t got = capture(m, nPoints, tiny, sizeof(tiny), &err, &fatal);
            printf("   frame %3u into a 1-byte buffer: err=%d reported %3u "
                   "(want %3u) fatal=%u\n",
                   (unsigned)wantFrame, err, (unsigned)got,
                   (unsigned)wantPayload, (unsigned)fatal);
            TEST_RESULT(std::string("a ") + std::to_string((unsigned)wantFrame) +
                            "-byte frame read into a one-byte buffer still reports "
                            "its full length of " +
                            std::to_string((unsigned)wantPayload) + " bytes",
                        err == ERR_BUFFER_TOO_SMALL && got == wantPayload &&
                        fatal == 0);
        }

        // The link must still be usable: three frames were abandoned mid-flight.
        int err = 0; uint8_t fatal = 0;
        uint16_t got = capture(m, 59, buf, sizeof(buf), &err, &fatal);
        printf("   a full 59-point capture afterwards: err=%d decoded %u bytes\n",
               err, (unsigned)got);
        TEST_RESULT("after three frames are abandoned mid-payload the very next "
                    "capture is read in full",
                    err == 0 && fatal == 0 && got == 118);
    }
    tfhReset(m);

    // =====================================================================
    // 6. THE EXTENDED SIZE FIELD IS SIXTEEN BITS, NOT EIGHT.
    //
    //    Past 127 the length lives in a little-endian uint16 (RS485.c:627), and
    //    that field has boundaries of its own. Three sizes probe them:
    //
    //      123 points -> frame 255: the LOW byte of the size field is 0xFF,
    //                    the escape byte itself, appearing inside the frame.
    //      124 points -> frame 257: the first length whose HIGH byte is
    //                    non-zero, so a decoder that read only one of the two
    //                    bytes would report 1 instead of 257.
    //      180 points -> frame 369: comfortably into two-byte territory
    //                    (0x0171), and a 360-byte payload. That is the same
    //                    payload size tm_hall_capture section 6 already reads
    //                    back exactly (60 points on three channels), so the
    //                    byte count is known to fit both the ESP32's 4096-byte
    //                    RX buffer and the host build's tty buffer; only the
    //                    SHAPE of the size field is new here.
    // =====================================================================
    printf("\n--- 6. the 16-bit extended size field ---\n");
    {
        tfhReset(m);
        const uint32_t pointCounts[] = { 123, 124, 180 };
        for (unsigned i = 0; i < sizeof(pointCounts) / sizeof(pointCounts[0]); i++) {
            uint32_t nPoints = pointCounts[i];
            uint16_t wantPayload = (uint16_t)(nPoints * 2u);
            uint16_t wantFrame = extFormTotal(wantPayload, true);
            int err = 0; uint8_t fatal = 0;
            memset(buf, 0, sizeof(buf));
            uint16_t got = capture(m, nPoints, buf, sizeof(buf), &err, &fatal);
            printf("   %lu points -> frame %3u (size bytes %02X %02X): decoded %3u "
                   "(want %3u) err=%d fatal=%u\n",
                   (unsigned long)nPoints, (unsigned)wantFrame,
                   (unsigned)(wantFrame & 0xFF), (unsigned)(wantFrame >> 8),
                   (unsigned)got, (unsigned)wantPayload, err, (unsigned)fatal);
            TEST_RESULT(std::string("a frame of ") + std::to_string((unsigned)wantFrame) +
                            " bytes decodes to exactly " +
                            std::to_string((unsigned)wantPayload) + " payload bytes",
                        err == 0 && fatal == 0 && got == wantPayload);
        }
    }
    tfhReset(m);

    // =====================================================================
    // 7. THE BOUNDARY BELONGS TO THE FRAME, NOT TO THE PAYLOAD.
    //
    //    Turning CRC32 off removes four bytes from every reply (RS485.c:590-592
    //    for the short form, RS485.c:636 for the extended one). If the size
    //    rule is really about the TOTAL frame, then the payload size that lands
    //    on 127 must move by exactly two points when those four bytes go away:
    //    59 points with CRC32 on, 61 points with it off.
    //
    //    Asserting that is a sharper statement than "both framings work"
    //    (which tm_crc32_matrix already establishes across the command set).
    //    Here the claim is that the SAME request yields the same payload length
    //    in both framings while occupying different amounts of wire, which is
    //    only true if the receiver is subtracting the CRC from a total it took
    //    from the size field rather than assuming a layout.
    // =====================================================================
    printf("\n--- 7. removing the CRC32 moves the boundary by two points ---\n");
    {
        tfhReset(m);

        // The reference measurement, taken in the default framing: 59 points,
        // a 127-byte frame, 118 payload bytes.
        int errRef = 0; uint8_t fRef = 0;
        memset(buf, 0, sizeof(buf));
        uint16_t gotRef = capture(m, 59, buf, sizeof(buf), &errRef, &fRef);

        bool switched = crcOff(m);
        TEST_RESULT("CRC32 can be turned off without losing the link", switched);

        uint16_t got61 = 0, got59 = 0;
        int err61 = -99, err59 = -99;
        uint8_t f61 = 0xFF, f59 = 0xFF;
        if (switched) {
            memset(buf, 0, sizeof(buf));
            got61 = capture(m, 61, buf, sizeof(buf), &err61, &f61);
            memset(buf, 0, sizeof(buf));
            got59 = capture(m, 59, buf, sizeof(buf), &err59, &f59);
        }
        printf("   CRC32 on : 59 points -> frame %u, decoded %u\n",
               (unsigned)extFormTotal(118, true), (unsigned)gotRef);
        printf("   CRC32 off: 61 points -> frame %u, decoded %u (want 122); "
               "59 points -> frame %u, decoded %u (want 118)\n",
               (unsigned)extFormTotal(122, false), (unsigned)got61,
               (unsigned)extFormTotal(118, false), (unsigned)got59);
        TEST_RESULT("with CRC32 off it is 61 points, not 59, whose frame length "
                    "is exactly 127 -- and it decodes to 122 payload bytes",
                    switched && err61 == 0 && f61 == 0 && got61 == 122 &&
                    extFormTotal(122, false) == EXT_MARKER_DECODED);
        // The same request, measured in both framings: identical payload,
        // different amounts of wire. Only a receiver that takes the total from
        // the size field and then subtracts the CRC can get both right.
        TEST_RESULT("the same 59-point request returns the same 118 payload bytes "
                    "in both framings, four bytes shorter on the wire",
                    errRef == 0 && fRef == 0 && gotRef == 118 &&
                    switched && err59 == 0 && f59 == 0 && got59 == gotRef);

        restoreFraming(m);
        m->getStatus();
        TEST_RESULT("the default CRC32 framing is restored and the link works",
                    m->getError() == 0 && m->isCRC32Enabled());
    }
    tfhReset(m);

    // =====================================================================
    // 8. BOTH ENCODINGS COEXIST ON THE SAME LINK, BACK TO BACK.
    //
    //    The choice of form is made per handler, not per size. Read multipurpose
    //    buffer (cmd 35) shows both: an EMPTY buffer is answered with the short
    //    form -- a single data_type byte of 0 in an 8-byte frame
    //    (main.c:793-801) -- while a non-empty one uses the extended form
    //    regardless of how little data it holds (main.c:811). Capture (cmd 7)
    //    is always extended even for two bytes of payload.
    //
    //    So a one-point capture is a 2-byte payload delivered in the LONGER
    //    encoding, immediately after a 1-byte payload delivered in the shorter
    //    one. If the receiver ever carried state from one frame's form into the
    //    next, this is where it would show.
    //
    //    (The non-empty multipurpose case is not probed here: filling that
    //    buffer requires going to closed loop, and the output stage stays off.)
    // =====================================================================
    printf("\n--- 8. short form and extended form back to back ---\n");
    {
        tfhReset(m);
        uint16_t mpSize = 0;
        uint8_t mpBuf[64];
        memset(mpBuf, 0xAA, sizeof(mpBuf));
        m->readMultipurposeBuffer(mpBuf, sizeof(mpBuf), &mpSize);
        bool mpOk = (m->getError() == 0);
        printf("   empty multipurpose buffer: err=%d decoded %u byte(s), value %u, "
               "frame %u (short form)\n",
               m->getError(), (unsigned)mpSize, (unsigned)mpBuf[0],
               (unsigned)shortFormTotal(1, true));
        TEST_RESULT("an empty multipurpose buffer answers with exactly one payload "
                    "byte, carried by the short form",
                    mpOk && mpSize == 1);
        TEST_RESULT("that byte is the data-type marker 0, meaning the buffer is empty",
                    mpOk && mpBuf[0] == 0);

        // Immediately afterwards, the smallest possible extended-form reply.
        int err = 0; uint8_t fatal = 0;
        memset(buf, 0, 16);
        uint16_t got = capture(m, 1, buf, sizeof(buf), &err, &fatal);
        printf("   one-point capture: decoded %u byte(s), frame %u (extended form) "
               "err=%d fatal=%u\n",
               (unsigned)got, (unsigned)extFormTotal(2, true), err, (unsigned)fatal);
        TEST_RESULT("a two-byte payload still arrives correctly in the extended "
                    "form, straight after a short-form reply",
                    err == 0 && fatal == 0 && got == 2);
    }
    // NOTE: deliberately NO reset here. Section 9 has to observe the state the
    // probing actually left behind, and a reset between the two would clear
    // exactly the residue it claims to be looking for.

    // =====================================================================
    // 9. NO RESIDUE. Length probing must not leave the machine altered: reading
    //    replies of unusual sizes, abandoning some of them mid-frame and
    //    switching the framing twice should all be invisible afterwards.
    //
    //    Read BEFORE resetting, or the check is vacuous. The same pattern (a
    //    queue-depth and fatal-error read taken straight after a run of
    //    captures, with no intervening reset) is what tm_hall_capture section 7
    //    does after ten consecutive captures, and it passes -- so this is a
    //    proven-safe way to observe rather than a new risk.
    // =====================================================================
    printf("\n--- 9. nothing was left behind ---\n");
    {
        uint8_t queued = m->getNQueuedItems();
        bool queueOk = (m->getError() == 0) && (queued == 0);
        uint8_t latched = tfhFatal(m);
        printf("   queue depth %u, latched fatal %u\n",
               (unsigned)queued, (unsigned)latched);
        TEST_RESULT("no queue slots were consumed by any of the length probes",
                    queueOk);
        TEST_RESULT("no fatal error was latched anywhere in the module",
                    latched == 0);

        tfhFreshOpen(m);
        bool ok = false;
        int64_t moved = tfhMove(m, (int32_t)(TFH_CPR / 4), TFH_TPS, &ok);
        printf("   confidence move: ok=%d moved %lld of %lld\n",
               (int)ok, (long long)moved, (long long)(TFH_CPR / 4));
        TEST_RESULT("an ordinary move still works after the whole sweep",
                    ok && llabs((long long)(moved - TFH_CPR / 4)) < 200);
    }

    tfhReset(m);
}
