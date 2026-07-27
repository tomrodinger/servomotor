// =============================================================================
//  Servomotor Arduino library — API USAGE BY COMMAND CATEGORY
// =============================================================================
//  A DESIGN-REVIEW sketch (not a working motor program). For every *category*
//  of command it shows exactly what a user writes: how they declare variables,
//  structs, and buffers, and how they read results.
//
//  Legend:
//     [NOW]      = works with the library exactly as it is today.
//     [PROPOSED] = the caller-provided-buffer design for variable-length
//                  results (the BUG-25/26/27 fix). Shown so we can judge the
//                  ergonomics from the user's side before implementing it.
//
//  Design rule for variable-length results (strings OR binary blobs):
//     the CALLER owns the buffer (a stack array or a `static` array) and passes
//     a pointer + its size. The library also fills a caller-supplied
//     "actual size" variable (passed by pointer). No malloc, no heap,
//     deterministic RAM. Success/failure is read through motor.getError(),
//     exactly like every other command.
// =============================================================================

#include <Servomotor.h>

// ESP32 RS485 pins (other boards omit the pin args). Alias 'X' == device alias.
#define ALIAS      'X'
#define RS485_RXD  5
#define RS485_TXD  4

void setup() {
  Serial.begin(115200);

  // ---------------------------------------------------------------------------
  //  SETUP: construct the motor, then pick the units you want to work in.
  //  Every value you pass or read is in these units; the library converts
  //  to/from the device's internal representation for you.
  //
  //  DEFAULTS: if you never call a setXUnit(), the library starts in the FIRST
  //  unit listed for each type in unit_conversions_M3.json — identical to the
  //  Python library. Today those defaults are:
  //     position=shaft_rotations  time=seconds  velocity=rotations_per_second
  //     acceleration=rot/s^2      temperature=celsius
  //     current=internal_current_units   voltage=millivolts
  //  (Setting them explicitly, as below, is good practice regardless.)
  // ---------------------------------------------------------------------------
#if defined(ESP32)
  Servomotor motor(ALIAS, Serial1, RS485_RXD, RS485_TXD);
#else
  Servomotor motor(ALIAS, Serial1);
#endif

  motor.setPositionUnit(PositionUnit::SHAFT_ROTATIONS);      // rotations
  motor.setTimeUnit(TimeUnit::SECONDS);                      // seconds
  motor.setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND); // rot/s
  motor.setTemperatureUnit(TemperatureUnit::CELSIUS);        // deg C

  // After every command you may check the error channel. 0 == success.
  // (Shown once here; it applies to every call below.)
  //   if (motor.getError() != 0) { /* handle comms/device error */ }


  // ===========================================================================
  //  CATEGORY 1 — no input, no output   (fire-and-forget)   [NOW]
  //  You just call it. The only result is motor.getError().
  // ===========================================================================
  motor.enableMosfets();
  motor.disableMosfets();
  motor.systemReset();


  // ===========================================================================
  //  CATEGORY 2 — ONE input parameter, no output   [NOW]
  //  Pass a plain value in the current unit.
  // ===========================================================================
  motor.setMaximumVelocity(2.0f);   // 2.0 rot/s  (uses the velocity unit above)


  // ===========================================================================
  //  CATEGORY 3 — MULTIPLE input parameters, no output   [NOW]
  //  Pass each value positionally, each in its own unit.
  // ===========================================================================
  motor.trapezoidMove(1.0f, 2.0f);  // move 1.0 rotation over 2.0 seconds
  motor.goToPosition(0.0f, 2.0f);   // go to absolute 0.0 rotation over 2.0 s


  // ===========================================================================
  //  CATEGORY 4 — no input, ONE scalar output   [NOW]
  //  The value is the return value. Declare a variable of the obvious type.
  // ===========================================================================
  float position = motor.getPosition();      // float, in rotations
  float tempC    = motor.getTemperature();   // float, in deg C
  Serial.print(F("position="));  Serial.println(position, 3);
  Serial.print(F("tempC="));     Serial.println(tempC, 1);


  // ===========================================================================
  //  CATEGORY 5 — no input, MULTI-FIELD (struct) output   [NOW]
  //  Declare the response struct, assign the call to it, read its fields.
  //  The type is named <command>Response (or ...Converted for unit-converted
  //  results). No buffers needed — it's a fixed-size struct returned by value.
  // ===========================================================================
  getStatusResponse status = motor.getStatus();
  Serial.print(F("statusFlags=0x")); Serial.println(status.statusFlags, HEX);
  Serial.print(F("fatalError="));    Serial.println(status.fatalErrorCode);

  getComprehensivePositionResponseConverted pos = motor.getComprehensivePosition();
  Serial.print(F("commanded=")); Serial.println(pos.commandedPosition, 3);
  Serial.print(F("hall="));      Serial.println(pos.hallSensorPosition, 3);

  // A struct output that itself contains a sub-struct (version number):
  getFirmwareVersionResponse fw = motor.getFirmwareVersion();
  Serial.print(F("fw ")); Serial.print(fw.firmwareVersion.major);
  Serial.print('.');      Serial.print(fw.firmwareVersion.minor);
  Serial.print('.');      Serial.println(fw.firmwareVersion.patch);


  // ===========================================================================
  //  CATEGORY 6 — FIXED-SIZE ARRAY input + struct output   [NOW]
  //  You declare the input array yourself; the fixed-size result comes back
  //  inside a struct (also a fixed array — no length ambiguity).
  // ===========================================================================
  uint8_t pingData[10] = {0,1,2,3,4,5,6,7,8,9};
  pingResponse pong = motor.ping(pingData);     // echoes the 10 bytes back
  Serial.print(F("ping echo[0]=")); Serial.println(pong.responsePayload[0]);


  // ===========================================================================
  //  CATEGORY 7 — ARRAY-OF-STRUCTS input (multimove)   [NOW]
  //  You declare and fill an array of move items, plus a bitmask that says
  //  which items are velocity vs acceleration moves.
  // ===========================================================================
  multimoveListConverted_t moves[3];
  moves[0] = { 2.0f, 1.0f };   // value=2.0 rot/s,   duration=1.0 s
  moves[1] = { 1.0f, 0.5f };   // value=1.0 rot/s^2, duration=0.5 s
  moves[2] = { 0.0f, 0.2f };   // value=0.0,         duration=0.2 s  (stop)
  uint32_t moveTypes = 0b010;  // bit i: 0=velocity move, 1=acceleration move
  motor.multimove(3, moveTypes, moves);


  // ===========================================================================
  //  CATEGORY 8 — VARIABLE-LENGTH STRING output   [PROPOSED]
  //  A string whose length is not known in advance ("Servomotor" today, but the
  //  protocol must handle any length). You provide the buffer AND a variable to
  //  receive the actual length (both by pointer). The library copies the string
  //  into your buffer, null-terminates it, and writes the length. No malloc.
  //  The call itself is void; check motor.getError() like everywhere else.
  // ===========================================================================
  char     nameBuf[64];        // caller owns the buffer
  uint16_t nameLen;            // library fills this with the string length
  motor.getProductDescription(nameBuf, sizeof(nameBuf), &nameLen);
  if (motor.getError() == 0) {
    Serial.print(F("product name (")); Serial.print(nameLen);
    Serial.print(F(" chars): "));      Serial.println(nameBuf);   // null-terminated
  } else if (motor.getError() == COMMUNICATION_ERROR_BUFFER_TOO_SMALL) {
    Serial.println(F("nameBuf too small — make it bigger"));
  }


  // ===========================================================================
  //  CATEGORY 9 — VARIABLE-LENGTH BINARY BLOB output   [PROPOSED]
  //  Same pattern. A `static` array keeps big buffers off the stack on small
  //  boards. You pass buffer + size, and a pointer to a variable that receives
  //  the number of bytes actually written.
  // ===========================================================================

  //  9a) no input, variable-length output — Read multipurpose buffer
  static uint8_t blob[512];    // caller owns the buffer
  uint16_t       blobLen;      // library fills this with the byte count
  motor.readMultipurposeBuffer(blob, sizeof(blob), &blobLen);
  if (motor.getError() == 0) {
    Serial.print(F("read ")); Serial.print(blobLen); Serial.println(F(" bytes"));
    // ... use blob[0 .. blobLen-1] ...
  } else if (motor.getError() == COMMUNICATION_ERROR_BUFFER_TOO_SMALL) {
    Serial.println(F("blob buffer too small"));
  }

  //  9b) MULTIPLE inputs + variable-length output — Capture hall sensor data.
  //      The input parameters come first (as usual), then buffer + size, then
  //      the pointer to the actual-length variable.
  static uint8_t capture[1024];   // caller owns the buffer
  uint16_t       capLen;          // library fills this
  motor.captureHallSensorData(
      /*captureType*/        1,
      /*nPointsToRead*/      100,
      /*channelsBitmask*/    0b111,   // channels 0,1,2
      /*timeStepsPerSample*/ 32,
      /*nSamplesToSum*/      1,
      /*divisionFactor*/     1,
      capture, sizeof(capture), &capLen);   // <-- buffer, size, &actualLength
  if (motor.getError() == 0) {
    Serial.print(F("captured ")); Serial.print(capLen); Serial.println(F(" bytes"));
  }


  // ===========================================================================
  //  CATEGORY 10 — address a specific motor by UNIQUE ID   [NOW]
  //  Every command has an overload whose FIRST argument is a 64-bit unique ID,
  //  so you can talk to one device on a shared bus without changing its alias.
  //  The return value / struct / error behavior is otherwise identical.
  // ===========================================================================
  uint64_t uid = 0x0123456789ABCDEFULL;
  motor.enableMosfets(uid);                    // void command, by unique ID
  float p2 = motor.getPosition(uid);           // <-- STILL returns the float
  Serial.print(F("pos of that motor=")); Serial.println(p2, 3);
  // The proposed variable-length calls get the same uniqueId overload, e.g.:
  //   motor.readMultipurposeBuffer(uid, blob, sizeof(blob), &blobLen);


  // ===========================================================================
  //  CATEGORY 11 — RAW variants: skip unit conversion   [NOW]
  //  Every unit-bearing command has a ...Raw twin that takes/returns integers
  //  in the device's internal units (encoder counts, timesteps). Use these for
  //  exact control or to avoid float math on tiny boards.
  // ===========================================================================
  motor.trapezoidMoveRaw(3276800, 62500);   // 1 rotation (counts) over 2 s (timesteps)
  int64_t posCounts = motor.getPositionRaw();
  Serial.print(F("posCounts=")); Serial.println((long)posCounts);
}

void loop() {
  // Demonstration only — nothing here.
}
