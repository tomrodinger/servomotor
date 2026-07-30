# Arduino Essentials: Checking for Errors and Setting Up Your Environment

Read this section before you write anything beyond the example above. It covers the two things
that account for most Arduino support cases: not checking for errors, and an Arduino IDE setting
that hides your serial output.

## Every call can fail, and a failure is silent unless you ask

None of the library's command methods return a success flag. Instead, each one records the outcome
of its last exchange with the motor, and you read it with `getError()`:

```cpp
motor.enableMosfets();
int e = motor.getError();
if (e != 0) {
  Serial.print("enableMosfets failed, getError() = ");
  Serial.println(e);
}
```

`getError()` returns `0` when the command succeeded. It is updated by every command that talks to
the motor, so read it immediately after the call you care about — the next command overwrites it.
The unit setters (`setPositionUnit()`, `setTimeUnit()`, and friends) are host-side only: they send
nothing, and they leave `getError()` untouched.

**This matters more than it looks.** When the motor latches a fatal error it disables itself and
rejects every subsequent command. Nothing about your sketch changes: the calls still return, the
`delay()`s still elapse, the loop still runs. The motor simply stops moving. Without `getError()`
there is no symptom to see, which is exactly how a working sketch turns into "the motor randomly
stops working".

### What the value means

**Zero** means success.

**A positive value is the motor's own fatal error code**, reported by the motor in its reply. Look
it up in the Error Codes section of this document — the numbers are the same ones the red LED
blinks out. Once a fatal error is latched, every command except *Get status* and *System reset*
comes back with that same code until you reset the device.

**A negative value is a host-side communication failure** — the motor never answered, or the reply
did not survive the trip. These are defined in `Communication.h`:

| Value | Name | What it usually means |
|---|---|---|
| -1 | `COMMUNICATION_ERROR_TIMEOUT` | No reply within 1 second. Wrong alias, wrong wiring or RX/TX pins, motor unpowered, motor sitting in the bootloader, or a CRC32 setting mismatch. This is by far the most common one. |
| -2 | `COMMUNICATION_ERROR_DATA_WRONG_SIZE` | The reply arrived but its payload was not the expected length for this command — usually a firmware/library version mismatch. |
| -3 | `COMMUNICATION_ERROR_BAD_RESPONSE_CHAR` | The reply's response byte was neither 253 nor 252. Something else is talking on the bus, or bytes were lost. |
| -5 | `COMMUNICATION_ERROR_BUFFER_TOO_SMALL` | You passed a buffer smaller than the data the motor sent (the variable-length calls: `getProductDescription()`, `readMultipurposeBuffer()`, `captureHallSensorData()`). Enlarge the buffer and call again. |
| -6 | `COMMUNICATION_ERROR_CRC32_MISMATCH` | The reply's checksum did not match. Electrical noise, missing common ground, or an unterminated bus. |
| -7 | `COMMUNICATION_ERROR_BAD_FIRST_BYTE` | The first byte of the reply was not a valid size byte. Usually leftover bytes in the receive buffer from an earlier partial exchange. |
| -9 | `COMMUNICATION_ERROR_PACKET_TOO_SMALL` | The declared packet size was too small to be a valid reply. |

Two more codes, `-4` (`COMMUNICATION_ERROR_BAD_STATUS_CHAR`) and `-8`
(`COMMUNICATION_ERROR_BAD_THIRD_BYTE`), are defined in `Communication.h` but no code path ever
returns them. Do not write handlers for them.

A timeout is also the normal outcome of a **broadcast**: a motor addressed at alias 255 executes
the command but deliberately sends no reply, so every broadcast command blocks for the full second
and then leaves `getError()` at `-1`. That `-1` is expected, not a fault.

### The trap: a failed read returns 0, not an error

Every value-returning method returns a **zero-initialised default** when the call fails. There is
no error sentinel, no NaN, no exception. So this line:

```cpp
Serial.println(motor.getHallSensorPosition());   // prints "0.00"
```

prints `0.00` both when the shaft is genuinely at zero and when the motor never answered at all.
The same applies to `getPosition()`, `getSupplyVoltage()`, `getNQueuedItems()`, and to the
struct-returning calls such as `getStatus()`, `getComprehensivePosition()` and `getProductInfo()`,
whose fields all come back zeroed.

**Never trust a returned value until you have checked `getError()` for that call.**

```cpp
float pos = motor.getHallSensorPosition();
if (motor.getError() != 0) {
  Serial.println("position read FAILED - the 0.00 below is not real");
} else {
  Serial.println(pos);
}
```

Three shapes hide the failure even better than a plain zero, and are worth knowing about:

- `getTemperature()` runs the zeroed raw value through the unit conversion, so a **failed** call
  returns exactly `32.00` with the unit set to Fahrenheit, or `273.15` with Kelvin. Both look like
  perfectly plausible readings. Only Celsius (the default) gives `0`.
- `getMaxPidError()` returns `{0.0, 0.0}` on failure, which reads as flawless tracking — and it is
  not the documented "no data since the last read" sentinel (which is `min > max`).
- `readMultipurposeBuffer()` and `captureHallSensorData()` set `*actualSize = 0` but leave **your
  buffer untouched**, so it may still hold bytes from an earlier call. And in the
  `-5` (buffer too small) case they set `*actualSize` to the number of bytes that would have been
  *required*, not the number stored — so a nonzero `actualSize` does not mean you have data.

### *Get status* is the one asymmetric command

*Get status* is answered normally even while a fatal error is latched — that is the whole point of
it. So on a motor that has faulted:

- `motor.getStatus()` succeeds, and `getError()` reads **0** for that call.
- The fault is in the returned struct instead: `getStatus().fatalErrorCode` is **nonzero**.
  (`statusFlags` is not useful here: in the fatal state every flag reads 0.)
- Every *other* command on that same motor returns the error code through `getError()`.

`systemReset()` is answered normally in the fatal state too, so it also leaves `getError()` at 0 —
which is what you want, since it is the command that clears the fault.

A health check written only against `getError()` therefore reports "no error" on a dead motor.
Check both. Note that a `getStatus()` that itself failed returns an all-zero struct, which is
indistinguishable from a healthy motor unless you check `getError()` as well — which is why the
helper below checks it twice.

### A helper worth pasting into every sketch

```cpp
// Returns true (and explains itself) if the last call did not succeed.
bool failed(Servomotor &motor, const char *what) {
  int e = motor.getError();
  if (e == 0) return false;
  Serial.print("[FAIL] ");
  Serial.print(what);
  Serial.print(" -> getError() = ");
  Serial.print(e);
  Serial.println(e > 0 ? "  (motor fatal error - see the Error Codes section)"
                       : "  (communication failure - check wiring, alias and power)");
  return true;
}

// Returns 0 when the motor is healthy. Nonzero means trouble:
//   > 0  a fatal error code from the motor (see the Error Codes section)
//   < 0  a communication failure
// Costs one extra round trip, so in a tight loop call it every N iterations
// rather than after every single command.
int motorTrouble(Servomotor &motor) {
  if (motor.getError() != 0) return motor.getError();  // the command just issued failed
  getStatusResponse status = motor.getStatus();        // still answered while a fault is latched
  if (motor.getError() != 0) return motor.getError();  // the status read itself failed
  return (int)status.fatalErrorCode;
}
```

Used together:

```cpp
motor.trapezoidMove(1.0f, 1.0f);
if (failed(motor, "trapezoidMove")) return;
delay(1100);

// Some faults (a stall, a position-deviation trip) happen asynchronously, after the
// command that caused them already returned success. Check again once the move is done.
int code = motorTrouble(motor);
if (code != 0) {
  Serial.print("motor faulted after the move, code ");
  Serial.println(code);
  motor.systemReset();
  delay(1500);          // keep the bus silent while it reboots - see the golden rules
}
```

## Setting up the Arduino environment

### Install the right library

In the Arduino IDE's Library Manager, search for `Servomotor` and install the one spelled
**Servomotor** with a lower-case "m", by **Gearotons**, category *Communication*, repository
`https://github.com/tomrodinger/Servomotor_Arduino_Library`.

The same search also returns **ServoMotor** with a capital "M" by RCmags — an unrelated DIY-servo
library. Its version number is higher than ours, so it tends to appear first in the list. It will
not compile against this documentation, and the class name it provides differs from ours by that
one letter, which makes the resulting error message confusing.

### ESP32-S3: turn on "USB CDC On Boot" or your serial monitor stays blank

On ESP32-S3 boards the Arduino IDE defaults **Tools > USB CDC On Boot** to **Disabled**, which
routes `Serial` to the hardware UART pins (GPIO43/44) instead of to the USB port. On a board whose
only connector is the native USB port, the result is a completely empty serial monitor while the
sketch runs perfectly — no output, no error, nothing.

If you see no output at all from an ESP32-S3, set **Tools > USB CDC On Boot > Enabled** before
suspecting your code or your wiring. (On the command line this is the `CDCOnBoot=cdc` fragment of
the FQBN.)

### The library prints to `Serial` on every command

Every command method prints to `Serial`, unconditionally — there is no flag to switch it off in the
current version. And it is usually more than one line per call: a unit-converting method echoes each
of its arguments and then calls the internal `...Raw()` variant, which prints as well. A single
`motor.trapezoidMove(2.0f, 3.0f)` produces four lines:

```
[Motor] trapezoidMove called.
  displacement in chosen unit: 2.00
  duration in chosen unit: 3.00
[Motor] trapezoidMoveRaw called.
```

This is helpful while bringing a board up, and worth knowing about before you put a command inside a
fast polling loop, where it will dominate your serial output and slow the loop down.

Note that `Serial` (the console) and the RS485 port are different ports: the motor is on `Serial1`
by default, so these messages do not disturb the motor.

### Reading large payloads

The Arduino library's 1-second timeout is a budget for the **entire** reply, and it waits for the
whole payload to be sitting in the UART receive buffer at once. That makes the two bulk reads —
`captureHallSensorData()` and `readMultipurposeBuffer()` — sensitive to the buffer size in a way the
Python library is not (Python restarts its timeout on every byte received). If you need a payload
larger than the platform's default receive buffer (256 bytes on ESP32, 64 on classic AVR), call
`Serial1.setRxBufferSize(n)` **before** constructing the `Servomotor` object — the constructor is
what opens the port, so anything that must precede `begin()` has to happen first.

### Small things that confuse people

- **The alias prints as a number.** `Servomotor motor('X', ...)` reports
  `[Motor] Initialized with standard addressing, Alias: 88`, because `'X'` is character code 88.
  That is the same alias, printed in decimal.
- **The constructor's defaults are already correct** for this product: alias `'X'`, port `Serial1`,
  baud `230400`. You do not need to pass a baud rate. Pass your RX/TX pins on boards that need them,
  as in the example above — note the parameter order is `(alias, serialPort, rxPin, txPin, baud)`,
  so overriding the baud rate means passing the pins too.
- **The RX/TX pin arguments only take effect on ESP32.** On sam, samd, rp2040, stm32 and renesas the
  platform's default `Serial1` pins are used and the arguments are ignored.
- **The port is opened once per sketch, by the first `Servomotor` you construct.** If you drive
  several motors, they share one bus and one port — construct additional objects with the same
  serial port and pins. A second object naming a different port or baud rate will not reconfigure
  anything.
- **The receive timeout is 1 second.** A command against a motor that is not there costs a full
  second before `getError()` returns `-1`. A loop that polls a missing motor will feel frozen.
- **A reply that starts but never finishes times out like any other failure.** A collision, a motor
  rebooting mid-reply, or noise on an unbiased RS485 pair can deliver the beginning of a frame and
  nothing more. That returns `-1` after the usual one-second budget, and the library discards the
  remains of the abandoned frame so they cannot be read as the head of your next reply. (In library
  versions before 0.10.1 this case hung forever with no timeout and no error, and leftover bytes
  surfaced as a spurious `-7` on the following command.)
- **Stray bytes can still linger in some situations** — most often after *Detect devices*, which
  produces several replies over about a second. The library exposes no flush of its own, so if you
  suspect the buffer is out of step, drain it yourself before normal traffic resumes:
  `while (Serial1.available()) Serial1.read();`
