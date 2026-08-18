# Know-How, Best Practices, and Gotchas

Everything in this section was verified against the firmware source code and the project's working test programs. Follow these practices and the motor will behave predictably; ignore them and you will hit fatal errors, lockups, or silent misbehavior.

## The golden rules

1. After 'System reset' (and after power-on), wait at least 0.5 seconds before sending ANYTHING on the bus, and keep the bus completely silent during that wait. The device passes through a bootloader window before launching the application. Any valid packet addressed to it (or broadcast) in that window pins it in the bootloader, where normal commands do not work ('Get status' then returns flags 0x0001). The bundled test programs wait 1.5 s for extra margin; 0.5 s is the working minimum with 2x margin. If a device does get pinned in the bootloader, send 'System reset' again and wait the full delay. Evidence: bench-measured boundary — probes 150-200 ms after reset pinned the device in the bootloader 5/5 times; at 250 ms and later, 0/5, so the window ends between 200 and 250 ms; probing early pins RELIABLY, not occasionally.
2. Begin every session with 'System reset' (then the post-reset wait). This guarantees a known-clean state: MOSFETs disabled, default limits and gains, CRC32 enabled, no latched fatal error, position and clock zeroed.
3. After 'Enable MOSFETs' the motor is immediately ready to use, but it may twitch or rotate slightly as the rotor snaps to the nearest commutation step (bench-measured up to ~4 degrees). Only if you are about to zero the position precisely or arm a tight position-deviation limit, let that transient settle first (about 0.3 s) — zeroing or arming tight limits during the twitch gives a corrupted zero or a spurious fatal error 45. For twitch-sensitive mechanisms, enter closed loop directly with go_to_closed_loop() instead: its engagement is dramatically gentler (bench-measured ~0.01 degrees).
4. Every velocity or acceleration move sequence MUST end with the motor back at zero velocity before the queue empties. A velocity move does not stop by itself — the motor holds the last commanded velocity, and when the queue runs dry at nonzero velocity the firmware raises fatal error 18 (ERROR_RUN_OUT_OF_QUEUE_ITEMS) within one control tick. End velocity sequences with move_with_velocity(0, 0.01) queued back-to-back with the moving segments; end acceleration sequences with a mirrored deceleration segment.
<!--LANG:PYTHON-->
5. Fatal errors LATCH. Once any fatal error trips, the motor disables itself, the red LED blinks the error code, and every command except 'Get status' and 'System reset' fails (each failing command's error reply carries the code). Bench-verified exhaustively: 12 probed commands — ping, all position/time/temperature readers, enable/disable MOSFETs, emergency stop, zero position, reset time, detect devices — ALL receive the error reply, and NONE of them un-latches the fault; 'System reset' alone recovers. First find out WHICH error it is — a FatalError exception carries the code (e.args[0] in Python), or read it with 'Get status' — and look it up so you can address the cause. Then clear it with system_reset() followed by the post-reset wait (0.5 s or more of bus silence).
<!--LANG:END-->
<!--LANG:ARDUINO-->
5. Fatal errors LATCH. Once any fatal error trips, the motor disables itself, the red LED blinks the error code, and every command except 'Get status' and 'System reset' fails (each failing command's error reply carries the code). Bench-verified exhaustively: 12 probed commands — ping, all position/time/temperature readers, enable/disable MOSFETs, emergency stop, zero position, reset time, detect devices — ALL receive the error reply, and NONE of them un-latches the fault; 'System reset' alone recovers. THIS IS THE SINGLE MOST COMMON REASON A WORKING SKETCH "STOPS WORKING": nothing about your program changes, every call still returns, and the motor simply ignores everything from then on. First find out WHICH error it is — getError() returns the code from any ordinary command, or read getStatus().fatalErrorCode — and look it up so you can address the cause. Then clear it with systemReset() followed by the post-reset wait (0.5 s or more of bus silence). See the Arduino Essentials section for how to check.
<!--LANG:END-->
6. NEVER send 'Test mode' with values 10-13 (LED tests): they lock the firmware up after replying, and only a power cycle (or the physical reset button) recovers the device. As of firmware 0.15.4.0, 'Test mode' 0 safely clears all test modes; in OLDER firmware value 0 also locks the device up — 'System reset' always safely clears test modes on any firmware.
7. An empty queue does not always mean motion is finished: if a commanded velocity exceeded what the motor can physically reach (~8.6 rot/s unloaded), the commanded position runs ahead of the rotor and, in closed loop, the motor keeps moving after the queue empties until it catches up (bench-measured: 0.35 s of extra travel after a 1 s profile at an unattainable speed). For true motion-complete, require queue empty AND position settled (two consecutive equal hall readings, or commanded-minus-measured within tolerance). Practically: poll 'Get n queued items' to 0, then allow an extra 0.1-0.3 s for mechanical settling before reading exact positions (polling at full rate is harmless; a 0.01-0.1 s sleep between polls just saves host CPU). Alternatively sleep the commanded duration plus ~5% margin (the motor's clock is accurate to well under 1%; +0.41% offset measured on the bench).
8. The motion queue holds at most 32 items, shared by all move commands. A 'Trapezoid move' or 'Go to position' normally occupies 3 slots (accelerate/coast/decelerate — degenerate phases still take a slot, including a zero-displacement dwell), so at most 10 such moves fit. BUT SINCE FIRMWARE 0.15.12.0 IT CAN BE 2, AND THE CONDITION IS COUNTERINTUITIVE: when the acceleration ramp works out to one control tick or less, the planner emits a constant-velocity segment plus a one-tick stop instead of three segments, and then 16 moves fit instead of 10. The ramp is `maximum velocity / maximum acceleration`, so this happens exactly when you make the speed limit LOW while leaving the acceleration limit high — a slow, precise axis with the 12,000 rot/s^2 factory default. Bench-measured on both a rack motor and the bench motor: at the default speed limit exactly 10 moves were accepted before fatal error 17; with the speed limit at 0.3 rot/s, exactly 16 were. So LOWERING the speed limit INCREASES how many moves fit — the opposite of what most people would guess, and worth knowing if you size a plan against the queue. Do not hard-code either number; poll 'Get n queued items'. 'Move with velocity'/'Move with acceleration' take 1 slot each, and a 'Multimove' entry takes 1 slot per entry; 'Move with velocity'/'Move with acceleration' take 1 slot each. The item currently executing is included in the count. Exceeding the queue is fatal error 17 — and bench-verified, the fault does not just reject the extra item: it ABORTS the entire in-flight choreography (MOSFETs drop, queue cleared). When streaming moves, poll 'Get n queued items' and stay below the cap.
9. Also bench-verified planner algebra: a trapezoid or go-to-position queued after motion that ends at velocity v0 superimposes — it travels an EXTRA v0 x duration and ends at v0, not at rest (measured: MWV(1 rot/s, 1 s) + trapezoid(2 rot, 1 s) landed at exactly 4.0 rot, not 3.0). Only queue them after motion that ends at rest, or account for the superposition.
10. Safety limits live in the CURRENT position frame — 'Zero position' silently moves your fences to new physical locations. Set safety limits AFTER homing and zeroing, and re-send them after any later zeroing if the fence protects real hardware (details under Motion control patterns).

## Positioning accuracy: absolute moves converge, and nothing drifts far

A single move delivers up to ONE COUNT LESS than commanded. This is integer division inside the
planner, not a mechanical effect, and it is visible with the MOSFETs off: ask for 1,638,400 counts
and the commanded position lands on 1,638,399. One count is 0.3 parts per million of a revolution,
so it never matters mechanically — but it decides which command to build a repeatable machine on.

- **'Go to position' CONVERGES.** It is absolute, so the second call sees the one-count error and
  corrects it. Bench-measured, four identical calls to the same target: `1638399, 1638400, 1638400,
  1638400`. It is not idempotent on the first call; it is convergent, and idempotent from the
  second onwards. If you need to land exactly on a coordinate, issue the absolute move twice, or
  simply re-issue it at each cycle of your loop, which most programs do anyway.
- **Relative 'Trapezoid move' does NOT self-correct**, but it also does not accumulate the way you
  would fear. Bench-measured, one full rotation split into 10, 50 and 200 relative moves gave a
  total shortfall of **1 count in every case** — not one count per move. Across every regime
  measured the total stayed within a few counts: 1-2 counts over a rotation, about 8 counts over
  400 rotations, about 5 counts over 5,000 rotations. The internal position keeps sub-count
  precision, so the error is bounded rather than cumulative.

Practical rule: use relative moves freely; if a machine must return to an exact coordinate, use an
absolute 'Go to position' for that step, and expect the first one to be a count short.

## A stale limit from a previous program is a real hazard

This is what golden rule 2 is protecting you from, made concrete. Limits are volatile but they
survive until the next reset or power cycle, NOT until your program exits. A program that sets a
tight limit and exits leaves the motor configured that way for whatever runs next.

Bench-demonstrated: program A sets the speed limit to 0.2 rot/s and exits. Program B starts without
a reset and commands an ordinary one-rotation move — and gets fatal error 16. Nothing in program B
is wrong; it inherited a limit it never set and cannot see, because there is no getter for it.

Always begin with 'System reset' and the post-reset wait. If you are debugging a program that
"works sometimes", check whether something else ran first.

## Recommended startup sequence

Almost every problem people hit is a SEQUENCING problem rather than a misunderstanding of an individual command. The command reference tells you what each command does on its own; this is the order to do them in.

1. Set your units. This is host-side only — nothing is sent to the motor, and units survive a device reset because they never lived on the device.
2. 'System reset', then wait at least 0.5 s with a completely silent bus (golden rule 1). This clears any fatal error left over from a previous run and gives you a known state.
3. 'Set maximum motor current' to a working value (150-200 internal units is typical). Check for an error.
4. 'Set maximum velocity' and 'Set maximum acceleration' explicitly, even if the defaults would do. Explicit limits make your program immune to a firmware default changing under it — which has happened (see the firmware version notes below).
5. Choose ONE of the two engagement paths, not both:
   - Open loop: 'Enable MOSFETs', wait about 0.3 s for the commutation snap to settle, then 'Zero position'.
   - Closed loop (preferred when the mechanism is sensitive): 'Zero position' FIRST, then 'Go to closed loop', which enables the MOSFETs itself and engages far more gently. Zeroing first is what prevents the closed-loop lurch described below.
6. Optional homing: 'Zero position', then 'Homing', then compare the distance actually travelled against the distance you asked for, then 'Zero position' again to set your origin. Homing does not zero anything itself.
7. Set 'Set safety limits' and 'Set max allowable position deviation' now, AFTER homing and zeroing — they are interpreted in the current position frame, so zeroing later silently moves them.
8. Move. To know a move has finished, poll 'Get n queued items' to 0 and then allow 0.1-0.3 s for mechanical settling (golden rule 7).
9. When you are done, 'Disable MOSFETs'.

Two rules that are easy to state and easy to get wrong:

- Enable the MOSFETs ONCE for a burst of work rather than toggling them around every move — but disable them when the machine will be idle for a long time. An enabled idle motor holds with coil current and warms the driver PCB, and the over-temperature fatal error (40, at roughly 80 C) is only monitored while the MOSFETs are enabled, so it is the energised state that carries the thermal risk. How close you get to the cutoff depends on your current limit, the ambient temperature and how the motor is mounted; an unloaded bench M17 at 24 V sits around 38-45 C even after a long working day, so this is about power and margin rather than an imminent trip. Note the trade-off: a later open-loop 'Enable MOSFETs' can twitch the shaft by up to about 4 degrees, so on a twitch-sensitive mechanism prefer to leave it energised, or re-engage via 'Go to closed loop'.
- After ANY reset, everything volatile is gone: motor current, velocity and acceleration limits, safety limits, position deviation limit, PID gains, the zeroed position, the MOSFET state, and closed-loop mode. Re-apply all of it. Only your host-side unit selection survives.

## Recommended program skeleton

There is no required program structure — write your program however you like. The skeleton below is just a recommended starting point that handles the common pitfalls for you:

<!--LANG:PYTHON-->
```python
import argparse, time, servomotor

parser = argparse.ArgumentParser()
parser.add_argument('-p', '--port', help='serial port device')
parser.add_argument('-P', '--PORT', action='store_true', help='interactive port menu')
parser.add_argument('-a', '--alias', default='X', help='device alias')
args = parser.parse_args()

servomotor.set_serial_port_from_args(args)   # must precede open_serial_port()
servomotor.open_serial_port()
motor = servomotor.M3(args.alias, time_unit='seconds',
                      position_unit='shaft_rotations',
                      velocity_unit='rotations_per_second',
                      acceleration_unit='rotations_per_second_squared',
                      verbose=0)
try:
    motor.system_reset()
    time.sleep(1.5)                          # golden rule 1
    # ... your work here ...
finally:
    try:
        motor.disable_mosfets()              # best-effort cleanup
    except Exception:
        pass
    servomotor.close_serial_port()
```

Notes on the skeleton:

- You do not have to pass -p every time. The port is resolved in this order, highest priority first: an explicit -p; -P, which jumps straight to the interactive menu (you asked to choose, so nothing pre-empts that); the SERVOMOTOR_PORT environment variable; the saved default from the last successful run; and finally the interactive menu. So setting SERVOMOTOR_PORT once names the port for every program without any file at all.
- Where the saved default lives (library 0.12.2 and later): a per-user config directory — ~/Library/Application Support/servomotor on macOS, ~/.config/servomotor on Linux, the local AppData servomotor folder on Windows, or $XDG_CONFIG_HOME/servomotor if you have set that. A default saved by an older library, which kept it inside the installed package, is still read, so upgrading does not lose it. Saving can never fail a run: if the location is not writable the library warns and carries on, because the port is already open and remembering it is only a convenience. (Library versions before 0.12.1 did fail there — an uncaught PermissionError after the port was already open, which a normal system-wide install hit every time.)
- LINUX PERMISSIONS, the most common first-run failure there: serial devices belong to the dialout group (uucp on some distributions), so a fresh account gets 'Permission denied' opening /dev/ttyUSB0 even though the port plainly exists and the adapter is plugged in. Fix it once with `sudo usermod -a -G dialout $USER`, then log out and back in — group membership is applied only to NEW logins, so the change appears to do nothing until you do. Library 0.12.1 and later prints this advice when it sees a permission error.
- The M3 constructor's verbose parameter defaults to 2 (prints every packet in hex). Pass verbose=0 for production use.
- The library's default read timeout is 1.2 s. A TimeoutError on a normally-addressed command is essentially always a real fault: wrong port, wrong alias, device in bootloader, device locked up, or a CRC-state mismatch. (One historical exception: firmware before 0.15.4.0 stayed silent when reading an empty multipurpose buffer, so that read timed out to mean "buffer empty"; current firmware responds instead.)
- If a program may leave nonstandard device state behind (changed PID gains, tightened limits, changed current limit), finish with a defensive system_reset() so the next user starts clean.
<!--LANG:END-->
<!--LANG:ARDUINO-->
```cpp
#include <Servomotor.h>

#define ALIAS 'X'
#define RS485_RXD 5     // change these for your board
#define RS485_TXD 4

// Returns true (and explains itself) if the previous command did not succeed.
bool failed(Servomotor &motor, const char *what) {
  int e = motor.getError();
  if (e == 0) return false;
  Serial.print("[FAIL] "); Serial.print(what);
  Serial.print(" -> getError() = "); Serial.println(e);
  return true;
}

void setup() {
  Serial.begin(115200);          // on ESP32-S3 also set Tools > USB CDC On Boot > Enabled
  Servomotor motor(ALIAS, Serial1, RS485_RXD, RS485_TXD);

  // Host-side only: sends nothing, and survives a device reset.
  motor.setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
  motor.setTimeUnit(TimeUnit::SECONDS);

  motor.systemReset();
  if (failed(motor, "systemReset")) return;
  delay(1500);                   // golden rule 1: keep the bus silent

  motor.setMaximumMotorCurrent(200, 200);
  if (failed(motor, "setMaximumMotorCurrent")) return;

  // ... your work here, checking getError() after each call ...

  motor.disableMosfets();
}

void loop() {
}
```

Notes on the skeleton:

- No command method returns a success flag. getError() carries the outcome of the most recent command: 0 on success, a positive value for a fatal error reported by the motor, a negative value for a host-side communication failure. Read it immediately after the call you care about — the next command overwrites it. The host-side unit setters do not touch it, so after calling one of those getError() still reports the previous command's result.
- The Arduino library's read timeout is 1 second, and it is a budget for the whole reply. A command sent to a motor that is not there therefore costs a full second before getError() returns -1. (The 1.2 s figure quoted for the Python library does not apply here.)
- The library prints a "[Motor] ... called." line to Serial for every command. This is unconditional in the current version; there is no flag to silence it.
- Construct the Servomotor object after any Serial1.begin() of your own, or simply pass the RX/TX pins to the constructor as above — the constructor opens the port.
- If a program may leave nonstandard device state behind (changed PID gains, tightened limits, changed current limit), finish with a defensive systemReset() so the next run starts clean.
<!--LANG:END-->

## Units — read this before your first move

<!--LANG:PYTHON-->
- Set units explicitly in the M3 constructor to make your programs self-documenting. The defaults are the first unit of each list: seconds, shaft_rotations, rotations_per_second, rotations_per_second_squared, celsius, internal_current_units, millivolts. (Older library versions defaulted the time unit to 'timesteps' — the raw 32-microsecond internal unit — which made trapezoid_move(1.0, 2.0) mean 64 microseconds; if you may be running an old library, pass time_unit='seconds' explicitly.)
- Available unit names (exact strings): time: seconds, milliseconds, minutes, microseconds, timesteps; position: shaft_rotations, degrees, radians, encoder_counts; velocity: rotations_per_second, rpm, degrees_per_second, radians_per_second, counts_per_second, counts_per_timestep; acceleration: the same family with _squared (except rpm, whose acceleration unit is rpm_per_second); current: internal_current_units, milliamps, amps; voltage: millivolts, volts; temperature: celsius, fahrenheit, kelvin. Units can be changed at runtime with set_position_unit() etc.
<!--LANG:END-->
<!--LANG:ARDUINO-->
- Set units explicitly with setPositionUnit(), setTimeUnit(), setVelocityUnit(), setAccelerationUnit(), setCurrentUnit(), setVoltageUnit() and setTemperatureUnit() to make your programs self-documenting. These are host-side only: they send nothing to the motor, they take effect immediately, and they survive a device reset. The defaults are SHAFT_ROTATIONS, SECONDS, ROTATIONS_PER_SECOND, ROTATIONS_PER_SECOND_SQUARED, INTERNAL_CURRENT_UNITS, VOLTS and CELSIUS.
- The unit names are C++ enum values: TimeUnit::{SECONDS, MILLISECONDS, MINUTES, MICROSECONDS, TIMESTEPS}; PositionUnit::{SHAFT_ROTATIONS, DEGREES, RADIANS, ENCODER_COUNTS}; VelocityUnit::{ROTATIONS_PER_SECOND, RPM, DEGREES_PER_SECOND, RADIANS_PER_SECOND, COUNTS_PER_SECOND, COUNTS_PER_TIMESTEP}; AccelerationUnit with the same family plus _SQUARED (and RPM_PER_SECOND); CurrentUnit::{INTERNAL_CURRENT_UNITS, MILLIAMPS, AMPS}; VoltageUnit::{MILLIVOLTS, VOLTS}; TemperatureUnit::{CELSIUS, FAHRENHEIT, KELVIN}.
- Every unit conversion goes through 32-bit float, so very large positions lose resolution: past about 5 shaft rotations a float can no longer represent every encoder count, and at 1000 rotations the granularity is roughly 0.03 degrees. Setter values are truncated toward zero rather than rounded. When exactness matters, use the ...Raw() variants of the methods, which take and return internal units with no conversion.
<!--LANG:END-->
- Key conversion constants: 1 shaft rotation = 3,276,800 encoder counts (M17/M23; always confirm with 'Get product specs', which returns the update frequency and counts per rotation). The internal time base is 31,250 timesteps per second (one timestep = 32 microseconds, which is also the control-loop period).
- Every converted value is rounded to the nearest integer internal unit before transmission; values smaller than half an internal unit become zero.
- The clock commands ('Get current time', and the masterTime input of 'Time sync') use microseconds on the wire, and the library converts them correctly from/to whatever time unit you configured. The time-error output of 'Time sync' is the exception: it is always returned as raw microseconds with no conversion. (Older library versions mis-scaled these two commands by ~32x in any unit except 'timesteps' — if you may be running an old library, read the clock with time_unit='timesteps', which then passes raw microseconds through.)
- (Older library versions only: the velocity unit named 'counts_per_timestep' carried a conversion factor ~104.86x too large. It is correct in the current library.)
<!--LANG:PYTHON-->
- Do not hand-roll internal-unit conversions copied from older scripts: legacy demos (ball_throwing_demo.py, ball_juggling_demo.py, magnetic_disk_machine/*) hardcode per-rotation values such as 4,320,000 or 4,752,000 (64*1350*50 or *55) that do not match current motors, and use an obsolete protocol module. Use the library's unit system.
<!--LANG:END-->

## Driving many motors on one bus — measured on a 35-motor rack

Everything else in this document describes one motor. These numbers come from a bus with
thirty-five M17s on it, all on firmware 0.15.12.0, with the MOSFETs off so only the commanded
position moves.

- **A broadcast move is exact, not approximate.** One 'Trapezoid move' sent to address 255 moved
  all 35 motors by *identically* 819,199 counts of a commanded 819,200 — spread ZERO across the
  whole bus. Broadcast is therefore the right primitive for coordinated multi-axis motion: it is
  one packet, it starts every axis on the same control tick, and every axis performs the same
  arithmetic.
- **Addressing motors individually costs about 4 ms each.** Queueing the same move on 35 motors one
  at a time took 142 ms end to end. That is fine for setup, but it means individually-addressed
  commands cannot start axes together — the last motor begins moving ~140 ms after the first. Use
  broadcast when simultaneity matters, and individual addressing only for per-axis parameters.
- **A supervisory polling loop tops out around 7 full sweeps per second.** Reading the position of
  all 35 motors in a loop sustained 262 reads/second, i.e. 7.5 complete sweeps/second, with **zero
  errors over 11,000 reads**. Budget accordingly: at 35 axes you get roughly 7 Hz of state, and
  polling harder just queues behind the bus.
- **One motor faulting does NOT disturb the others.** With one motor deliberately faulted, five
  others were commanded and all five moved correctly. A single broadcast 'System reset' then
  cleared the fault on every motor at once (35/35 clean). A fault is genuinely per-device, and
  bus-wide recovery is one packet.

### The gotcha that will bite you here

Unit settings are **host-side, per-motor-object, and sticky**. They persist until you change them
and are silently applied to every later reading. Nothing errors.

The same device clock, read five ways within a second of each other:

| time unit | value reported |
|---|---|
| seconds | 1.8069 |
| milliseconds | 1814.5360 |
| microseconds | 1818352.0000 |
| timesteps | 56585.1562 |
| minutes | 0.0304 |

In a multi-motor script it is very easy to set a unit for one phase of the program and then read
something unrelated in a later phase. That exact mistake — setting `timesteps` for a motion
section, then reading clocks further down — produced an apparent **4,150-second disagreement**
between motors that was really 133 milliseconds of read-order skew. The number looked like a
serious hardware fault and was a unit label. **Set the unit immediately before you read, or set it
once at startup and never change it.**

## Addressing and multi-motor buses

- Aliases are one byte, usable range 0-251. Values 252/253/254 are reserved by the protocol (attempting to assign them latches fatal error 50); 255 means broadcast and also serves as "no alias assigned".
<!--LANG:PYTHON-->
- Broadcast (alias 255) commands execute on every device but produce NO responses (the library returns [] immediately). You cannot read anything via broadcast. The one exception is 'Detect devices', which is designed to be broadcast and collects one response per device.
- GOTCHA: if you omit the alias, some CLI paths default to 255 (broadcast) — query commands then appear to succeed while returning nothing. Always pass a real alias for anything that expects a reply.
- A wrong alias or wrong unique ID produces a TimeoutError, not an error message — timeout is the only symptom of misaddressing. An UNSUPPORTED COMMAND ID is also silently dropped (bench-verified: no response, no error, device unaffected), so a timeout can mean either "nobody home" or "command unknown to this firmware".
- Device discovery: use servomotor.detect_devices_iteratively(n_detections=3). Each round does broadcast reset, 1.5 s wait, flush_receive_buffer(), then 'Detect devices'; results are merged across rounds by unique ID because single rounds can miss devices (responses arrive at random 0-950 ms offsets to avoid collisions). Each entry gives the 64-bit unique ID and the current alias.
<!--LANG:END-->
<!--LANG:ARDUINO-->
- Broadcast (alias 255) commands execute on every device but produce NO responses. You cannot read anything via broadcast, and because the library still waits for a reply that never comes, every broadcast command blocks for the full 1-second timeout and then leaves getError() at -1. That -1 is expected, not a fault. The one exception is 'Detect devices', which is designed to be broadcast and collects one response per device.
- A wrong alias or wrong unique ID gives you -1 (timeout) rather than an error message — a timeout is the only symptom of misaddressing. An unsupported command ID is also silently dropped, so -1 can mean either "nobody home" or "command unknown to this firmware".
- Device discovery: call detectDevices() for the first reply, then detectDevicesGetAnotherResponse() repeatedly to collect the rest, since every device answers after its own random 0-950 ms delay. Expect to run several rounds and merge the results by unique ID — a single round can miss devices. Drain any stray receive bytes between rounds with `while (Serial1.available()) Serial1.read();`.
<!--LANG:END-->
- GOTCHA: after 'Detect devices' the device ignores ALL bus traffic for about 1 second (its collision-avoidance window). Wait at least 1.1 s before sending anything else. Queued motion is NOT disturbed — a move in flight continues and completes normally across the window (bench-verified); only hosts that need to keep STREAMING new segments must not run detection mid-path (the queue could starve into fatal error 18). The detect reply itself can take over 1.2 s end to end (random 0-950 ms delay); the library swallows the terminating read timeout for this multiple-response command, so a missed device shows up as an empty or partial result (never a TimeoutError) — retry and merge rounds.
- 'Set device alias' saves to flash and immediately REBOOTS the device. Keep the bus SILENT and wait at least 0.5 s before addressing it at the new alias — exactly like the post-reset rule. Bench-measured: with a silent bus the device answered 0.3 s after the command, but continuously polling it during the reboot stretched recovery to over 1.2 s (and risks the bootloader-pinning trap of golden rule 1). Broadcasting this command sets the SAME alias on every device on the bus — almost never what you want.
<!--LANG:PYTHON-->
- Devices with alias 255 (unassigned) or duplicated aliases must be addressed by their 64-bit unique ID. The M3 constructor accepts it as an int (e.g. 0x0123456789ABCDEF) or as a string of exactly 16 hex digits (e.g. M3("0123456789ABCDEF")); other multi-character strings are parsed as decimal alias numbers. (Older library versions did not parse hex strings in the constructor — there, pass an int or use servomotor.string_to_alias_or_unique_id() first.) The -a command-line flag can carry a 16-hex-digit unique ID two ways: the servomotor_command.py CLI converts it with servomotor.string_to_alias_or_unique_id(), and, with the current library, test programs that pass -a straight to the M3 constructor also accept it — the constructor parses a 16-hex-digit string as a unique ID (M3.py). Only older library versions, whose constructor did not parse hex strings, required the CLI-style conversion first.
- To control several motors, either create one M3 object per motor (different aliases) or create one object and retarget it with motor.use_this_alias_or_unique_id(...). Note that, unlike the M3 constructor, use_this_alias_or_unique_id() stores its argument as-is with no string parsing and no reserved-alias validation. Pass an already-parsed integer — ord('X') for a one-byte alias, or int('0123456789ABCDEF', 16) for a 64-bit unique ID; a single character or hex string stored here is not converted and fails later with a TypeError deep in send_command (the 'X' <= 255 comparison). Keep one extra M3(255) instance for broadcast operations like a bus-wide 'System reset' or emergency 'Disable MOSFETs'.
<!--LANG:END-->
<!--LANG:ARDUINO-->
- Devices with alias 255 (unassigned) or duplicated aliases must be addressed by their 64-bit unique ID. Call motor.useUniqueId(0x0123456789ABCDEFULL) to retarget an object permanently, or use the per-call overloads that take a uniqueId as their first argument (for example motor.trapezoidMove(uniqueId, 1.0f, 1.0f)). Switch back with motor.useAlias('X').
- To control several motors on one bus, create one Servomotor object per motor, all naming the same HardwareSerial port. Only the FIRST object constructed actually opens the port — it fixes the port, baud rate and pins for every object afterwards — so construct them all against the same port, and set any RX buffer size you need before the first construction.
<!--LANG:END-->
- Fleet-scale figures (measured on a 39-motor bus): a full-fleet position sweep takes ~150 ms (~6.7 Hz fleet telemetry; per-transaction RTT is uniform across devices at ~3.6 ms regardless of bus position); a single-round 'Detect devices' found all 39 in 13 of 15 tries (the 3-round default is ample); free-running clocks across a 39-unit population spanned ~5000 ppm (so two unsynced motors can drift apart ~5 ms per second — use 'Time sync' or generous margins); round-robin syncing the whole fleet at the bus's full rate held every motor within ~250 microseconds of the master. Phased choreography works by broadcasting 'Reset time' then queueing per-motor [dwell-until-slot, move] pairs with dwells computed against the shared epoch. One faulted motor does not disturb the others (verified: 38/38 executed a broadcast move around a latched neighbor) and rejoins after a solo reset. GOTCHA: a synchronized fleet-wide hard stop can glitch the HOST's receiver (one in-flight reply corrupted in each of two trials, device counters clean) — read telemetry after, not during, mass transients, and wrap such reads in retry+flush.
- The bus stays fully usable while motors execute moves — pinging at 100 Hz during high-speed motion works. You only need bus silence during post-reset windows.

## Motion control patterns

- 'Trapezoid move' is RELATIVE (a signed displacement from the end of previously queued motion); 'Go to position' is ABSOLUTE. Successive commands chain correctly because both plan from the predicted end-of-queue position, not the live position.
- Both commands assume the motor starts the move at rest. If the preceding queued motion ends at nonzero velocity, the profiles superimpose and the motor will NOT stop at the expected position. Only queue them when prior motion ends at rest.
- A move's success response confirms validation and queueing only — it says nothing about motion completion.
- MIGRATION NOTE (firmware 0.15.8.0): the boot-default velocity limit dropped from a buggy 68 rot/s to the datasheet 560 RPM (9.333 rot/s), and the acceleration default became 12,000 rot/s^2. Any pre-0.15.8.0 program that commands faster than 560 RPM now gets fatal error 16/28 at queue time — add an explicit 'Set maximum velocity' call first (the limit raises up to 18.67 rot/s). Three of this project's own long-standing test programs needed exactly this one-line change.
- Set 'Set maximum velocity', 'Set maximum acceleration', and safety limits BEFORE queueing moves — trajectory planning and validation use the values in effect at queue time, and limits are enforced by REJECTING violating moves with a latched fatal error (15/16/26/27/28), not by clamping them. 'Set maximum motor current' is different: it is not a planning input and takes effect immediately, at any time, even mid-move.
- Safety limits are interpreted in the CURRENT position frame: 'Zero position' shifts which physical locations they refer to (bench-demonstrated — after zeroing at 2.5 rotations, a fence set at 3.0 sat at physical 5.5 and the motor happily drove past the old physical fence). Set safety limits AFTER homing and zeroing, and re-send them after any later zeroing if the fence protects real hardware. Do not send an inverted window (lower > upper): firmware 0.15.5.0 and later rejects it cleanly with fatal error 34; firmware up to 0.15.4.0 stored it unvalidated, faulted with error 25 on the next 32-microsecond tick, and the race with the command's own reply could lock the device up completely until a power cycle.
- Speed and acceleration out of the box: the attainable speed of an unloaded M17 is about 8.6 rotations/second (~520 RPM), just below the default velocity limit of 9.33 rot/s (the datasheet's 560 RPM; firmware 0.15.8.0 and later). The default acceleration limit is likewise 12,000 rot/s² — just above the ~11,000 rot/s² unloaded spin-up measured at 24 V (~9,300 at 20 V). So out of the box the motor's own physics, not the limits, constrain motion. Commanding an unattainable speed raises no error while the deviation stays inside the allowed limit — the motor just falls behind and catches up later (see golden rule 7). Under load, the attainable speed drops further.
- Changing the limits: they can be raised (silently clamped at 2x/4x the defaults) or lowered with 'Set maximum velocity'/'Set maximum acceleration', with rejection errors 16/28/15 enforcing them immediately.
- Supply voltage: the speed ceiling is INTRINSIC to the drive, not the supply voltage — 40 units measured 8.59-8.65 rot/s identically at 20 V and 24 V. Supply voltage buys ACCELERATION instead (median unloaded spin-up ~9,300 rot/s² at 20 V vs ~11,000 at 24 V).
- Wire-format ceilings to know: the u32 limit field could carry a request up to ~39.06 rot/s, and MOVE commands carry velocity as a signed i32 (a ~19.5 rot/s wire ceiling), but on firmware 0.15.8.0 neither wire cap is the binding constraint — 'Set maximum velocity' silently clamps the stored limit at 18.67 rot/s (2x the default), and any move faster than the stored limit is rejected with fatal error 16, so 18.67 rot/s is the real ceiling on both the settable limit and the fastest commandable move.
- Choosing how to stop mid-motion (bench-measured from 5 rotations/second, free shaft): a commanded stop — 'Reset time', which clears the queue and forces velocity to zero with MOSFETs still on — halted in ~0.02 rotations, about HALF the travel of 'Emergency stop' or 'Disable MOSFETs' (~0.05 rotations), which remove torque and let the rotor freewheel. With real load inertia the gap grows. For the shortest physical stop keep the MOSFETs on and command zero velocity; use 'Emergency stop' when you want torque removed.
- GOTCHA: do not lower 'Set maximum velocity' while moves are executing — the limit takes effect immediately and the runtime check trips fatal error 16 on the next control tick if the executing velocity now exceeds it. Change limits only while stopped with an empty queue.
- 'Set maximum acceleration' with 0, or any standalone move command with a duration of 0, is rejected with fatal error 34 (ERROR_PARAMETER_OUT_OF_RANGE) as of firmware 0.15.4.0. Exception: a zero-duration entry inside a 'Multimove' is silently dropped and consumes no queue slot (verified in firmware: add_to_queue returns before incrementing the queue count). (Older firmware accepted both: the zero max-acceleration caused a divide-by-zero in later planning, and a zero-duration move was a silent no-op that still returned success.)
- 'Set maximum velocity' with 0 is rejected with fatal error 34 as of firmware 0.15.6.0. (Firmware through 0.15.5.0 accepted it, and then every trapezoid/go-to-position was silently planned as a zero-motion dwell — success response, queue drains over the commanded duration, shaft never moves.) Beware that any limit below half an internal unit rounds down to 0 during unit conversion, so a tiny requested limit can hit this rejection unexpectedly.
- THE SPEED YOU ASK FOR IS NOT THE SPEED THAT GETS VALIDATED. 'Trapezoid move' and 'Go to position' take a displacement and a duration, so it is natural to compute displacement/duration and check it against your velocity limit. The firmware never looks at that average. It builds a fixed ramp of rampTime = maxVelocity/maxAcceleration (truncated down to a whole 32-microsecond timestep), then sizes the profile so that peakVelocity = displacement / (duration - rampTime) and peakAcceleration = peakVelocity / rampTime, and it validates those two. Because rampTime does not shrink with the move, the peak approaches TWICE the average as the duration approaches 2 x rampTime, and exceeds twice below that. A move whose average is comfortably inside your limit can therefore still be rejected. Check the peak, not the average, whenever you are near a limit.
- Which error you get: for a too-short 'Trapezoid move', 'Go to position' or 'Homing', the rejection is fatal error 15 (ERROR_ACCEL_TOO_HIGH), not 16 or 28. The planner's derived acceleration is checked first and is always reached at or before the velocity limit, so the acceleration check is what fires. Error 28 (ERROR_PREDICTED_VELOCITY_TOO_HIGH) belongs to the acceleration-type commands — 'Move with acceleration' and the acceleration segments of 'Multimove' — where a legal acceleration applied for too long would end above the velocity limit. Error 16 (ERROR_VEL_TOO_HIGH) belongs to the velocity-type commands, and is additionally re-checked every control tick while moves execute.
- Worked example, if the shape is easier to see with numbers: with a 5 rot/s velocity limit and a 1000 rot/s^2 acceleration limit, rampTime is 0.005 s. A move of 0.04 rotations over 0.01 s averages 4 rot/s — apparently fine — but the duration is exactly 2 x rampTime, so the profile degenerates to a pure triangle with no coast phase at all, peaks at 8 rot/s and needs 1600 rot/s^2. It is rejected with fatal error 15. (Below 2 x rampTime there is no coast segment, which is also why such a move occupies 2 of the 32 queue slots rather than the usual 3.)
- Pre-validate trapezoid durations in closed form instead of try/except: the minimum accepted duration is displacement/maxVelocity + maxVelocity/maxAcceleration (bench-verified to 4 decimal places; shorter durations reject with fatal error 15). One refinement to be aware of: the firmware truncates rampTime down to a whole timestep, so the velocity actually reached at the top of the ramp can be slightly below maxVelocity, which makes that formula very slightly optimistic — at the M17 boot defaults the ramp reaches 9.216 rather than 9.333 rot/s, and the formula under-estimates the minimum duration by about 1.3% of its displacement/maxVelocity term. Add a few percent of margin when pre-validating rather than submitting the computed minimum exactly. The velocity and acceleration limits are INCLUSIVE — a move at exactly the limit is accepted; 0.2% over rejects.
- A single 'Go to position' move is limited to a displacement of about +/-655 shaft rotations (a signed 32-bit count range); split longer travels.
- Timed dwell inside a motion sequence: queue trapezoid_move(0, t) as an in-queue pause — the whole choreography (move, dwell, move) then runs from the queue without host timing.
- Streaming motion (continuous paths): keep feeding segments and never let the queue empty mid-path. Two proven patterns: (a) low-latency ramping — keep only ~3 segments buffered, blocking while get_n_queued_items() >= 3; (b) throughput streaming — count segments locally and only query the queue when your local estimate approaches 32. Perform side-channel traffic (time sync, telemetry) only when at least 3 segments are buffered, at most one side-channel transaction per segment queued.
- Multi-axis coordination: send the same segment duration to every motor back-to-back so segment boundaries stay aligned; wait for all queues to empty between phases; sequence potentially colliding axes (e.g. retract Z before long XY traverses).
- For long random/burn-in motion on limited-travel rigs, bias each random move to pull the cumulative displacement back toward zero.

## Closed loop, calibration, and homing

- Motors are calibrated at the FACTORY, and the calibration persists in flash. A new motor works in closed loop out of the box — you normally never need to run 'Start calibration'. Recalibrate only if closed-loop control misbehaves or after hardware service.
- If you do calibrate: requirements are a shaft completely free to rotate (remove all loads), empty queue, no test mode active, and the device in open-loop mode (i.e. freshly reset — which also leaves the MOSFETs disabled; calibration enables them itself). The motor spins about 1.5 turns back and forth for roughly 20-60 s (product-dependent; about 20 s measured on an M17).
- Calibration lifecycle: the success response only means calibration STARTED. Keep the bus COMPLETELY QUIET while it runs — polling during calibration can disturb the measurement and reduce its accuracy, and when calibration finishes the firmware saves to flash and AUTOMATICALLY REBOOTS, so a poll landing in the post-reboot bootloader window pins the device in the bootloader. Instead: wait a generous fixed time (60 s covers all products), then send 'System reset', wait the post-reset delay, and verify a clean baseline with 'Get status' (application mode, no fatal error, MOSFETs disabled).
- Entering closed loop: just call go_to_closed_loop() — it enables the MOSFETs by itself (skipping a separate 'Enable MOSFETs' may even give a gentler engagement), optionally after setting PID constants and motor current. Poll 'Get status' until the closed-loop bit (bit 2) sets, with a ~6 s timeout; if it never sets, the motor probably needs calibration.
- ZERO BEFORE ENTERING CLOSED LOOP, and understand why. On M17, M2 and M23, 'Go to closed loop' does NOT synchronise the measured position to the commanded one and does not clear the PID state — it only enables the MOSFETs, reloads the commutation offset, and switches the mode. (Only the M1 code path re-aligns; its source comment even says it is "so that the motor does not move when we go into closed loop mode".) So if a position error is standing when the loop closes, the PID acts on it on the very next 32-microsecond tick and drives it out at full authority. Any error beyond about 52,000 counts (0.016 rotation, 5.8 degrees) at the default gains saturates the controller, which then applies a full 90-electrical-degree commutation lead at the configured maximum motor current until the error is gone.
- CRITICALLY: that correction is NOT speed limited. 'Set maximum velocity' governs the motion planner — it is checked when a move is queued and against the planner's own velocity every tick — but the closed-loop correction has no planned velocity to clamp; in closed loop the commutation angle is slaved to the rotor, so the loop is a torque command, not a speed command. Nothing in the firmware bounds how fast the shaft moves while it closes a standing error. Bench-measured on one M17: a 0.49-rotation standing error present when the loop closed was driven out at about 8 rotations/second average — essentially the motor's unloaded top speed of ~8.6 rot/s — with no acceleration ramp. On a real mechanism that is a safety concern, not just a surprise.
- The fix is simply to call zero_position() BEFORE go_to_closed_loop(). Zeroing atomically sets the commanded position, the measured hall position, the velocity and the PID state (integral term, previous error, filtered error change) all to zero, so there is nothing left to correct. Its only precondition is an empty queue. Note the ordering difference between the two engagement paths: with a bare 'Enable MOSFETs' you enable first, wait ~0.3 s for the commutation snap to settle, and zero after (golden rule 3); with go_to_closed_loop() there is no such snap to wait out, so zero first and enter closed loop second.
- To see the error before you act on it, read 'Get comprehensive position': the difference between the commanded and hall-sensor fields IS what the PID will act on. To make an unexpected lurch fail safe instead of fast, arm 'Set max allowable position deviation' — the deviation check is unconditional on control mode, so a limit tighter than the standing error trips fatal error 45 and removes power instead of allowing the correction. Lowering 'Set maximum motor current' before entering reduces the FORCE of the correction but not its top speed, because nothing in the loop limits speed.
- Nothing observable tells you a correction is in progress: no status bit reflects it, and the queue stays at 0 items throughout, so 'Get n queued items' returning 0 does not mean the shaft is stationary.
- Homing drives the motor until it detects a collision by following-error (threshold: 50,000 counts between commanded and measured position), then shifts the commanded position 50,000 counts back toward the measured position and holds there, MOSFETs still enabled. Requirements: closed-loop mode first (otherwise fatal error 13), empty queue (otherwise fatal error 8).
- ALWAYS CALL 'Zero position' IMMEDIATELY BEFORE 'Homing'. This is the single easiest way to make homing fail silently. The 50,000-count collision threshold is only 0.0153 shaft rotations — about 5.5 degrees on M17/M23 — and it is tested on the very FIRST 32-microsecond control tick after homing starts, before the homing move has advanced at all. So any position error already standing when you call homing is read as an instant hard stop: homing aborts, the success response you already received says nothing, the homing (bit 4) and busy (bit 6) status flags are set and cleared again within one tick so your first status poll sees them already clear, and no error is raised. The shaft performs none of the homing travel. There is no flag and no error code anywhere that distinguishes this from a real homing.
- A standing error that large is easy to arrive at, and the standard homing recipe below actively encourages it: neither 'Enable MOSFETs' nor 'Go to closed loop' re-aligns the commanded position to the measured one on M17/M23, the hall position keeps tracking the shaft even while the MOSFETs are off (so anything that moves the shaft by hand, by gravity or by back-driving leaves an offset), and a move that ended with following error leaves one too. At a deliberately low motor current the unloaded motor can park with a PERMANENT ~0.6-rotation standing error and no error flag — about 39 times the collision threshold. Reducing the current for a soft approach and then homing is therefore exactly the sequence that walks into this trap. Zero between the two.
- Verify the homing afterwards, because the status flags cannot: compare the distance actually travelled against the maxDistance you asked for. Much less means a hard stop was found; the full amount means none was. Travelled almost nothing (well under 50,000 counts) means the false-collision trap, not a hard stop.
- Homing recipe: REDUCE the motor current first ('Set maximum motor current' to a gentle value, e.g. 50-100 internal units versus the ~200 working default) so the motor presses softly into the hard stop; then call zero_position() so no standing error remains; give a signed maxDistance larger than the full travel (sign = direction) and a generous maxDuration; poll 'Get status' bit 4 (homing) until clear, or poll the queue to empty, budgeting maxDuration plus ~2 s; compare the travelled distance against maxDistance to confirm a stop was actually found. Then restore the working current, and call zero_position() again to establish your origin — homing itself does NOT zero the position or set any limits.
- One boundary worth knowing: the silent-failure window is a standing error between 50,001 counts and the max allowable position deviation (default 2 shaft rotations = 6,553,600 counts). Above that limit you get fatal error 45 instead of a silent no-op, which is at least visible.
- The homing APPROACH SPEED is approximately maxDistance/maxDuration — the firmware paces the internal move across the full maxDuration (bench: 2 rotations with a 5 s budget crawled at ~0.4 rot/s). A generous maxDuration is not just a timeout, it directly makes the approach gentler; conversely a short duration rams the stop fast. During homing both bit 4 (homing) and bit 6 (busy) are set — homing is the one common operation where the busy bit is actually observable. Curiosity: homing with maxDistance 0 is accepted and simply holds the homing/busy state for the full maxDuration without moving; maxDuration 0 rejects with fatal error 34.
- If homing never hits an obstacle it travels the full maxDistance and stops — indistinguishable from a collision by status alone; compare positions to tell.
- For repeatable origins, home 2-3 times and average; back off ("relieve") 10-50 degrees between runs.
- After homing and zeroing, set 'Set safety limits' and/or 'Set max allowable position deviation' so later collisions fault out safely instead of grinding.
- PID constants ('Set PID constants', u32 P, I, D) apply immediately, are not validated, have no read-back, and reset to firmware defaults on reset. Known-good M17 values (also the firmware defaults): P=2000, I=5, D=175000. GOTCHA: D values below 32 are quantized to zero derivative action.
- 'Set max allowable position deviation' continuously watches how far the measured position has drifted from the commanded position; if the difference exceeds the limit, fatal error 45 trips (asynchronously — the offending move itself returns success). The default is 2 shaft rotations. The parameter is a signed 64-bit value on the wire and the firmware takes its absolute value, so a negative input acts as its positive equivalent. The deviation check stays armed even with MOSFETs disabled: back-driving a disabled motor more than the limit trips error 45.
- GOTCHA: 'Disable MOSFETs' does NOT stop or clear queued motion — the commanded position keeps advancing invisibly while the rotor stands still, and the deviation check then trips fatal error 45. Stop motion (queue empty or 'Emergency stop') before or together with disabling.
- Temporary torque release in closed loop: 'Disable MOSFETs' keeps the closed-loop mode bit, and a later 'Enable MOSFETs' resumes closed-loop control directly — no second 'Go to closed loop' needed — with a gentle re-engagement (bench-measured ~540 counts ≈ 0.06 degrees, versus ~4 degrees for a cold open-loop enable). CAVEAT: the commanded position is still held from before the disable, so if the shaft moved while unpowered (gravity, hand), the PID yanks it back on re-enable and can trip the deviation limit — re-zero or re-command first if the shaft may have moved.

## Reading state

- get_status() returns [statusFlags, fatalErrorCode]. Bits: 0 = in bootloader (if set, all other bits are invalid/zero), 1 = MOSFETs enabled, 2 = closed-loop mode, 3 = calibrating, 4 = homing, 5 = go-to-closed-loop in progress (M1 product only), 6 = busy. A clean post-reset baseline is flags with bits 0 and 1 clear and fatalErrorCode 0. GOTCHA: no bit reflects ordinary queued motion — bit 6 (busy) stays 0 during trapezoid/velocity moves (verified by polling at 20 Hz through a move; it covers long-running tasks like calibration and homing) — so detect motion via 'Get n queued items' plus position-settled, per golden rule 7. Also note the closed-loop bit (bit 2) SURVIVES 'Disable MOSFETs' (drivers off, mode retained) but is CLEARED by any fatal error. In the fatal-error state the flags all read 0 as of firmware 0.15.4.0 (the MOSFETs are off and nothing is running); in older firmware they were frozen at stale pre-error values, so only fatalErrorCode was trustworthy there.
<!--LANG:PYTHON-->
- Return shapes from M3 methods: single-output commands return a bare scalar; multi-output commands return a flat list; broadcast returns []. Version numbers arrive least-significant-first: [development, patch, minor, major] — reverse for display.
<!--LANG:END-->
<!--LANG:ARDUINO-->
- Return shapes from Servomotor methods: single-output commands return a bare scalar (float for the unit-converted variants, an integer type for the ...Raw() variants); multi-output commands return a struct named after the method, for example getStatusResponse from getStatus() with fields statusFlags and fatalErrorCode. Version numbers arrive least-significant-first — reverse them for display. Remember that on failure these return an all-zero value rather than an error, so check getError() first.
<!--LANG:END-->
- 'Get position' (commanded) vs 'Get hall sensor position' (measured): compare them to detect stalls or missed motion; at rest expect the hall reading within a few hundred counts of the commanded one. 'Get comprehensive position' returns all three (commanded, hall, external encoder) in one round trip and is safe to poll at 20 Hz. The external encoder field is a raw count from an optional external quadrature encoder — it reads 0 when none is fitted, is never zeroed by 'Zero position', and is NOT in motor position units. GOTCHA: the library nonetheless runs it through the motor's position unit conversion like the other two fields (the default shaft_rotations divides it by 3,276,800); to see the raw count, select the encoder-counts position unit, use the raw variant of the call, or multiply back by the position conversion factor.
- 'Get max PID error' returns [min, max] and RESETS the window on every read. min > max (the sentinels 2147483647 / -2147483648) means "no data since last read" — it only accumulates in closed loop (and in closed loop the window is never empty, since the PID runs every tick). Because the library converts the values into your position unit, the sentinels come out looking like plausible numbers — [+655.36, -655.36] in shaft_rotations — so detect them by min > max, never by magnitude. After entering closed loop, read it once and discard, then read again after your move to measure tracking quality. The values come back converted into your selected POSITION unit (easy to misread as raw counts). Typical unloaded figures: idle holding dither about +/-500 counts (+/-0.05 degrees); worst tracking error during a brisk 2-rotation/1-second trapezoid about 5000 counts (0.55 degrees).
- 'Get debug values' also resets some of its fields on read (profiler max-times, hall-delta stats) — values are per-read-window, not cumulative; the hall-delta average is meaningless when no samples accumulated since the last read.
- 'Get temperature' reads a sensor on the driver PCB; the conversion table covers roughly 33-307 C and out-of-range readings clamp to 0 (so 0 means "below ~33 C", not freezing). A working motor under load should show the value climbing.
- 'Get supply voltage' returns the bus voltage (internally in tenths of a volt); allow ~0.2 s after reset before the first read for the ADC to settle. Expect your PSU voltage within a few percent.
- 'Read multipurpose buffer' has read-once semantics: a successful read CLEARS the buffer. Reading an EMPTY buffer returns a single byte of 0 (the data-type tag for "nothing stored") as of firmware 0.15.4.0; older firmware sent no response at all, so the read timed out and the timeout meant "buffer empty".

## Communication robustness

- CRC32 is enabled by default (and after every reset). Leave it on. With CRC enabled, a packet sent without a CRC (or corrupted) is silently DROPPED — the symptom is a TimeoutError, not an error reply. If a device stops answering right after you toggled CRC state, your framing no longer matches its setting; recover by sending 'System reset' first without a CRC and, if that times out, again with one. On a MULTI-DROP bus, never mix CRC modes at all: a device switched to no-CRC treats the 4 CRC bytes of every CRC'd frame it hears — including broadcasts meant for the whole fleet — as excess payload and latches fatal error 51 (bench-verified on a 39-motor bus: one mode-mismatched motor faulted on the first CRC'd broadcast while the other 38 diverged silently). Also never use alias-addressed queries when aliases are duplicated (all holders answer at once — pure collision garbage; bench-verified with 39 same-alias responders); address by unique ID instead.
<!--LANG:PYTHON-->
- The M3 object controls its outgoing CRC32 via motor.set_crc32_enabled(True/False) — after sending 'CRC32 control' 0 to the device, call motor.set_crc32_enabled(False) and the high-level methods keep working. (Older library versions had no such control — the wrapper always appended a CRC32, and CRC-disabled operation required servomotor.communication.execute_command(..., crc32_enabled=False).) One command still needs execute_command instead of the M3 wrapper: 'Multimove' (mixed-unit move list in raw internal units).
<!--LANG:END-->
<!--LANG:ARDUINO-->
- The Servomotor object controls its outgoing CRC32 via motor.enableCRC32() and motor.disableCRC32(), with motor.isCRC32Enabled() to read the current setting. It starts enabled, matching the device. After sending crc32Control(0) to the device, call motor.disableCRC32() so your framing matches again — and vice versa.
<!--LANG:END-->
- 'Get communication statistics' returns six u32 counters in this order: [crc32_errors, packet_decode_errors, first_bit_errors, framing_errors, overrun_errors, noise_errors]; the input flag (1) resets them after reading. These count silently-dropped garbage — poll them to monitor bus health (they should stay at 0 on a healthy bus).
- If you suspect you sent a garbled or partial packet, keep the bus idle for at least 100 ms — the firmware's receiver resynchronizes after 100 ms of silence (bench-measured threshold: 95 ms of delay was not enough, 105 ms was — the documented figure is exact). Distinguish the failure modes: a CORRUPTED packet (bad CRC) is dropped and self-clears — the very next command works immediately, no wait needed; only a TRUNCATED/incomplete frame jams the parser and needs the 100 ms silence. Each malformation feeds a specific 'Get communication statistics' counter (LSB-clear first byte -> firstBitErrorCount, bad CRC -> crc32ErrorCount, bad declared size -> packetDecodeErrorCount).
- The protocol's CRC32 is the standard CRC-32 (zlib/PNG polynomial, little-endian on the wire) — verified against Python's zlib.crc32. Useful when implementing the protocol on a new platform.
<!--LANG:PYTHON-->
- servomotor.detect_devices_iteratively() flushes the receive buffer itself (before each round and before returning). If you drive 'Detect devices' at a lower level, or suspect stray bytes in the host receive buffer for any other reason, call servomotor.flush_receive_buffer() before normal traffic resumes.
<!--LANG:END-->
<!--LANG:ARDUINO-->
- A reply that starts and never finishes (a collision, a motor rebooting mid-reply, noise on an unbiased pair) times out with getError() == -1 after the usual one-second budget, and the library discards the remains of that abandoned frame so they cannot be read as the head of the next reply. In library versions before 0.10.1 this case HUNG FOREVER with no timeout and no error, and any leftover bytes surfaced as a spurious -7 on the following command — if you are on an older library, upgrade.
- The Arduino library still does not drain stale receive bytes before sending a request, and exposes no flush of its own. Bytes can therefore still linger in some situations — most often after 'Detect devices', which produces several replies over about a second. If you suspect the buffer is out of step, drain it yourself before normal traffic resumes: `while (Serial1.available()) Serial1.read();`
- The Arduino read timeout (1 second) covers the WHOLE reply and requires the entire payload to be buffered at once, unlike the Python library, whose timeout restarts on each received byte. Long streamed responses that work from Python can therefore time out on Arduino unless you enlarge the UART receive buffer first.
<!--LANG:END-->
- Verify the link before trusting motion: ping the device 10-100 times with random 10-byte payloads and require exact echoes. 'Ping' requires exactly 10 bytes.
- Under normal conditions reads do not fail — bench validation ran hundreds of commands with zero communication failures. A few long-running stress tests wrap status reads in a retry loop (their comments call the reads "unreliable" after heavy motion), but a dedicated stress investigation could not reproduce any failure on current firmware/library (49,360 reads across 729 aggressive motion segments, zero failures, all six device-side communication counters at 0) — those comments predate current firmware/library revisions and are not a latent bug. If you ever do see read failures, the first diagnostic is 'Get communication statistics' (then check wiring, grounding, CRC state) rather than routinely retrying them away.
- NEVER transmit a second command before the previous command's reply has fully arrived (or timed out). The bus is half duplex: a device starts replying within microseconds, so a back-to-back second request collides with that reply on the shared wire pair and BOTH are destroyed (bench-verified: two valid packets in one write produced only collision garbage; the same two packets a few milliseconds apart worked perfectly). The library's request/reply methods already enforce this — the rule matters when writing raw bytes or broadcasting rapid-fire sequences.
- Only one process can own the serial port; close it before letting another tool open the same device.

## Error handling and recovery

Every code below is a LATCHING fatal error: the motor disables itself, the red LED blinks the code, and every subsequent command except 'Get status' and 'System reset' comes back with that same code. Only 'System reset' clears it — 'Emergency stop' does not (and is itself not a fatal error). After the reset the device is back to its power-on defaults, so every recovery ends with "re-establish your settings". The Error Codes section at the end of this document lists all of them with full causes and solutions; this table is just the short list of the ones customers actually hit, described in terms of what you did wrong rather than what the firmware detected.

| Code | What you most likely did | What to do |
|---|---|---|
| 45 position deviation too large | Disabled the MOSFETs while the queue was still running (the commanded position keeps advancing while the shaft stands still); the shaft was back-driven or hand-turned; the motor current is too low for the load; or you commanded a speed the motor cannot physically reach | Stop motion before disabling the MOSFETs; check the load and raise 'Set maximum motor current'; slow down. The check is armed even while the MOSFETs are off. |
| 18 run out of queue items | A velocity or acceleration move sequence ended with the motor still at nonzero velocity, or a streaming path let the queue run dry. A velocity move does NOT stop by itself | Always end such a sequence with a segment that brings velocity to zero, queued back-to-back with the moving segments; when streaming, keep the queue fed |
| 15 accel too high | A 'Trapezoid move', 'Go to position' or 'Homing' duration too short for the distance. Note this is an ACCELERATION error even though it feels like a speed problem — see the peak-versus-average discussion above | Lengthen the duration (minimum is roughly displacement/maxVelocity + maxVelocity/maxAcceleration, plus a few percent) or raise 'Set maximum acceleration' |
| 16 vel too high | A 'Move with velocity' (or a Multimove velocity segment) above 'Set maximum velocity'. Also fires DURING execution if you lower the velocity limit while moves are running | Reduce the velocity or raise the limit; change limits only while stopped with an empty queue |
| 28 predicted velocity too high | A 'Move with acceleration' (or a Multimove acceleration segment) whose acceleration is legal but is applied for long enough to end above the velocity limit | Use a smaller acceleration, a shorter duration, or raise the velocity limit |
| 8 queue not empty | 'Zero position', 'Go to closed loop', 'Homing' or 'Start calibration' sent while moves were still queued. These four are the only commands with this requirement | Poll 'Get n queued items' to 0 first, or clear the queue with 'Emergency stop' (also disables the MOSFETs) or 'Reset time' (leaves them on) |
| 17 queue is full | More than 32 queued items. A trapezoid move or go-to-position normally takes 3 slots, so about 10 of them fit; velocity and acceleration moves take 1 each | Pace with 'Get n queued items'. Note the fault aborts the whole in-flight sequence, it does not merely reject the extra item |
| 34 parameter out of range | A duration of 0 on any move, a maxDuration of 0 on homing, a 0 passed to 'Set maximum velocity' or 'Set maximum acceleration', or 'Set safety limits' with lower greater than upper | Validate before sending. Watch for a small duration rounding down to 0 during unit conversion |
| 13 not in closed loop | 'Homing' in open loop. Homing is the ONLY command that requires closed loop; everything else works in open loop | 'Go to closed loop' first and let it complete, then home |
| 19 motor busy | A move-queueing command, 'Go to closed loop' or 'Start calibration' sent while a calibration or a homing was still running. It does NOT mean "still moving" — ordinary queued motion never sets the busy state | Wait for status bit 6 (busy) to clear. During calibration, do not poll at all: wait a fixed time, then 'System reset' |
| 23 max pwm voltage too high | 'Set maximum motor current' given a motorCurrent or regenerationCurrent above the product ceiling — 390 on M17. The same ceiling applies to both parameters | Range-check both values. Remember they are internal current units, not amps |
| 40 overheat | Sustained high torque, poor airflow, or a stall. Monitored only while the MOSFETs are enabled — which is why leaving a machine energised and idle for hours can trip it | Let it cool, then reset. Reduce load, duty cycle or current limit; improve heat sinking; disable the MOSFETs between work bursts |
| 14 overvoltage | Regenerated energy from decelerating an inertial load, or the shaft spun by hand while powered, pushed the supply rail up | Brake more gently, add bulk capacitance or use a supply that can absorb current, do not spin the shaft forcefully while powered |
| 51 command size wrong | A payload that does not match the command's expected size. The realistic field cause is a multi-drop bus with mixed CRC32 settings: a device switched to no-CRC reads the 4 CRC bytes of every CRC'd frame it hears as excess payload | Never mix CRC modes on one bus; keep CRC32 enabled everywhere |
| 50 bad alias | 'Set device alias' with 252, 253 or 254, which the protocol reserves | Use 0-251, or 255 for "no alias assigned" |

Two codes are internal-consistency faults rather than user errors: 30 (control loop took too long) and 42 (position discrepancy). If you see either, report it to the manufacturer along with your firmware version.

<!--LANG:PYTHON-->
- Exception types from the library: TimeoutError (no response — wrong address/port, device dead, in bootloader, CRC mismatch; on firmware before 0.15.4.0 also an empty multipurpose-buffer read), FatalError (the device answered with a nonzero error code; e.args[0] is the code from the error reference in this document), CommunicationError (malformed/corrupt response), PayloadError (defined and exported by the package but not raised by any current code path), NoAliasOrUniqueIdSet. All of these are the library's own exception classes (plain Exception subclasses) — in particular servomotor.TimeoutError is NOT Python's built-in TimeoutError, so catch servomotor.TimeoutError (or import it), not the builtin; a bare `except TimeoutError:` will not catch it.
- A rejected command can surface two ways depending on timing: the call itself raises FatalError, OR the call returns normally and the error appears shortly afterwards in get_status()[1] (asynchronous checks like position deviation always do the latter). Robust code checks both: catch FatalError, and after risky operations sleep ~0.2 s and poll get_status().
- GOTCHA (firmware up to 0.15.4.0; fixed in 0.15.5.0): a fatal error that fires in the fraction of a millisecond while a command's reply is still transmitting can corrupt that exchange — the reply may be truncated (host sees TimeoutError), or an extra unsolicited error packet may arrive and shift every later reply by one (host sees FatalError raised against the wrong command). If a state-changing command times out or errors unexpectedly, call servomotor.flush_receive_buffer(), then probe with 'Get status'. In the worst timing (observed on the bench with inverted safety limits), the device can lock up completely — silent to every command including 'Get status' and 'System reset' — and only a power cycle recovers it. Firmware 0.15.5.0 closes all of this: an in-flight reply is completed before the fatal-error state takes over, the duplicate error packet is suppressed, and the hardware-verified result is clean error delivery even with a fatal firing during heavy reply traffic.
- Recovery recipe from any fatal error (this is golden rule 5 applied, plus two follow-up steps): read and note the error code ('Get status' or the FatalError exception); system_reset(); wait the post-reset delay (per golden rule 1: 0.5 s or more of bus silence); then verify get_status() == clean baseline; and re-establish your settings (units are host-side and survive, but limits, gains, zeroed position, and enabled state are gone).
- 'Emergency stop' is NOT a fatal error: it disables the MOSFETs and clears the queue, and you can resume with enable_mosfets() without any reset. (Firmware before 0.15.4.0 had a bug here: a mid-motion 'Emergency stop' or 'Reset time' left an internal planner variable stale, making later acceleration-type moves validate incorrectly — on old firmware, send 'System reset' after a mid-motion stop before queueing new moves. Fixed in 0.15.4.0.)
- GOTCHA: 'Reset time' also clears the motion queue and halts motion instantly (it shares the stop path with 'Emergency stop' but leaves MOSFETs enabled). Do not send it mid-motion unless you intend an abrupt stop.
- On KeyboardInterrupt during motion: emergency_stop(), then disable_mosfets().
- GOTCHA: several library input errors (wrong argument count, out-of-range integer for the wire type, unknown command, malformed alias) call exit(1) instead of raising — a long-running application should validate values before calling the library.
- A watchdog for unattended loops: put a hard time limit on any loop that commands motion, so a logic bug cannot run the motor forever.
<!--LANG:END-->
<!--LANG:ARDUINO-->
- Nothing throws. The library reports every outcome through getError(): 0 on success, a POSITIVE value equal to the motor's own fatal error code (look it up in the Error Codes section), or a NEGATIVE host-side communication code. The negative codes you can actually get are -1 timeout (no reply within one second: wrong alias, wrong pins, unpowered motor, device in the bootloader, CRC-mode mismatch, or a broadcast, which never replies), -2 wrong payload size, -3 bad response character, -5 your buffer was too small, -6 CRC32 mismatch, -7 bad first byte (the receiver is out of frame), and -9 packet too small. See the Arduino Essentials section for the full treatment.
- A rejected command can surface two ways depending on timing: getError() is nonzero immediately after the call, OR the call succeeds and the error appears shortly afterwards in getStatus().fatalErrorCode (asynchronous checks such as position deviation always do the latter). Robust code checks both: check getError() after every command, and after risky operations wait about 0.2 s and then read getStatus().
- GOTCHA (firmware up to 0.15.4.0; fixed in 0.15.5.0): a fatal error that fires in the fraction of a millisecond while a command's reply is still transmitting can corrupt that exchange — the reply may be truncated (you see -1), or an extra unsolicited error packet may arrive and shift every later reply by one (you see an error attributed to the wrong command). If a state-changing command times out or errors unexpectedly, drain the port (`while (Serial1.available()) Serial1.read();`) and then probe with getStatus(). In the worst timing the device can lock up completely and only a power cycle recovers it. Firmware 0.15.5.0 closes all of this.
- Recovery recipe from any fatal error (golden rule 5 applied, plus two follow-up steps): read and note the error code (getStatus().fatalErrorCode, or the positive getError() from any ordinary command); systemReset(); wait the post-reset delay (0.5 s or more of bus silence, 1.5 s for margin); verify getStatus() reads a clean baseline; and re-establish your settings — your unit selection is host-side and survives, but limits, gains, zeroed position and enabled state are gone.
- 'Emergency stop' is NOT a fatal error: it disables the MOSFETs and clears the queue, and you can resume with enableMosfets() without any reset. (Firmware before 0.15.4.0 had a bug here: a mid-motion emergencyStop() or resetTime() left an internal planner variable stale, making later acceleration-type moves validate incorrectly — on old firmware, send systemReset() after a mid-motion stop before queueing new moves. Fixed in 0.15.4.0.)
- GOTCHA: resetTime() also clears the motion queue and halts motion instantly (it shares the stop path with emergencyStop() but leaves the MOSFETs enabled). Do not send it mid-motion unless you intend an abrupt stop.
- A watchdog for unattended loops: put a hard time limit on any loop that commands motion, so a logic bug cannot run the motor forever. Remember every command blocks the sketch for up to a full second when a motor does not answer, and the library never calls yield().
<!--LANG:END-->

## Upgrading the firmware

Firmware upgrades go over the same RS485 bus as everything else, using the Python tool. The upgrade protocol itself has no Arduino-side implementation — but that does NOT mean a motor wired to an Arduino or ESP32 is unreachable. If the motor's only RS485 connection is to a microcontroller, flash that board with a TRANSPARENT USB-to-RS485 BRIDGE and run the ordinary Python tool through it. A working bridge is in this repository at `Arduino_library/ESP32S3_RS485_Bridge/`; it is about twenty lines and simply relays bytes both ways. This is strongly preferable to reimplementing the upgrade in C++, because the protocol stays in the tested Python and because the same bridge then gives every other host tool access to that motor. Bench-verified: an M17 whose only link was an ESP32-S3 was upgraded 0.15.9.0 -> 0.15.12.0 through such a bridge, and the whole 111-module test suite was then run against it. If you write your own bridge, read the two gotchas at the end of this section first — both cost a full test run to find.

- Get the tool: `pip3 install --upgrade servomotor`. Since library version 0.12.0 this installs an `upgrade_firmware` command directly, along with `servomotor_command`, `detect_and_set_alias_all_devices` and `show_device_information_for_all_devices`. No repository checkout is needed. Do not install pyserial — a copy is bundled inside the package and is what the library actually imports.
- Use library version 0.12.2 or later. ON WINDOWS, 0.12.0 and every earlier version that bundled pyserial could not open a serial port at all — and the failure is easy to misdiagnose, because `import servomotor` SUCCEEDS. A working import is therefore not evidence that your version is good. The error appears only when you actually open a port, as `ModuleNotFoundError: No module named 'serial'` raised from `from serial import win32` (the bundled pyserial's Windows backend used absolute imports; fixed in 0.12.1). Separately, versions before 0.12.1 aborted with an uncaught PermissionError when they could not save the remembered serial port into the installed-package directory — which a normal system-wide install hit every time. 0.12.2 moves that file to a per-user config directory and adds the SERVOMOTOR_PORT environment variable, so you can name the port once instead of passing -p to every command below.
- The `.firmware` image files are NOT part of the pip package; obtain them separately.
- CHECK COMPATIBILITY FIRST. Run `show_device_information_for_all_devices -p <PORT>` and note the Product Code and Firmware Compatibility Code. The file name encodes both: `servomotor_M17_fw0.15.9.0_scc3_hw1.5.firmware` means model `M17` and compatibility code `3`. Both must match your device exactly.
- A MISMATCH IS SILENT. The check happens in the device's bootloader, which simply ignores any page whose codes do not match and says nothing on the bus; the explanatory message goes only to an internal debug port you cannot see. The tool has no way to notice.
- ADDRESS ONE MOTOR: `upgrade_firmware -p <PORT> -a <ALIAS> <file.firmware>`. The tool's DEFAULT is the broadcast address 255, which flashes every matching device on the bus simultaneously but receives no replies at all — so in broadcast mode it prints "Firmware page N written successfully" for every page and exits successfully even when absolutely nothing was written. Use `-a <ALIAS>` so a problem shows up as a timeout rather than as a false success. (If a device still has no alias, assign one first with `detect_and_set_alias_all_devices`.)
- Be aware that the tool's first action is a BROADCAST 'System reset', so every motor on the bus reboots into its bootloader for the duration of the transfer, not just the one you are flashing. Do not upgrade a bus that is mid-motion or safety-critical.
- VERIFY AFTERWARDS — the tool does not. Run `servomotor_command -p <PORT> -a <ALIAS> get_firmware_version` and confirm the new version with inBootloader = 0. (Note: `show_device_information_for_all_devices` fails with an assertion error against a device sitting in the bootloader, so use `servomotor_command` for this check.)
- YOUR CALIBRATION AND ALIAS SURVIVE. An upgrade only rewrites the application flash pages. The device alias, the three hall-sensor midlines, the commutation position offset and the motor-phases-reversed flag live in a separate settings page that the burn routine refuses to write. You do not need to recalibrate after an upgrade.
- AN INTERRUPTED UPGRADE CANNOT BRICK THE MOTOR. The bootloader lives in pages the upgrade also refuses to write, and it verifies the application by CRC32 on every boot. A half-written image simply fails that check, so the device stays in the bootloader, still answering RS485 — recognisable by a fast-blinking green LED (about 10 Hz, versus the brief once-per-second blip of the running application) and by 'Get status' bit 0 being set. Recovery is to run the same `upgrade_firmware` command again.

## Two ways to fault a motor with a setting, not a move

Both of these are commands that are perfectly legal, return normally in most circumstances, and
nonetheless take the machine down. Neither is documented anywhere else.

**Lowering the speed limit while a move is running faults that move.** The obvious way to slow a
machine down is to reduce 'Set maximum velocity'. Do that mid-move and you get fatal error 16, an
abrupt uncommanded stop, and a latched fault — the opposite of the intended effect. The cause is
that the limit is enforced in two places with different scopes: a move is validated against the
limits when it is QUEUED, but the control loop also re-checks the LIVE velocity against the
CURRENT limit on every one of its 31,250 ticks per second. Lowering the ceiling below the speed a
move is already travelling at therefore faults a move that was legal when queued.
Bench-measured: dropping the ceiling from 5 to 2 rot/s during a move that peaks near 0.5 rot/s is
harmless; dropping it to 0.2 rot/s faults immediately with code 16.
**To slow a machine down, stop it first, or only ever RAISE the limit mid-flight.**

**Setting a safety fence that excludes the current position usually loses its own reply.** A fence
with lower < upper that simply does not contain where the motor currently is, is accepted and
correctly faults with error 25 within one control tick. But the fence goes live before the
command's reply is sent, and the fault fires from the control loop, so the reply is frequently
never transmitted. Bench-measured over 30 trials: **10 of 30 lost the reply (33%)**, versus 0 of 30
for a fence that includes the current position. The fault itself is reliable — code 25 was raised
in 29 of 30.

So a timeout from 'Set safety limits' does NOT mean the motor is gone. Treat it as "check status":
read 'Get status' and you will find error 25. Note that the firmware already guards the closely
related case — an INVERTED fence (lower > upper) is rejected cleanly with error 34 before the
limits are applied, precisely to avoid this race — so only the excludes-current-position case
behaves this way.

## Diagnosing a faulted motor: you get the error number and nothing else

Golden rule 5 says every command except 'Get status' and 'System reset' fails once a fault latches.
The consequence is worth stating separately, because it shapes how you should write monitoring
code: **no telemetry survives a fault.** Temperature, supply voltage, position, the device clock,
even 'Get product info' and 'Ping' all return the latched error code instead of their data. The
single number from 'Get status' is the entire diagnostic surface.

That is a sound fail-safe — a faulted machine should refuse to act — but it means the readings you
would want in order to work out WHY a motor stopped (did it overheat? did the supply sag? where was
it?) are unavailable at exactly the moment you need them.

**Therefore: log telemetry continuously while things are working, not after they break.** Polling
is cheap and provably harmless — bench-measured, polling any getter continuously through a move
leaves the endpoint bit-identical to an undisturbed reference, and reads consume no queue slots.

### If you write your own USB-to-RS485 bridge, two traps

Both produce the same symptom: short exchanges work perfectly, sustained traffic fails
intermittently, and the failure looks exactly like a firmware fault under load. Both were found the
hard way, by a 111-module suite reporting six modules "failing under load" that were nothing of the
kind.

1. **Yield to the scheduler when idle.** A `loop()` that polls both ports and never blocks starves
   the RTOS idle task, and on an ESP32 the task watchdog eventually resets the board mid-run. The
   host then sees a plain read timeout. One `delay(1)` when no bytes moved fixes it; without it a
   200-cycle soak died at cycle 96.
2. **Do not use `readBytes()` to move the data.** It inherits Stream's one-second timeout, so if
   the driver reports N bytes available but returns fewer, the loop blocks for up to a second in
   one direction and cannot relay the reply travelling the other way. The host times out on a
   command the motor answered correctly. Use `read()` in a loop bounded by `available()`, so
   neither direction can ever stall the other. Fixing this took one test module from 22 passed /
   18 failed to 40 passed / 0 failed.

Also: enlarge both receive buffers. USB CDC delivers a 2,067-byte firmware page far faster than a
230,400 baud wire can carry it, and the default CDC receive buffer is only 256 bytes.

## Behaviour by firmware version

Read your firmware version before anything else — 'Get firmware version' — and check this list before assuming a behaviour described elsewhere in this document applies to you. Version numbers arrive least-significant-first. Everything below is M17; the current release is 0.15.12.0.

| Firmware | What changed, and how you would notice |
|---|---|
| 0.15.3.4 | PID overhaul. With an integral gain of 10 or more the motor no longer hunts after a move; the derivative term actually acts (it previously had a sticky offset proportional to kD); holding torque no longer collapses when the maximum motor current is set below about 64; changing PID constants mid-operation no longer produces a derivative kick. |
| 0.15.4.0 | Validation and lock-up hardening. A duration of 0 on any move command, and a value of 0 to 'Set maximum acceleration', now raise fatal error 34 instead of silently succeeding. 'Test mode' 0 now clears test modes instead of hanging the device until a power cycle. Reading an EMPTY multipurpose buffer now replies with a single 0x00 byte instead of staying silent — code that treated a read timeout as "buffer empty" must be changed. 'Get status' in the fatal state now reports all flags 0 instead of stale pre-error values. A 'Multimove' with more than 32 moves gives a clean fatal error 24. After a mid-motion 'Emergency stop' or 'Reset time' you can queue new moves directly, with no intervening reset. |
| 0.15.5.0 | Fatal-error/reply race fixed. A fatal error firing while a reply is transmitting no longer truncates that reply, no longer emits an extra unsolicited error packet that shifts every later reply by one, and no longer hard-hangs the device (which previously needed a power cycle). 'System reset' now always works in the fatal state. 'Set safety limits' with lower greater than upper is rejected cleanly with fatal error 34 instead of faulting one control tick later. |
| 0.15.6.0 | 'Set maximum velocity' with 0 is rejected with fatal error 34. Previously a zero limit was accepted and every trapezoid/go-to-position was then silently planned as a zero-motion dwell — success reply, queue occupied for the full duration, shaft never moved. |
| 0.15.7.0 | Boot-default motion limits corrected: maximum velocity 68 to 34 rot/s, maximum acceleration 2,000 to 500 rot/s^2. Also new in this release: 'Set maximum velocity' and 'Set maximum acceleration' can RAISE the limit above the boot default at all — earlier firmware clamped every request at the boot default itself. |
| 0.15.8.0 | Boot defaults aligned to the product specification: maximum velocity 560 RPM (9.3333 rot/s), maximum acceleration 12,000 rot/s^2. THIS IS THE MOST LIKELY MIGRATION HAZARD. Relative to 0.15.6.0 and earlier the default velocity limit is about 7.3x LOWER (68 to 9.3333 rot/s) while the default acceleration limit is 6x HIGHER (2,000 to 12,000 rot/s^2). Any program written against older firmware that commands faster than 560 RPM now gets a fatal error at queue time and needs one explicit 'Set maximum velocity' call. The maximum SETTABLE velocity also drops, to a firmware clamp of 18.67 rot/s — requests above it are silently clamped and still reply success. |
| 0.15.9.0 | 'Set max allowable position deviation' with the extreme negative wire value now saturates to the maximum limit instead of storing a negative limit that latched fatal error 45 on the next control tick. That landmine existed in 0.15.4.0 through 0.15.8.0. |
| 0.15.12.0 | Trapezoid-move planner fixes, all three customer-visible. (a) A move is no longer SILENTLY IGNORED when the acceleration limit is very high relative to the speed limit. Setting a speed limit below about 0.384 rotations/second while leaving the 12,000 rotations/second^2 factory-default acceleration — an ordinary slow, precise axis — used to make every 'Trapezoid move' and 'Go to position' return success and never move the shaft. They now move correctly. (b) A move that is far too aggressive is no longer silently WRONG. The planner's acceleration is now saturated rather than truncated into a 32-bit field, so instead of moving an arbitrary distance (measured: 8,191 counts where 3,276,800 were asked for) or not at all, the move is rejected with fatal error 15. (c) When the acceleration ramp works out shorter than one 32-microsecond control tick, the move is queued as a constant-velocity segment plus a stop instead of a three-segment ramp. That removes an intermittent fatal error 18 that struck roughly one ordinary slow move in five. Note the consequences: some very short moves that used to be rejected are now performed (they were always within the speed limit), and an over-commanded short move may now report fatal error 16 rather than 15. |

Boot defaults on the current firmware, for reference: maximum velocity 9.3333 rot/s (560 RPM), maximum acceleration 12,000 rot/s^2, maximum motor and regeneration current 200 internal units (of a 390 ceiling), maximum allowable position deviation 2 shaft rotations, PID constants P=2000 I=5 D=175000. All of these are volatile and return on every reset.

## Clock synchronization (multi-motor coordinated motion)

- Purpose: 'Time sync' does not set the motor's clock — it disciplines the clock RATE (trims the internal oscillator) so motors converge on the master's timebase. Establish a common epoch first: broadcast 'Reset time' to alias 255 (all clocks zero simultaneously — remember this also stops motion, so do it before motion starts) and record the host's epoch at that instant.
- Then send time_sync(master_time_since_epoch) to EACH motor individually every 0.1 s (10 Hz), in whatever time unit the M3 object is configured with (the library converts to the command's microsecond wire unit; e.g. with the default 'seconds', pass time.time() - epoch directly). (Older library versions converted this command incorrectly in every unit except 'timesteps' — there, pass raw microseconds with time_unit='timesteps'.) GOTCHA: a broadcast 'Time sync' is a complete no-op in current firmware — each motor must be addressed individually. (Verified both in the firmware source, where the sync action sits inside the not-broadcast branch of the handler, and by bench experiment: 30 broadcast syncs left the oscillator trim untouched, while the same 30 syncs sent addressed shifted the clock rate by about 0.7%.)
- Budget about a minute of syncing before trusting tight synchronization (bench-measured: a -3.4 ms initial offset converged to within ±200 microseconds in ~40 s at 10 Hz). Steady-state error should sit within ~5000 microseconds; occasional single-sample spikes are host-side USB jitter, not drift.
- The sync reply returns the motor's clock error in microseconds (positive = motor behind master) — track it as a health metric.

## Performance envelope (what the test programs demonstrate works)

- Queued sequence timing is accurate to about +/-0.1 s over multi-second sequences; execution starts essentially immediately when the first move enters an empty queue. Bench-measured device clock rate offset versus the host: +0.41% (+4100 ppm) fast over a 120 s free-run, consistent with the "well under 1%" spec, so a ~5% wait margin on timed moves is ample.
- Commanded (desired) position readback lands within 50-80 counts of the target in the tests (under 0.003% of a rotation; profile discretization); hall-measured position pass/fail tolerances are much looser: ~500 counts at rest, ~20,000 counts tracking an open-loop move. Closed-loop holding tolerance used for pass/fail: ~4000 counts (0.12% of a rotation). Position reads agree across unit systems to within 1 count (commanded) / ~1000 counts (measured, due to jitter between sequential reads).
- End-of-move accuracy for an identical unloaded 2-rotation/1-second trapezoid (bench, M17): open loop landed 504 counts short (~0.055 degrees — commutation-step quantization), closed loop 37 counts (~0.004 degrees) — about 14x tighter. The position-deviation watchdog is hard to trip with attainable commands on a free shaft (even an instantaneous 8 rot/s velocity step stayed within a 0.2-rotation limit); its practical role is catching stalls, overloads, and unattainable commands.
- Decelerating pumps energy back into the supply: a hard 10-to-0 rot/s stop of just the free rotor produced a +0.4 V blip on a 20 V bench supply (no sag was measurable during the matching acceleration). With significant load inertia and a supply that cannot sink current, regeneration is what pushes the rail toward the overvoltage fatal (error 14) — budget for it or brake more gently.
- Move durations from 0.32 ms (10 timesteps) up to minutes work; the theoretical minimum segment is 1 timestep = 32 microseconds.
- The RS485 link at 230400 baud carries roughly 23 KB/s. Measured command round trip ('Get status' via a USB adapter, 200 samples): median 3.4 ms, 95th percentile ~5 ms, worst outlier 15 ms — and statistically identical while the motor executes a move, so ~290 status reads/second are attainable idle or moving. Rapid enable/disable cycling (50 back-to-back) and continuous pinging at up to ~100 Hz pacing throughout a 70 s move sequence are demonstrated reliable, with zero failures.
<!--LANG:PYTHON-->
- The library's read timeout restarts with every received byte, which is why a multi-second 'Capture hall sensor data' stream completes despite the nominal 1.2 s timeout; a fatal error mid-capture kills the stream instead (the read times out — probe 'Get status' afterward).
<!--LANG:END-->
<!--LANG:ARDUINO-->
- The Arduino library's 1-second read timeout is a single budget for the whole reply and it does NOT restart per byte, so a long streamed response such as 'Capture hall sensor data' is much more constrained here than in the Python library. Keep captures small, and enlarge the UART receive buffer before constructing the Servomotor object if you need a large payload. A fatal error mid-capture kills the stream (the read times out — probe getStatus() afterward).
<!--LANG:END-->
- Current setting guidance from working programs: ~150-200 internal units as a general working value; up to 390 for demanding moves; ~10-50 for deliberately compliant/soft behavior (catching at 20, soft homing at 50; the main homing test homes at 200 and uses 10 as the deliberately-too-weak value); the current limit doubles as a programmable force limit. The units map directly to PWM drive duty (bench: open-loop holding PWM telemetry reads exactly setting/2). Two measured cautions: at very low current (20) the unloaded motor still runs but parks with a PERMANENT ~0.6-rotation standing error and NO error flag (too weak to overcome its own friction — check |commanded − measured| after moves if you run soft); and raising current does NOT buy top speed (free-shaft top speed measured mildly LOWER at 390 than at 50-100).
- Closed-loop tuning envelope (bench, free shaft, 1-rot/0.6-s test move): tracking error improves from P=500 to P≈4000 and then degrades sharply — in-motion oscillation starts between P=4000 and 8000, and at P=64000 the motor buzzes even at rest; none of this raises a fatal error (the signature is a large 'Get max PID error' span and audible buzz, not a fault). D values below 32 are confirmed identical to zero, and D only becomes visibly effective around 10^5 (default 175000 is mid-range; ~700k damps a P=8000 oscillation best). The PID arithmetic is exact and inspectable live via Test mode 3: P-term = P x error, output = (P+I+D) >> 11. The integral anti-windup clamp works: even I=500 with seconds of forced windup overshoots by a bounded ~23k counts regardless of windup duration.
