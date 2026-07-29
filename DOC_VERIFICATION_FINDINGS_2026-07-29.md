# Source-verification findings, 2026-07-29

**Origin:** a real customer support case (Gearotons M17-40 + ESP32-S3, July 2026). The AI that
handled the case wrote `DOCUMENTATION_IMPROVEMENT_REPORT.md` (lives outside this repo, at
`/Users/tom/Projects/protein_purification_machine_Jeffs_lab/`). Acting on it, every technical
claim in that report — and a good deal of surrounding code — was re-verified against firmware,
Arduino library and Python package source by a 32-agent verification sweep.

**What was already done** (committed and pushed, `1b56b63`, `1d159c3`, `36c387b`):
the documentation work itself. See those commit messages for the full list.

**What this file is for:** everything the sweep found that was *not* acted on, because it is a
code change rather than a documentation change, or because it is outside the scope of that task.
All of it is source-verified with file:line unless explicitly marked otherwise. None of it is
bench-verified.

---

## 1. Corrections the report itself needed

Recorded because the report is otherwise excellent and may be consulted again. Three of its
claims were wrong and are already fixed in the shipped docs:

| Report claim | Truth |
|---|---|
| A too-short trapezoid move raises fatal error **28** | It raises **15** (ERROR_ACCEL_TOO_HIGH). `add_to_queue` checks the derived acceleration (`motor_control.c:1748-1750`) before the predicted velocity (`:1755-1757`), and for a trapezoid queued from rest the acceleration limit is always reached first. Verified by exhaustively sweeping the exact integer math: 299,030 random parameter sets, zero cases where 28 fired without 15 firing first. `motor_commands.json:69` ('Go to position') already said this correctly. |
| The peak is the "**cruise** speed" | Wrong for exactly the short moves the finding is about. When `duration <= 2 x rampTime` the clamp at `motor_control.c:450-452` drives the coast segment to zero duration and it is dropped (`:1735-1742`) — the profile is a pure triangle with no cruise phase, and the move takes 2 of the 32 queue slots instead of 3. Say **peak**. |
| `upgrade_firmware.py` is not shipped by pip; the tool validates model/compatibility before writing | Both false as of `c2df14c`. It ships as a console script since servomotor **0.12.0** (`PyPi_distribution/pyproject.toml:21-25`). And the tool never queries the device — the check is device-side in the bootloader (`bootloader_STM32G031/Src/main.c:182-191`) and is invisible to the host, so a broadcast upgrade prints "page N written successfully" and exits 0 even when nothing was written. |

Also corrected: the report attributed the boot-default limit change to 0.15.8.0 as "halved
velocity, quartered acceleration". That describes the **0.15.7.0** BUG-23 fix. Net change from
<=0.15.6.0 to >=0.15.8.0 is velocity **/7.3** (68 -> 9.3333 rot/s) and acceleration **x6**
(2,000 -> 12,000 rot/s^2).

---

## 2. Firmware — open issues

### 2.1 Divide-by-zero in `compute_trapezoid_move` (STATIC ANALYSIS ONLY — needs a bench probe)

`motor_control.c:449` computes `delta_t1 = max_velocity / max_acceleration` in integer
arithmetic. When internal `max_velocity < max_acceleration` this truncates to **0**, and
`:455-456` then computes `denominator = (delta_t1 + delta_t2) * delta_t1 = 0` followed by
`numerator / denominator`.

Reachable with **legal, non-clamped** limit settings on M17 — roughly whenever
`maxAcceleration >= 31250 x maxVelocity`:
- `Set maximum velocity` 1 rot/s + `Set maximum acceleration` 48000 rot/s^2
- `Set maximum velocity` 0.5 rot/s + `Set maximum acceleration` 20000 rot/s^2

`set_max_velocity` (`:3259-3271`) and `set_max_acceleration` (`:3278-3290`) reject only 0, not
this combination. On Cortex-M0+ the aeabi 64-bit divide-by-zero helper does not trap, so the
likely symptom is a silently wrong acceleration (typically 0 -> success reply followed by a
zero-motion dwell) rather than a crash — the same failure mode already documented for
`maxVelocity = 0` in knowhow.md. **Probe on the bench before deciding whether to document or fix.**

### 2.2 Dead code that invites mis-citation

- `MAX_HOMING_ERROR` = 50000 at `motor_control.c:51` is referenced nowhere. The live constant is
  `HOMING_MAX_POSITION_ERROR` at `motor_control.c:1404`. Cite only the latter.
- `get_max_velocity()` / `get_max_acceleration()` (`motor_control.c:3273`, `:3292`, declared
  `motor_control.h:209`, `:213`) are called from nowhere in `firmware/Src` or
  `common_source_files`, and would truncate an int64 into an int32 if used. **There is no command
  to read back the velocity or acceleration limit.** The only readback is `Get debug values`
  (cmd 45), whose `maxVelocity`/`maxAcceleration` come back in **raw internal units with no unit
  conversion** — divide by 450,359,962,737.05 (velocity) or 14,411,519.45 (acceleration) for
  rot/s and rot/s^2 on M17.

### 2.3 Latent inconsistency (harmless today)

`motor_control.c:122` (and the simulator path at `:3650`) initialise the regeneration limit from
`DEFAULT_MAX_MOTOR_PWM_VOLTAGE`, not `DEFAULT_MAX_MOTOR_REGEN_PWM_VOLTAGE`. Both are 200 for
every product (`motor_control.h:121-140`), so nothing misbehaves — but the two would silently
diverge if anyone ever set them differently.

### 2.4 Stale source comments — do not use as data points

- `common_source_files/device_status.h:12-13` still says bits 5 and 6 are "Not used, set to 0".
  The `#define`s at `:21-22` and the code at `motor_control.c:3376-3381` prove otherwise
  (bit 5 = go-to-closed-loop, bit 6 = motor busy).
- `common_source_files/settings.h:11` says `BOOTLOADER_N_PAGES 5 // 8 kB bootloader`.
  5 x 2048 = **10 kB**; `memory_layout.ld:40` correctly reserves 10K.
- `firmware/Src/temperature.c:12` (commented out) claims ADC 14000 is "about 70 degrees C".
  Through the firmware's own conversion it is 56 C.

---

## 3. Arduino library — open issues

`Servomotor.h`/`.cpp` are AUTO-GENERATED. Fix generated behaviour in
`autogeneration/generate_command_code_module/`, not in the output.
`Communication.cpp`/`.h` are hand-written.

### 3.1 Receive hang on a truncated reply (highest severity here)

`Communication.cpp` `getResponse()` passes `TIMEOUT_MS - (millis() - startTime)` into
`receiveBytes` (call sites `:289, 327, 363, 407, 427, 442, 633`). Once elapsed exceeds
`TIMEOUT_MS` (1000, `:6`) that expression is evaluated in **unsigned** arithmetic and wraps,
arriving in the `int32_t timeout_ms` parameter as a negative number. `:660` then compares
`uint32_t` against `int32_t`, the negative converts back to ~4.29e9, the condition can never be
true, and the busy-wait at `:659` (`while (_serial.available() < numBytes)`) never exits.

Reachable when a valid first size byte arrives and the rest of the packet never does — bus
collision, a motor rebooting mid-reply (system reset / calibration), or noise on an unbiased
RS485 pair. **Symptom: the sketch freezes, with no timeout and no error.**

### 3.2 No way to recover from stray receive bytes

The defensive pre-request drain is commented out at `Communication.cpp:139-142`, and
`Communication::flush()` (`:637-642`) is unreachable from user code — `_comm` is private
(`Servomotor.h:550`) and `Servomotor` exposes no flush. Leftover bytes from a timed-out exchange
get parsed as the next response, surfacing as `getError()` of -7, -3 or -6. The documented
workaround is user-side (`while (Serial1.available()) Serial1.read();`), which is what the docs
now say — but exposing a flush would be better.

### 3.3 Timeout semantics differ from Python, and bulk reads are constrained

The 1-second timeout is a budget for the **whole** reply and does not restart per byte (the
Python library's does). Worse, `receiveBytes` waits for the entire chunk to be simultaneously
buffered (`:659`), so any payload larger than the platform's UART RX ring can *never* satisfy the
condition — ESP32 core default is 256 bytes, classic AVR 64. `captureHallSensorData()` and
`readMultipurposeBuffer()` are the affected calls. Customers must call
`Serial1.setRxBufferSize(n)` **before** constructing the `Servomotor` (the constructor opens the
port). Documented; not fixed.

### 3.4 Smaller items

- **No RS485 DE/RE direction control** anywhere (zero `pinMode`/`digitalWrite` in
  `Communication.*`/`Servomotor.*`). The library assumes an auto-direction transceiver. Anyone
  wiring a plain MAX485 with a manual DE pin has no hook.
- **The shipped examples construct the motor as a local inside `setup()`**
  (`example_one_move.cpp`, `example_one_move_two_motors.cpp`, `example_trapezoid_move.cpp`), so
  the object is destroyed when `setup()` returns. Gives customers no template for `loop()`.
- **`library.properties:9` omits `avr`** from `architectures`, so Uno/Nano/Mega users see the
  library as incompatible — though on ATmega328P `Serial1` does not exist anyway.
- **Float precision.** All unit conversion goes through 32-bit float
  (`AutoGeneratedUnitConversions.*`). Past 16,777,216 counts = 5.12 shaft rotations a float
  cannot represent every count; at 1000 rotations granularity is ~256 counts (~0.028 deg).
  `getCurrentTime()` casts a uint64 microsecond count to float — after an hour of uptime,
  ~0.4 ms resolution. Setters **truncate** toward zero rather than round
  (`Servomotor.cpp:192`). Use the `...Raw()` variants when exactness matters. Documented.
- **The port is opened once per sketch** — `static bool s_commSerialOpened`
  (`Communication.cpp:16`, guarded `:111`, set `:121`). The first `Servomotor` constructed fixes
  port, baud and pins for every object afterwards. RX/TX pin arguments are honoured **only on
  ESP32** (`:112-120`) and silently ignored elsewhere. Documented.

---

## 4. Documentation and JSON inaccuracies still open

These were found but NOT fixed, because they sit in files outside the task's scope. Each is
customer-visible.

| Where | Problem |
|---|---|
| `motor_commands.json` cmd 17 'Go to closed loop' | Says it "runs a current-sensor baseline check ... which can raise fatal error 22, ERROR_CURRENT_SENSOR_FAILED". **False on M17**: `check_current_sensor_and_enable_mosfets()` for M17 (`motor_control.c:3501-3511`) only sets `TIM1->CCR1 = 0` and enables the MOSFETs. The baseline check exists only in the M1/M2 (`:3456-3499`) and M23 (`:3514-3538`) variants. This text is mirrored into `Servomotor.cpp:742` and **both** generated API documents. |
| `motor_commands.json` cmd 21 'Trapezoid move' | Lists ERROR_ACCEL_TOO_HIGH and ERROR_PREDICTED_VELOCITY_TOO_HIGH without saying which fires for a too-short duration. It is **15**. 'Go to position' (`:69`) already states this correctly — make them match. |
| `motor_commands.json:562,572` | Declares the 'Set maximum motor current' InternalUnit as `arbitrary_units`, a name that **no longer exists** in the generated unit tables (renamed to `internal_current_units` at `autogeneration/autogenerate_unit_conversions.py:502`). Verify current-unit conversion still resolves in both libraries. |
| `PyPi_distribution/README.md:16-19` | Still lists "pyserial 3.5+" under Requirements. pyserial is **vendored** (`servomotor/vendor/serial/`, imported relatively) and `pyproject.toml` declares no dependencies at all. This text renders on the **PyPI project page**. |
| `Arduino_library/README.md:130-146` | The communication-error list omits **-9** (PACKET_TOO_SMALL) and -4, and documents **-8** as "the third byte in the response (expected to be the Command Byte) was invalid" — wrong twice over: responses contain no command byte, and -8 is never returned by any code path. The -1 timeout breakdown also lists a "Command byte" stage that does not exist in `getResponse()`. (The class-name and debug-output errors in this same file were fixed in `1d159c3`.) |
| `Arduino_library/API_usage_by_category.ino:41` | States the voltage default is `millivolts`; the generated code is `VoltageUnit::VOLTS` (`Servomotor.cpp:18`). Not currently shipped by the deploy script, but wrong. |
| `servomotor_command.py:19` | Help text says "For example ENABLE_MOSFETS_COMMAND". Name matching strips spaces/underscores and compares against the JSON `CommandString`, so that resolves to `None` -> "Unknown command". The working form is `enable_mosfets` (no `_COMMAND` suffix). |
| `show_device_information_for_all_devices.py:94` | Bare `assert(in_bootloader == 0)`. Crashes with an AssertionError against a device sitting in the bootloader — **exactly** the recovery scenario after a failed firmware upgrade. Use `servomotor_command ... get_firmware_version` there instead. |

---

## 5. Facts worth keeping (verified, already in the shipped docs)

- **Homing's false-collision trap.** `HOMING_MAX_POSITION_ERROR` = 50,000 counts
  (`motor_control.c:1404`) = 0.0153 rotation = **5.49 deg on M17/M23**. Compared on the **first**
  32 us control tick (`:2539-2541`, before `handle_queued_movements()` at `:2554`), so a standing
  error aborts homing before it starts. The angle is product-specific: M1 = 28.1 deg,
  M2 = 3.94 deg. The silent-failure window is 50,001 .. 6,553,600 counts; above that you get
  fatal error 45 instead.
- **The closed-loop lurch is torque-limited but NOT speed-limited.** `max_velocity` is only ever
  compared against planner variables (`:1755`, `:1799`, `:2105`), never against the rotor. What
  actually bounds the correction: the PID output clamp (+/-51,200 at defaults), the 90-electrical-
  degree commutation lead clamp (16,384 counts), the current/PWM clamp, and the integral clamp
  (25% of authority). On M17/M23 `start_go_to_closed_loop_mode()` (`:1140-1158`) does **not**
  re-align measured to commanded — only the M1 path does (`:1131`).
- **Minimum trapezoid duration** = `displacement/vEff + rampTime`, where `rampTime` is
  `maxVelocity/maxAcceleration` truncated **down** to a whole 32 us timestep and
  `vEff = maxAcceleration x rampTime <= maxVelocity`. The simpler
  `displacement/maxVelocity + maxVelocity/maxAcceleration` is slightly optimistic (~1.3% of the
  first term at M17 boot defaults, where rampTime = 24 steps = 768 us and vEff = 9.216 rot/s).
  Add a few percent of margin.
- **Current M17 boot defaults** (all RAM-only, restored on every reset): max velocity 9.3333 rot/s
  (internal 4,203,359,652,212), max acceleration 12,000 rot/s^2 (internal 172,938,225,691),
  motor and regeneration current 200 (ceiling 390), max allowable position deviation 2 shaft
  rotations (6,553,600 counts), PID P=2000 I=5 D=175000. Settable ceilings: 18.67 rot/s (2x) and
  48,000 rot/s^2 (4x), silently clamped, still replying success.
- **`error_codes.json` is trustworthy.** Unlike the `Description` fields elsewhere, it is
  hand-verified and is the generator input for `common_source_files/error_text.h`. Spot-checked
  against firmware for 25 codes with zero discrepancies. **Prefer its prose over newly-invented
  cause/fix wording.**
- **Flash map:** page size 2048, 32 pages, 64 KB. Pages 0-4 = product info + bootloader (10 KB,
  never written by an upgrade), 5-30 = application (52 KB), 31 = non-volatile settings (alias,
  three hall midlines, commutation offset, phases-reversed flag). `burn_firmware_page()` rejects
  anything outside 5-30 (`common_source_files/settings.c:131`), which is why calibration and
  alias survive an upgrade. Caveat: page 31 has no magic number or checksum
  (`settings.c:9-12` is a raw memcpy), so that guarantee holds only while the struct layout is
  unchanged between the two firmware versions.

---

## 6. Process note for whoever picks this up

`API_documentation/autogeneration/` now renders `knowhow.md` into **both** the Python and Arduino
documents from one source, using `<!--LANG:PYTHON-->` / `<!--LANG:ARDUINO-->` blocks plus a
curated method-name substitution table. Full details in
`API_documentation/autogeneration/README.md`. After editing any shared source, regenerate and
check leakage in both directions:

```bash
cd API_documentation/autogeneration && python3 M17_generate_api_documentation.py
cd .. && grep -c 'LANG:' M17_servomotor_*_API_documentation.md        # must be 0
grep -c 'get_status(\|zero_position(\|M3(' M17_servomotor_Arduino_API_documentation.md
grep -c 'getError\|Serial1' M17_servomotor_Python_API_documentation.md
```

The generated `.md` and `.pdf` files are build outputs. Never edit them directly.
