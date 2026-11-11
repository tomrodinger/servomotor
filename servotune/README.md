# ServoTune browser bench

This is a local adaptation of Matthias Wandel's October 2026 Gearotons
ServoTune. The UI, serialized transport, immediate-target loop and plot are
rewritten code; this is not Matthias's unchanged application. His original
experiment used timed position moves. The application has no npm, CDN or internet dependencies.

Run from the repository root:

```sh
python3 servotune/serve.py
```

The launcher selects a free port, prints its loopback URL and opens the browser
in **Demo mode**. To choose a port or leave browser opening to the user:

```sh
python3 servotune/serve.py --port 8765 --no-browser
```

Stop the local server with Ctrl-C. The server serves only this directory and
listens only on `127.0.0.1`.

## Without a motor

Open the printed URL ending in `/?demo=1`, or open the base URL and press
**Start demo**. The yellow banner explicitly identifies simulated data. Demo
mode requires no serial-port permission. Try:

1. Enable closed loop.
2. Send a target, or send a target and read position.
3. Start alternating targets; continuous scope capture starts automatically.
4. Stop test to hold the last target, or Drive off / Emergency stop to disable
   the demo outputs.
5. Edit PID values and press Write PID; Read PID returns them. Reset motor
   restores the defaults.

Sending a target before enabling closed loop demonstrates fatal error 13.
Sending one after Drive off demonstrates error 55. Reset motor recovers either
case. The demo consumes/produces actual binary packets with CRC and fragmented
responses. Its displayed motion is an illustrative first-order response, not a
model of the physical motor, PID gains or timing. This proves that the browser
app runs; it does not establish physical firmware acceptance.

The demo models the compact command-50 reply of stable 0.17.0.0. For
compatibility QA, `/?demo=1&demo-legacy=1` models the 24-byte reply of 0.17.0.2.

## With a motor

Use a desktop browser exposing Web Serial (Chrome or Edge), served from this
loopback URL. Disconnect the demo, set the motor address, then press Connect
serial and select the USB–RS485 adapter. Use a 16-digit unique ID if several
motors share an alias. The client deliberately does not broadcast discovery.

Connection queries the firmware version, product specifications, PID gains
(when supported) and status. It never writes PID settings or enables outputs
automatically. Stable **0.17.0.0** supports immediate targets and the compact
command-50 reply. Development **0.17.0.2** uses the legacy 24-byte reply;
development **0.17.0.3+** uses the compact eight-byte reply. These gates explicitly
recognize stable .0, whose version bytes sort below the development builds.
Development .1 remains disabled because of its historical ISR overhead.
The exact stable image passed 35,148+ physical canary updates with zero
communication errors through Python/C++ clients. Stable/legacy browser demo
checks also passed; browser hardware acceptance was not part of this release.
Release evidence and timing limitations: `firmware/immediate_target_validation/2026-11-11/release_0.17.0.0/hardware/HARDWARE_RESULTS.txt`.
The previous stable 0.16.0.0 supports PID readback, without immediate targets.
Existing .2 motors remain supported without schema guessing or motion retries.

The target entry is in degrees relative to the current session zero, using
counts-per-rotation queried from the device. Enable closed loop deliberately
before sending targets. Both commands require an empty trajectory queue and
enabled closed-loop position control; invalid requests latch a fatal error
and disable outputs. Large reference steps have no trajectory speed profile.

**Stop test** stops generating updates and retains the last accepted target.
The scope continues sampling so the settling response remains visible.
**Drive off** and **Emergency stop** disable outputs. **Disconnect** only
closes the serial connection; it does not command a physical stop or remove
holding torque. Use the motor controls before disconnecting when needed.

The alternating test steps between the entered angle and its negative. Green
is the accepted reference, drawn as held steps; blue is measured Hall position,
sampled continuously throughout each move. The scope uses the existing Get
comprehensive position command (37) to read reference and measured position in
one reply. Combined target/position replies (50) also mark each accepted step.
The latter alone sample the motor just as the reference changes, before it has
time to move; they cannot establish the ensuing step response. The scope does
not smooth, invert or synthesize measured overshoot. Gaps over 100 ms are left
blank, with individual sample markers, rather than interpolated into ramps.
**Stop scope** pauses the plot, including combined-reply samples, without
stopping the alternating test. Starting the test again resumes capture if needed.

## Transaction handling

`protocol.mjs` keeps exactly one request outstanding until its entire response
including CRC arrives. Other callers wait in a host-side transaction queue.
Scope polling and alternating targets schedule their next request after the
previous reply. Position samples are not unsolicited motor transmissions.

The Communication panel updates once per second, independently of scope
capture. It shows the total successful request/reply transactions in the
current connection, completed transactions per second over the latest interval,
and average time per transaction in milliseconds (elapsed milliseconds divided
by completed transactions). The calculation uses actual elapsed time if a
browser timer is delayed. Time includes intentional pauses between requests;
it measures the effective transaction rate rather than isolated reply latency.
An interval with no completed transactions shows 0 transactions/s and no time
per transaction. Reconnecting starts a new total and measurement interval.

The parser handles fragmented/coalesced USB chunks, extended packet lengths,
the existing compact success ACK, and data/error replies. Response byte 2 in
normal short data/error replies is an error code, not a command echo. The
pending transaction identifies the payload schema. A fatal error cancels
already queued work. Stop/disable/reset actions discard queued work before
submitting their command; an existing on-wire transaction must finish first.

Timeouts, invalid frames, CRC errors and failed writes latch a connection
fault and stop further transmissions until reconnect. Commands are never
automatically retried. Reopening drains input for 150 ms before the first
request. Normal reset is silent; the firmware's fatal-error handler sends a
compact acknowledgement before reset. The client accepts either behavior and
reserves the full 1.5-second quiet interval. One
browser client must own the adapter; this does not arbitrate multiple masters.

Command 50 is `Set immediate position target and get position`; its compact
reply contains only measured Hall position. Stable 0.17.0.0 and development .6
validate with interrupts enabled, mask only the position stores, then read
cached Hall atomically with a separate mask. An ISR may run between the write
and read. It takes no new measurement and does not wait for a control tick.
Success confirms reference installation, not controller execution or arrival.
The earlier .3/.4 builds return a first-tick sample; .5 copies cached Hall in
the write mask. These timing differences do not alter the compact wire schema.

The client retains the exact requested target for the chart and computes
position error locally. The Last
target reply panel labels that value as Target sent; it does not present it as
an echoed field. Device timestamps, status flags, control mode and queue depth
are absent from the compact reply. Explicit status queries still update the
motor status display. The older 24-byte reply is decoded for compatibility,
with the same simplified display. The chart uses browser receipt times for
both reply formats. Wrong reply lengths fault the connection; the client never
retries a motion command or guesses a different schema after sending it.

## Tests

```sh
node servotune/test_protocol.mjs
node servotune/test_scope.mjs
python3 python_programs/test_host_immediate_target.py
```

Optional browser QA uses Playwright and an installed Chromium browser:

```sh
SERVOTUNE_URL='http://127.0.0.1:8765/?demo=1' node servotune/test_browser.cjs
```

Install Playwright in your test environment if needed, or set
`PLAYWRIGHT_MODULE` to an existing Playwright installation. No Playwright
dependency is needed to run ServoTune itself. Browser QA covers load, PID
read/write, both targets, automatic continuous capture when starting only the
alternating test, scope pause/resume, settling capture, fatal errors, reset and
disconnect. Scope tests check step geometry, measured overshoot preservation
and sparse capture gaps.
Protocol tests exercise both command-50 schemas, exact signed-i64 values,
schema mismatches without retries, delayed fragmented replies, concurrent callers,
coalesced packets, compact ACKs, CRC errors, timeouts/late replies, cancelled
work and write failures. The Python host test compiles the real firmware
handlers, queue logic and control ISR, including full signed-i64 targets.

Hardware browser QA is available in `test_browser_hardware.cjs` with
`test_hardware_serial_bridge.py`. It runs the production browser UI, stream and
transaction code against one real motor through a test-only stdio/UART bridge.
This is used because the available Chromium automation interface cannot select
the native serial chooser. It does not replace the native Web Serial adapter
selection in the delivered app. The test restricts every packet to the specified
unique ID, uses requested limits of 50 internal current units each for motor
and regeneration current (approximately 256 mA using the library's conversion
of 0.195 internal units/mA) and +/-1 degree targets, then restores disabled
startup defaults. These are configured limits, not measured current. The bridge
selects internal current units explicitly. Set `SERVOTUNE_PORT`, `SERVOTUNE_UID`,
`SERVOTUNE_URL`, `SERVOTUNE_REPORT`, and optionally `PLAYWRIGHT_MODULE` before
running it. The test expects stable 0.17.0.0 by default; set
`SERVOTUNE_FIRMWARE_VERSION=0.17.0.2` to test compatibility on an older motor.
Use `SERVOTUNE_FIRMWARE_VERSION=0.17.0.0` for the currently flashed canary; the bridge
also accepts development 0.17.0.6.
Versions 0.17.0.4 and 0.17.0.5 are also accepted by the bridge and retain the
same compact command-50 reply. The bridge verifies the exact
expected version before beginning motion tests.
Exact stable release evidence and limitations are recorded in
`firmware/immediate_target_validation/2026-11-11/release_0.17.0.0/hardware/HARDWARE_RESULTS.txt`. The release qualified one canary with Python/C++ clients;
its browser checks used simulated motors and do not qualify native serial
chooser behavior or physical browser operation.

A bookmarked URL may prefill fields without connecting or issuing commands:
`/?address=04245C407D26CCD0&target=1`. Use the actual rack adapter in Chrome's
chooser, then Enable closed loop and send a target. The 2026-11-05 rack adapter
was `/dev/cu.usbserial-113210`; adapter paths can change on reconnection.
