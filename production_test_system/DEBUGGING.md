# Debugging a hang / livelock (runbook)

Read this first if the test system **wedges**: the UI/detection is stuck, a bus
shows `running` forever, "1 detection queued" never clears, or the machine fans
spin up (high CPU). This happened once — three bus-worker threads spun at 100%
CPU and we had **no way to see what they were doing** (macOS SIP blocks
`py-spy`/`lldb` from attaching to the Xcode-Python the venv is built on) and **no
durable logs** (the in-memory log had drained). The mechanisms below exist so
that never happens again. They are always on and add no meaningful overhead.

## When it's wedged, do this (in order)

1. **Get the thread stacks — the single most useful action.** HTTP stays
   responsive even when the workers are wedged:

   ```
   curl http://localhost:8000/api/debug/stacks
   ```

   This prints every thread's Python stack. Find the three `bus-A/B/C` threads:
   whatever file:line they sit on **is the bug**. (Also saved to the durable log,
   see below.)

2. **Check the durable log** for what led up to it (worker phase transitions,
   errors with full tracebacks, and one `VITALS` line per minute showing CPU and
   per-bus state over time):

   ```
   curl http://localhost:8000/api/debug/log?lines=400
   #   ...or read the file directly (survives restarts):
   #   data/diagnostics.log   (and data/diagnostics.log.old after rotation)
   ```

   If the watchdog already saw the high-CPU signature it will have **auto-dumped
   the stacks into this log** (`WATCHDOG ALERT ...`) — so the evidence may
   already be captured even before you ran step 1.

3. **Backup stack dump without HTTP** (SIP-immune, no attach, no sudo). Find the
   server PID and signal it; stacks are appended to `data/faulthandler.log`:

   ```
   lsof -nP -iTCP:8000 -sTCP:LISTEN -t           # -> PID
   kill -USR1 <PID>
   tail -n 200 data/faulthandler.log
   ```

4. **Quick health probe** (machine-readable): `curl http://localhost:8000/api/debug/vitals`

**Capture stacks BEFORE restarting.** A restart fixes the symptom but destroys
the evidence. Grab steps 1–3 first, then restart to recover.

## What's installed (see `backend/diagnostics.py`)

- `GET /api/debug/stacks` — all-thread Python stacks on demand.
- `GET /api/debug/log` / `GET /api/debug/vitals` — durable log tail / health.
- `data/diagnostics.log` — durable log: app/worker events (teed from the app
  log), full exception tracebacks from bus workers, one `VITALS` line/minute, and
  any watchdog auto-dumps. Rotates at ~16 MB to `.old`.
- **Watchdog thread** — logs vitals every 60 s and, on **sustained high CPU**
  (≥1.5 cores for ~3 min — the livelock fingerprint), auto-dumps all stacks to
  the durable log. A false trigger (e.g. a long PNG-generation burst) only costs
  one extra dump in the log; it never interferes with a run.
- **`SIGUSR1` faulthandler** — `kill -USR1 <pid>` dumps all stacks to
  `data/faulthandler.log`. Works while a thread is spinning in a C extension
  (sqlite3) and needs no debugger, so it bypasses the SIP restriction.

All of this is observational only — it never kills threads, cancels work, or
touches the RS485 bus.

## Note on the original incident (root cause still unknown)

The trigger was **not** reproducible by yanking the adapters during Phase 9
(Phase 9 is ~5 s; the operator does not unplug mid-test). The symptoms were:
three bus workers pegged at 100% CPU spinning in **SQLite reads** (no DB writes,
no serial I/O), state frozen at `running / phase 9`, holding stale serial fds
from before a USB re-enumeration, with a detection queued behind the stuck run.
If it recurs, `GET /api/debug/stacks` will show the exact loop immediately —
paste that output to pin the root cause.

## The fix (deferred — implement once a stack dump pins the loop)

Only diagnostics were added so far. Once `/api/debug/stacks` (or the watchdog
auto-dump) shows the exact spinning line, fix the **disconnect intolerance** that
let a dead/changed USB adapter wedge a run. The hardening, roughly in order:

1. **Detect the disconnect.** Open the serial port with a `write_timeout` (it is
   currently `None` in `transport.py:SerialTransport.__init__` → a `write()` to a
   vanished FTDI can block forever), and treat `OSError`/`serial.SerialException`
   as a **fatal bus error** that aborts the current job instead of being swallowed
   as a generic `Exception`.
2. **Guarantee cleanup on every exit path.** Ensure `bus_worker._run_sequence`
   (and `_run_detect_passes`) always reset `running`/`detecting`/`pending_detections`
   and `close()` the transport in `finally`, even on a hard serial fault — so the
   queue never wedges behind a stuck job and stale fds are not leaked.
3. **No unpaced loops.** Whatever loop the dump implicates: give it a wall-clock
   cap and a mandatory minimum sleep so an instantly-failing serial call (dead fd
   returns immediately, defeating the read timeout) can never spin a core.
4. **Tolerate USB re-enumeration.** A replug gives the adapter a new device node;
   detect a stale fd and reopen the port by path before a job (each job already
   calls `_open_transport`, so the main gap is not getting *stuck* in the old one).

Add a regression test that simulates a mid-run serial disconnect (raise from the
transport) and asserts the run aborts cleanly with flags reset and the queue
drainable.
