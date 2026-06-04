# Servomotor Production Test System

A backend-driven system for production-testing servomotors over RS485, built to
the specification in [`../PRODUCTION_TEST_PROPOSAL.md`](../PRODUCTION_TEST_PROPOSAL.md)
(the authoritative spec).  It runs a 15-phase test sequence on up to 144 motors
across three independent RS485 buses, stores all raw data, and scores pass/fail
in a separate, re-runnable post-processing stage.

## Quick start

```bash
cd production_test_system
./run.sh --sim       # simulated rack — no hardware needed; try the whole UI
# or
./run.sh             # real hardware; assign the three serial ports in the UI
```

Then open <http://localhost:8000/>.

To run the tests:

```bash
. .venv/bin/activate
python -m pytest
```

## Design (how it maps to the spec)

* **One async worker thread per RS485 bus** (`bus_worker.py`), coordinated by
  `runner.py`.  **All test execution runs in the backend** — closing or
  reloading the browser never stops a run.  A WebSocket streams live state.
* **Two strictly separate stages:**
  * **Stage A — collection** (`phases/`, one module per phase): runs on the
    hardware and stores *all raw data* to SQLite (large arrays as binary blobs).
    No metrics, no PNGs, no pass/fail during collection.  A `system_reset`
    (+1 s bootloader-exit delay) is issued between phases.
  * **Stage B — post-processing** (`evaluation.py`, `png_generation.py`):
    triggered from Tab 2.  *Run Evaluation* derives metrics and applies the
    current criteria → fresh `phase_eval` rows (re-runnable; tagged with a
    criteria version).  *Generate PNGs* renders disposable plots into
    `data/plots/`, named by unique ID + plot type.
* **Database** (`database.py`): SQLite in WAL mode — `motors`, `phase_data`
  (raw, never overwritten), `phase_eval` (derived + pass/fail).  Keyed on the
  8-byte hardware unique ID; `first_detected` is written once.
* **Untested-vs-tested colouring** (`detection.py`): each detected motor in the
  test set is shown **green** when no test work has been recorded for it yet, and
  **orange** (`⚠ already tested`) once it has any `phase_data` or `phase_eval` row
  (`Database.has_any_test_data`).  Plain detection does *not* turn a motor orange —
  re-running detection any number of times leaves a fresh motor green, so the
  operator can spot which motors on a freshly loaded rack still need testing.  The
  colour is decided once per session and preserved across detection passes (run to
  beat RS485 collisions) so a re-detect never flips a colour mid-session.
* **Settings** (`settings.py`): one JSON file, **atomic writes**
  (temp → fsync → `os.replace`), reloaded at startup.
* **Library reuse**: the existing `servomotor` control library
  (`../python_programs/servomotor`) is reused for the command tables, unit
  conversions, CRC and packet encoding/decoding.  Because that library keeps its
  serial port in a module global (single-bus), `transport.py` adds a per-bus
  transport that reuses the library's encoder/decoder while owning its own
  serial port — so three buses run truly in parallel.  `simulator.py` is a
  drop-in command-level simulator used for hardware-free development and tests.

## Layout

```
backend/
  bootstrap.py        put servomotor lib on sys.path (reuse, not duplicate)
  config.py           paths + rack constants
  units.py            unit conversions (loaded from the library's own factors)
  transport.py        per-bus RS485 transport (reuses library encode/decode)
  simulator.py        command-level virtual rack (no hardware)
  motor_client.py     high-level per-motor command helpers
  settings.py         atomic JSON settings
  database.py         SQLite WAL storage
  state.py            live, in-memory state for the WebSocket
  detection.py        detect passes + untested(green)/tested(orange) colour logic
  bus_worker.py       one worker thread per bus (jobs: detect / run / reset)
  runner.py           coordinator + LED-test reconciliation
  phase_defs.py       phase metadata, params, criteria, measured items
  phases/             phase01..phase15 — Stage A collectors
  analysis/           hall peak-finder + thermal best-fit (pure functions)
  evaluation.py       Stage B metric derivation + criteria
  png_generation.py   Stage B plot rendering
  blobs.py            binary (de)serialization for large arrays
  demo.py             optional simulated rack for --sim
  app.py              FastAPI app: REST + WebSocket + static frontend
frontend/             plain HTML/JS/CSS (no build step)
tests/                pytest suite + simulator harness
data/                 (gitignored) SQLite DB, plots, settings.json
```

## Hardware bring-up notes (verify against firmware)

* Motor driver is the **AT5833**; encoder disk has **50 magnets** (N_POLES=50).
* **Calibration auto-reboots** — never send `system_reset` after `start_calibration`;
  keep the bus quiet for the hold, then read status once (pass only if zero).
* **Setting the alias auto-reboots**; every `system_reset` is followed by a 1 s delay.
* **Phase 8** peak-finder mirrors firmware `handle_calibration_logic`; the
  raw `HALL_PEAK_FIND_THRESHOLD = 2000` becomes **8000** here (captured data is
  4× = 64/16).
* **Phase 10 (overvoltage)** uses firmware test modes 74 (22 V) / 75 (26 V),
  added in **fw 0.15.1.0**; a trip is reported as fatal `ERROR_OVERVOLTAGE` (14).
  Enabled by default.
* **Phase 11 (thermal)** runs to the overtemperature cutoff (`ERROR_OVERHEAT`,
  fatal 40) or a max time, and arms a tight deviation limit after spin-up to
  catch a driver cut-out. **Phase 9** uses a low-current closed-loop move and a
  PID-deviation band (current is in internal units, 0–390). See the per-phase
  sections in `../PRODUCTION_TEST_PROPOSAL.md`.
* **Phase 15 (LED test)** uses cmd 36 mode **13** (both LEDs) and locks the motor
  until a power cycle, so it runs **last**.
* The default thresholds are placeholders to be tuned from real captures via the
  Tab 2 / per-phase histograms.
* **Bring-up lessons** (collision-tolerant detection, serialized firmware flash,
  staggered calibration, trapezoid spins, etc.) are documented in the
  "Implementation Notes & Hardware Gotchas" section of the proposal.

### Phase 1 firmware flashing
Phase 1 broadcast-flashes the target release (default **0.15.1.0**) then reads
the version back. Flashing is **serialized across buses** (`_FLASH_LOCK` in
`firmware_flash.py`) — flashing all three at once corrupted a bus on the bench.
Validated on the rack (buses flashed 48/48 cleanly). Toggle off via the Phase 1
`flash_enabled` param to verify versions only.
