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

## Troubleshooting

If the system **wedges** — UI/detection stuck, a bus stuck `running`, "1
detection queued" that never clears, or high CPU — see **[`DEBUGGING.md`](DEBUGGING.md)**.
Short version: hit **`GET /api/debug/stacks`** first (it shows exactly what each
bus worker is doing), and check the durable log at `data/diagnostics.log`. These
always-on diagnostics (`backend/diagnostics.py`) capture the evidence so the
cause can be pinned without a restart.

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
    `data/plots/`, named by unique ID + plot type.  *Run Evaluation* grades the
    motors picked by the **Tab-2 Scope selector** — only the test set, or every
    motor in the database (`Runner.evaluate`) — the same selector that filters
    the device table and plots.  Each motor is graded on every enabled phase
    **plus any phase that has collected data even if that phase is now disabled**
    (`_phases_to_evaluate`), so a disabled phase's leftover data still resolves
    to green/red instead of being stuck on blue; a disabled phase never affects
    the motor's overall pass/fail.
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
* **Live collection-status grid** (`state.py`, `frontend/app.js`, `style.css`):
  the *Rack — live collection status* panel shows one tile per detected motor,
  and every tile is a **15-cell mini-grid** (P1…P15, laid out 3 rows × 5
  columns). Each cell is the live status of that one phase, in five colours:
  **grey** = no data collected, **yellow** = under test right now, **blue** =
  data collected and awaiting evaluation, **green** = evaluated and passed,
  **red** = evaluated and failed. The status is derived in `phase_grid_from_db`
  (per phase: no `phase_data` → grey; data with no up-to-date `phase_eval` →
  blue; an eval at/after the latest data → green when *pass or operator-cleared*,
  else red; re-collecting newer data reverts the cell to blue, i.e. "pending
  re-evaluation"). A freshly detected motor is seeded from the database, so prior
  results show immediately; during a run, each phase **auto-evaluates the instant
  a motor finishes it** — `CollectionContext.store` writes the `phase_data` row,
  immediately evaluates just that (motor, phase), and colours the cell green/red
  (the active phase shows yellow while collecting, and several motors tested in
  parallel show several yellow cells at once). The blue "pending evaluation"
  state therefore only appears transiently or as a fallback if auto-evaluation
  can't run. *Run Evaluation* (Tab 2) re-grades everything and refreshes the
  whole grid via `Runner.refresh_grid_from_db` (e.g. after a criteria change).
  The Phase 15 LED test holds its cell yellow while it awaits the human yes/no,
  then goes straight to green/red once the observation is graded
  (`Runner._grade_led`).
* **Enabled-phase tab colouring** (`frontend/app.js`, `style.css`): each
  per-phase tab (P1…P15) is tinted **light green** when that phase's
  *Phase enabled* checkbox is checked, and left the default **grey** when it is
  disabled — so the operator can scan the tab strip and see which tests will run
  without opening each tab.  The colour is applied from `settings.phases[N].enabled`
  when the tabs are built and toggled live (`setTabEnabledColour`) the moment the
  checkbox changes; the green survives the active-tab highlight (the blue underline
  still marks the selected tab).
* **Device identification — Database tab** (`frontend/app.js`, `runner.py`):
  each device row has two *locate-it-in-the-rack* buttons.  **Flash Green**
  re-triggers the Identify command (cmd 41) so the green LED flashes continuously
  until cancelled (the button toggles to *Cancel Green Flashing*); a single
  per-bus coordinator loop (`Runner._identify_loop`) services every flashing
  device on that bus, so **any number flash at once** over the shared RS485 line
  (the bus's worker is busy while flashing — Detect/Run on it wait until the
  flashes are cancelled or the cap elapses).  **Red and Green Solid** sends test
  mode 13 (both LEDs solid) which is the most visible but **locks up the motor**
  (recover only by power-cycling), so it arms with a 3 s *Lockup Imminent! Undo*
  window before firing, then disables both buttons (*You must Power Cycle the
  Device*).  Solid works while other devices on the same bus keep flashing.  A
  5-minute safety cap and the global **Cancel** both stop flashing.
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
  runner.py           coordinator + LED-test reconciliation + identify (flash/solid)
  phase_defs.py       phase metadata, params, criteria, measured items
  phases/             phase01..phase15 — Stage A collectors
  analysis/           hall peak-finder + thermal best-fit (pure functions)
  evaluation.py       Stage B metric derivation + criteria
  png_generation.py   Stage B plot rendering
  blobs.py            binary (de)serialization for large arrays
  demo.py             optional simulated rack for --sim
  app.py              FastAPI app: REST + WebSocket + static frontend (no-cache)
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
