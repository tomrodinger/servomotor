# Client-library release handoff — 2026-11-12

Python **0.13.0** and Arduino **0.11.0** are released. This document records
the completed work and remaining follow-ups so the conversation can be cleared.

## Releases and source

| Component | Published version | Release / notes |
| --- | --- | --- |
| Python | 0.13.0 | [PyPI](https://pypi.org/project/servomotor/0.13.0/), [release notes](https://github.com/tomrodinger/servomotor/releases/tag/pylib-v0.13.0) |
| Arduino | 0.11.0 | [Release, ZIP and checksum](https://github.com/tomrodinger/Servomotor_Arduino_Library/releases/tag/v0.11.0) |
| M17 firmware | 0.17.0.0 | [Firmware release](https://github.com/tomrodinger/servomotor/releases/tag/v0.17.0.0), [image index](firmware/firmware_releases/README.md) |

[PR #6](https://github.com/tomrodinger/servomotor/pull/6) was squash-merged as
`f0b3673257b1e23f1ff2e5a80cd141419e1bec52`. Its tree exactly matches tested
head `898ff261d39c1322c73a99252c58024f46a0dfdf`. The Python tag is
`pylib-v0.13.0`; the dedicated Arduino repository tag is `v0.11.0`, commit
`36800b13a0d866d6a0e32bc8ad1803ea5241360f`. Firmware and client-library
versions are independent. The monorepo's latest release remains firmware
0.17.0.0; Python release notes were published with `--latest=false`.

Public downloads were checked against the qualified candidates:

| Artifact | SHA-256 |
| --- | --- |
| Python wheel | `523e6b659ab99517cb7863596b8527f8eb0e5e91db55c516c20caca5f861c2bb` |
| Python source archive | `28f0962c3b9be54cc9d6c6b618f8b975bf39e262e0f3801c3e1e2571e81f93e5` |
| Arduino release ZIP | `042e26b487b0d6524910fd99cc34ca28eef0dafa4ed6a00e221c016ffa6ccf98` |
| Arduino Library Manager ZIP | `8d6b251ddf14540313acf522b422870f8b39fde818f3a72a2b376600474d5355` |

All 43 Python package source/data files and five CLI modules match the reviewed
source. All 25 Arduino package files match the staged release; its runtime
sources match those used for physical acceptance. No package needs republishing.

## Release follow-ups

**Arduino Library Manager: complete.** The one-time local check ran on
**2026-11-12 at 18:16 Asia/Shanghai (10:16 UTC)** and confirmed that 0.11.0
is indexed. The index was last modified at 10:00:18 UTC. The
[Library Manager download](https://downloads.arduino.cc/libraries/github.com/tomrodinger/Servomotor-0.11.0.zip)
passed its registry checksum check, and all 25 files matched the qualified
candidate. Its archive checksum differs from the separately packaged GitHub
release ZIP; both contain the expected files.

The local check completed and its desktop-notification command succeeded. It
did not start another AI turn or access motors. No release-check timer remains
pending. Its receipt, result and log are saved locally:

```text
library_validation/2026-11-12-release/arduino_followup/timer.json
library_validation/2026-11-12-release/arduino_followup/result.json
library_validation/2026-11-12-release/arduino_followup/timer.log
```

`result.json` records `indexed=true` and `candidate_files_matched=25`.
For future releases, inspect the [official per-repository log](https://downloads.arduino.cc/libraries/logs/github.com/tomrodinger/Servomotor_Arduino_Library/)
and Arduino's [hourly indexing process](https://github.com/arduino/library-registry/blob/main/FAQ.md#how-can-i-publish-a-new-release-once-my-library-is-in-the-list).

**PyPI trusted publishing:** [workflow run 43192171653](https://github.com/tomrodinger/servomotor/actions/runs/43192171653)
failed at upload with HTTP 403: the token was not allowed to upload to project
`servomotor`. The existing manual PyPI credential configuration successfully
published both qualified artifacts. Public PyPI metadata and the simple package
index list 0.13.0. Repair the project's trusted-publisher registration/permission
for `tomrodinger/servomotor`, workflow `publish-python.yml`, environment `pypi`
before relying on another automated release. Credentials were not copied into
the repository. Do not move released tags or retry uploading existing artifacts.

## Command behavior agreed during review

- Command 48 reads the configured raw PID gains; firmware 0.16.0.0+ supports it.
- Commands 49/50 set an immediate position reference. They require enabled
  closed-loop POSITION operation, an empty queue, zero commanded/planned
  velocity, and valid configured safety/deviation limits. A nonempty queue
  raises fatal error 8; the commands never cancel queued motion.
- The target and empty-queue planner baseline are updated together with a very
  short interrupt mask containing only the stores. The baseline is the starting
  position for a later queued move. Validation runs with interrupts enabled;
  do not put validation or function calls inside that critical section.
- Command 50 separately copies the latest stored Hall position atomically.
  The target update and Hall read need not be atomic together. It does not
  recalculate Hall position from ADC readings or wait for the next control tick;
  the returned measurement can precede application of the target.
- The canonical name is **Set immediate position target and get position**,
  with Python `set_immediate_position_target_and_get_position()` and Arduino
  `setImmediatePositionTargetAndGetPosition[Raw]()`. Keep descriptions concise;
  field definitions already provide their types. See the [protocol specification](common_source_files/PROTOCOL_SPEC.md)
  for exact fields, guards and fatal errors.
- The latest accepted reference is held when updates stop. PID history and
  protections are retained. Success confirms reference installation, not arrival.
  There is no trajectory velocity/acceleration profile for physical catch-up.
- These bindings accept stable 0.17.0.0 or development 0.17.0.3+ compact replies.
  They reject the 24-byte command-50 replies from development .1/.2 without
  retrying motion. ServoTune retains its separate legacy .2 decoder.

Arduino command code was generated through
`autogeneration/generate_command_code.py`, including its templates/helpers;
fresh regeneration matches the committed bindings. Change the generator for
future binding fixes rather than editing generated command methods alone.

## Validation completed

| Scope | Result |
| --- | --- |
| PID readback on 27 rack M17 motors | 5,543 checks; 2,386 gain round trips passed |
| Python on the stable firmware canary | 5,000 immediate updates, both CRC modes, zero communication errors |
| Arduino runtime through the desktop transport | 1,087 assertions and 512 reference updates passed |
| Legacy command-50 schema | Python and C++ rejected old replies in both CRC modes |
| Arduino running on the physical ESP32-S3 | Three consecutive new-command runs, 1,158 assertions / 512 reference updates each |
| Selected additional on-device modules | 190 regression checks plus 22 product checks; total successful qualification checks 3,686 |
| User-observed motion | `motion_basic`: 18 checks passed; two immediate 180-degree steps reached within 5 degrees in about 60 / 67 ms |
| Host and packaging | All 17 host programs; isolated generation/package checks passed |
| Post-merge portability | [All five CI jobs](https://github.com/tomrodinger/servomotor/actions/runs/43191692247) passed on Linux, Windows and macOS |
| Public PyPI wheel | Fresh installed wheel passed 24 command-contract tests and all four CLI help checks |

The dedicated ESP32 motor needed an upgrade from 0.15.12.0 to the existing
stable 0.17.0.0 image before new-command testing. Its factory/bootloader and
settings pages were preserved. No rack motor was reflashed during library
acceptance. The original ESP32 full flash image was backed up locally.

At the initial current setting of 50, the immediate-target hold check missed
its half-degree tolerance. At 100 (below the M17 default of 200), three runs
passed with the same 500-ms wait and position tolerance. This was a test fixture
adjustment; no runtime change was needed. The fast half-turn demonstrations
used reset-default PID/current limits and retained protections. The user observed
both the ordinary spin and a fast half-turn and explicitly approved deployment.

The suite has **120 registered modules; the complete suite was not run**.
Rack-wide PID coverage does not mean every rack motor ran compact command-50
motion: 26 rack motors retained legacy development firmware. Loaded-motion and
physical power-cycle qualification were not performed. Motors were left
disabled/healthy with default PID, empty queues and CRC enabled. The regular
Arduino suite was restored to the ESP32 after the demonstrations and left idle.

## Continuing after clearing context

The remaining release follow-up is PyPI trusted-publisher permission repair;
Arduino indexing and its timer are complete. No additional motor motion or
package release is pending. Hardware tests are in
`Arduino_library/Servomotor_TestSuite_ESP32S3/`; `run immediate_position` selects
the new-command module and `run motion_basic` selects ordinary visible motion.
The USB console uses 230400 baud; board UART pins are TX4/RX5.

Detailed device identities, port/topology records, logs, binaries, flash backup,
timer scripts and deployment receipts remain **local** under:

```text
library_validation/2026-11-12/
library_validation/2026-11-12-esp32/
library_validation/2026-11-12-release/
```

Read `library_validation/2026-11-12-release/PRIVATE_HANDOFF.md` for the exact local
hardware mapping and recovery paths. Do not push the separate local
`prepare/library-command-updates` branch or raw hardware evidence. Recheck USB
identity before any later physical test; saved port names can change. The user
explicitly prohibited communicating with the separate adapter-testing jig hub.
Use the verified motor rack or the dedicated ESP32 only, and select the intended
motor by UID. Preserve the user's existing PCB preferences edit and unrelated
untracked files; they are outside this release work.
