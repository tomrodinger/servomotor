# Handoff: (3) full hardware test run on 0.10.0  +  (4) tag-driven PyPI CI for the library

> **STATUS: ✅ ALL TASKS COMPLETE (2026-06-23)**
>
> - **Task 3 — hardware suite on published 0.10.0:** `run_all_tests.py` against the bench M17
>   (`/dev/cu.usbserial-210`, alias `X`=88) importing PyPI `servomotor==0.10.0` (verified
>   `__file__` → site-packages). **63/63 PASSED**, 0 failed/skipped, 813.9 s — incl. the firmware
>   test's 3 reflashes (`0.15.2.0→0.14.0.0→0.15.2.0`) and both alias tests (alias self-restored to 88).
>   Results: `/tmp/hwtest_root/RESULTS.txt`. Device left healthy on fw 0.15.2.0.
> - **Task 4 — tag-driven publishing:** added `.github/workflows/publish-python.yml` (trigger
>   `pylib-v*` + `workflow_dispatch`, OIDC trusted publishing) and `PyPi_distribution/prepare_src.sh`
>   (shared staging, called by both the workflow and `build_and_submit_to_PyPi.sh`). Build path
>   validated locally (build + `twine check` PASS, no cruft). **TestPyPI dry-run run #28002861795
>   succeeded** — OIDC auth worked (attestations generated, reached the upload endpoint;
>   `skip-existing` handled the pre-existing 0.10.0). PyPI publisher (env `pypi`) + TestPyPI publisher
>   (env `testpypi`) configured by Tom.
> - **Task 5 — merge:** PR #3 `production-test-system → main` merged (merge commit); `main` has the
>   hardened deploy script, the workflow, and `prepare_src.sh`. A follow-up commit added
>   `skip-existing` to the TestPyPI step.
> - **Release flow going forward:** bump `python_programs/servomotor/__init__.py __version__` →
>   commit → `git tag pylib-vX.Y.Z && git push --tags`. (Also documented in `CLAUDE.md`.)
>
> The detail below is the original handoff, kept for reference.

---

# Handoff: (3) full hardware test run on 0.10.0  +  (4) tag-driven PyPI CI for the library

Two independent tasks for an executing AI. Context: `servomotor` **0.10.0** was just published to
PyPI (cross-platform refactor; verified against the bench M17). The deploy script
`build_and_submit_to_PyPi.sh` was hardened (cruft strip + version guard + artifact validation) and
committed on branch `production-test-system` (not pushed). Mirror that work where noted.

Bench hardware (all tasks): one **M17 on `/dev/cu.usbserial-210`**, alias **88**,
unique_id `0x99856389A2B46555`. Three other USB-RS485 adapters are connected but **EMPTY**.
Shaft is **free, nothing attached** → motion is safe.

---

## Task 3 — Run the full hardware test suite using the DEPLOYED 0.10.0 library

Goal: run the library's hardware tests against the bench motor, importing the **PyPI-published
`servomotor==0.10.0`**, not the local working-tree source. Report pass/fail per test.

**THE KEY GOTCHA — make sure you test the published library, not the local source.**
The local package dir `python_programs/servomotor/` sits right next to the `test_*.py` files, so
running `python test_X.py` from `python_programs/` imports the LOCAL copy (cwd/script dir is on
`sys.path`) and **shadows** the installed one. To force the installed version:
1. `python3 -m venv /tmp/hwtest && source /tmp/hwtest/bin/activate`
2. `pip install servomotor==0.10.0`
3. Run the tests from a directory that is NOT `python_programs/` and does NOT contain a `servomotor/`
   dir (e.g. copy the needed `test_*.py` + any helper modules they import into `/tmp/hwtest_run/`).
4. **Verify** before running: `python -c "import servomotor; print(servomotor.__file__)"` must point
   into `…/site-packages/servomotor/`, NOT into `python_programs/`.

**Before running anything**, read `run_all_tests.py` to learn how it selects tests and what args it
needs (expect a port `-p /dev/cu.usbserial-210` and alias `-a 88`).

**DO NOT run these without explicit intent — they change motor state or firmware:**
- `test_firmware_upgrade.py` — **reflashes firmware.** Skip unless deliberately flashing.
- `test_set_device_alias.py` / `test_correct_and_incorrect_addressing.py` — **change the alias (88).**
  If you run them, set the alias back to **88** afterward.
- Anything that writes non-volatile settings (e.g. set max current) — note it reverts on reset in
  0.10.0, but still review first.

Deliverable: a results table (test → pass/fail/skipped), plus any failures with output. There is a
`hardware_tests/RESULTS.txt` convention in the `servomotor-mcp` repo you can follow for format.

---

## Task 4 — Tag-driven GitHub Actions trusted publishing for the library (mirror `servomotor-mcp`)

Goal: publish `servomotor` to PyPI from a git tag via OIDC (no token), like
`github.com/Gearotons/servomotor-mcp` already does — replacing manual `build_and_submit_to_PyPi.sh`
runs for releases (keep the script as a manual/TestPyPI fallback).

**Repo facts:**
- `github.com/tomrodinger/servomotor` is a **MONOREPO** (firmware, bootloader, mechanical, micropython,
  `python_programs/`…). The Python package builds from **`python_programs/PyPi_distribution/`**.
- Version source of truth: **`python_programs/servomotor/__init__.py` `__version__`** (now `0.10.0`),
  read by `PyPi_distribution/pyproject.toml` via `[tool.setuptools.dynamic]`.

**Build a workflow `.github/workflows/publish-python.yml` (repo root) that:**
1. Triggers on a **python-specific tag**, e.g. `pylib-v*` (do NOT use plain `v*` — this repo also
   versions firmware/bootloaders; a generic tag would collide). Add `workflow_dispatch` too.
2. Replicates the prep in `build_and_submit_to_PyPi.sh`: copy `python_programs/servomotor` →
   `python_programs/PyPi_distribution/src/`, **strip** `*.bak *.bak2 *.old *.working *.backup *.pyc
   __pycache__`, and copy the 4 standalone modules (`terminal_formatting.py`, `servomotor_command.py`,
   `detect_and_set_alias_all_devices.py`, `show_device_information_for_all_devices.py`) into
   `PyPi_distribution/src/`. (Best: factor this prep into a small shared script the local deploy
   script and the workflow both call, so they can't drift.)
3. `python -m build` from `python_programs/PyPi_distribution`, then `twine check dist/*`.
4. Publish with `pypa/gh-action-pypi-publish@release/v1`, `environment: pypi`,
   `permissions: id-token: write`. (See the `servomotor-mcp` repo's `.github/workflows/publish.yml`
   as the template.)

**ONE-TIME on pypi.org (Tom must do this — needs PyPI login):** the `servomotor` project already
exists, so under **the `servomotor` project → Settings → Publishing → add a publisher** (not a
"pending" publisher) with: Owner `tomrodinger`, Repository `servomotor`, Workflow
`publish-python.yml`, Environment `pypi`.

**Validate on TestPyPI first** (PyPI permanently rejects re-used versions): use `workflow_dispatch`
pointed at TestPyPI, or bump `__version__` and push a `pylib-vX.Y.Z` tag once the publisher is set up.

**Release flow once live:** edit `__version__` → commit → `git tag pylib-v0.11.0 && git push --tags`.

Guardrail: do not change the package's import name, the `servomotor` PyPI project owner, or break the
existing `build_and_submit_to_PyPi.sh` (it's the manual fallback and the prep reference).

---

## Task 5 — Merge `production-test-system` into `main`

Do this **before Task 4** so the CI workflow (and the hardened `build_and_submit_to_PyPi.sh`) live on
`main`. The hardened deploy script was pushed to `origin/production-test-system` (commit
`bc8781f …Harden PyPI deploy script…`) and needs to reach `main`, along with the rest of that branch's
work.

- Repo: `github.com/tomrodinger/servomotor` (monorepo). `main` flows from `production-test-system` via
  PRs (see the existing "Merge pull request #2 from tomrodinger/production-test-system" on `main`).
- Preferred: open a PR `production-test-system → main` on GitHub, review, and merge — keeps history/CI
  consistent with how this repo already merges.
- Confirm afterward that `main` contains the hardened `python_programs/build_and_submit_to_PyPi.sh`.
- Heads-up: the local working tree has regenerable build-staging edits under
  `python_programs/PyPi_distribution/src/` (byproducts of the 0.10.0 build) — discard them
  (`git checkout -- python_programs/PyPi_distribution/src`) rather than committing them into the merge.
