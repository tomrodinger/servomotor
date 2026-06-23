#! /usr/bin/env bash
set -euo pipefail

# Stage the src/ layout for building the `servomotor` PyPI distribution.
#
# Single source of truth for the build staging, shared by BOTH:
#   - python_programs/build_and_submit_to_PyPi.sh  (manual / TestPyPI deploys)
#   - .github/workflows/publish-python.yml          (tag-driven trusted publishing)
# so the two can never drift. It is idempotent: safe to re-run.
#
# It copies the live `servomotor` package + the standalone/CLI modules into
# PyPi_distribution/src/, stripping editor/backup/compiled cruft so none of it
# can leak into a release artifact. No network, no build, no upload.

# Resolve paths relative to THIS script so it works from any cwd (local or CI).
DIST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # .../python_programs/PyPi_distribution
PP_DIR="$(cd "${DIST_DIR}/.." && pwd)"                      # .../python_programs

echo "Staging servomotor sources into ${DIST_DIR}/src ..."

# --- Ensure src layout exists and is fresh -----------------------------------------------
mkdir -p "${DIST_DIR}/src"
rm -rf "${DIST_DIR}/src/servomotor"
cp -R "${PP_DIR}/servomotor" "${DIST_DIR}/src/"

# Strip editor/backup/compiled cruft so it can never leak into a release artifact,
# regardless of what is lying around in the working tree (e.g. M3.py.bak, *.old,
# motor_commands.json.backup, __pycache__). The real sources + *.json data stay.
find "${DIST_DIR}/src/servomotor" \
  \( -name '*.bak' -o -name '*.bak2' -o -name '*.old' -o -name '*.working' \
     -o -name '*.backup' -o -name '*.pyc' -o -name '__pycache__' \) \
  -exec rm -rf {} + 2>/dev/null || true

# Include standalone module and CLI modules into the distribution tree
# - terminal_formatting.py as a top-level module (declared in pyproject's py-modules)
cp -f "${PP_DIR}/terminal_formatting.py" "${DIST_DIR}/src/"

# - CLI modules (declared in [project.scripts] and py-modules)
cp -f "${PP_DIR}/servomotor_command.py" "${DIST_DIR}/src/"
cp -f "${PP_DIR}/detect_and_set_alias_all_devices.py" "${DIST_DIR}/src/"
cp -f "${PP_DIR}/show_device_information_for_all_devices.py" "${DIST_DIR}/src/"

echo "Staging complete."
