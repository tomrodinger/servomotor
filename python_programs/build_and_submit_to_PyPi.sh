#! /usr/bin/env bash
set -euo pipefail

# Resolve repo root to be robust when invoked from anywhere
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Configuration
VENV_DIR="${ROOT_DIR}/.publish-venv"

# Ensure src layout exists and is fresh
mkdir -p "${ROOT_DIR}/PyPi_distribution/src"
rm -rf "${ROOT_DIR}/PyPi_distribution/src/servomotor"
cp -R "${ROOT_DIR}/servomotor" "${ROOT_DIR}/PyPi_distribution/src/"

# Include standalone module and CLI modules into the distribution tree
# - terminal_formatting.py as a top-level module (declared in pyproject's py-modules)
cp -f "${ROOT_DIR}/terminal_formatting.py" "${ROOT_DIR}/PyPi_distribution/src/"

# - CLI modules (declared in [project.scripts] and py-modules)
cp -f "${ROOT_DIR}/servomotor_command.py" "${ROOT_DIR}/PyPi_distribution/src/"
cp -f "${ROOT_DIR}/detect_and_set_alias_all_devices.py" "${ROOT_DIR}/PyPi_distribution/src/"
cp -f "${ROOT_DIR}/show_device_information_for_all_devices.py" "${ROOT_DIR}/PyPi_distribution/src/"

# Create an isolated virtual environment for build tooling (avoids PEP 668 issues)
if [ ! -d "${VENV_DIR}" ]; then
  python3 -m venv "${VENV_DIR}"
fi

# Use venv's python explicitly
PY="${VENV_DIR}/bin/python"

# Upgrade pip and install build + twine inside the venv
"${PY}" -m pip install --upgrade pip
"${PY}" -m pip install --upgrade build twine

# Build the distribution from the PyPi_distribution directory
pushd "${ROOT_DIR}/PyPi_distribution" > /dev/null
rm -rf dist
"${PY}" -m build

# Upload to TestPyPI (requires either ~/.pypirc with 'testpypi' or TWINE_USERNAME/TWINE_PASSWORD set)
"${PY}" -m twine upload --repository testpypi dist/*
popd > /dev/null

echo "Build and upload completed."
