#! /usr/bin/env bash
set -euo pipefail

# Resolve repo root to be robust when invoked from anywhere
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Configuration
VENV_DIR="${ROOT_DIR}/.publish-venv"

# Single source of truth for the version: servomotor/__init__.py __version__
# (pyproject reads it via [tool.setuptools.dynamic] version = {attr = "servomotor.__version__"}).
VERSION=$(grep -o "__version__ = ['\"][^'\"]*['\"]" "${ROOT_DIR}/servomotor/__init__.py" | sed "s/__version__ = ['\"]//g" | sed "s/['\"]//g")
if [ -z "${VERSION}" ]; then
  echo "ERROR: could not read __version__ from servomotor/__init__.py" >&2
  exit 1
fi
echo "servomotor version to publish: ${VERSION}"

# --- Choose the target index up front so we can fail fast before building ----------------
echo ""
echo "Select deployment target:"
echo "1) TestPyPI (test.pypi.org)"
echo "2) PyPI (pypi.org) - PRODUCTION"
echo ""
read -p "Enter your choice (1 or 2): " choice

case $choice in
    1)
        REPOSITORY="testpypi"
        INDEX_HOST="test.pypi.org"
        ;;
    2)
        REPOSITORY="pypi"
        INDEX_HOST="pypi.org"
        ;;
    *)
        echo "Invalid choice. Deployment cancelled."
        exit 1
        ;;
esac

# --- Pre-flight: refuse to (re)publish a version that already exists on the chosen index --
# PyPI permanently rejects re-uploading an existing version, so catch the "forgot to bump
# __version__" mistake here instead of after a full build + a confusing twine error.
HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" "https://${INDEX_HOST}/pypi/servomotor/${VERSION}/json" || echo "000")
if [ "${HTTP_CODE}" = "200" ]; then
    echo "ERROR: servomotor ${VERSION} is already published on ${INDEX_HOST}." >&2
    echo "       Bump __version__ in servomotor/__init__.py before publishing." >&2
    exit 1
fi

# Production gets an explicit confirmation gate.
if [ "${REPOSITORY}" = "pypi" ]; then
    echo ""
    echo "⚠️  WARNING: You are about to deploy to PRODUCTION PyPI!"
    echo "This will make version ${VERSION} publicly available."
    echo ""
    read -p "Are you sure you want to proceed? (yes/no): " confirm
    if [[ $confirm != "yes" ]]; then
        echo "Deployment cancelled."
        exit 0
    fi
fi
echo "Deploying ${VERSION} to ${REPOSITORY}..."

# --- Ensure src layout exists and is fresh -----------------------------------------------
mkdir -p "${ROOT_DIR}/PyPi_distribution/src"
rm -rf "${ROOT_DIR}/PyPi_distribution/src/servomotor"
cp -R "${ROOT_DIR}/servomotor" "${ROOT_DIR}/PyPi_distribution/src/"

# Strip editor/backup/compiled cruft so it can never leak into a release artifact,
# regardless of what is lying around in the working tree (e.g. M3.py.bak, *.old,
# motor_commands.json.backup, __pycache__). The real sources + *.json data stay.
find "${ROOT_DIR}/PyPi_distribution/src/servomotor" \
  \( -name '*.bak' -o -name '*.bak2' -o -name '*.old' -o -name '*.working' \
     -o -name '*.backup' -o -name '*.pyc' -o -name '__pycache__' \) \
  -exec rm -rf {} + 2>/dev/null || true

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

# --- Validate the built artifacts BEFORE uploading ---------------------------------------
# 1) twine's own metadata/readme checks.
"${PY}" -m twine check dist/*

# 2) Belt-and-suspenders: fail if any backup/compiled cruft slipped into the sdist or wheel.
LEAKED=""
for art in dist/*.tar.gz; do
    LEAKED+=$(tar -tzf "${art}" | grep -E '\.(bak|bak2|old|working|backup|pyc)$' || true)
done
for art in dist/*.whl; do
    LEAKED+=$("${PY}" -c "import sys,zipfile;print('\n'.join(zipfile.ZipFile(sys.argv[1]).namelist()))" "${art}" \
              | grep -E '\.(bak|bak2|old|working|backup|pyc)$' || true)
done
if [ -n "${LEAKED}" ]; then
    echo "ERROR: forbidden backup/compiled files leaked into the build artifact:" >&2
    echo "${LEAKED}" >&2
    popd > /dev/null
    exit 1
fi
echo "Artifact validation passed (twine check OK; no backup/compiled cruft in sdist/wheel)."

# Upload to selected repository
"${PY}" -m twine upload --repository "${REPOSITORY}" dist/*
popd > /dev/null

echo "Build and upload of servomotor ${VERSION} to ${REPOSITORY} completed successfully!"
