#!/usr/bin/env bash
set -euo pipefail

# arduino_esp32c3_build_upload.sh - Compile examples and upload to ESP32-C3 using arduino-cli

trap 'echo "[ERROR] Script failed at line ${LINENO}"; exit 1' ERR

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"
OUT_ROOT="${OUT_ROOT:-"${REPO_DIR}/out/arduino"}"
FQBN="${FQBN:-${ARDUINO_FQBN:-esp32:esp32:esp32c3}}"
LIB_COPY_SCRIPT="${REPO_DIR}/copy_stuff_to_Arduino.sh"
LIB_DEST="${HOME}/Documents/Arduino/libraries/Servomotor"

# Autodiscover examples: all files matching example_*.cpp at repo root
discover_examples() {
  local f bn name
  for f in "${REPO_DIR}"/example_*.cpp; do
    [[ -f "$f" ]] || continue
    bn="$(basename "$f")"
    name="${bn%.cpp}"
    printf "%s:%s\n" "$name" "$bn"
  done
}

usage() {
  cat <<'EOF'
Usage:
  arduino_esp32c3_build_upload.sh list
  arduino_esp32c3_build_upload.sh compile-all [--fqbn FQBN] [--no-copy]
  arduino_esp32c3_build_upload.sh compile --example NAME [--fqbn FQBN] [--no-copy]
  arduino_esp32c3_build_upload.sh upload  --example NAME [-p|--port PORT] [--fqbn FQBN] [--no-copy]

Defaults:
  FQBN: esp32:esp32:esp32c3  (override via --fqbn or env ARDUINO_FQBN)
  OUT_ROOT: ./out/arduino    (override via env OUT_ROOT)

Examples:
  ./tools/arduino_esp32c3_build_upload.sh list
  ./tools/arduino_esp32c3_build_upload.sh compile-all
  ./tools/arduino_esp32c3_build_upload.sh compile --example One_Move
  ./tools/arduino_esp32c3_build_upload.sh upload  --example One_Move --port /dev/cu.usbmodemXXXX
EOF
}

log() { echo "[arduino-cli] $*"; }

ensure_cli() {
  if ! command -v arduino-cli >/dev/null 2>&1; then
    echo "arduino-cli not found in PATH"
    exit 1
  fi
}

ensure_core() {
  # Ensure index URL is present and the esp32 core is available
  mkdir -p "${HOME}/Library/Arduino15" >/dev/null 2>&1 || true
  arduino-cli config init --overwrite >/dev/null 2>&1 || true
  # Add Espressif index if missing
  if ! arduino-cli config dump | grep -q 'package_esp32_index.json'; then
    arduino-cli config set board_manager.additional_urls "https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json" >/dev/null
  fi
  # Update index with retries
  for i in 1 2 3; do
    if arduino-cli core update-index >/dev/null 2>&1; then break; fi
    sleep 2
  done
  if ! arduino-cli core list | grep -q '^esp32:esp32'; then
    log "Installing esp32:esp32 core (this may take a while)..."
    arduino-cli core install esp32:esp32
  fi
}

sync_library() {
  if [[ -x "${LIB_COPY_SCRIPT}" ]]; then
    log "Syncing library to Arduino libraries folder..."
    bash "${LIB_COPY_SCRIPT}" >/dev/null
  else
    echo "Copy script not found: ${LIB_COPY_SCRIPT}"
    exit 1
  fi
}

list_examples() {
  echo "Available examples:"
  local pair name src
  while IFS= read -r pair; do
    IFS=':' read -r name src <<<"$pair"
    printf "  - %s (src: %s)\n" "$name" "$src"
  done < <(discover_examples)
}

sketch_dir_for() {
  local name="$1"
  echo "${LIB_DEST}/examples/${name}"
}

compile_one() {
  local name="$1"
  local sketch_dir
  sketch_dir="$(sketch_dir_for "$name")"
  if [[ ! -d "$sketch_dir" ]]; then
    echo "Sketch directory not found: $sketch_dir"
    exit 1
  fi
  local out_dir="${OUT_ROOT}/${FQBN}/${name}"
  mkdir -p "$out_dir"
  log "Compiling ${name} -> ${out_dir}"
  arduino-cli compile --fqbn "${FQBN}" --build-path "$out_dir" "$sketch_dir"
}

compile_all() {
  local pair name _src
  while IFS= read -r pair; do
    IFS=':' read -r name _src <<<"$pair"
    compile_one "$name"
  done < <(discover_examples)
}

detect_port() {
  # Prefer OS-specific USB serial device patterns to avoid Bluetooth ports
  local port os
  os="$(uname -s 2>/dev/null || echo Unknown)"

  if [[ "$os" == "Darwin" ]]; then
    # Common macOS USB serial device names
    for pattern in /dev/cu.usbmodem* /dev/cu.usbserial* /dev/tty.usbmodem* /dev/tty.usbserial*; do
      port="$(ls $pattern 2>/dev/null | head -n1)" || true
      [[ -n "${port:-}" ]] && break
    done
  elif [[ "$os" == "Linux" ]]; then
    # Common Linux USB serial device names
    for pattern in /dev/ttyUSB* /dev/ttyACM*; do
      port="$(ls $pattern 2>/dev/null | head -n1)" || true
      [[ -n "${port:-}" ]] && break
    done
  fi

  # Fallback to arduino-cli, but exclude Bluetooth and non-USB pseudo-ports
  if [[ -z "${port:-}" ]]; then
    port="$(arduino-cli board list | awk 'NR>1 && ($1 ~ /^\/dev\/(cu|tty)\.(usb|usbserial|usbmodem|ttyUSB|ttyACM)/) {print $1; exit}')" || true
  fi

  echo "${port:-}"
}

upload_one() {
  local name="$1"
  local port="${2:-}"
  local sketch_dir
  sketch_dir="$(sketch_dir_for "$name")"
  local out_dir="${OUT_ROOT}/${FQBN}/${name}"
  if [[ ! -d "$out_dir" ]]; then
    log "Build artifacts for ${name} not found, compiling first..."
    compile_one "$name"
  fi
  if [[ -z "$port" ]]; then
    port="$(detect_port)"
  fi
  if [[ -z "$port" ]]; then
    echo "Serial port not found. Specify with --port."
    exit 1
  fi
  log "Uploading ${name} (FQBN=${FQBN}) to ${port}"
  arduino-cli upload --fqbn "${FQBN}" -p "${port}" --input-dir "${out_dir}" "${sketch_dir}"
}

main() {
  local sub="${1:-help}"; shift || true
  local no_copy="0"
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --fqbn) FQBN="$2"; shift 2 ;;
      -e|--example) EXAMPLE_NAME="$2"; shift 2 ;;
      -p|--port) PORT="$2"; shift 2 ;;
      --no-copy) no_copy="1"; shift ;;
      -h|--help) usage; exit 0 ;;
      *) break ;;
    esac
  done

  case "$sub" in
    list)
      list_examples
      ;;
    compile-all)
      ensure_cli
      ensure_core
      if [[ "$no_copy" != "1" ]]; then
        sync_library
      fi
      compile_all
      ;;
    compile)
      : "${EXAMPLE_NAME:?--example NAME is required}"
      ensure_cli
      ensure_core
      if [[ "$no_copy" != "1" ]]; then
        sync_library
      fi
      compile_one "$EXAMPLE_NAME"
      ;;
    upload)
      : "${EXAMPLE_NAME:?--example NAME is required}"
      ensure_cli
      ensure_core
      if [[ "$no_copy" != "1" ]]; then
        sync_library
      fi
      upload_one "$EXAMPLE_NAME" "${PORT:-}"
      ;;
    help|*)
      usage
      ;;
  esac
}

main "$@"