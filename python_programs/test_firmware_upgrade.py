#!/usr/bin/env python3
"""
Per-command test for "Firmware upgrade" (cmd 23).

Uses ONLY the new firmware/bootloader protocol (proper size encoding with
LSB=1 + CRC32 on every packet). The legacy protocol is never exercised.

Sequence (per decision #7 in TEST_MODERNIZATION_PLAN.md):
  1. Install LATEST firmware. Read Get firmware version (cmd 25); assert
     the four version bytes match the firmware filename and inBootloader=0.
  2. Install a KEPT OLDER firmware (a known-good earlier release retained
     in the repo). Read version, assert match. This proves the device can
     be downgraded after the upgrade in step 1.
  3. Install LATEST firmware again. Read version, assert match. Device
     ends on the latest firmware — that is the desired end state and the
     reason this test runs first in the suite (see
     TEST_MODERNIZATION_PLAN.md "Test ordering & dependencies").

The page-by-page bootloader handshake is implemented inline here using
servomotor.execute_command (no subprocess delegation). Each firmware page
is sent alias-addressed (NOT broadcast) so the bootloader ACKs every page
and any per-page failure surfaces immediately as a TimeoutError or
FatalError instead of silently dropping bytes.

Firmware files come from ../firmware/firmware_releases/ filtered by the
device's actual model / firmware-compatibility / hardware version (read
live via Get product info). "Latest" is the highest-version match.

The "kept older firmware" filename is hardcoded to a specific release
that MUST remain in the repo (see KEPT_OLDER_FIRMWARE_NAME below). If
you change it, keep the file in firmware_releases/.

Caveat: three full reflashes plus version checks add a couple of minutes
to a suite run and write the device flash three times. Approved in the
2026-05-19 review.
"""

import argparse
import binascii
import re
import sys
import time
from pathlib import Path

import servomotor
from servomotor import M3


KEPT_OLDER_FIRMWARE_NAME = "servomotor_M17_fw0.14.0.0_scc3_hw1.5.firmware"

FIRMWARE_UPGRADE_COMMAND = 23
SYSTEM_RESET_COMMAND     = 27
GET_FIRMWARE_VERSION_COMMAND = 25

FLASH_PAGE_SIZE                 = 2048
BOOTLOADER_N_PAGES              = 5
FIRST_FIRMWARE_PAGE_NUMBER      = BOOTLOADER_N_PAGES
LAST_FIRMWARE_PAGE_NUMBER       = 30
MODEL_CODE_LENGTH               = 8
FIRMWARE_COMPATIBILITY_CODE_LENGTH = 1
FIRMWARE_PAGE_NUMBER_LENGTH     = 1

# Empirically determined in upgrade_firmware.py: the bootloader's
# "waiting-for-firmware" window opens immediately after the reset and
# lasts a few hundred ms. The first cmd 23 must land inside that window.
WAIT_FOR_RESET_INTO_BOOTLOADER_S = 0.07
# Time to let the application firmware boot after the bootloader hands off.
POST_FLASH_BOOT_DELAY_S          = 3.0
# Reset delay used when reopening the port for the version check.
POST_RESET_DELAY_S               = 1.5

FIRMWARE_RELEASES_DIR = Path(__file__).resolve().parent.parent / "firmware" / "firmware_releases"

# servomotor_<MODEL>_fw<MAJOR>.<MINOR>.<BUGFIX>.<DEV>_scc<SCC>_hw<HW>.firmware
FIRMWARE_FILENAME_PATTERN = re.compile(
    r"^servomotor_(?P<model>[A-Za-z0-9]+)_fw"
    r"(?P<major>\d+)\.(?P<minor>\d+)\.(?P<bugfix>\d+)\.(?P<dev>\d+)"
    r"_scc(?P<scc>\d+)_hw(?P<hw>[A-Za-z0-9]+(?:\.[A-Za-z0-9]+)*)\.firmware$"
)


def parse_firmware_filename(name):
    m = FIRMWARE_FILENAME_PATTERN.match(name)
    if not m:
        return None
    return {
        "name": name,
        "model": m.group("model"),
        "major": int(m.group("major")),
        "minor": int(m.group("minor")),
        "bugfix": int(m.group("bugfix")),
        "dev": int(m.group("dev")),
        "scc": int(m.group("scc")),
        "hw": m.group("hw"),
        "version": (
            int(m.group("major")),
            int(m.group("minor")),
            int(m.group("bugfix")),
            int(m.group("dev")),
        ),
    }


def pretty_version(v):
    return f"{v[0]}.{v[1]}.{v[2]}.{v[3]}"


def list_matching_firmwares(model, scc, hw):
    if not FIRMWARE_RELEASES_DIR.is_dir():
        raise FileNotFoundError(f"firmware releases directory not found: {FIRMWARE_RELEASES_DIR}")
    matches = []
    for entry in FIRMWARE_RELEASES_DIR.iterdir():
        info = parse_firmware_filename(entry.name)
        if info is None:
            continue
        if info["model"] == model and info["scc"] == scc and info["hw"] == hw:
            info["path"] = entry
            matches.append(info)
    return matches


def resolve_firmwares(model, scc, hw):
    """Return (latest_info, older_info) — older_info is the named kept-older file."""
    matches = list_matching_firmwares(model, scc, hw)
    if not matches:
        raise FileNotFoundError(
            f"No firmware files in {FIRMWARE_RELEASES_DIR} match model={model!r} scc={scc} hw={hw!r}"
        )
    latest = max(matches, key=lambda f: f["version"])
    older = next((f for f in matches if f["name"] == KEPT_OLDER_FIRMWARE_NAME), None)
    if older is None:
        raise FileNotFoundError(
            f"Kept older firmware {KEPT_OLDER_FIRMWARE_NAME!r} is missing from "
            f"{FIRMWARE_RELEASES_DIR}. This file must remain in the repo for this test."
        )
    if older["version"] == latest["version"]:
        raise AssertionError(
            f"The kept-older firmware {older['name']!r} has the same version as the "
            f"latest match {latest['name']!r}. Pick a different KEPT_OLDER_FIRMWARE_NAME."
        )
    return latest, older


def read_and_prepare_firmware(filename):
    """Mirror of upgrade_firmware.py:read_binary() + the size/CRC32 fix-up.

    Returns (model_code: bytes, scc: int, pages: list[bytes 2048-each]).
    """
    with open(filename, "rb") as fh:
        raw = fh.read()
    if len(raw) < MODEL_CODE_LENGTH + FIRMWARE_COMPATIBILITY_CODE_LENGTH + (FLASH_PAGE_SIZE - 4):
        raise ValueError(f"firmware file {filename} is too small: {len(raw)} bytes")
    model_code = raw[:MODEL_CODE_LENGTH]
    scc = int.from_bytes(raw[MODEL_CODE_LENGTH:MODEL_CODE_LENGTH + FIRMWARE_COMPATIBILITY_CODE_LENGTH],
                         byteorder="little")
    body = bytearray(raw[MODEL_CODE_LENGTH + FIRMWARE_COMPATIBILITY_CODE_LENGTH:])

    # Pad to a multiple of 4.
    while len(body) & 0x03:
        body.append(0)

    # The first 32-bit slot originally held the application stack pointer; the
    # startup script moved it to position 9, so the bootloader reuses slot 0
    # for "firmware size in 32-bit words minus 1" and appends the firmware
    # CRC32 (computed over body[4:]) at the very end.
    firmware_size = (len(body) >> 2) - 1
    firmware_crc = binascii.crc32(bytes(body[4:]))
    body = firmware_size.to_bytes(4, "little") + bytes(body[4:]) + firmware_crc.to_bytes(4, "little")

    # Pad the final page out to FLASH_PAGE_SIZE with zeros.
    if len(body) % FLASH_PAGE_SIZE != 0:
        body += b"\x00" * (FLASH_PAGE_SIZE - (len(body) % FLASH_PAGE_SIZE))
    n_pages = len(body) // FLASH_PAGE_SIZE
    if n_pages == 0:
        raise ValueError("firmware payload is empty after preparation")
    if FIRST_FIRMWARE_PAGE_NUMBER + n_pages - 1 > LAST_FIRMWARE_PAGE_NUMBER:
        raise ValueError(
            f"firmware needs {n_pages} pages, would exceed the available flash "
            f"(pages {FIRST_FIRMWARE_PAGE_NUMBER}..{LAST_FIRMWARE_PAGE_NUMBER})"
        )
    pages = [body[i:i + FLASH_PAGE_SIZE] for i in range(0, len(body), FLASH_PAGE_SIZE)]
    return model_code, scc, pages


def assert_models_match(file_model_code, file_scc, file_hw_in_name,
                        device_model, device_scc, device_hw):
    """The firmware file's model/scc must match the device or the bootloader rejects."""
    file_model_str = file_model_code.rstrip(b"\x00 ").decode("ascii", errors="replace")
    if file_model_str != device_model:
        raise AssertionError(
            f"firmware model code {file_model_str!r} != device model {device_model!r}"
        )
    if file_scc != device_scc:
        raise AssertionError(
            f"firmware scc {file_scc} != device scc {device_scc}"
        )
    if file_hw_in_name != device_hw:
        raise AssertionError(
            f"firmware hw {file_hw_in_name!r} (from filename) != device hw {device_hw!r}"
        )


def install_firmware(alias, info, verbose):
    """Install one firmware file via the new protocol, alias-addressed (each page ACKed).

    Caller must have the serial port open. Sequence:
      1. system_reset → device drops into bootloader.
      2. Sleep WAIT_FOR_RESET_INTO_BOOTLOADER_S so the first cmd 23 lands inside
         the bootloader's "waiting for firmware" window.
      3. For each page: cmd 23 (firmware_upgrade) with [model+scc+page#+data]
         payload. execute_command with crc32_enabled=True. The bootloader
         replies with a success packet per page — execute_command will raise
         on timeout or fatal error.
      4. system_reset → bootloader jumps to the new application firmware.
    """
    model_code, scc, pages = read_and_prepare_firmware(info["path"])
    print(f"\n  loaded {info['name']}  ({len(pages)} pages, {len(pages) * FLASH_PAGE_SIZE} bytes)")
    print(f"  model_code={model_code!r}  scc={scc}  expected version={pretty_version(info['version'])}")

    print("  → resetting device into the bootloader...")
    servomotor.execute_command(SYSTEM_RESET_COMMAND, bytearray(),
                               alias_or_unique_id=alias, crc32_enabled=True, verbose=verbose)
    time.sleep(WAIT_FOR_RESET_INTO_BOOTLOADER_S)

    started = time.time()
    for i, page_data in enumerate(pages):
        page_number = FIRST_FIRMWARE_PAGE_NUMBER + i
        payload = bytearray()
        payload += model_code
        payload += int(scc).to_bytes(FIRMWARE_COMPATIBILITY_CODE_LENGTH, "little")
        payload += int(page_number).to_bytes(FIRMWARE_PAGE_NUMBER_LENGTH, "little")
        payload += page_data
        # alias-addressed: bootloader sends a success packet; execute_command
        # raises TimeoutError or FatalError on failure.
        servomotor.execute_command(FIRMWARE_UPGRADE_COMMAND, [payload],
                                   alias_or_unique_id=alias, crc32_enabled=True, verbose=verbose)
        if verbose < 2:
            print(f"    page {page_number}/{FIRST_FIRMWARE_PAGE_NUMBER + len(pages) - 1} ok", end="\r")
    if verbose < 2:
        print()
    elapsed = time.time() - started
    print(f"  → {len(pages)} pages written in {elapsed:.1f}s; resetting to start new firmware...")
    servomotor.execute_command(SYSTEM_RESET_COMMAND, bytearray(),
                               alias_or_unique_id=alias, crc32_enabled=True, verbose=verbose)


def get_device_info(alias, verbose):
    """Read Get product info and parse model / scc / hw / unique_id."""
    motor = M3(alias, verbose=verbose)
    pi = motor.get_product_info(verbose=verbose)
    # pi = [productCode, firmwareCompatibility, hardwareVersion, serialNumber, uniqueId, reserved]
    product_code = pi[0].strip().strip("\x00").strip()
    firmware_compat = int(pi[1])
    hw_list = pi[2]  # [patch, minor, major]
    unique_id = int(pi[4])
    # Match firmware-filename hw form: "major.minor" (with ".patch" only if non-zero).
    hw_string = f"{hw_list[2]}.{hw_list[1]}"
    if hw_list[0] != 0:
        hw_string += f".{hw_list[0]}"
    return product_code, firmware_compat, hw_string, unique_id


def read_firmware_version(alias, verbose):
    motor = M3(alias, verbose=verbose)
    fv = motor.get_firmware_version(verbose=verbose)
    version_value, in_bootloader = fv[0], int(fv[1])
    if isinstance(version_value, list):
        dev, patch, minor, major = version_value
    else:
        dev = version_value & 0xFF
        patch = (version_value >> 8) & 0xFF
        minor = (version_value >> 16) & 0xFF
        major = (version_value >> 24) & 0xFF
    return (major, minor, patch, dev), in_bootloader


def install_and_verify(alias, info, device_model, device_scc, device_hw, label, verbose):
    print(f"\n========== {label}: install {info['name']} ==========")
    assert_models_match(read_and_prepare_firmware(info['path'])[0], info['scc'], info['hw'],
                        device_model, device_scc, device_hw)

    install_firmware(alias, info, verbose)

    print(f"  → waiting {POST_FLASH_BOOT_DELAY_S}s for application firmware to boot...")
    time.sleep(POST_FLASH_BOOT_DELAY_S)

    # Quiet, single get_firmware_version. (No system_reset before reading — a
    # reset right after install would drop us back into the bootloader.)
    got, in_bootloader = read_firmware_version(alias, verbose)
    print(f"  → Get firmware version: {pretty_version(got)} (inBootloader={in_bootloader})")
    if in_bootloader != 0:
        raise AssertionError(f"[{label}] device is still in the bootloader after install "
                             f"(inBootloader={in_bootloader})")
    if got != info['version']:
        raise AssertionError(f"[{label}] firmware version mismatch: device reports "
                             f"{pretty_version(got)}, expected {pretty_version(info['version'])}")


def main():
    parser = argparse.ArgumentParser(description="Per-command test for 'Firmware upgrade' (cmd 23).")
    parser.add_argument('-p', '--port', help='Serial port device')
    parser.add_argument('-P', '--PORT', action='store_true', help='Show available ports and prompt for selection')
    parser.add_argument('-a', '--alias', default='X', help='Alias of the device to upgrade (default: X)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose output')
    args = parser.parse_args()

    if not args.port and not args.PORT:
        parser.error("either -p/--port or -P/--PORT must be specified")

    verbose_level = 2 if args.verbose else 0
    servomotor.set_serial_port_from_args(args)
    # execute_command takes an int (alias byte or unique-ID u64); the M3 wrapper
    # accepts the string form but execute_command does not, so resolve once here.
    alias_int = servomotor.string_to_alias_or_unique_id(args.alias)

    success = False
    failure_message = ""
    try:
        servomotor.open_serial_port()
        try:
            print(f"Identifying device at alias {args.alias!r} (resolved to {alias_int}) on {args.port}...")
            model, scc, hw, unique_id = get_device_info(alias_int, verbose_level)
            print(f"  model={model!r}  scc={scc}  hw={hw!r}  unique_id=0x{unique_id:016X}")

            starting_version, _ = read_firmware_version(alias_int, verbose_level)
            print(f"  starting firmware version: {pretty_version(starting_version)}")

            latest, older = resolve_firmwares(model, scc, hw)
            print(f"\nLatest firmware:      {latest['name']}  ({pretty_version(latest['version'])})")
            print(f"Kept older firmware:  {older['name']}  ({pretty_version(older['version'])})")

            install_and_verify(alias_int, latest, model, scc, hw, "Phase 1 (latest)",  verbose_level)
            install_and_verify(alias_int, older,  model, scc, hw, "Phase 2 (older)",   verbose_level)
            install_and_verify(alias_int, latest, model, scc, hw, "Phase 3 (latest)",  verbose_level)

            success = True
        finally:
            servomotor.close_serial_port()

    except Exception as e:
        failure_message = str(e) if str(e) else type(e).__name__

    if success:
        print("\nPASSED")
        return 0
    print(f"\nFAILED: {failure_message}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
