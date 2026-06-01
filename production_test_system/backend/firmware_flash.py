"""Broadcast firmware flashing for Phase 1.

This is a faithful port of the *new-protocol* broadcast flash in
``python_programs/upgrade_firmware.py`` (the bench-validated upgrader), adapted
to run over a per-bus :class:`~backend.transport.Transport` so all motors on a
bus are flashed in one shot.  The page encoding is the library's cmd 23
("Firmware upgrade", a ``firmware_page`` payload), so no encoding is duplicated.

The constants and inter-page/post-reset delays below are copied verbatim from
``upgrade_firmware.py`` (empirically determined there).  Because flashing is
inherently hardware-timing-sensitive, **this path has not been validated on the
bench from inside the test system** — verify it against a single unit during
hardware bring-up before relying on it for a full rack.  Until then, the proven
standalone ``upgrade_firmware.py`` remains the fallback.
"""

from __future__ import annotations

import glob
import os
import time
import zlib
from typing import Callable, Optional

from . import config
from .transport import Transport

# --- constants copied from upgrade_firmware.py (do not change without re-deriving) ---
FLASH_PAGE_SIZE = 2048
BOOTLOADER_N_PAGES = 5
FIRST_FIRMWARE_PAGE_NUMBER = BOOTLOADER_N_PAGES
LAST_FIRMWARE_PAGE_NUMBER = 30
WAIT_FOR_RESET_TIME = 0.07
DELAY_AFTER_EACH_PAGE = 0.18
MODEL_CODE_LENGTH = 8
FIRMWARE_COMPATIBILITY_CODE_LENGTH = 1
FIRMWARE_PAGE_NUMBER_LENGTH = 1
CRC32_SIZE = 4
MINIMUM_FIRMWARE_SIZE = FLASH_PAGE_SIZE - CRC32_SIZE


def find_firmware_file(version: str, product: str = "M17",
                       scc: int = 3, hw: str = "1.5") -> Optional[str]:
    """Locate the released ``.firmware`` image for a target version."""
    preferred = os.path.join(
        config.FIRMWARE_RELEASES_DIR,
        "servomotor_%s_fw%s_scc%d_hw%s.firmware" % (product, version, scc, hw))
    if os.path.exists(preferred):
        return preferred
    matches = glob.glob(os.path.join(config.FIRMWARE_RELEASES_DIR,
                                     "*fw%s*.firmware" % version))
    return matches[0] if matches else None


def _read_binary(filename: str):
    with open(filename, "rb") as fh:
        data = fh.read()
    if len(data) - MODEL_CODE_LENGTH - FIRMWARE_COMPATIBILITY_CODE_LENGTH < MINIMUM_FIRMWARE_SIZE:
        raise ValueError("firmware file too small: %s" % filename)
    model_code = data[0:MODEL_CODE_LENGTH]
    compat = int.from_bytes(
        data[MODEL_CODE_LENGTH:MODEL_CODE_LENGTH + FIRMWARE_COMPATIBILITY_CODE_LENGTH],
        "little")
    firmware = data[MODEL_CODE_LENGTH + FIRMWARE_COMPATIBILITY_CODE_LENGTH:]
    return model_code, compat, firmware


def _prepare_image(data: bytes) -> bytes:
    """Pad to a multiple of 4, prepend the 32-bit firmware size, append CRC32
    (exactly as upgrade_firmware.py does before page programming)."""
    data = bytearray(data)
    while len(data) & 0x03:
        data += b"\x00"
    firmware_size = (len(data) >> 2) - 1
    crc = zlib.crc32(bytes(data[4:])) & 0xFFFFFFFF
    return firmware_size.to_bytes(4, "little") + bytes(data[4:]) + crc.to_bytes(4, "little")


def flash_all_broadcast(transport: Transport, version: str,
                        log: Optional[Callable[[str], None]] = None,
                        product: str = "M17", scc: int = 3, hw: str = "1.5") -> None:
    """Broadcast-flash every motor on this bus to ``version``.

    Raises FileNotFoundError if no matching firmware image is found.
    """
    def _log(m):
        if log:
            log(m)

    path = find_firmware_file(version, product, scc, hw)
    if not path:
        raise FileNotFoundError(
            "no firmware image for version %s in %s" % (version, config.FIRMWARE_RELEASES_DIR))
    model_code, compat, firmware = _read_binary(path)
    image = _prepare_image(firmware)
    _log("Flashing %s (%d bytes) to all motors on the bus." % (os.path.basename(path), len(image)))

    transport.transact(255, "System reset")
    time.sleep(WAIT_FOR_RESET_TIME)

    page_number = FIRST_FIRMWARE_PAGE_NUMBER
    offset = 0
    while offset < len(image):
        if page_number > LAST_FIRMWARE_PAGE_NUMBER:
            raise ValueError("firmware too big to fit in flash")
        chunk = image[offset:offset + FLASH_PAGE_SIZE]
        if len(chunk) < FLASH_PAGE_SIZE:
            chunk = chunk + bytes(FLASH_PAGE_SIZE - len(chunk))
        payload = (model_code
                   + int(compat).to_bytes(FIRMWARE_COMPATIBILITY_CODE_LENGTH, "little")
                   + int(page_number).to_bytes(FIRMWARE_PAGE_NUMBER_LENGTH, "little")
                   + chunk)
        transport.transact(255, "Firmware upgrade", [payload])
        time.sleep(DELAY_AFTER_EACH_PAGE)   # broadcast: no ack, so pace the writes
        offset += FLASH_PAGE_SIZE
        page_number += 1

    transport.transact(255, "System reset")
    time.sleep(config.BOOTLOADER_EXIT_DELAY_S)
    _log("Firmware flash complete (%d pages)." % (page_number - FIRST_FIRMWARE_PAGE_NUMBER))
