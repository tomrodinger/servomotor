"""Regression tests for the two fixes found during hardware bring-up:
collision-tolerant detection, and the serialized broadcast firmware flash.
"""

from typing import Any, List

import pytest

from backend.transport import Transport, CommunicationError, command_id_of
from backend.simulator import RackSimulator, SimMotor
from backend import detection, firmware_flash


class FlakyDetectTransport(Transport):
    """Wraps a real sim transport; the first ``fail_n`` detect reads collide."""

    def __init__(self, inner, fail_n):
        self._inner = inner
        self._fail_n = fail_n
        self._detect_calls = 0

    @property
    def port(self):
        return "FLAKY"

    @property
    def is_simulated(self):
        return True

    def transact(self, address, command, inputs=None, *, crc32_enabled=True, timeout=None):
        if command_id_of(command) == command_id_of("Detect devices"):
            self._detect_calls += 1
            if self._detect_calls <= self._fail_n:
                raise CommunicationError("simulated detect collision")
        return self._inner.transact(address, command, inputs, crc32_enabled=crc32_enabled,
                                    timeout=timeout)

    def flush_input(self):
        self._inner.flush_input()

    def close(self):
        self._inner.close()


def test_detection_retries_through_collisions():
    rack = RackSimulator()
    for i in range(5):
        rack.add_motor("A", SimMotor(0x100 + i))
    flaky = FlakyDetectTransport(rack.transport_for("A"), fail_n=4)
    found = detection.run_detect_pass(flaky, reset_before=False)
    assert len(found) == 5          # collisions retried; full set still found


def test_detect_aliases_bounded_when_a_motor_is_absent():
    # A motor that never responds must not cause an infinite loop: detect_aliases
    # is hard-bounded by max_passes and simply omits the absent motor.
    from backend import config
    config.BOOTLOADER_EXIT_DELAY_S = 0.01
    rack = RackSimulator()
    rack.add_motor("A", SimMotor(0x1))
    rack.add_motor("A", SimMotor(0x2))
    t = rack.transport_for("A")
    found = detection.detect_aliases(t, [0x1, 0x2, 0xDEADBEEF], max_passes=3)
    assert 0x1 in found and 0x2 in found      # present motors read back
    assert 0xDEADBEEF not in found            # absent motor omitted, no hang


def test_detection_gives_up_after_max_attempts():
    rack = RackSimulator()
    rack.add_motor("A", SimMotor(0x1))
    # every attempt collides -> returns empty rather than hanging/raising
    flaky = FlakyDetectTransport(rack.transport_for("A"),
                                 fail_n=detection.DETECT_MAX_ATTEMPTS + 5)
    assert detection.run_detect_pass(flaky, reset_before=False) == []


class RecordingTransport(Transport):
    def __init__(self):
        self.calls: List[Any] = []

    @property
    def port(self):
        return "REC"

    @property
    def is_simulated(self):
        return True

    def transact(self, address, command, inputs=None, *, crc32_enabled=True, timeout=None):
        self.calls.append((address, command, len(inputs[0]) if inputs else 0))
        return []

    def flush_input(self):
        pass

    def close(self):
        pass


def test_firmware_image_prep_shape():
    raw = bytes(range(256)) * 20      # > one page
    image = firmware_flash._prepare_image(raw)
    assert len(image) % 4 == 0
    # layout: [size word][body][crc32]; size word = (len(image)-4)/4 - 1
    import struct
    size_words = struct.unpack_from("<I", image, 0)[0]
    assert (size_words + 1) * 4 + 4 == len(image)


def test_find_firmware_file():
    assert firmware_flash.find_firmware_file("0.15.0.0") is not None
    assert firmware_flash.find_firmware_file("9.9.9.9") is None


def test_flash_sends_expected_broadcasts():
    rec = RecordingTransport()
    firmware_flash.flash_all_broadcast(rec, "0.15.0.0")
    resets = [c for c in rec.calls if c[1] == "System reset"]
    pages = [c for c in rec.calls if c[1] == "Firmware upgrade"]
    assert len(resets) == 2                      # one before, one after
    assert len(pages) >= 1
    assert all(addr == 255 for addr, _, _ in rec.calls)   # all broadcast
    assert all(sz == firmware_flash.FLASH_PAGE_SIZE +
               firmware_flash.MODEL_CODE_LENGTH +
               firmware_flash.FIRMWARE_COMPATIBILITY_CODE_LENGTH +
               firmware_flash.FIRMWARE_PAGE_NUMBER_LENGTH
               for a, c, sz in rec.calls if c == "Firmware upgrade")
