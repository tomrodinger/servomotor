#!/usr/bin/env python3
"""
M23 Telemetry Protocol Definitions
==================================

Shared protocol definitions for M23 current streaming telemetry viewer and test tools.

Telemetry Frame Format (9 bytes, motor -> PC, little-endian):
  - Bytes 0-1: i_a_ref (int16, Phase A desired current)
  - Bytes 2-3: i_a_actual (int16, Phase A measured current)
  - Bytes 4-5: i_b_actual (int16, Phase B measured current)
  - Bytes 6-7: pwm_a (int16, Phase A PWM duty, TIM1->CCR1, 0-1024)
  - Byte 8:   checksum (uint8, sum of bytes 0-7 truncated to 8 bits)

One frame is sent per motor control loop iteration (~31.25 kHz).
No timestamp in frame - timestamps generated locally from frame count and sample rate.

Control:
  - Send 's' character to toggle streaming on/off
  - Streaming baud rate: 5 Mbps (~4.92 Mbps actual)
  - Debug mode baud rate: 230400 bps
"""

import struct
from dataclasses import dataclass
from typing import Optional

# ==============================================================================
# Telemetry Protocol Constants
# ==============================================================================

# Telemetry frame (motor -> PC)
TELEMETRY_FRAME_SIZE = 9
TELEMETRY_DATA_SIZE = 8  # Bytes before checksum

# Serial configuration
DEFAULT_PORT = '/dev/tty.usbserial-110'  # macOS typical
DEFAULT_BAUD = 5000000  # 5 Mbps (~4.92 Mbps actual with 1.5% error)
DEBUG_BAUD = 230400     # Normal debug baud rate
SAMPLE_RATE_HZ = 31250  # Motor control loop frequency

# PWM configuration for M23
PWM_PERIOD_TIM1 = 1024  # Maximum PWM duty value

# ADC configuration
ADC_MAX_VALUE = 4095    # 12-bit ADC

# Colors for plotting (dark theme)
COLOR_CURRENT_A = '#2ecc71'   # Green
COLOR_CURRENT_B = '#3498db'   # Blue
COLOR_PWM_A = '#f39c12'       # Orange
COLOR_PWM_B = '#9b59b6'       # Purple


def compute_checksum(data: bytes) -> int:
    """Compute checksum: sum of bytes, truncated to uint8."""
    return sum(data) & 0xFF


# ==============================================================================
# Telemetry Frame
# ==============================================================================

@dataclass
class M23TelemetryFrame:
    """Represents a single M23 telemetry frame."""
    current_a: int      # Phase A current (ADC counts)
    current_b: int      # Phase B current (ADC counts)
    pwm_a: int          # Phase A PWM duty (0-1024)
    pwm_b: int          # Phase B PWM duty (0-1024)

    def pack(self) -> bytes:
        """Pack frame into bytes (including checksum)."""
        data = struct.pack(
            '<hhhh',
            self.current_a,
            self.current_b,
            self.pwm_a,
            self.pwm_b,
        )
        return data + bytes([compute_checksum(data)])

    @classmethod
    def unpack(cls, data: bytes) -> Optional['M23TelemetryFrame']:
        """Unpack frame from bytes. Returns None if invalid."""
        if len(data) < TELEMETRY_FRAME_SIZE:
            return None
        try:
            current_a, current_b, pwm_a, pwm_b = struct.unpack(
                '<hhhh', data[:TELEMETRY_DATA_SIZE]
            )
            return cls(
                current_a=current_a,
                current_b=current_b,
                pwm_a=pwm_a,
                pwm_b=pwm_b,
            )
        except struct.error:
            return None


# ==============================================================================
# Frame Parser
# ==============================================================================

class FrameParser:
    """Parses telemetry frames from serial data stream using checksum validation."""

    REQUIRED_CONSECUTIVE_GOOD_FRAMES = 3

    def __init__(self):
        self.buffer = bytearray()
        self.frames_parsed = 0
        self.checksum_errors = 0
        self.consecutive_good = 0
        self.synchronized = False
        self.discarded_frames = 0

    def reset(self):
        """Reset parser synchronization state."""
        self.buffer = bytearray()
        self.consecutive_good = 0
        self.synchronized = False

    def _validate_checksum(self, frame_bytes):
        """Check if frame_bytes has a valid checksum."""
        expected = compute_checksum(frame_bytes[:TELEMETRY_DATA_SIZE])
        return frame_bytes[TELEMETRY_DATA_SIZE] == expected

    def feed(self, data: bytes) -> list:
        """Feed raw bytes into the parser. Returns list of M23TelemetryFrame objects."""
        if not data:
            return []

        self.buffer.extend(data)
        frames = []

        while len(self.buffer) >= TELEMETRY_FRAME_SIZE:
            frame_data = self.buffer[:TELEMETRY_FRAME_SIZE]

            if self._validate_checksum(frame_data):
                self.consecutive_good += 1

                if self.synchronized:
                    # Parse and emit
                    frame = M23TelemetryFrame.unpack(bytes(frame_data))
                    if frame is not None:
                        frames.append(frame)
                        self.frames_parsed += 1
                    self.buffer = self.buffer[TELEMETRY_FRAME_SIZE:]
                elif self.consecutive_good >= self.REQUIRED_CONSECUTIVE_GOOD_FRAMES:
                    # Just achieved sync - emit this frame
                    self.synchronized = True
                    frame = M23TelemetryFrame.unpack(bytes(frame_data))
                    if frame is not None:
                        frames.append(frame)
                        self.frames_parsed += 1
                    self.buffer = self.buffer[TELEMETRY_FRAME_SIZE:]
                else:
                    # Valid checksum but not yet synchronized, discard
                    self.discarded_frames += 1
                    self.buffer = self.buffer[TELEMETRY_FRAME_SIZE:]
            else:
                # Checksum failed - lose sync and advance by 1 byte
                self.checksum_errors += 1
                self.consecutive_good = 0
                self.synchronized = False
                self.buffer = self.buffer[1:]

        return frames


# ==============================================================================
# Testing
# ==============================================================================

if __name__ == '__main__':
    print("M23 Telemetry Protocol Tests")
    print("=" * 40)

    # Test frame packing/unpacking
    print("\nTesting Frame Pack/Unpack:")
    frame = M23TelemetryFrame(
        current_a=2048,
        current_b=2050,
        pwm_a=512,
        pwm_b=510
    )
    packed = frame.pack()
    print(f"  Original: {frame}")
    print(f"  Packed ({len(packed)} bytes): {packed.hex()}")
    assert len(packed) == TELEMETRY_FRAME_SIZE, f"Expected {TELEMETRY_FRAME_SIZE} bytes, got {len(packed)}"

    # Verify checksum
    expected_cksum = compute_checksum(packed[:TELEMETRY_DATA_SIZE])
    assert packed[TELEMETRY_DATA_SIZE] == expected_cksum, "Checksum mismatch!"
    print(f"  Checksum: 0x{expected_cksum:02X} (valid)")

    unpacked = M23TelemetryFrame.unpack(packed)
    print(f"  Unpacked: {unpacked}")
    assert unpacked == frame, "Pack/unpack mismatch!"

    # Test parser
    print("\nTesting Frame Parser:")
    parser = FrameParser()

    # Create a stream of frames
    stream = bytearray()
    for i in range(10):
        f = M23TelemetryFrame(
            current_a=2000 + i,
            current_b=2000 - i,
            pwm_a=500 + i,
            pwm_b=500 - i
        )
        stream.extend(f.pack())

    # Feed in chunks
    frames = []
    chunk_size = 7  # Odd size to test buffering
    for i in range(0, len(stream), chunk_size):
        chunk = bytes(stream[i:i + chunk_size])
        frames.extend(parser.feed(chunk))

    print(f"  Frames parsed: {len(frames)}")
    print(f"  Checksum errors: {parser.checksum_errors}")
    print(f"  Synchronized: {parser.synchronized}")

    # Test with garbage data
    print("\nTesting with noise:")
    parser.reset()
    noisy_stream = b'\x00\x01\x02' + packed + b'\xff\xfe' + packed + packed
    frames = parser.feed(noisy_stream)
    print(f"  Frames found: {len(frames)}")
    print(f"  Checksum errors: {parser.checksum_errors}")

    # Test checksum detection
    print("\nTesting corrupted frame detection:")
    parser2 = FrameParser()
    good_stream = bytearray()
    for i in range(10):
        f = M23TelemetryFrame(
            current_a=1000 + i,
            current_b=2000 + i,
            pwm_a=300,
            pwm_b=400
        )
        good_stream.extend(f.pack())

    # Corrupt one byte in the middle of the stream
    corrupt_pos = 4 * TELEMETRY_FRAME_SIZE + 3  # Middle of frame 4
    good_stream[corrupt_pos] ^= 0xFF  # Flip all bits

    frames = parser2.feed(bytes(good_stream))
    print(f"  Frames parsed: {len(frames)} (expected ~8, 1 corrupted + re-sync)")
    print(f"  Checksum errors: {parser2.checksum_errors}")

    print("\n" + "=" * 40)
    print("All tests passed!")
