#!/usr/bin/env python3
"""
M23 Telemetry Protocol Definitions
==================================

Shared protocol definitions for M23 current streaming telemetry viewer and test tools.

Telemetry Frame Format (10 bytes, motor → PC, little-endian):
  - Bytes 0-1: Sync word 0xABCD
  - Bytes 2-3: current_a (int16, Phase A current in ADC counts, 0-4095)
  - Bytes 4-5: current_b (int16, Phase B current in ADC counts, 0-4095)
  - Bytes 6-7: pwm_a (int16, Phase A PWM duty, TIM1->CCR1, 0-1024)
  - Bytes 8-9: pwm_b (int16, Phase B PWM duty, TIM1->CCR2, 0-1024)

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

# Telemetry frame (motor → PC)
TELEMETRY_FRAME_SIZE = 10
TELEMETRY_SYNC_WORD = 0xABCD
TELEMETRY_SYNC_BYTES = struct.pack('<H', TELEMETRY_SYNC_WORD)

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


# ==============================================================================
# Telemetry Frame
# ==============================================================================

@dataclass
class M23TelemetryFrame:
    """Represents a single M23 telemetry frame."""
    current_a: int      # Phase A current (ADC counts, 0-4095)
    current_b: int      # Phase B current (ADC counts, 0-4095)
    pwm_a: int          # Phase A PWM duty (0-1024)
    pwm_b: int          # Phase B PWM duty (0-1024)

    def pack(self) -> bytes:
        """Pack frame into bytes (including sync word)."""
        return struct.pack(
            '<Hhhhh',
            TELEMETRY_SYNC_WORD,
            self.current_a,
            self.current_b,
            self.pwm_a,
            self.pwm_b,
        )

    @classmethod
    def unpack(cls, data: bytes) -> Optional['M23TelemetryFrame']:
        """Unpack frame from bytes. Returns None if invalid."""
        if len(data) < TELEMETRY_FRAME_SIZE:
            return None
        try:
            sync, current_a, current_b, pwm_a, pwm_b = struct.unpack(
                '<Hhhhh', data[:TELEMETRY_FRAME_SIZE]
            )
            if sync != TELEMETRY_SYNC_WORD:
                return None
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
    """Parses telemetry frames from serial data stream."""

    REQUIRED_CONSECUTIVE_GOOD_FRAMES = 5

    def __init__(self):
        self.buffer = bytearray()
        self.frames_parsed = 0
        self.sync_errors = 0
        self.consecutive_good = 0
        self.synchronized = False
        self.discarded_frames = 0

    def reset(self):
        """Reset parser synchronization state."""
        self.buffer = bytearray()
        self.consecutive_good = 0
        self.synchronized = False

    def feed(self, data: bytes) -> list:
        """Feed raw bytes into the parser. Returns list of M23TelemetryFrame objects."""
        if not data:
            return []

        self.buffer.extend(data)
        frames = []

        while len(self.buffer) >= TELEMETRY_FRAME_SIZE:
            # Look for sync word
            sync_pos = self.buffer.find(TELEMETRY_SYNC_BYTES)

            if sync_pos == -1:
                # No sync found, keep only last byte (could be start of sync)
                if len(self.buffer) > 1:
                    self.sync_errors += len(self.buffer) - 1
                    self.buffer = self.buffer[-1:]
                if self.consecutive_good > 0:
                    self.consecutive_good = 0
                break

            if sync_pos > 0:
                # Discard bytes before sync
                self.sync_errors += sync_pos
                self.buffer = self.buffer[sync_pos:]
                self.consecutive_good = 0

            if len(self.buffer) < TELEMETRY_FRAME_SIZE:
                break

            # Check if next frame also starts with sync
            next_sync_valid = False
            if len(self.buffer) >= TELEMETRY_FRAME_SIZE + 2:
                next_sync = self.buffer[TELEMETRY_FRAME_SIZE:TELEMETRY_FRAME_SIZE + 2]
                next_sync_valid = (next_sync == TELEMETRY_SYNC_BYTES)

            # Parse the frame
            frame_data = bytes(self.buffer[:TELEMETRY_FRAME_SIZE])
            frame = M23TelemetryFrame.unpack(frame_data)

            if frame is not None:
                # Basic validation: ADC values should be in valid range
                if not (0 <= frame.current_a <= ADC_MAX_VALUE and
                        0 <= frame.current_b <= ADC_MAX_VALUE):
                    # Invalid data, might be false sync
                    self.consecutive_good = 0
                    self.sync_errors += 1
                    self.buffer = self.buffer[1:]
                    continue

                if next_sync_valid or len(self.buffer) < TELEMETRY_FRAME_SIZE + 2:
                    self.consecutive_good += 1

                    if not self.synchronized:
                        self.discarded_frames += 1
                        if self.consecutive_good >= self.REQUIRED_CONSECUTIVE_GOOD_FRAMES:
                            self.synchronized = True
                    else:
                        frames.append(frame)
                        self.frames_parsed += 1

                    self.buffer = self.buffer[TELEMETRY_FRAME_SIZE:]
                else:
                    # Next frame doesn't have valid sync - false positive
                    self.consecutive_good = 0
                    self.sync_errors += 1
                    self.buffer = self.buffer[1:]
            else:
                # Invalid frame
                self.consecutive_good = 0
                self.sync_errors += 1
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
    print(f"  Sync errors: {parser.sync_errors}")
    print(f"  Synchronized: {parser.synchronized}")

    # Test with garbage data
    print("\nTesting with noise:")
    parser.reset()
    noisy_stream = b'\x00\x01\x02' + packed + b'\xff\xfe' + packed + packed
    frames = parser.feed(noisy_stream)
    print(f"  Frames found: {len(frames)}")
    print(f"  Sync errors: {parser.sync_errors}")

    print("\n" + "=" * 40)
    print("All tests passed!")
