"""High-level per-motor command helpers built on a :class:`Transport`.

Phases address motors through a :class:`MotorClient`, which sends commands via
the bus's transport (real or simulated) and decodes the library's parsed
responses into convenient Python values.  This is a thin convenience layer over
the *reused* library encoder/decoder — it adds no protocol logic of its own.

All positions/velocities here are in the motor's **internal units** (encoder
counts, timesteps); callers convert to/from rotations/seconds via ``units``.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

from .transport import Transport, ALL_ALIAS


class MotorClient:
    def __init__(self, transport: Transport, unique_id: int):
        self.transport = transport
        self.unique_id = unique_id

    # -- low level ----------------------------------------------------------
    def _query(self, command: Any, inputs: Optional[List[Any]] = None,
               timeout: Optional[float] = None) -> List[Any]:
        """Send to this motor, return the single parsed response field list."""
        responses = self.transport.transact(self.unique_id, command, inputs or [],
                                             timeout=timeout)
        if not responses:
            return []
        return responses[0]

    def _send(self, command: Any, inputs: Optional[List[Any]] = None,
              timeout: Optional[float] = None) -> None:
        self.transport.transact(self.unique_id, command, inputs or [], timeout=timeout)

    # -- identity / status --------------------------------------------------
    def get_status(self) -> Tuple[int, int]:
        r = self._query("Get status")
        return (int(r[0]), int(r[1])) if r else (0, 0)

    def get_product_info(self) -> Dict[str, Any]:
        r = self._query("Get product info")
        product_type, scc, hw, serial, uid, _ = r
        patch, minor, major = hw[0], hw[1], hw[2]
        hw_version = "%d.%d" % (major, minor) if patch == 0 else "%d.%d.%d" % (major, minor, patch)
        return {
            "product_type": product_type.rstrip("\x00 ") if isinstance(product_type, str) else product_type,
            "scc": int(scc),
            "hw_version": hw_version,
            "serial_number": int(serial),
            "unique_id": int(uid),
        }

    def get_firmware_version(self) -> str:
        r = self._query("Get firmware version")
        v = r[0]                      # [dev, patch, minor, major]
        dev, patch, minor, major = v[0], v[1], v[2], v[3]
        return "%d.%d.%d.%d" % (major, minor, patch, dev)

    def get_supply_voltage_raw(self) -> int:
        r = self._query("Get supply voltage")
        return int(r[0])

    def get_temperature(self) -> int:
        r = self._query("Get temperature")
        return int(r[0])

    def ping(self, payload: bytes) -> bytes:
        r = self._query("Ping", [payload])
        return bytes(r[0]) if r else b""

    def get_communication_statistics(self, reset: int = 0) -> Dict[str, int]:
        r = self._query("Get communication statistics", [reset])
        keys = ["crc_errors", "decode_errors", "first_bit_errors",
                "framing_errors", "overrun_errors", "noise_errors"]
        return {k: int(v) for k, v in zip(keys, r)}

    # -- motion / calibration ----------------------------------------------
    def start_calibration(self) -> None:
        self._send("Start calibration")

    def set_max_current(self, current: int, regen: int = 0) -> None:
        self._send("Set maximum motor current", [int(current), int(regen)])

    def enable_mosfets(self) -> None:
        self._send("Enable MOSFETs")

    def go_to_closed_loop(self) -> None:
        self._send("Go to closed loop")

    def disable_mosfets(self) -> None:
        self._send("Disable MOSFETs")

    def zero_position(self) -> None:
        self._send("Zero position")

    def set_max_velocity(self, velocity_internal: int) -> None:
        self._send("Set maximum velocity", [int(velocity_internal)])

    def trapezoid_move(self, displacement_counts: int, time_steps: int) -> None:
        self._send("Trapezoid move", [int(displacement_counts), int(time_steps)])

    def go_to_position(self, position_counts: int, time_steps: int) -> None:
        self._send("Go to position", [int(position_counts), int(time_steps)])

    def move_with_velocity(self, velocity_internal: int, time_steps: int) -> None:
        self._send("Move with velocity", [int(velocity_internal), int(time_steps)])

    def get_position(self) -> int:
        r = self._query("Get position")
        return int(r[0])

    def get_hall_position(self) -> int:
        r = self._query("Get hall sensor position")
        return int(r[0])

    def get_comprehensive_position(self) -> Tuple[int, int, int]:
        r = self._query("Get comprehensive position")
        return int(r[0]), int(r[1]), int(r[2])

    def get_max_pid_error(self) -> Tuple[int, int]:
        r = self._query("Get max PID error")
        return int(r[0]), int(r[1])

    def set_max_allowable_position_deviation(self, deviation_counts: int) -> None:
        self._send("Set max allowable position deviation", [int(deviation_counts)])

    def capture_hall_sensor_data(self, capture_type: int, n_points: int,
                                 channels_bitmask: int, time_steps_per_sample: int,
                                 n_samples_to_sum: int, division_factor: int,
                                 timeout: Optional[float] = None) -> bytes:
        r = self._query("Capture hall sensor data",
                        [capture_type, n_points, channels_bitmask,
                         time_steps_per_sample, n_samples_to_sum, division_factor],
                        timeout=timeout)
        return bytes(r[0]) if r else b""

    def identify(self) -> None:
        self._send("Identify")

    def test_mode(self, mode: int) -> None:
        self._send("Test mode", [int(mode)])

    def leds_solid(self) -> None:
        """Hold both LEDs (green+red) steadily on via test mode 13.

        WARNING: the firmware's LED test modes (10-13) acknowledge the command
        and then ``while(1)`` with interrupts disabled — the motor stays lit but
        is locked up and stops answering the bus.  Recovery requires a physical
        power cycle (a System reset will NOT bring it back)."""
        self._send("Test mode", [13])

    def set_alias(self, alias: int) -> None:
        self._send("Set device alias", [int(alias)])

    def system_reset(self) -> None:
        self._send("System reset")
