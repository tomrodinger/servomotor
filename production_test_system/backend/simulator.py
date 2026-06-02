"""Command-level RS485 simulator — a virtual rack of motors.

This implements :class:`~backend.transport.Transport` without any hardware, by
modelling each motor's state and responding to commands the same way the
firmware would.  Responses are produced in the *parsed* shape that the
library's ``interpret_single_response`` returns, so the bus workers and phase
code cannot tell a simulated bus from a real one.

The point is to test **everything**: detection, the phase runner, all 15
phases' collection, the database, evaluation/criteria, and PNG generation —
using motors with deliberately injected faults (dead magnet, weak current,
thermal runaway, comms errors, calibration failure, burn-in fatal, ...).

Timing: position reads interpolate against wall-clock since a move started, so
a continuous spin (Phase 6) yields a realistic dense stream while a settled
step (Phase 7) reads its target.  Tests use short durations.
"""

from __future__ import annotations

import random
import struct
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from . import units
from .transport import (Transport, FatalError, TimeoutError as RS485Timeout,
                        command_id_of, ALL_ALIAS)

# Current below which a "good" motor is too weak to turn (Phase 9 relies on it).
STALL_CURRENT = 50


@dataclass
class MotorProfile:
    """Knobs that make a simulated motor pass or fail specific phases."""
    # Identity / firmware
    firmware_version: str = "0.15.1.0"
    product_type: str = "M17"
    scc: int = 3
    hw_version: str = "1.5"
    # Electrical
    supply_voltage: float = 24.0
    ambient_temp: float = 25.0
    # Comms
    ping_drop_rate: float = 0.0       # fraction of pings that time out
    crc_errors: int = 0
    # Calibration
    calibration_fails: bool = False
    # Hall tracking (Phases 6/7): tracking error in rotations
    tracking_error_rot: float = 0.0002
    # Hall waveform (Phase 8)
    hall_span: int = 50000            # avg peak - avg valley
    hall_offset: int = 7000           # valley level
    dead_magnet: bool = False         # inject one weak magnet
    extra_hall_noise: int = 0
    # Current control (Phase 9): if True the motor turns even at low current
    current_limit_broken: bool = False
    # Thermal (Phase 11): degrees C rise per second under load
    thermal_slope: float = 0.08
    # Burn-in (Phases 12/13)
    openloop_skips: bool = False      # trips the deviation limit -> fatal
    closedloop_fatal: bool = False
    pid_error_scale: float = 20000.0  # typical max PID error magnitude
    # Overvoltage (Phase 10): a good motor trips at the 22 V threshold (mode 74)
    # and does NOT trip at 26 V (mode 75) on a 24 V rack.
    ov_no_trip_low: bool = False      # faulty: fails to trip at 22 V
    ov_false_trip_high: bool = False  # faulty: trips even at 26 V
    # LED (Phase 15) is decided by the human, not the device.


@dataclass
class _Move:
    start_t: float
    start_counts: float
    target_counts: float
    duration_s: float
    weak: bool = False                # too weak to actually move (hall frozen)


@dataclass
class SimMotor:
    unique_id: int
    alias: int = 255
    profile: MotorProfile = field(default_factory=MotorProfile)
    serial_number: int = 0

    def __post_init__(self):
        self._rng = random.Random(self.unique_id & 0xFFFFFFFF)
        self.fatal_error = 0
        self.mosfets_on = False
        self.max_current = 200
        self.max_deviation = 2_000_000_000
        self.calibration_done = False
        self._calibrating = False
        self._abs_counts = 0.0        # absolute commanded position, counts
        self._hall_abs = 0.0          # actual shaft position (lags if too weak)
        self._move: Optional[_Move] = None
        self._led_locked = False
        self._temp = self.profile.ambient_temp
        self._load_start: Optional[float] = None
        self._pid_min = 0
        self._pid_max = 0
        self._move_count = 0

    # -- helpers ------------------------------------------------------------
    def _commanded_counts(self) -> float:
        if self._move is None:
            return self._abs_counts
        elapsed = time.monotonic() - self._move.start_t
        if self._move.duration_s <= 0:
            frac = 1.0
        else:
            frac = max(0.0, min(1.0, elapsed / self._move.duration_s))
        cur = self._move.start_counts + frac * (self._move.target_counts - self._move.start_counts)
        if frac >= 1.0:
            self._abs_counts = self._move.target_counts
            # a too-weak move leaves the shaft where it started, permanently
            self._hall_abs = (self._move.start_counts if self._move.weak
                              else self._move.target_counts)
            self._move = None
        return cur

    def _hall_counts(self, commanded: float) -> float:
        if self._move is not None:
            actual = self._move.start_counts if self._move.weak else commanded
        else:
            actual = self._hall_abs
        err = self.profile.tracking_error_rot * units.COUNTS_PER_ROTATION
        # deterministic small wobble plus the profile's tracking error
        return actual + self._rng.uniform(-err, err)

    def _update_temp(self) -> None:
        if self._load_start is not None and self.mosfets_on:
            elapsed = time.monotonic() - self._load_start
            self._temp = self.profile.ambient_temp + self.profile.thermal_slope * elapsed
        else:
            self._temp = self.profile.ambient_temp

    # -- command dispatch ---------------------------------------------------
    def handle(self, command_id: int, inputs: List[Any]) -> List[Any]:
        """Return the parsed response field list, or raise FatalError/timeout."""
        if self._led_locked:
            # Firmware LED test mode disables interrupts: the device is silent.
            raise RS485Timeout("device locked in LED test mode")

        name = _CMD_NAMES.get(command_id)

        # A latched fatal error blocks most commands (mirrors the firmware).
        if self.fatal_error and command_id not in (_CMD["system_reset"], _CMD["get_status"]):
            raise FatalError(self.fatal_error)

        handler = getattr(self, "_cmd_" + name, None) if name else None
        if handler is None:
            return []
        return handler(inputs)

    # success-response commands return [] (empty parsed response)
    def _cmd_system_reset(self, _):
        self.fatal_error = 0
        self.mosfets_on = False
        self.max_current = 200
        self.max_deviation = 2_000_000_000
        self._move = None
        self._load_start = None
        self._calibrating = False
        self._move_count = 0
        return []

    def _cmd_get_status(self, _):
        flags = 0
        if self._calibrating:
            # calibration finished by the time the host reads status after the hold
            self._calibrating = False
            if self.profile.calibration_fails:
                self.fatal_error = 2   # some fatal code
            else:
                self.calibration_done = True
        status_flag = 0
        return [status_flag, self.fatal_error]

    def _cmd_get_product_info(self, _):
        p = self.profile
        major, minor, patch = (p.hw_version.split(".") + ["0", "0", "0"])[:3]
        hw = [int(patch), int(minor), int(major)]   # u24: [patch, minor, major]
        return [p.product_type, p.scc, hw, self.serial_number, self.unique_id, 0]

    def _cmd_get_firmware_version(self, _):
        parts = [int(x) for x in (self.profile.firmware_version.split(".") + ["0"] * 4)[:4]]
        major, minor, patch, dev = parts
        return [[dev, patch, minor, major], 0]   # u32: [dev, patch, minor, major]

    def _cmd_get_supply_voltage(self, _):
        return [int(round(self.profile.supply_voltage * units.VOLTS_FACTOR))]

    def _cmd_get_temperature(self, _):
        self._update_temp()
        return [int(round(self._temp))]

    def _cmd_ping(self, inputs):
        if self.profile.ping_drop_rate and self._rng.random() < self.profile.ping_drop_rate:
            raise RS485Timeout("simulated ping drop")
        payload = inputs[0] if inputs else b""
        if isinstance(payload, str):
            payload = payload.encode("utf-8")
        return [bytes(payload)]

    def _cmd_get_communication_statistics(self, inputs):
        return [self.profile.crc_errors, 0, 0, 0, 0, 0]

    def _cmd_start_calibration(self, _):
        self._calibrating = True
        self.mosfets_on = True
        return []

    def _cmd_set_maximum_motor_current(self, inputs):
        self.max_current = int(inputs[0]) if inputs else self.max_current
        return []

    def _cmd_enable_mosfets(self, _):
        self.mosfets_on = True
        return []

    def _cmd_disable_mosfets(self, _):
        self.mosfets_on = False
        return []

    def _cmd_zero_position(self, _):
        self._abs_counts = 0.0
        self._hall_abs = 0.0
        self._move = None
        return []

    def _cmd_set_maximum_velocity(self, inputs):
        return []

    def _start_linear_move(self, target_counts: float, duration_s: float):
        start = self._commanded_counts()
        weak = (self.max_current < STALL_CURRENT and not self.profile.current_limit_broken)
        self._move = _Move(time.monotonic(), start, target_counts, duration_s, weak=weak)
        if self.mosfets_on:
            if self._load_start is None:
                self._load_start = time.monotonic()
        # open-loop deviation trip (Phase 12)
        if (self.profile.openloop_skips and
                self.max_deviation < 0.05 * units.COUNTS_PER_ROTATION):
            self.fatal_error = 3   # deviation-limit fatal

    def _cmd_trapezoid_move(self, inputs):
        disp = int(inputs[0]); dur = int(inputs[1])
        dur_s = dur / units.TIMESTEPS_PER_SECOND
        target = self._commanded_counts() + disp
        self._start_linear_move(target, dur_s)
        # closed-loop burn-in fatal injection (deterministic: a few moves in)
        self._move_count += 1
        if self.profile.closedloop_fatal and self._move_count >= 3:
            self.fatal_error = 4
        # PID-error model: with adequate current the servo keeps up (small
        # error); underpowered (low max current) it lags and the error grows
        # with the demanded speed.  A motor whose current limit is broken keeps
        # up even at a low setting (small error) -> Phase 9 catches it.
        demanded_rps = abs(disp) / units.COUNTS_PER_ROTATION / max(dur_s, 0.05)
        eff_current = 200 if self.profile.current_limit_broken else self.max_current
        noise = abs(self._rng.gauss(0, self.profile.pid_error_scale))
        if eff_current >= STALL_CURRENT:
            self._pid_max = int(noise)
        else:
            # ~1.64e6 per rot/s reproduces the ~5.9e6 deviation observed on the
            # bench for the Phase 9 move (1.8 rot in 0.5 s = 3.6 rot/s).
            self._pid_max = int(demanded_rps * 1_640_000 + noise)
        self._pid_min = -int(abs(self._rng.gauss(0, self.profile.pid_error_scale)))
        return []

    def _cmd_go_to_position(self, inputs):
        target = int(inputs[0]); dur = int(inputs[1])
        self._start_linear_move(float(target), dur / units.TIMESTEPS_PER_SECOND)
        return []

    def _cmd_move_with_velocity(self, inputs):
        vel = int(inputs[0]); dur = int(inputs[1])   # internal velocity, time
        duration_s = dur / units.TIMESTEPS_PER_SECOND
        vel_counts_per_s = vel / (units.VEL_PER_ROT_PER_S / units.COUNTS_PER_ROTATION)
        target = self._commanded_counts() + vel_counts_per_s * duration_s
        self._start_linear_move(target, duration_s)
        return []

    def _cmd_get_position(self, _):
        return [int(round(self._commanded_counts()))]

    def _cmd_get_hall_sensor_position(self, _):
        commanded = self._commanded_counts()
        return [int(round(self._hall_counts(commanded)))]

    def _cmd_get_comprehensive_position(self, _):
        commanded = self._commanded_counts()
        hall = self._hall_counts(commanded)
        return [int(round(commanded)), int(round(hall)), 0]

    def _cmd_get_max_pid_error(self, _):
        mn, mx = self._pid_min, self._pid_max
        self._pid_min = 0
        self._pid_max = 0      # cmd 39 read-and-reset
        return [mn, mx]

    def _cmd_set_max_allowable_position_deviation(self, inputs):
        self.max_deviation = int(inputs[0]) if inputs else self.max_deviation
        return []

    def _cmd_identify(self, _):
        return []

    def _cmd_test_mode(self, inputs):
        mode = int(inputs[0]) if inputs else 0
        if 10 <= mode <= 13:
            self._led_locked = True
        elif mode == 74:      # OV threshold 22 V: trips on a 24 V rack
            if not self.profile.ov_no_trip_low:
                self.fatal_error = 14      # ERROR_OVERVOLTAGE
        elif mode == 75:      # OV threshold 26 V: should NOT trip on 24 V
            if self.profile.ov_false_trip_high:
                self.fatal_error = 14
        # The firmware acks the command, then the comparator trips asynchronously,
        # so we still return a success ack here.
        return []

    def _cmd_capture_hall_sensor_data(self, inputs):
        # inputs: captureType, nPoints, channelsBitmask, timeStepsPerSample,
        #         nSamplesToSum, divisionFactor
        n_points = int(inputs[1])
        return [self._synth_hall_waveform(n_points)]

    def _synth_hall_waveform(self, n_points: int) -> bytes:
        """Build a synthetic 3-channel raw hall capture (u16 little-endian).

        ~50 magnet periods per rotation over ~1.4 rotations => ~70 extrema per
        channel.  Three channels are 120 degrees apart.  ``dead_magnet`` weakens
        one peak so Phase 8's per-magnet check can catch it.
        """
        import math
        p = self.profile
        # 50 extrema per rotation = 25 sine cycles per rotation; over ~1.4
        # rotations that is ~35 cycles => ~70 extrema per channel.
        cycles = (50 / 2.0) * 1.4
        period_samples = n_points / cycles
        out = bytearray()
        mid = p.hall_offset + p.hall_span / 2.0
        amp = p.hall_span / 2.0
        dead_at = int(n_points * 0.5)
        for i in range(n_points):
            phase = 2 * math.pi * cycles * i / n_points
            for ch in range(3):
                scale = 1.0
                # Weaken one full magnet cycle on channel 0 (a damaged magnet).
                if p.dead_magnet and ch == 0 and abs(i - dead_at) < period_samples / 2:
                    scale = 0.35
                val = mid + scale * amp * math.sin(phase + ch * 2 * math.pi / 3.0)
                if p.extra_hall_noise:
                    val += self._rng.uniform(-p.extra_hall_noise, p.extra_hall_noise)
                v = int(max(0, min(65535, round(val))))
                out += struct.pack("<H", v)
        return bytes(out)


_CMD = {
    "system_reset": command_id_of("System reset"),
    "get_status": command_id_of("Get status"),
}

# Map command id -> the snake_case handler suffix used above.
_CMD_NAMES: Dict[int, str] = {}
for _label in [
    "System reset", "Get status", "Get product info", "Get firmware version",
    "Get supply voltage", "Get temperature", "Ping",
    "Get communication statistics", "Start calibration",
    "Set maximum motor current", "Enable MOSFETs", "Disable MOSFETs",
    "Zero position", "Set maximum velocity", "Trapezoid move", "Go to position",
    "Move with velocity", "Get position", "Get hall sensor position",
    "Get comprehensive position", "Get max PID error",
    "Set max allowable position deviation", "Identify", "Test mode",
    "Capture hall sensor data",
]:
    _CMD_NAMES[command_id_of(_label)] = _label.replace(" ", "_").lower()


class RackSimulator:
    """Holds the simulated motors for all three buses."""

    def __init__(self):
        self.buses: Dict[str, Dict[int, SimMotor]] = {}

    def add_motor(self, bus_id: str, motor: SimMotor,
                  detect_prob: float = 1.0) -> SimMotor:
        motor._detect_prob = detect_prob          # type: ignore[attr-defined]
        self.buses.setdefault(bus_id, {})[motor.unique_id] = motor
        return motor

    def remove_motor(self, bus_id: str, unique_id: int) -> None:
        """Physically pull a motor from the rack (for LED-test reconciliation)."""
        self.buses.get(bus_id, {}).pop(unique_id, None)

    def power_cycle(self, bus_id: str) -> None:
        """Power-cycle the bus: recovers every motor from LED-test lockup."""
        for m in self.buses.get(bus_id, {}).values():
            m._led_locked = False
            m.fatal_error = 0
            m.mosfets_on = False
            m._load_start = None

    def transport_for(self, bus_id: str, port: str = "SIM") -> "SimulatedTransport":
        self.buses.setdefault(bus_id, {})
        return SimulatedTransport(self, bus_id, port)


class SimulatedTransport(Transport):
    """A :class:`Transport` backed by a :class:`RackSimulator` bus."""

    def __init__(self, rack: RackSimulator, bus_id: str, port: str = "SIM"):
        self._rack = rack
        self._bus_id = bus_id
        self._port = port
        self._rng = random.Random(hash(bus_id) & 0xFFFFFFFF)

    @property
    def port(self) -> str:
        return self._port

    @property
    def is_simulated(self) -> bool:
        return True

    def _motors(self) -> Dict[int, SimMotor]:
        return self._rack.buses.get(self._bus_id, {})

    def transact(self, address, command, inputs=None, *, crc32_enabled=True,
                 timeout=None):
        if inputs is None:
            inputs = []
        command_id = command_id_of(command)
        detect_id = command_id_of("Detect devices")

        if command_id == detect_id:
            results: List[List[Any]] = []
            for m in self._motors().values():
                prob = getattr(m, "_detect_prob", 1.0)
                if m._led_locked:
                    continue
                if self._rng.random() <= prob:
                    results.append([m.unique_id, m.alias])
            return results

        set_alias_id = command_id_of("Set device alias")

        if address == ALL_ALIAS:
            # Broadcast: deliver to every motor, expect no responses.
            for m in self._motors().values():
                try:
                    if command_id == set_alias_id and inputs:
                        m.alias = int(inputs[0])
                    else:
                        m.handle(command_id, list(inputs))
                except (FatalError, RS485Timeout):
                    pass
            return []

        # Addressed to a specific motor (by unique id in extended addressing).
        motor = self._motors().get(address)
        if motor is None:
            raise RS485Timeout("no simulated motor at address %r" % (address,))
        if command_id == set_alias_id and inputs:
            motor.alias = int(inputs[0])
            return [[]]
        parsed = motor.handle(command_id, list(inputs))
        # success-response commands -> one empty parsed response
        return [parsed]

    def flush_input(self) -> None:
        pass

    def close(self) -> None:
        pass
