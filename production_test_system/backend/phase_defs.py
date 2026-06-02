"""Single source of truth for the 15 test phases (metadata only).

Each :class:`PhaseDef` records what a phase does, its **collection (input)
parameters** (Stage A) and its **pass/fail criteria** (Stage B), plus the list
of **measured items** that the per-phase UI tab renders (a histogram with
threshold lines, or a categorical/boolean count summary).

This module holds *defaults and structure only* — live values are kept in
``settings.py`` (persisted to JSON) so the operator can tune them.  The
defaults here mirror ``PRODUCTION_TEST_PROPOSAL.md`` and, per that document,
are deliberately *placeholders to be tuned from real captures* via the Tab 2
histograms.
"""

from __future__ import annotations

from dataclasses import dataclass, field, asdict
from typing import Any, Dict, List, Optional


@dataclass
class Param:
    """One editable parameter (a collection input or a pass/fail criterion)."""
    key: str
    label: str
    type: str                      # "str" | "int" | "float" | "bool"
    default: Any
    unit: Optional[str] = None
    help: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class MeasuredItem:
    """A derived metric shown on the phase tab beside its criterion.

    ``kind`` is ``"histogram"`` for numeric distributions (drawn with vertical
    threshold lines) or ``"categorical"`` for boolean/enum observations (drawn
    as a count/bar summary).  ``threshold_keys`` names the criteria params whose
    current values are drawn as vertical lines on the histogram.
    """
    key: str
    label: str
    kind: str = "histogram"
    unit: Optional[str] = None
    threshold_keys: List[str] = field(default_factory=list)
    per_channel: bool = False       # Phase 8 metrics are per hall channel

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class PhaseDef:
    number: int
    key: str
    name: str
    short: str
    description: str
    power: str
    parallelism: str
    enabled_default: bool = True
    requires_calibration: bool = False
    params: List[Param] = field(default_factory=list)
    criteria: List[Param] = field(default_factory=list)
    measured: List[MeasuredItem] = field(default_factory=list)
    firmware_dependency: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        return d


# ---------------------------------------------------------------------------
# The 15 phases.
# ---------------------------------------------------------------------------

PHASES: List[PhaseDef] = [
    PhaseDef(
        number=1, key="firmware", name="Firmware", short="MCU flash + RS485",
        power="none", parallelism="broadcast flash; sequential verify",
        description=(
            "Broadcast-flash all motors to the target release, then read the "
            "firmware version back from each device individually."),
        params=[
            Param("target_firmware_version", "Target firmware version", "str",
                  "0.15.1.0", help="Latest release; flashed to every motor. "
                                   "0.15.1.0 adds the Phase 10 OV test modes."),
            Param("flash_enabled", "Broadcast-flash before verifying", "bool", True,
                  help="If off, the version is only read back and verified "
                       "(no flashing). The broadcast-flash path is bench-pending."),
        ],
        criteria=[],   # pass = version equals target AND device responded
        measured=[
            MeasuredItem("firmware_version", "Firmware version read back",
                         kind="categorical"),
            MeasuredItem("responded", "Responded after flash", kind="categorical"),
        ],
    ),
    PhaseDef(
        number=2, key="identity", name="Identity", short="MCU, RS485",
        power="none", parallelism="yes",
        description="Read product info (cmd 22) from each device.",
        params=[
            Param("expected_product_type", "Expected product type", "str", "M17"),
            Param("expected_hw_version", "Expected hardware version", "str", "1.5"),
            Param("expected_scc", "Expected software-compat code", "int", 3),
        ],
        criteria=[],
        measured=[
            MeasuredItem("product_type", "Product type", kind="categorical"),
            MeasuredItem("hw_version", "Hardware version", kind="categorical"),
            MeasuredItem("scc", "Software-compat code", kind="categorical"),
        ],
    ),
    PhaseDef(
        number=3, key="electrical", name="Electrical baseline",
        short="Voltage divider, thermistor, MCU", power="none", parallelism="yes",
        description=("Read supply voltage (cmd 38), temperature (cmd 42) and "
                     "status (cmd 16) — an at-rest sanity check."),
        params=[],
        criteria=[
            Param("voltage_nominal", "Nominal supply voltage", "float", 24.0, "V"),
            Param("voltage_tolerance_pct", "Voltage tolerance", "float", 10.0, "%"),
            Param("temp_min", "Temperature min", "float", 10.0, "°C"),
            Param("temp_max", "Temperature max", "float", 45.0, "°C"),
        ],
        measured=[
            MeasuredItem("supply_voltage", "Supply voltage", unit="V",
                         threshold_keys=["voltage_nominal"]),
            MeasuredItem("temperature", "Board temperature", unit="°C",
                         threshold_keys=["temp_min", "temp_max"]),
            MeasuredItem("status_clean", "Status flags clean", kind="categorical"),
        ],
    ),
    PhaseDef(
        number=4, key="comms", name="Comms quality",
        short="RS485 transceiver, MCU UART", power="none", parallelism="yes",
        description="Send pings (cmd 31) and read comm statistics (cmd 47).",
        params=[
            Param("ping_count", "Ping count", "int", 1000),
        ],
        criteria=[
            Param("required_success_rate", "Required success rate", "float", 100.0, "%"),
            Param("max_crc_errors", "Max CRC errors", "int", 0),
        ],
        measured=[
            MeasuredItem("success_rate", "Ping success rate", unit="%",
                         threshold_keys=["required_success_rate"]),
            MeasuredItem("crc_errors", "CRC error count",
                         threshold_keys=["max_crc_errors"]),
        ],
    ),
    PhaseDef(
        number=5, key="calibration", name="Calibration", short="Full motion chain",
        power="full", parallelism="8 per section",
        description=("Send start_calibration (cmd 6) to a batch of 8, keep the "
                     "bus quiet for the hold time, then read status once. The "
                     "motor auto-reboots — never send system_reset here."),
        params=[
            Param("start_stagger_s", "Delay between start-calibration sends",
                  "float", 0.5, "s",
                  "Pace the 8 start-calibration commands (one per this interval) "
                  "to avoid command-buffer overflow during calibration."),
            Param("quiet_hold_s", "Bus-quiet hold time", "float", 30.0, "s",
                  "Bus stays completely silent during this window."),
        ],
        criteria=[],   # pass = status word is perfectly zero
        measured=[
            MeasuredItem("status_zero", "Calibration status zero", kind="categorical"),
        ],
    ),
    PhaseDef(
        number=6, key="spin_tracking", name="Continuous-spin tracking",
        short="Hall sensors, AT5833, windings (dynamic)", power="full",
        parallelism="one motor at a time", requires_calibration=True,
        description=("Open-loop, max current. Spin forward 1.5 rotations then "
                     "back, polling get_comprehensive_position (cmd 37) as fast "
                     "as the bus allows. The full (commanded, hall) stream is "
                     "stored as a blob."),
        params=[
            Param("spin_rotations", "Spin distance each direction", "float", 1.5, "rot"),
            Param("spin_velocity", "Spin velocity", "float", 0.5, "rot/s"),
            Param("max_current", "Motor current", "int", 200,
                  help="Max motor current (cmd 28) for the open-loop spin."),
        ],
        criteria=[
            Param("deviation_threshold_rot", "Max hall-vs-commanded deviation",
                  "float", 0.001, "rot", "Shared with Phase 7."),
        ],
        measured=[
            MeasuredItem("max_deviation", "Max hall-vs-commanded deviation",
                         unit="rot", threshold_keys=["deviation_threshold_rot"]),
        ],
    ),
    PhaseDef(
        number=7, key="hall_linearity", name="Hall linearity & hysteresis",
        short="Hall sensors, AT5833, windings (static)", power="full",
        parallelism="8 at once", requires_calibration=True,
        description=("Open-loop, max current. Step through 2000 evenly-spaced "
                     "positions over one rotation CW then CCW, settling 50 ms "
                     "each, reading cmd 37. 4000 pairs stored as a blob."),
        params=[
            Param("n_steps", "Steps per rotation", "int", 2000),
            Param("settle_ms", "Settle time per step", "int", 50, "ms"),
            Param("max_current", "Motor current", "int", 200),
        ],
        criteria=[
            Param("deviation_threshold_rot", "Max hall-vs-commanded deviation",
                  "float", 0.001, "rot", "Shared with Phase 6."),
        ],
        measured=[
            MeasuredItem("max_deviation", "Max hall-vs-commanded deviation",
                         unit="rot", threshold_keys=["deviation_threshold_rot"]),
        ],
    ),
    PhaseDef(
        number=8, key="hall_waveform", name="Hall waveform & peak analysis",
        short="Hall signal quality + 50-magnet disk", power="default",
        parallelism="one motor at a time", requires_calibration=True,
        description=("Capture raw 3-channel hall waveform (capture type 1, 4000 "
                     "points, bitmask 7, sum 64 / div 16) while spinning 1.4 "
                     "rotations. Peak/valley analysis is done in post-processing."),
        params=[
            Param("capture_type", "Capture type", "int", 1),
            Param("n_points", "Points to capture", "int", 4000),
            Param("channels_bitmask", "Channels bitmask", "int", 7),
            Param("time_steps_per_sample", "Time steps per sample", "int", 1),
            Param("n_samples_to_sum", "Samples summed per point", "int", 64),
            Param("division_factor", "Division factor", "int", 16),
            Param("spin_rotations", "Spin distance", "float", 1.4, "rot",
                  "Same distance the firmware uses for calibration (358/256)."),
        ],
        criteria=[
            Param("peak_find_hysteresis", "Peak-find hysteresis", "int", 8000,
                  help="Firmware 2000 x (n_samples_to_sum/division_factor)."),
            Param("extrema_count_min", "Extrema count min", "int", 68),
            Param("extrema_count_max", "Extrema count max", "int", 71),
            Param("saturation_min", "Saturation band min", "int", 1000),
            Param("saturation_max", "Saturation band max", "int", 64535),
            Param("span_min", "Minimum span (avg peak - avg valley)", "int", 40000),
            Param("peak_max_dev_from_avg", "Max peak deviation from avg", "int", 2000),
            Param("valley_max_dev_from_avg", "Max valley deviation from avg", "int", 2000),
            Param("max_adjacent_peak_dev", "Max adjacent-peak deviation", "int", 2000),
            Param("max_adjacent_valley_dev", "Max adjacent-valley deviation", "int", 2000),
            Param("peak_spacing_min", "Peak spacing min", "float", 0.0, "samples",
                  "Set from real captures."),
            Param("peak_spacing_max", "Peak spacing max", "float", 1000.0, "samples",
                  "Set from real captures."),
        ],
        measured=[
            MeasuredItem("extrema_count", "Extrema count", per_channel=True,
                         threshold_keys=["extrema_count_min", "extrema_count_max"]),
            MeasuredItem("hall_min", "Min hall value", per_channel=True,
                         threshold_keys=["saturation_min"]),
            MeasuredItem("hall_max", "Max hall value", per_channel=True,
                         threshold_keys=["saturation_max"]),
            MeasuredItem("span", "Span (avg peak - avg valley)", per_channel=True,
                         threshold_keys=["span_min"]),
            MeasuredItem("max_peak_from_avg", "Max peak-from-avg deviation",
                         per_channel=True, threshold_keys=["peak_max_dev_from_avg"]),
            MeasuredItem("max_valley_from_avg", "Max valley-from-avg deviation",
                         per_channel=True, threshold_keys=["valley_max_dev_from_avg"]),
            MeasuredItem("max_adjacent_peak_dev", "Max adjacent-peak deviation",
                         per_channel=True, threshold_keys=["max_adjacent_peak_dev"]),
            MeasuredItem("max_adjacent_valley_dev", "Max adjacent-valley deviation",
                         per_channel=True, threshold_keys=["max_adjacent_valley_dev"]),
            MeasuredItem("peak_spacing_min", "Min peak spacing", per_channel=True,
                         unit="samples", threshold_keys=["peak_spacing_min"]),
            MeasuredItem("peak_spacing_max", "Max peak spacing", per_channel=True,
                         unit="samples", threshold_keys=["peak_spacing_max"]),
        ],
    ),
    PhaseDef(
        number=9, key="current_control", name="Current control",
        short="Current-control path (AT5833 / cmd 28)", power="low",
        parallelism="broadcast, all at once", requires_calibration=True,
        description=("Closed-loop, low current. Broadcast a low current limit, "
                     "go to closed loop, then broadcast a fast move (1.8 rot in "
                     "0.5 s) that the motor cannot keep up with at low current. "
                     "After a 5 s wait, read the max PID deviation (cmd 39). It "
                     "must fall within a band: too small means the current limit "
                     "is not actually limiting; too large flags a different "
                     "fault."),
        params=[
            Param("low_current", "Low motor current (internal units, 0-390)", "int", 20,
                  help="Raw internal current units (NOT amps/mA) — same value the "
                       "firmware receives via cmd 28. Range 0-390."),
            Param("move_rotations", "Move distance", "float", 1.8, "rot"),
            Param("move_time_s", "Commanded move time", "float", 0.25, "s",
                  "Tuned so the low current limits the speed (PID error ~13x the "
                  "high-current value) while the motor still reaches the target. "
                  "Faster (e.g. 0.1 s) hits the motor's speed ceiling at any "
                  "current and stops discriminating."),
            Param("wait_s", "Wait for the move", "float", 5.0, "s"),
        ],
        criteria=[
            Param("pid_error_min", "Min max-PID deviation", "float", 40000.0,
                  help="Below this the motor kept up too well -> current limit "
                       "not working (high current ~9e3 on the bench). Good motors "
                       "ranged 8.3e4-3.1e6; tune from the histogram."),
            Param("pid_error_max", "Max max-PID deviation", "float", 5000000.0,
                  help="Above this flags a different fault. Tune from the histogram."),
            Param("position_tolerance_rot", "Final position tolerance", "float",
                  0.1, "rot", "Max |commanded - hall| at the end; the motor must "
                              "have reached the target."),
        ],
        measured=[
            MeasuredItem("max_pid_deviation", "Max PID deviation at low current",
                         threshold_keys=["pid_error_min", "pid_error_max"]),
            MeasuredItem("position_error", "Final position error (commanded - hall)",
                         unit="rot", threshold_keys=["position_tolerance_rot"]),
        ],
    ),
    PhaseDef(
        number=10, key="overvoltage", name="Overvoltage protection",
        short="Overvoltage comparator + threshold PWM", power="none",
        parallelism="8 at once", enabled_default=True,
        firmware_dependency=("Requires firmware >= 0.15.1.0 (cmd 36 test modes "
                             "74 = 22 V and 75 = 26 V). Resolved in 0.15.1.0."),
        description=("Enter the 22 V test mode (cmd 36 mode 74 — should trip on a "
                     "24 V supply), reset, enter the 26 V mode (mode 75 — should "
                     "not trip), reset. A trip is reported as fatal code "
                     "ERROR_OVERVOLTAGE in the status word."),
        params=[
            Param("settle_s", "Settle time before reading status", "float", 0.5, "s",
                  "Time after entering a test mode before reading the trip."),
            # The 22 V / 26 V thresholds are fixed in the firmware test modes
            # (74 / 75); there are no editable setpoints here.
        ],
        criteria=[],
        measured=[
            MeasuredItem("tripped_low", "22 V mode tripped", kind="categorical"),
            MeasuredItem("tripped_high", "26 V mode tripped", kind="categorical"),
        ],
    ),
    PhaseDef(
        number=11, key="thermal", name="Thermal",
        short="Thermistor / thermal behaviour under load", power="full",
        parallelism="8 per section", requires_calibration=True,
        description=("Take a fresh baseline temperature, run the motor at full "
                     "power while logging temperature (cmd 42) every second for "
                     "the configured duration. Full series stored as a blob."),
        params=[
            Param("duration_s", "Thermal run duration", "float", 120.0, "s"),
            Param("max_current", "Motor current", "int", 200),
            Param("spin_velocity", "Spin velocity", "float", 1.0, "rot/s"),
        ],
        criteria=[
            Param("rise_min", "Temperature rise min", "float", 5.0, "°C"),
            Param("rise_max", "Temperature rise max", "float", 20.0, "°C"),
            Param("start_temp_min", "Best-fit start temp min", "float", 10.0, "°C"),
            Param("start_temp_max", "Best-fit start temp max", "float", 45.0, "°C"),
            Param("slope_min", "Best-fit slope min", "float", 0.0, "°C/s"),
            Param("slope_max", "Best-fit slope max", "float", 1.0, "°C/s"),
            Param("r_value_min", "Best-fit R minimum", "float", 0.9),
        ],
        measured=[
            MeasuredItem("temp_rise", "Temperature rise", unit="°C",
                         threshold_keys=["rise_min", "rise_max"]),
            MeasuredItem("fit_start_temp", "Best-fit start temperature", unit="°C",
                         threshold_keys=["start_temp_min", "start_temp_max"]),
            MeasuredItem("fit_slope", "Best-fit slope", unit="°C/s",
                         threshold_keys=["slope_min", "slope_max"]),
            MeasuredItem("fit_r", "Best-fit R value",
                         threshold_keys=["r_value_min"]),
        ],
    ),
    PhaseDef(
        number=12, key="openloop_burnin", name="Open-loop burn-in",
        short="Full motion chain under sustained load", power="full",
        parallelism="random 8 per section", requires_calibration=True,
        description=("Repeatedly pick 8 random motors and spin each a random "
                     "distance/direction for a fixed interval, for the full "
                     "duration. A tight deviation limit (cmd 44) catches "
                     "skipped steps. Any fatal error fails the motor."),
        params=[
            Param("duration_s", "Burn-in duration", "float", 12600.0, "s",
                  "Default 3.5 hours."),
            Param("spin_interval_s", "Spin interval", "float", 5.0, "s"),
            Param("deviation_tolerance_rot", "Deviation tolerance (cmd 44)",
                  "float", 0.01, "rot"),
            Param("max_current", "Motor current", "int", 200),
        ],
        criteria=[],   # pass = no fatal error during the phase
        measured=[
            MeasuredItem("no_fatal_error", "No fatal error", kind="categorical"),
        ],
    ),
    PhaseDef(
        number=13, key="closedloop_burnin", name="Closed-loop burn-in",
        short="Closed-loop servo performance under load", power="low",
        parallelism="all 48 per section, staggered", requires_calibration=True,
        description=("All motors run closed-loop for the full duration, many "
                     "random trapezoid moves each. After each move (move "
                     "duration + 5 %) read max PID error (cmd 39, read-and-reset)."
                     " Full per-move PID series stored as a blob."),
        params=[
            Param("duration_s", "Burn-in duration", "float", 12600.0, "s",
                  "Default 3.5 hours."),
            Param("move_duration_s", "Trapezoid move duration", "float", 5.0, "s"),
            Param("quiet_margin_pct", "Post-move quiet margin", "float", 5.0, "%"),
            Param("max_move_magnitude_rot", "Max move magnitude", "float", 5.0, "rot"),
        ],
        criteria=[
            Param("max_pid_deviation_threshold", "Max PID deviation threshold",
                  "float", 100000.0, help="Overall max PID error must stay below."),
        ],
        measured=[
            # Histogram the full per-move PID-deviation distribution (the
            # evaluator also checks the overall max against the threshold).
            MeasuredItem("pid_series", "PID deviation (per move)",
                         threshold_keys=["max_pid_deviation_threshold"]),
            MeasuredItem("no_fatal_error", "No fatal error", kind="categorical"),
        ],
    ),
    PhaseDef(
        number=14, key="set_alias", name="Set factory-default alias",
        short="MCU settings (alias)", power="none",
        parallelism="broadcast set; per-device readback",
        description=("Broadcast set-alias to 'X' (cmd 21), wait for the "
                     "auto-reboot, then read the alias back from each device by "
                     "unique ID (cmd 22)."),
        params=[
            Param("factory_alias", "Factory alias", "str", "X"),
            Param("reboot_delay_s", "Post-set reboot delay", "float", 1.0, "s"),
        ],
        criteria=[],   # pass = alias read back equals factory_alias
        measured=[
            MeasuredItem("alias", "Alias read back", kind="categorical"),
        ],
    ),
    PhaseDef(
        number=15, key="led_test", name="LED test", short="Green + red LEDs",
        power="LEDs only", parallelism="command all, then human walk-through",
        description=("Drive both LEDs solid on via the firmware LED test mode "
                     "(cmd 36, modes 10-13). This LOCKS the motor until a power "
                     "cycle, so it must be the LAST phase. A human confirms; on "
                     "failure the removal-and-ping reconciliation runs."),
        params=[],
        criteria=[],   # pass = human confirmation "yes"
        measured=[
            MeasuredItem("leds_lit", "LEDs lit (human confirm)", kind="categorical"),
        ],
    ),
]

PHASES_BY_NUMBER: Dict[int, PhaseDef] = {p.number: p for p in PHASES}
PHASES_BY_KEY: Dict[str, PhaseDef] = {p.key: p for p in PHASES}


def all_phases() -> List[PhaseDef]:
    return PHASES


def get_phase(number: int) -> PhaseDef:
    return PHASES_BY_NUMBER[number]
