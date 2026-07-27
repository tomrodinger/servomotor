#include "test_registry.h"

// Execution order rationale:
//  - pure-computation tests first (no motor needed),
//  - cheap reads next, then calibration EARLY so every later closed-loop /
//    position-dependent test runs against a freshly calibrated motor
//    (mirrors the Python suite's ordering),
//  - motion depth, stress, and behavior tests in the middle,
//  - fleet tests (multi_device, broadcast) near the end,
//  - addressing LAST (it changes the alias + reboots the motor twice).
// Timeouts are generous: a timeout marks the test CRASH and the suite moves on.
const TestDef TF_TESTS[] = {
    //  name                    fn                        knownBugs         timeout  needsMotor
    { "unit_conversions",      tm_unit_conversions,      "",                  60, false },
    { "units_roundtrip",       tm_units_roundtrip,       "",                  60, false },
    { "product_info",          tm_product_info,          "",                  90, true  },
    { "temperature",           tm_temperature,           "",                  60, true  },
    { "telemetry",             tm_telemetry,             "",                 150, true  },
    { "system_status",         tm_system_status,         "",                 150, true  },
    { "status_matrix",         tm_status_matrix,         "",                  90, true  },
    { "comms",                 tm_comms,                 "",                 150, true  },
    { "ping_stress",           tm_ping_stress,           "",                  90, true  },
    { "wrong_addressing",      tm_wrong_addressing,      "",                  60, true  },
    { "time",                  tm_time,                  "",                 150, true  },
    { "calibration",           tm_calibration,           "",                 240, true  },
    { "diagnostics",           tm_diagnostics,           "",                 240, true  },
    { "motion_basic",          tm_motion_basic,          "",                 240, true  },
    { "move_velocity",         tm_move_velocity,         "",                 120, true  },
    { "move_accel",            tm_move_accel,            "",                 120, true  },
    { "multimove",             tm_multimove,             "",                 240, true  },
    { "fast_short_moves",      tm_fast_short_moves,      "",                 120, true  },
    { "queue_stress",          tm_queue_stress,          "",                 150, true  },
    { "emergency_stop",        tm_emergency_stop,        "",                  90, true  },
    { "comm_during_motion",    tm_comm_during_motion,    "",                 120, true  },
    { "velocity_accuracy",     tm_velocity_accuracy,     "",                 120, true  },
    { "voltage_under_load",    tm_voltage_under_load,    "",                  90, true  },
    { "settings",              tm_settings,              "",                 240, true  },
    { "safety_pid",            tm_safety_pid,            "",                 240, true  },
    { "safety_fence",          tm_safety_fence,          "",                 150, true  },
    { "pid_dynamics",          tm_pid_dynamics,          "",                 180, true  },
    { "closed_loop",           tm_closed_loop,           "",                 180, true  },
    { "homing",                tm_homing,                "",                 300, true  },
    { "comprehensive_position",tm_comprehensive_position,"",                  90, true  },
    { "actuators",             tm_actuators,             "",                 150, true  },
    { "error_injection",       tm_error_injection,       "",                 150, true  },
    { "varlen_bugs",           tm_varlen_bugs,           "",                 240, true  },
    { "enable_reliability",    tm_enable_reliability,    "",                 120, true  },
    { "multi_device",          tm_multi_device,          "",                 300, true  },
    { "broadcast",             tm_broadcast,             "",                 150, true  },
    { "addressing",            tm_addressing,            "",                 240, true  },

    // ---- Round 3: edge-case / bug-hunting modules ----
    // Numeric / queue / protocol boundaries first (mostly no motion), then
    // state-machine + physical edges, then the heavier real-usage patterns.
    { "overflow_guards",       tm_overflow_guards,       "",                 120, true  },
    { "float_edges",           tm_float_edges,           "",                 150, true  },
    { "dwell_and_tiny",        tm_dwell_and_tiny,        "",                 150, true  },
    { "queue_edges",           tm_queue_edges,           "",                 300, true  },
    { "multimove_edges",       tm_multimove_edges,       "",                 120, true  },
    { "limit_boundaries",      tm_limit_boundaries,      "",                 150, true  },
    { "goto_edges",            tm_goto_edges,            "",                 180, true  },
    { "position_frames",       tm_position_frames,       "",                 240, true  },
    { "raw_protocol",          tm_raw_protocol,          "",                 120, true  },
    { "payload_size_attacks",  tm_payload_size_attacks,  "",                 120, true  },
    { "bad_command_byte",      tm_bad_command_byte,      "",                  90, true  },
    { "uid_edges",             tm_uid_edges,             "",                 120, true  },
    { "timeout_recovery",      tm_timeout_recovery,      "",                 180, true  },
    { "rapid_fire",            tm_rapid_fire,            "",                 300, true  },
    { "mosfet_interactions",   tm_mosfet_interactions,   "",                 180, true  },
    { "settings_persistence",  tm_settings_persistence,  "",                 240, true  },
    { "limit_reset_edges",     tm_reset_edges,           "",                 180, true  },
    { "fatal_state_matrix",    tm_fatal_state_matrix,    "",                 150, true  },
    { "calibration_rejections",tm_calibration_rejections,"",                 180, true  },
    { "homing_edges",          tm_homing_edges,          "",                 180, true  },
    { "bootloader_window",     tm_bootloader_window,     "",                 120, true  },
    { "version_info",          tm_version_info_consistency,"",                60, true  },
    { "sensor_consistency",    tm_sensor_consistency,    "",                 180, true  },
    { "actuator_edges",        tm_actuator_edges,        "",                 150, true  },
    { "alias_edges",           tm_alias_edges,           "",                 300, true  },
    { "two_motors",            tm_two_motors,            "",                 300, true  },
};

const int TF_N_TESTS = sizeof(TF_TESTS) / sizeof(TF_TESTS[0]);
