#pragma once
#include <cstdint>

// One entry per test module. `knownBugs` non-empty means the test contains
// assertions that are EXPECTED to fail until the named registry bugs are fixed
// (they assert the CORRECT behavior on purpose). `timeoutSec` is the per-test
// watchdog: if the test hangs longer than this, the harness force-reboots and
// records the test as CRASH.
struct TestDef {
    const char* name;
    void (*fn)();
    const char* knownBugs;   // "" = a failure here is unexpected
    uint16_t timeoutSec;
    bool needsMotor;         // false = pure computation (runs without a motor)
};

extern const TestDef TF_TESTS[];
extern const int TF_N_TESTS;

// Module entry points (one tm_*.cpp each)
void tm_unit_conversions();
void tm_units_roundtrip();
void tm_product_info();
void tm_temperature();
void tm_telemetry();
void tm_system_status();
void tm_status_matrix();
void tm_comms();
void tm_ping_stress();
void tm_wrong_addressing();
void tm_time();
void tm_calibration();
void tm_diagnostics();
void tm_motion_basic();
void tm_move_velocity();
void tm_move_accel();
void tm_multimove();
void tm_fast_short_moves();
void tm_queue_stress();
void tm_emergency_stop();
void tm_comm_during_motion();
void tm_velocity_accuracy();
void tm_voltage_under_load();
void tm_settings();
void tm_safety_pid();
void tm_safety_fence();
void tm_pid_dynamics();
void tm_closed_loop();
void tm_homing();
void tm_comprehensive_position();
void tm_actuators();
void tm_error_injection();
void tm_varlen_bugs();
void tm_enable_reliability();
void tm_multi_device();
void tm_broadcast();
void tm_addressing();
// Round 3 — edge-case / bug-hunting modules
void tm_overflow_guards();
void tm_multimove_edges();
void tm_queue_edges();
void tm_raw_protocol();
void tm_payload_size_attacks();
void tm_bad_command_byte();
void tm_float_edges();
void tm_alias_edges();
void tm_uid_edges();
void tm_rapid_fire();
void tm_mosfet_interactions();
void tm_settings_persistence();
void tm_calibration_rejections();
void tm_homing_edges();
void tm_limit_boundaries();
void tm_bootloader_window();
void tm_version_info_consistency();
void tm_position_frames();
void tm_two_motors();
void tm_planner_arithmetic();
void tm_move_composition();
void tm_unit_equivalence();
void tm_queue_accounting();
void tm_setting_timing();
void tm_determinism();
void tm_error_taxonomy();
void tm_position_accumulator();
void tm_crc32_matrix();
void tm_multimove_bitmask();
void tm_stop_semantics();
void tm_time_domain();
void tm_fatal_state_commands();
void tm_queue_underrun();
void tm_readonly_purity();
void tm_move_type_equivalence();
void tm_soak();
void tm_command_state_matrix();
void tm_duration_extremes();
void tm_reset_completeness();
void tm_deviation_tracking();
void tm_timeout_recovery();
void tm_actuator_edges();
void tm_sensor_consistency();
void tm_fatal_state_matrix();
void tm_dwell_and_tiny();
void tm_goto_edges();
void tm_reset_edges();

void tm_acceleration_move_semantics();
void tm_current_and_pid_settings();
void tm_device_identity();
void tm_diagnostics_readouts();
void tm_goto_absolute_semantics();
void tm_hall_capture();
void tm_multimove_capacity();
void tm_safety_fence_arithmetic();
void tm_telemetry_conversions();
void tm_velocity_move_semantics();
void tm_zero_position_semantics();
void tm_communication_statistics();
void tm_indicator_commands();
void tm_multipurpose_buffer();
void tm_packet_size_encoding();
void tm_position_source_consistency();
void tm_queue_ordering();
void tm_status_flag_semantics();
void tm_time_sync_protocol();
void tm_boundary_value_sweep();
void tm_command_latency_profile();
void tm_concurrent_command_pressure();
void tm_error_code_reachability();
void tm_firmware_compat_gating();
void tm_homing_arithmetic();
void tm_motion_timing_precision();
void tm_position_far_field();
void tm_power_on_defaults();
void tm_realistic_motion_patterns();
void tm_recovery_sequences();
void tm_repeated_command_idempotence();
void tm_response_field_integrity();
void tm_settings_interaction_matrix();
void tm_streaming_motion();
void tm_unit_extreme_values();