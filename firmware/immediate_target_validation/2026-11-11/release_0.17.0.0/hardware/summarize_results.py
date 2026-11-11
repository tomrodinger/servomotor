"""Summarize completed canary runs without opening a serial port."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re

OUT = Path(__file__).resolve().parent
ROOT = Path(__file__).resolve().parents[5]
def read(name):
    return json.loads((OUT / name).read_text())

run = read('physical_run.json')
upgrade = read('upgrade_canary.json')
acceptance = read('canary_extended.json')
planner = read('planner_followup.json')
soak = read('canary_soak.json')
timing = read('timing_windows.json')
before = read('inventory_before.json')
final = read('final_verification.json')
for report in (run, upgrade, acceptance, planner, soak, timing, before, final):
    assert report['passed']
for report in (acceptance, planner, soak, timing):
    assert report['cleanup_passed']
assert all(row['returncode'] == 0 for row in run['stages'])
assert acceptance['firmware'] == [[0,0,17,0],0]
assert acceptance['stress']['transactions'] == 10000
assert set(acceptance['fatal_errors']) == {8,13,19,27,45,51,55}
assert all(len(bytes.fromhex(row['rx'])) == (15 if row['crc32'] else 11) for row in acceptance['wire_samples'] if row['command']==50)
assert len(planner['moves']) == 16
assert len(soak['phases']) == 2
cpp = (OUT / 'cpp.log').read_text()
match = re.search(r'PASSED: (\d+) checks; (\d+) command-50 transactions', cpp)
assert match
other_before = {d['unique_id']:d for d in before['devices'] if d['unique_id'] != upgrade['unique_id']}
other_final = {d['unique_id']:d for d in final['devices'] if d['unique_id'] != upgrade['unique_id']}
assert other_final == other_before
assert len(final['devices']) == 27
canary = next(d for d in final['devices'] if d['unique_id'] == upgrade['unique_id'])
assert canary['firmware'] == [[0,0,17,0],0]
assert canary['status'] == [0,0] and canary['queue_depth'] == 0
assert canary['pid_constants'] == [2000,5,175000]
assert canary['communication_statistics'] == [0]*6
peaks = [acceptance['profiler_after'][10]] + [p['peak_control_isr_us'] for p in soak['phases']] + [p['calculation_max_us'] for p in timing['steady_retargeting']]
assert max(peaks) < 32
soak_updates = sum(p['command_49']+p['command_50'] for p in soak['phases'])
known_updates = acceptance['stress']['transactions'] + len(planner['moves']) + int(match[2]) + soak_updates + timing['steady_target_updates'] + timing['warmup_target_updates']
summary = {
 'timestamp_utc':datetime.now(timezone.utc).isoformat(),
 'firmware':'0.17.0.0','status':'Canary physical functional validation passed',
 'tested_uid':upgrade['unique_id'],'tested_motor_count':1,
 'only_opened_port':upgrade['port'],'adapter_jig_opened':False,
 'image_sha256':upgrade['image_sha256'],
 'responding_inventory_count':len(final['devices']),
 'other_responding_motors_unchanged':len(other_final),
 'reference_updates_at_least':known_updates,
 'requested_current_limits_internal_units':[50,50],
 'extended_python':{'transactions':acceptance['stress']['transactions'],
  'crc_split':acceptance['stress']['crc_transactions'],
  'fatal_codes_checked':sorted(set(acceptance['fatal_errors'])),
  'stress_communication_statistics':acceptance['communication_statistics_after_stress'],
  'peak_calculation_us':acceptance['profiler_after'][10],
  'peak_end_to_end_interval_us':acceptance['profiler_after'][20],
  'settled_positions':acceptance['settled_positions'],
  'wire_samples':acceptance['wire_samples']},
 'planner_followup':{'moves':len(planner['moves']),'both_commands_and_crc_modes':True,
  'communication_statistics':planner['communication_statistics_after']},
 'arduino_cpp':{'checks':int(match[1]),'command_50_transactions':int(match[2]),
  'coverage':'Native C++ Arduino bindings through real serial transport: raw/converted positions, configured/explicit UID, both CRC modes, tracking, full-width i64 rejection and cleanup'},
 'mixed_soak':{'reference_updates':soak_updates,'command_37_reads':sum(p['command_37'] for p in soak['phases']),'phases':soak['phases']},
 'timing':{'peak_calculation_us':max(peaks),'steady_windows':len(timing['steady_retargeting']),
  'disabled_idle_summary':timing['disabled_idle_summary'],
  'enable_transition':timing['enable_transition'],
  'steady_retargeting_summary':timing['steady_retargeting_summary'],
  'strict_isr_entry_interval_bound_established':False,'metric_note':timing['metric_note']},
 'final_canary':canary,
 'limits':['One canary only. No loaded motion, physical power-cycle or external ISR-entry timestamp capture.',
  'Physical UART tests check Hall data, framing and behavior; host interrupt-race tests and ARM disassembly establish the separate atomic write/read implementation.',
  'No browser hardware test was run in this acceptance. Package publication is separate.']}
(OUT / 'hardware_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
interval=timing['steady_retargeting_summary']['end_to_end_interval_max_us']
text=f'''0.17.0.0 separate target/Hall copies: physical canary validation

RESULT
PASSED on UID {upgrade['unique_id']}, upgraded from 0.17.0.6 to 0.17.0.0.
Image SHA256: {upgrade['image_sha256']}
Only {upgrade['port']} opened under the verified three-adapter motor-rack hub.
The adapter-testing jig and all other serial adapters were not opened.

COVERAGE
At least {known_updates:,} accepted immediate-reference updates.
- Python: 10,000 target/position stress transactions, half in each CRC mode; exact command 49/50 frames, tracking, PID preservation, fatal errors 8/13/19/27/45/51/55 and recovery.
- Planner: 16 timed moves directly after nonzero immediate references without reset/zero between the commands, both commands and CRC modes.
- Native C++ Arduino library: {match[1]} checks, {match[2]} command-50 transactions; raw/converted/UID overloads, CRC modes, i64 guards and cleanup.
- Mixed soak: {soak_updates:,} updates and 200 comprehensive-position reads; queue/output/reference/PID consistency, holding and communication counters.
- Timing: 100 warmup and 3000 steady updates over 30 windows; disabled and enable-transition measurements.
Requested current limit: 50 internal units each (about 256 mA by library conversion, not measured phase current). Reference range: +/-1 degree.

TIMING
Peak observed controller calculation: {max(peaks)} us (nominal period 32 us).
Steady-window maximum end-of-ISR intervals: {interval['min']}-{interval['max']} us, mean maximum {interval['mean']:.3f} us.
The existing interval profiler samples near ISR end, so these are not ISR-entry timestamps and do not establish a strict 32-us ISR-entry interval bound. Exact sampled/window values are in timing_windows.json.

FINAL STATE
Canary 0.17.0.0; status [0,0], queue 0, PID [2000,5,175000], CRC enabled, outputs disabled.
Communication error counters stayed unchanged during tests and were zero at final verification.
All 27 currently responding rack motors were verified; the other 26 have unchanged firmware, product data, PID, status, queue and error counters compared with preflight.

LIMITS
One canary only. No loaded-motion, physical power-cycle, browser or external ISR-entry timing test in this acceptance. UART behavior does not resolve the exact 32-us control-tick phase; host race checks and disassembly establish separate atomic writes and reads. Firmware publication is tracked by the release manifest.

EVIDENCE
hardware_summary.json, inventory_before.json, upgrade_canary.json, final_verification.json
canary_extended.json, planner_followup.json, canary_soak.json, cpp.log, timing_windows.json
physical_run.json, usb_scope.json, usb_topology.txt, test_source_hashes.json
'''
(OUT / 'HARDWARE_RESULTS.txt').write_text(text)
print(text)
