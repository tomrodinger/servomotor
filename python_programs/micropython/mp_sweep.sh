#!/bin/bash
# Drives mp_runner.py through the test list, machine-resetting between chunks
# so heap fragmentation doesn't kill later tests. Reads progress from
# /results.txt on the device.

set -u
PORT="${MP_PORT:-/dev/cu.usbmodem1201}"
LOG="${MP_LOG:-/tmp/mp_full_sweep.log}"
MAX_CHUNKS="${MP_MAX_CHUNKS:-30}"

echo "Sweep starting on $PORT; logging to $LOG; max chunks=$MAX_CHUNKS"
: > "$LOG"

for i in $(seq 1 "$MAX_CHUNKS"); do
    echo "===== chunk $i =====" | tee -a "$LOG"
    mpremote connect "$PORT" run /tmp/mp_runner.py 2>&1 | tee -a "$LOG"
    status=${PIPESTATUS[0]}

    if grep -q "All tests already done" "$LOG"; then
        echo "Sweep complete after chunk $i."
        break
    fi

    if [ $status -ne 0 ]; then
        echo "WARN: mpremote returned $status; resetting and retrying."
    fi

    # Hard-reset the device to free RAM / clear any peripheral state.
    echo "----- machine.reset between chunks -----" | tee -a "$LOG"
    mpremote connect "$PORT" exec "import machine; machine.reset()" >/dev/null 2>&1 || true
    sleep 4
done

echo "===== sweep finished =====" | tee -a "$LOG"
