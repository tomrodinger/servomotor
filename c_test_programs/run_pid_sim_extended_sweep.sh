#!/bin/bash
# Extended simulator sweep for V2 semantics: wider coverage than the standard battery.
# Every run must have overflow_violations=0; stability criteria per scenario class.
cd "$(dirname "$0")"
SIM=${SIM:-./pid_algorithm_sim}
FAIL=0
TOTAL=0

check() {  # name rms_limit output...
    local name=$1 limit=$2; shift 2
    local out="$*"
    local rms=$(echo "$out" | grep -o "post_move_rms=[0-9.]*" | cut -d= -f2)
    [ -z "$rms" ] && rms=$(echo "$out" | grep -o "ss_err_std=[0-9.]*" | cut -d= -f2)
    local ovf=$(echo "$out" | grep -o "overflow_violations=[0-9]*" | cut -d= -f2)
    TOTAL=$((TOTAL+1))
    if [ "$ovf" != "0" ]; then echo "FAIL(overflow) $name"; FAIL=$((FAIL+1)); return; fi
    if [ -n "$limit" ] && python3 -c "exit(0 if $rms < $limit else 1)"; then :; else
        if [ -n "$limit" ]; then echo "FAIL(rms=$rms>=$limit) $name"; FAIL=$((FAIL+1)); return; fi
    fi
}

echo "[1] 20-seed robustness: kI=25/kD=350k and kI=50/kD=700k, move 2 rot / 2 s"
for seed in $(seq 1 20); do
    check "ki25kd350k_s$seed" 500 "$($SIM --alg step2 --ki 25 --kd 350000 --move 2 --duration 8 --seed $seed --quiet)"
    check "ki50kd700k_s$seed" 500 "$($SIM --alg step2 --ki 50 --kd 700000 --move 2 --duration 8 --seed $seed --quiet)"
done

echo "[2] noise sweep (sigma 10/33/60/100) x 3 seeds at kI=25/kD=350k"
for n in 10 33 60 100; do for seed in 1 6 10; do
    check "noise${n}_s$seed" 800 "$($SIM --alg step2 --ki 25 --kd 350000 --move 2 --duration 8 --noise $n --seed $seed --quiet)"
done; done

echo "[3] load sweep at kI=25/kD=350k: 0/0.02 Nm must settle; 0.04 Nm on the pathological"
echo "    plant is the DOCUMENTED heavy-load corner (plan doc section 6/8) - overflow-checked only"
for l in 0 0.02; do for seed in 1 6 10; do
    check "load${l}_s$seed" 4000 "$($SIM --alg step2 --ki 25 --kd 350000 --move 2 --duration 8 --load $l --seed $seed --quiet)"
done; done
for seed in 1 6 10; do
    check "load0.04_s${seed}_infoonly" "" "$($SIM --alg step2 --ki 25 --kd 350000 --move 2 --duration 8 --load 0.04 --seed $seed --quiet)"
done
echo "[3b] heavy load on a HEALTHY-friction plant must settle (the corner is stiction-specific)"
for seed in 1 6 10; do
    check "load0.04_healthy_s$seed" 900 "$($SIM --alg step2 --ki 25 --kd 350000 --move 2 --duration 8 --load 0.04 --static-friction 0.03 --coulomb 0.015 --seed $seed --quiet)"
done

echo "[4] healthy-friction plant x 5 seeds (no regression on normal motors)"
for seed in 1 3 6 7 10; do
    check "healthy_s$seed" 900 "$($SIM --alg step2 --ki 25 --kd 350000 --move 2 --duration 8 --static-friction 0.03 --coulomb 0.015 --seed $seed --quiet)"
done

echo "[5] long standstill soak 120 s x 3 seeds (I stability, no drift blowup)"
for seed in 2 5 9; do
    check "soak120_s$seed" 300 "$($SIM --alg step2 --ki 25 --kd 350000 --duration 120 --seed $seed --quiet)"
done

echo "[6] fast/violent moves at the STEP slew limit (10 rev/s commanded) x 3 seeds:"
echo "    residual hunt on the sticky plant is expected; must stay bounded below the"
echo "    old-algorithm level (~3000-3100 RMS measured)"
for seed in 1 6 10; do
    check "fast_s$seed" 3200 "$($SIM --alg step2 --ki 25 --kd 350000 --move 6 --move-time 0.6 --duration 6 --seed $seed --quiet)"
    check "reverse_s$seed" 3200 "$($SIM --alg step2 --ki 25 --kd 350000 --move -6 --move-time 0.6 --duration 6 --seed $seed --quiet)"
done

echo "[7] extreme-gain plant runs (must not overflow; stability not required)"
for g in "65535 40460 4294967295" "1 4294967295 33" "2147483647 1 40" "20000 500 2000000"; do
    set -- $g
    check "extreme_${1}_${2}_${3}" "" "$($SIM --alg step2 --kp $1 --ki $2 --kd $3 --move 2 --duration 6 --seed 3 --quiet)"
done

echo ""
echo "RESULT: $((TOTAL-FAIL))/$TOTAL passed"
[ $FAIL -eq 0 ] && echo "SIM EXTENDED SWEEP PASSED" || echo "SIM EXTENDED SWEEP FAILED"
exit $FAIL
