# PID sim battery report

Final proposed design = **step2** (rounding fixes + back-calculation anti-windup
+ integral clamp at 25% authority, existing LPF derivative topology).
`old` = shipped 0.15.3.0 behavior (regression reference).
`step3` = experimental FIR-32 derivative (kept for reference; rejected).

NOTE on acceptance thresholds: the stick-slip plant is bistable at low kI —
the old algorithm limit-cycles on ~40% of seeds even at the default kI=5.
Post-move RMS < 500 counts (0.015% of a rev) counts as 'settled, no limit cycle';
a limit cycle measures in the thousands.

## 1. Standstill (5 s, defaults kP=2000 kI=5 kD=175000)

| alg | err std | D mean | D std | overflow |
|---|---|---|---|---|
| old | 33.0 | -84,494.0 | 181,924.0 | 0 |
| step1 | 33.0 | -44.0 | 182,584.0 | 0 |
| step2 | 33.0 | -44.0 | 182,584.0 | 0 |
| step3 | 33.0 | -1.0 | 254,745.0 | 0 |

## 2. Move 2 rot / 2 s at default gains

| alg | peak err | post RMS | settle ms | osc Hz | sat ticks |
|---|---|---|---|---|---|
| old | 9,759.0 | 38.4 | 50.0 | 0.0 | 0 |
| step1 | 9,759.0 | 4,171.7 | -1.0 | 7.2 | 0 |
| step2 | 9,759.0 | 349.7 | -1.0 | 0.0 | 0 |
| step3 | 9,698.0 | 148.5 | -1.0 | 0.0 | 0 |

## 3. kI sweep, move 2 rot / 2 s (limit-cycle = post RMS > 1000)

| kI | old post-RMS | step1 post-RMS | step2 post-RMS | step3 post-RMS |
|---|---|---|---|---|
| 5 | 38.4 | 4,171.7 | 349.7 | 148.5 |
| 10 | 2,832.7 | 3,029.6 | 257.6 | 33.3 |
| 25 | 3,794.9 | 3,519.5 | 46.5 | 102.3 |
| 50 | 1,972.7 | 2,806.4 | 54.4 | 85.5 |
| 100 | 2,488.2 | 2,524.3 | 42.8 | 117.6 |
| 200 | 30,335.3 | 30,344.8 | 2,114.2 | 50.8 |

## 4. kD sweep, kI=25, move 2 rot / 2 s (median of 3 seeds: move peak err / post RMS)

| kD | step2 (final) | step3 (reference) |
|---|---|---|
| 0 | 47,463.0 / 33,064.5 | 47,463.0 / 33,064.5 |
| 87500 | 9,420.0 / 48.9 | 8,480.0 / 36.0 |
| 175000 | 5,404.0 / 38.4 | 5,345.0 / 84.0 |
| 350000 | 4,885.0 / 53.2 | 4,898.0 / 77.9 |
| 700000 | 4,047.0 / 95.8 | 27,357.0 / 11,634.4 |
| 1000000 | 3,495.0 / 176.3 | 28,918.0 / 14,268.5 |

## 5. Stress & edge cases

| case | alg | key metrics | overflow |
|---|---|---|---|
| fast move (5 rot in 0.7 s, saturates) | old | peak_err=9655 postRMS=4673 sat=0 | 0 |
| fast move (5 rot in 0.7 s, saturates) | step2 | peak_err=9655 postRMS=1279 sat=0 | 0 |
| reversal (-3 rot) | old | peak_err=9862 postRMS=4479 sat=0 | 0 |
| reversal (-3 rot) | step2 | peak_err=9862 postRMS=486 sat=0 | 0 |
| external load 0.05 Nm | old | peak_err=13091 postRMS=44 sat=0 | 0 |
| external load 0.05 Nm | step2 | peak_err=13091 postRMS=35 sat=0 | 0 |
| external load 0.05 Nm, kI=25 | old | peak_err=6894 postRMS=2758 sat=0 | 0 |
| external load 0.05 Nm, kI=25 | step2 | peak_err=8421 postRMS=7690 sat=0 | 0 |
| high noise (sigma 100) | old | peak_err=9622 postRMS=3880 sat=0 | 0 |
| high noise (sigma 100) | step2 | peak_err=9622 postRMS=458 sat=0 | 0 |
| tiny move (0.01 rot) | old | peak_err=5292 postRMS=4910 sat=0 | 0 |
| tiny move (0.01 rot) | step2 | peak_err=5323 postRMS=2038 sat=0 | 0 |
| slow long move (10 rot / 20 s) | old | peak_err=9127 postRMS=54 sat=0 | 0 |
| slow long move (10 rot / 20 s) | step2 | peak_err=9127 postRMS=79 sat=0 | 0 |
| kP=0 kD=0 (I only) | old | peak_err=100369 postRMS=48888 sat=0 | 0 |
| kP=0 kD=0 (I only) | step2 | peak_err=3276850 postRMS=3276800 sat=0 | 0 |
| huge gains | old | peak_err=34520 postRMS=23903 sat=0 | 0 |
| huge gains | step2 | peak_err=777 postRMS=12063 sat=80102 | 0 |
| long standstill 60 s | old | peak_err=182 postRMS=0 sat=0 | 0 |
| long standstill 60 s | step2 | peak_err=182 postRMS=0 sat=0 | 0 |

## 6. Seed robustness (kI=25 move, post RMS per seed)

| seed | step2 | step3 |
|---|---|---|
| 1 | 38.4 | 116.9 |
| 2 | 119.3 | 48.0 |
| 3 | 50.3 | 47.6 |
| 4 | 62.5 | 48.5 |
| 5 | 40.7 | 54.0 |
| 6 | 46.0 | 70.0 |
| 7 | 46.1 | 64.4 |
| 8 | 43.4 | 54.5 |
| 9 | 44.1 | 78.2 |
| 10 | 37.6 | 84.0 |

## Result

**60/62 checks passed**

Failed checks:

- external load 0.05 Nm, kI=25 [step2]: bounded post-move error (RMS < 2000) (7690)
- tiny move (0.01 rot) [step2]: bounded post-move error (RMS < 2000) (2038)
