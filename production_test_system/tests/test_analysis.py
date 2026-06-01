from backend.simulator import SimMotor, MotorProfile
from backend.analysis import hall, thermal


def test_hall_good_vs_dead_magnet():
    good = SimMotor(0x1, profile=MotorProfile())
    res = hall.analyze_capture(good._synth_hall_waveform(4000), threshold=8000)
    for ch in range(3):
        m = res["channels"][ch]
        assert 68 <= m["extrema_count"] <= 71          # ~70 over 1.4 rotations
        assert m["span"] > 40000
        assert m["max_peak_from_avg"] < 2000           # good: peaks uniform

    dead = SimMotor(0x2, profile=MotorProfile(dead_magnet=True))
    res2 = hall.analyze_capture(dead._synth_hall_waveform(4000), threshold=8000)
    assert res2["channels"][0]["max_peak_from_avg"] > 2000   # damaged magnet caught


def test_peak_finder_alternates():
    samples = []
    import math
    for i in range(4000):
        samples.append(int(32000 + 25000 * math.sin(2 * math.pi * 35 * i / 4000)))
    ex = hall.find_extrema(samples, 8000)
    kinds = [k for k, _, _ in ex]
    # extrema strictly alternate peak/valley
    for a, b in zip(kinds, kinds[1:]):
        assert a != b


def test_thermal_fit():
    series = [[float(i), 25.0 + 0.5 * i] for i in range(20)]
    res = thermal.analyze(25.0, series)
    assert abs(res["fit_slope"] - 0.5) < 1e-6
    assert abs(res["fit_start_temp"] - 25.0) < 1e-6
    assert res["fit_r"] > 0.999
    assert abs(res["temp_rise"] - (series[-1][1] - 25.0)) < 1e-6
