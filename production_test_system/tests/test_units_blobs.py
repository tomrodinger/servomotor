from backend import units, blobs


def test_unit_roundtrips():
    assert units.rotations_to_counts(1.0) == round(units.COUNTS_PER_ROTATION)
    assert abs(units.counts_to_rotations(units.rotations_to_counts(1.5)) - 1.5) < 1e-6
    assert units.seconds_to_timesteps(1.0) == round(units.TIMESTEPS_PER_SECOND)
    assert abs(units.raw_voltage_to_volts(240) - 24.0) < 1e-9


def test_position_blob_roundtrip():
    samples = [(0, 100, 90), (0, 200, 195), (1, 50, 48)]
    out = blobs.unpack_position_stream(blobs.pack_position_stream(samples))
    assert out == samples


def test_temperature_blob_roundtrip():
    samples = [(0.0, 25.0), (1.0, 26.5), (2.0, 28.0)]
    out = blobs.unpack_temperature_series(blobs.pack_temperature_series(samples))
    assert len(out) == 3
    assert abs(out[1][1] - 26.5) < 1e-4


def test_pid_blob_roundtrip():
    samples = [(-10, 20), (-5, 5), (0, 100)]
    out = blobs.unpack_pid_series(blobs.pack_pid_series(samples))
    assert out == samples
