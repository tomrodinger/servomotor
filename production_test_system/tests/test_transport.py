from backend.simulator import RackSimulator, SimMotor, MotorProfile
from backend.motor_client import MotorClient
from backend import units


def _rack():
    rack = RackSimulator()
    rack.add_motor("A", SimMotor(0x1122334455667788, serial_number=7))
    rack.add_motor("A", SimMotor(0xAABBCCDDEEFF0011,
                                 profile=MotorProfile(firmware_version="0.14.0.0")))
    return rack


def test_detection_returns_all():
    t = _rack().transport_for("A")
    det = t.transact(255, "Detect devices")
    ids = sorted(u for u, a in det)
    assert ids == sorted([0x1122334455667788, 0xAABBCCDDEEFF0011])


def test_identity_and_firmware():
    t = _rack().transport_for("A")
    c = MotorClient(t, 0x1122334455667788)
    assert c.get_firmware_version() == "0.15.0.0"
    info = c.get_product_info()
    assert info["product_type"] == "M17" and info["hw_version"] == "1.5" and info["scc"] == 3
    assert c.ping(b"0123456789") == b"0123456789"


def test_weak_current_does_not_move():
    rack = _rack()
    t = rack.transport_for("A")
    c = MotorClient(t, 0x1122334455667788)
    c.set_max_current(5)            # below stall threshold
    c.enable_mosfets(); c.zero_position()
    c.trapezoid_move(units.rotations_to_counts(1.0), units.seconds_to_timesteps(0.05))
    import time; time.sleep(0.1)
    assert abs(units.counts_to_rotations(c.get_hall_position())) < 0.1   # stayed put


def test_broadcast_set_alias():
    rack = _rack()
    t = rack.transport_for("A")
    t.transact(255, "Set device alias", [ord("X")])
    det = dict((u, a) for u, a in t.transact(255, "Detect devices"))
    assert all(a == ord("X") for a in det.values())
