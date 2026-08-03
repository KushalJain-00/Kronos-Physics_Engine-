"""Tests for the Phase 3 completion constraints: WeldConstraint locks relative position and angle, MotorConstraint drives relative angular velocity to a target."""

import pytest
import math
from core.constraints import WeldConstraint, MotorConstraint, HingeConstraint
from core.rigidbody import RigidBody


def make_body(x=400.0, y=400.0, angle=0.0, size=10.0):
    b = RigidBody(x, y, 1.0, angle)
    b.set_shape([(-size, -size), (size, -size), (size, size), (-size, size)])
    return b


def test_weld_pulls_anchor_points_together():
    """The weld's hinge half must bring the two world anchor points to coincidence."""
    a = make_body(400, 400)
    b = make_body(400, 360)
    weld = WeldConstraint(a, b, (0, 20), (0, -20))
    for _ in range(10):
        weld.solve()
    wa = weld._get_world_anchor(a, (0, 20))
    wb = weld._get_world_anchor(b, (0, -20))
    dist = ((wa[0] - wb[0]) ** 2 + (wa[1] - wb[1]) ** 2) ** 0.5
    assert dist == pytest.approx(0.0, abs=1e-6)


def test_weld_preserves_relative_angle():
    """The weld's angle half must hold the relative angle at its construction-time value through external perturbation."""
    a = make_body(400, 400, angle=0.0)
    b = make_body(400, 360, angle=math.radians(15.0))
    weld = WeldConstraint(a, b, (0, 20), (0, -20))
    target = b.angle - a.angle
    b.angle += math.radians(20.0)
    for _ in range(10):
        weld.solve()
    rel = (b.angle - a.angle) % (2 * math.pi)
    if rel > math.pi:
        rel -= 2 * math.pi
    assert rel == pytest.approx(target, abs=1e-6)


def test_motor_drives_relative_angular_velocity_to_target():
    """A motor on a pinned stator must spin the free body up to the target relative angular velocity."""
    stator = make_body(400, 400)
    stator.pinned = True
    rotor = make_body(400, 400)
    motor = MotorConstraint(stator, rotor, target_angular_velocity=2.0)
    for _ in range(3):
        motor.solve()
    # stator is pinned (infinite inertia), so all correction lands on the rotor
    assert rotor.angular_velocity == pytest.approx(2.0, abs=1e-9)
    assert stator.angular_velocity == pytest.approx(0.0, abs=0)


def test_motor_holds_zero_by_default():
    """A motor with default target 0 must cancel any existing relative spin (brake)."""
    a = make_body(400, 400)
    b = make_body(400, 400)
    a.angular_velocity = 1.0
    b.angular_velocity = 3.0
    motor = MotorConstraint(a, b)
    for _ in range(3):
        motor.solve()
    assert b.angular_velocity - a.angular_velocity == pytest.approx(0.0, abs=1e-9)
