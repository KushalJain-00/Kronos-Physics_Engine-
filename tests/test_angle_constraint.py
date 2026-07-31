"""Regression tests for AngleConstraint.solve: in-range and exact-boundary angles must be untouched, out-of-range angles must be corrected with inertia-weighted magnitudes, zero-inertia bodies must never rotate, and inverted ranges must be rejected at construction."""

import pytest
from core.constraints import AngleConstraint
from core.rigidbody import RigidBody


def make_body(x=0.0, angle=0.0, inertia=None):
    b = RigidBody(x, 0.0, 1.0, angle)
    if inertia is not None:
        b.set_shape([(-5, -5), (5, -5), (5, 5), (-5, 5)])
    if inertia is not None:
        b.moment_of_inertia = inertia
    return b


def test_inverted_angle_range_rejected_at_construction():
    """A min_angle above max_angle is a contradiction and must be rejected with the module's ValueError convention."""
    a, b = make_body(), make_body()
    with pytest.raises(ValueError):
        AngleConstraint(a, b, max_angle=30.0, min_angle=60.0)


def test_in_range_angle_is_untouched():
    """An angle inside (min, max) violates nothing, so neither body's angle nor angular velocity may change."""
    a = make_body(angle=0.0, inertia=1.0)
    b = make_body(angle=20.0, inertia=1.0)
    c = AngleConstraint(a, b, max_angle=30.0, min_angle=10.0)
    a0, b0, wa0, wb0 = a.angle, b.angle, a.angular_velocity, b.angular_velocity
    c.solve()
    assert a.angle == pytest.approx(a0, abs=0)
    assert b.angle == pytest.approx(b0, abs=0)
    assert a.angular_velocity == pytest.approx(wa0, abs=0)
    assert b.angular_velocity == pytest.approx(wb0, abs=0)


def test_exact_boundary_angles_are_untouched():
    """Sitting exactly on min_angle or max_angle is not a violation — no correction may be applied."""
    a = make_body(angle=0.0, inertia=1.0)
    b = make_body(angle=30.0, inertia=1.0)
    c = AngleConstraint(a, b, max_angle=30.0, min_angle=10.0)
    a0, b0 = a.angle, b.angle
    c.solve()
    assert a.angle == pytest.approx(a0, abs=0)
    assert b.angle == pytest.approx(b0, abs=0)
    a.angle = 0.0
    b.angle = 10.0
    c.solve()
    assert a.angle == pytest.approx(0.0, abs=0)
    assert b.angle == pytest.approx(10.0, abs=0)


def test_above_max_angle_corrected_down():
    """When relative_angle exceeds max_angle the relative angle must shrink back toward the bound."""
    a = make_body(angle=0.0, inertia=1.0)
    b = make_body(angle=60.0, inertia=1.0)
    c = AngleConstraint(a, b, max_angle=30.0, min_angle=10.0, stiffness=1.0)
    c.solve()
    assert b.angle - a.angle == pytest.approx(30.0, abs=1e-9)


def test_below_min_angle_corrected_up():
    """When relative_angle drops below min_angle the relative angle must grow back toward the bound."""
    a = make_body(angle=0.0, inertia=1.0)
    b = make_body(angle=5.0, inertia=1.0)
    c = AngleConstraint(a, b, max_angle=30.0, min_angle=10.0, stiffness=1.0)
    c.solve()
    assert b.angle - a.angle == pytest.approx(10.0, abs=1e-9)


def test_correction_split_by_inverse_inertia():
    """The correction must be shared in inverse-inertia proportion — the body with 4x the inertia moves 4x less."""
    a = make_body(inertia=1.0)
    b = make_body(inertia=4.0)
    c = AngleConstraint(a, b, max_angle=10.0, min_angle=-10.0, stiffness=1.0)
    a.angle = 0.0
    b.angle = 50.0
    c.solve()
    assert a.angle == pytest.approx(32.0, abs=1e-9)
    assert b.angle == pytest.approx(42.0, abs=1e-9)


def test_zero_inertia_body_never_rotates():
    """A body with no shape (zero moment of inertia) must stay put — the partner takes the full correction."""
    a = make_body(inertia=0.0)
    b = make_body(inertia=1.0)
    c = AngleConstraint(a, b, max_angle=10.0, min_angle=-10.0, stiffness=1.0)
    a.angle = 0.0
    b.angle = 50.0
    c.solve()
    assert a.angle == pytest.approx(0.0, abs=1e-9)
    assert b.angle == pytest.approx(10.0, abs=1e-9)


def test_both_zero_inertia_solve_is_safe_noop():
    """Two zero-inertia bodies make total inverse inertia zero — solve must return without NaN or movement."""
    a = make_body(inertia=0.0)
    b = make_body(inertia=0.0)
    c = AngleConstraint(a, b, max_angle=10.0, min_angle=-10.0, stiffness=1.0)
    a.angle = 0.0
    b.angle = 50.0
    c.solve()
    assert a.angle == pytest.approx(0.0, abs=0)
    assert b.angle == pytest.approx(50.0, abs=0)


def test_repeated_solves_converge_to_range():
    """At stiffness 0.5 each solve halves the error, so repeated solves must drive the angle onto the bound (within the residual 0.5^20 ~ 1e-5)."""
    a = make_body(angle=0.0, inertia=1.0)
    b = make_body(angle=80.0, inertia=1.0)
    c = AngleConstraint(a, b, max_angle=30.0, min_angle=10.0, stiffness=0.5)
    for _ in range(20):
        c.solve()
    assert b.angle - a.angle == pytest.approx(30.0, abs=1e-3)


def test_solve_never_touches_angular_velocity():
    """Angular velocity corrections are deferred until a timestep-aware contract exists — solve must leave velocities untouched."""
    a = make_body(angle=0.0, inertia=1.0)
    b = make_body(angle=50.0, inertia=1.0)
    c = AngleConstraint(a, b, max_angle=30.0, min_angle=10.0, stiffness=1.0)
    a.angular_velocity = 2.0
    b.angular_velocity = -1.0
    c.solve()
    assert a.angular_velocity == pytest.approx(2.0, abs=0)
    assert b.angular_velocity == pytest.approx(-1.0, abs=0)
