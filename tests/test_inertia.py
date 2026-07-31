"""Tests that verify RigidBody's moment-of-inertia calculation (shoelace method) reproduces closed-form known results."""

import pytest
from core.rigidbody import RigidBody


RECT_SHAPE = [(-50, -25), (50, -25), (50, 25), (-50, 25)]


def test_moment_of_inertia_rectangle_matches_closed_form():
    """The shoelace inertia for an axis-aligned rectangle centered on the origin must equal m(w^2 + h^2)/12 exactly — any deviation means the polygon formula is wrong."""
    body = RigidBody(0, 0, 10.0, 0.0)
    body.set_shape(RECT_SHAPE)
    expected = 10.0 * (100**2 + 50**2) / 12
    assert body.moment_of_inertia == pytest.approx(expected, rel=1e-9)


def test_moment_of_inertia_scales_linearly_with_mass():
    """Moment of inertia is mass times a shape factor, so doubling the mass must double I for an identical shape."""
    body_a = RigidBody(0, 0, 2.0, 0.0)
    body_a.set_shape(RECT_SHAPE)
    body_b = RigidBody(0, 0, 5.0, 0.0)
    body_b.set_shape(RECT_SHAPE)
    ratio = body_b.moment_of_inertia / body_a.moment_of_inertia
    assert ratio == pytest.approx(5.0 / 2.0, rel=1e-9)
