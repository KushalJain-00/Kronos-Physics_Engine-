"""Tests that verify RigidBody's shoelace-based area and centroid helpers reproduce known analytic results for symmetric polygons."""

import pytest
from core.rigidbody import RigidBody


def test_polygon_area_shoelace_known_square():
    """The shoelace area of a 10x10 square centered at the origin must be exactly 100."""
    body = RigidBody(0, 0, 1.0, 0.0)
    body.set_shape([(-5, -5), (5, -5), (5, 5), (-5, 5)])
    assert body.area() == pytest.approx(100.0, rel=1e-9)


def test_center_of_mass_symmetric_polygon_is_at_origin():
    """A polygon with 4-fold symmetry about the origin must have its centroid exactly at the origin."""
    body = RigidBody(0, 0, 1.0, 0.0)
    body.set_shape([(-5, -5), (5, -5), (5, 5), (-5, 5)])
    cx, cy = body.center_of_mass()
    assert cx == pytest.approx(0.0, abs=1e-9)
    assert cy == pytest.approx(0.0, abs=1e-9)
