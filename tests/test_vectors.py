"""Tests that verify the Vector2D math primitives against their closed-form definitions."""

import pytest
from core.vectors import Vector2D


def test_add_sums_component_wise():
    v = Vector2D(1, 2).add(Vector2D(3, -4))
    assert v.x == pytest.approx(4.0, abs=1e-12)
    assert v.y == pytest.approx(-2.0, abs=1e-12)


def test_subtract_differences_component_wise():
    v = Vector2D(1, 2).sub(Vector2D(3, -4))
    assert v.x == pytest.approx(-2.0, abs=1e-12)
    assert v.y == pytest.approx(6.0, abs=1e-12)


def test_scalar_multiply_scales_both_components():
    v = Vector2D(3, -2).multiply(2.5)
    assert v.x == pytest.approx(7.5, abs=1e-12)
    assert v.y == pytest.approx(-5.0, abs=1e-12)


def test_dot_product_orthogonal_is_zero():
    assert Vector2D(3, 4).dot_product(Vector2D(-4, 3)) == pytest.approx(0.0, abs=1e-12)


def test_dot_product_parallel_equals_magnitude_product():
    v = Vector2D(3, 4)
    assert v.dot_product(v) == pytest.approx(v.magnitude() ** 2, abs=1e-12)


def test_normalize_returns_unit_length():
    v = Vector2D(3, 4).normalize()
    assert v.magnitude() == pytest.approx(1.0, abs=1e-12)
    assert v.x == pytest.approx(0.6, abs=1e-12)


def test_normalize_zero_vector_does_not_crash():
    """Defined behavior: normalizing the zero vector returns the zero vector rather than dividing by zero."""
    v = Vector2D(0, 0).normalize()
    assert v.x == pytest.approx(0.0, abs=0)
    assert v.y == pytest.approx(0.0, abs=0)


def test_distance_matches_pythagorean_triple():
    assert Vector2D(0, 0).distance(Vector2D(3, 4)) == pytest.approx(5.0, abs=1e-12)


def test_angle_known_values():
    x = Vector2D(1, 0)
    assert x.angle(Vector2D(1, 0)) == pytest.approx(0.0, abs=1e-9)
    assert x.angle(Vector2D(0, 1)) == pytest.approx(90.0, abs=1e-9)
    assert x.angle(Vector2D(-1, 0)) == pytest.approx(180.0, abs=1e-9)


def test_magnitude_zero_vector_is_zero():
    assert Vector2D(0, 0).magnitude() == pytest.approx(0.0, abs=0)
