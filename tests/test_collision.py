"""Tests that verify SAT collision detection reports overlaps, separations, and exact penetration geometry, per the Phase 2 claims in README.md."""

import pytest
from core.particles import Particle
from core.rigidbody import RigidBody
from core.vectors import Vector2D


def make_box(x, y):
    b = RigidBody(x, y, 1.0, 0.0)
    b.set_shape([(-5, -5), (5, -5), (5, 5), (-5, 5)])
    return b


def test_sat_detects_overlapping_boxes(world, box_body):
    """Two boxes overlapping by 4 units along x must be reported as a collision."""
    assert box_body.sat_collision(make_box(6, 0)) is not None


def test_sat_no_collision_for_separated_boxes(world, box_body):
    """Two boxes with a 1-unit gap must be reported as non-colliding (None)."""
    assert box_body.sat_collision(make_box(11, 0)) is None


def test_sat_collision_normal_direction_convention(world, box_body):
    """Pins the sign convention: the returned normal is the first edge normal with minimal overlap (ties go to the first axis tested), unit-normalized — the world later flips it toward the b1->b2 direction during position correction, so a regression here breaks separation logic."""
    n = box_body.sat_collision(make_box(6, 0))["normal"]
    assert n[0] == pytest.approx(-1.0, abs=1e-9)
    assert n[1] == pytest.approx(0.0, abs=1e-9)


def test_sat_penetration_depth_matches_known_overlap(world, box_body):
    """A box offset 3 units up must report penetration depth exactly 7 (full 10 minus the 3-unit offset)."""
    result = box_body.sat_collision(make_box(0, 3))
    assert result["depth"] == pytest.approx(7.0, abs=1e-9)


def test_particle_rigidbody_collision_detected(world, box_body):
    """A particle center inside the box (closer to an edge than its radius) must produce a contact manifold."""
    p = Particle(3, 0, 1.0)
    assert box_body.particle_collision(p) is not None


def test_no_false_positive_at_exact_zero_penetration(world, box_body):
    """Touching-but-not-overlapping boxes report depth 0 — a non-event that must produce zero positional correction and zero motion."""
    box_body.position.x = 400.0  # move off the origin: the left wall would otherwise bounce it
    box_body.position.y = 300.0
    b2 = make_box(410, 300)
    world.add_rigid_bodies(box_body)
    world.add_rigid_bodies(b2)
    world.gravity = Vector2D(0, 0)
    pos_a0, pos_b0 = box_body.position.x, b2.position.x
    world.step(0.01)
    assert box_body.position.x == pytest.approx(pos_a0, abs=1e-12)
    assert b2.position.x == pytest.approx(pos_b0, abs=1e-12)
