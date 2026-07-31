"""Regression tests for bugs caught by CodeRabbit AI review and documented in README.md: impulse sign errors, unnormalized collision axes, division-by-zero edge cases, and threading races."""

import threading

import pytest
from core.constraints import DistanceConstraint
from core.particles import Particle
from core.rigidbody import RigidBody
from core.vectors import Vector2D


def test_no_sign_error_in_normal_impulse_calculation(world, particle_pair):
    """Two approaching particles must separate after resolution — a sign error in the normal impulse would make them plow through each other."""
    p1, p2 = particle_pair
    p1.position.x = 395.5  # spawn overlapping so the collision happens in step 1
    p2.position.x = 404.5
    world.add_particle(p1)
    world.add_particle(p2)
    world.gravity = Vector2D(0, 0)
    world.step(0.01)
    n = (1.0, 0.0)
    rel = (p1.velocity.x - p2.velocity.x, p1.velocity.y - p2.velocity.y)
    assert rel[0] * n[0] + rel[1] * n[1] < 0


def test_collision_axis_is_always_normalized(world, box_body):
    """CodeRabbit caught unnormalized collision axes — both the SAT normal and the particle-collision normal must be unit length, including for rotated bodies."""
    b2 = RigidBody(6, 0, 1.0, 0.3)
    b2.set_shape([(-5, -5), (5, -5), (5, 5), (-5, 5)])
    n = box_body.sat_collision(b2)["normal"]
    assert (n[0] ** 2 + n[1] ** 2) ** 0.5 == pytest.approx(1.0, abs=1e-9)
    p = Particle(3, 0, 1.0)
    pn = box_body.particle_collision(p)["normal"]
    assert (pn[0] ** 2 + pn[1] ** 2) ** 0.5 == pytest.approx(1.0, abs=1e-9)


def test_no_division_by_zero_for_zero_length_distance_constraint(world):
    """Coincident anchor points make current_length 0 — the solver must skip instead of dividing by zero (CodeRabbit edge-case catch)."""
    a = Particle(400, 300, 1.0)
    b = Particle(400, 300, 1.0)
    c = DistanceConstraint(a, b, (0, 0), (0, 0), rest_length=10.0)
    c.solve()
    assert a.position.x == pytest.approx(400.0, abs=0)
    assert b.position.y == pytest.approx(300.0, abs=0)


def test_no_division_by_zero_for_zero_relative_velocity_in_friction(world, box_body):
    """Resting overlapping bodies have zero relative velocity — the Coulomb friction path must not divide by zero and must leave velocities untouched (CodeRabbit catch)."""
    b2 = RigidBody(6, 0, 1.0, 0.0)
    b2.set_shape([(-5, -5), (5, -5), (5, 5), (-5, 5)])
    world.add_rigid_bodies(box_body)
    world.add_rigid_bodies(b2)
    world.gravity = Vector2D(0, 0)
    for _ in range(10):
        world.step(0.01)
    assert box_body.velocity.x == pytest.approx(0.0, abs=1e-9)
    assert b2.velocity.x == pytest.approx(0.0, abs=1e-9)


def test_no_race_condition_reading_world_state_during_step(world):
    """The physics lock must make concurrent reads during step() safe — a torn read or exception here means the threading model regressed (CodeRabbit catch)."""
    p = Particle(400, 300, 1.0)
    world.add_particle(p)
    world.gravity = Vector2D(0, 0)
    errors = []

    def reader():
        try:
            for _ in range(200):
                with world.lock:
                    x, y = p.position.x, p.position.y
                if not (0 <= x <= world.width and 0 <= y <= world.height):
                    errors.append("out-of-bounds read")
        except Exception as e:  # noqa: BLE001
            errors.append(repr(e))

    t = threading.Thread(target=reader)
    t.start()
    for _ in range(200):
        world.step(0.01)
    t.join()
    assert errors == []
