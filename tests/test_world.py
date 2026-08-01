"""Tests that verify World's fixed-timestep stepping, gravity integration, clock semantics, and add/clear bookkeeping."""

import pytest
from core.chains_and_ropes import Link
from core.constraints import DistanceConstraint
from core.particles import Particle
from core.rigidbody import RigidBody
from core.springs import Spring
from core.vectors import Vector2D


def test_world_step_advances_simulation_time_by_dt(world, particle_pair):
    """One step must advance every particle's simulation clock by exactly dt — World itself has no clock, so Particle.time is the canonical one."""
    p1, p2 = particle_pair
    world.add_particle(p1)
    world.add_particle(p2)
    world.step(0.05)
    assert p1.time == pytest.approx(0.05, abs=1e-12)
    assert p2.time == pytest.approx(0.05, abs=1e-12)


def test_gravity_accelerates_particle_matches_analytic_v_equals_gt(world):
    """A particle at rest under constant gravity must follow v = g*t and y = y0 + 0.5*g*t^2 exactly — the Verlet update is exact for constant acceleration."""
    p = Particle(400, 300, 1.0)
    world.add_particle(p)
    world.gravity = Vector2D(0, -9.8)
    world.step(0.01)
    assert p.velocity.y == pytest.approx(-9.8 * 0.01, abs=1e-9)
    assert p.position.y == pytest.approx(300.0 - 0.5 * 9.8 * 0.01**2, abs=1e-9)


def test_wind_accelerates_particle_matches_analytic_v_equals_at(world):
    """A particle at rest under constant wind (gravity zeroed) must follow v = a*t exactly — like gravity, the Verlet update is exact for constant acceleration."""
    p = Particle(400, 300, 1.0)
    world.add_particle(p)
    world.gravity = Vector2D(0, 0)
    world.wind = Vector2D(10, 0)
    world.step(0.01)
    assert p.velocity.x == pytest.approx(10 * 0.01, abs=1e-9)


def test_add_and_remove_bodies_updates_world_state(world):
    """World has no per-object removal API — removal is clear(); this pins the add-then-clear round trip so counts always reflect state."""
    p = Particle(100, 100, 1.0)
    anchor = Particle(200, 200, 1.0)
    anchor.pinned = True
    b = RigidBody(300, 300, 1.0, 0.0)
    b.set_shape([(-5, -5), (5, -5), (5, 5), (-5, 5)])
    world.add_particle(p)
    world.add_spring(Spring(p, anchor, 10.0))
    world.add_rigid_bodies(b)
    world.add_constraint(DistanceConstraint(anchor, p, (0, 0), (0, 0), rest_length=10.0))
    world.add_link(Link(400, 400, 1.0))
    assert len(world.particles) == 1
    assert len(world.springs) == 1
    assert len(world.rigid_bodies) == 1
    assert len(world.constraints) == 1
    assert len(world.links) == 1
    world.clear()
    assert world.particles == []
    assert world.springs == []
    assert world.rigid_bodies == []
    assert world.constraints == []
    assert world.links == []
