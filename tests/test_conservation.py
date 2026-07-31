"""Tests that verify Kronos's collision response and integration conserve momentum, kinetic energy, and angular momentum for elastic collisions, backing the conservation-law claims in README.md."""

import math

import pytest
from core.particles import Particle
from core.rigidbody import RigidBody
from core.springs import Spring
from core.vectors import Vector2D


def test_elastic_collision_conserves_momentum(world, particle_pair, momentum_energy):
    """A head-on elastic collision exchanges velocities but the total momentum vector is invariant — equal-and-opposite impulses can only redistribute it."""
    p1, p2 = particle_pair
    p1.position.x = 395.5  # spawn overlapping so the collision happens in step 1
    p2.position.x = 404.5
    p1.velocity = Vector2D(3.0, 0.0)
    p2.velocity = Vector2D(-1.0, 0.0)
    world.add_particle(p1)
    world.add_particle(p2)
    world.gravity = Vector2D(0, 0)
    px0, py0, _ = momentum_energy(world)
    world.step(0.01)
    px1, py1, _ = momentum_energy(world)
    # exact impulse math: drift is pure float noise
    assert px1 == pytest.approx(px0, abs=1e-9)
    assert py1 == pytest.approx(py0, abs=1e-9)


def test_elastic_collision_conserves_kinetic_energy(world, particle_pair, momentum_energy):
    """At restitution=1.0 the impulse model conserves kinetic energy exactly — equal masses head-on simply exchange velocities, so 0.5*(9+1) stays 0.5*(1+9)."""
    p1, p2 = particle_pair
    p1.position.x = 395.5  # spawn overlapping so the collision happens in step 1
    p2.position.x = 404.5
    p1.velocity = Vector2D(3.0, 0.0)
    p2.velocity = Vector2D(-1.0, 0.0)
    world.add_particle(p1)
    world.add_particle(p2)
    world.gravity = Vector2D(0, 0)
    world.restitution = 1.0
    _, _, ke0 = momentum_energy(world)
    world.step(0.01)
    _, _, ke1 = momentum_energy(world)
    assert ke1 == pytest.approx(ke0, abs=1e-9)


def test_inelastic_collision_loses_kinetic_energy(world, particle_pair, momentum_energy):
    """At restitution=0.0 two equal masses colliding head-on come to rest together — kinetic energy goes to zero, the opposite of elastic behavior."""
    p1, p2 = particle_pair
    p1.position.x = 395.5  # spawn overlapping so the collision happens in step 1
    p2.position.x = 404.5
    world.add_particle(p1)
    world.add_particle(p2)
    world.gravity = Vector2D(0, 0)
    world.restitution = 0.0
    world.step(0.01)
    _, _, ke = momentum_energy(world)
    assert ke == pytest.approx(0.0, abs=1e-9)


def test_rigid_body_collision_conserves_angular_momentum(world):
    """An off-center impact exchanges linear and angular momentum between bodies, but total angular momentum about any fixed point (here the origin) is invariant — slop position-correction is the only non-conservative term, and this setup keeps it zero because both bodies' momenta lie along the collision normal."""
    b1 = RigidBody(400, 300, 1.0, 0.0)
    b2 = RigidBody(406, 302, 1.0, 0.0)
    for b in (b1, b2):
        b.set_shape([(-5, -5), (5, -5), (5, 5), (-5, 5)])
    b1.velocity = Vector2D(2.0, 0.0)
    world.add_rigid_bodies(b1)
    world.add_rigid_bodies(b2)
    world.gravity = Vector2D(0, 0)

    def L():
        total = 0.0
        for b in (b1, b2):
            total += b.position.x * (b.mass * b.velocity.y) - b.position.y * (b.mass * b.velocity.x)
            total += b.moment_of_inertia * b.angular_velocity
        return total

    l0 = L()
    world.step(0.01)
    assert L() == pytest.approx(l0, abs=1e-9)


def test_isolated_system_no_gravity_conserves_total_momentum(world, momentum_energy):
    """With zero external force and no boundary contact, total momentum of a multi-body system must not drift over 200 steps — every collision impulse is internal and momentum-conserving."""
    n = 8
    for i in range(n):
        theta = 2 * math.pi * i / n
        p = Particle(400 + 60 * math.cos(theta), 300 + 60 * math.sin(theta), 1.0)
        toward_center = (-math.cos(theta), -math.sin(theta))
        tangent = (-toward_center[1], toward_center[0])
        p.velocity = Vector2D(
            20.0 * toward_center[0] + 15.0 * tangent[0],
            20.0 * toward_center[1] + 15.0 * tangent[1],
        )
        world.add_particle(p)
    world.gravity = Vector2D(0, 0)
    px0, py0, _ = momentum_energy(world)
    for _ in range(200):
        world.step(0.01)
    px1, py1, _ = momentum_energy(world)
    # symmetric setup has zero initial momentum; speeds keep particles far from walls
    assert px1 == pytest.approx(px0, abs=1e-9)
    assert py1 == pytest.approx(py0, abs=1e-9)


def test_spring_energy_conservation_no_damping(world, momentum_energy):
    """Best integrator-error detector: with damping=0 and no gravity, total mechanical energy (KE + spring PE) must stay constant — any drift here exposes integration error in the Verlet update."""
    p1 = Particle(300, 300, 1.0)
    p2 = Particle(360, 300, 1.0)
    spring = Spring(p1, p2, length=50.0, k=0.1, damp=0.0)
    world.add_particle(p1)
    world.add_particle(p2)
    world.add_spring(spring)
    world.gravity = Vector2D(0, 0)

    def total_energy():
        _, _, ke = momentum_energy(world)
        length = p1.position.distance(p2.position)
        return ke + 0.5 * spring.spring_const * (length - spring.rest_length) ** 2

    e0 = total_energy()
    for _ in range(1000):
        world.step(0.001)
    # velocity-Verlet energy error is a bounded oscillation of order (w*dt)^2, ~1e-7 here; 1e-4 gives huge headroom
    assert total_energy() == pytest.approx(e0, rel=1e-4)
