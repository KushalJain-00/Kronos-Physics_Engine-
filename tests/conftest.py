"""Shared fixtures: fresh world, standard particle pair, standard box body, momentum/energy helper."""

import pytest
from core.particles import Particle
from core.rigidbody import RigidBody
from core.vectors import Vector2D
from simulation.world import World


@pytest.fixture
def world():
    """A fresh 800x600 world with default gravity and zero drag."""
    return World(800, 600)


@pytest.fixture
def particle_pair():
    """Two equal-mass (1.0) particles 20 units apart on the x-axis, heading toward each other at 2 units/s."""
    p1 = Particle(390, 300, 1.0)
    p2 = Particle(410, 300, 1.0)
    p1.velocity = Vector2D(2.0, 0.0)
    p2.velocity = Vector2D(-2.0, 0.0)
    return p1, p2


@pytest.fixture
def box_body():
    """A 10x10 axis-aligned square rigid body of mass 1 centered at the origin."""
    body = RigidBody(0, 0, 1.0, 0.0)
    body.set_shape([(-5, -5), (5, -5), (5, 5), (-5, 5)])
    return body


@pytest.fixture
def momentum_energy():
    """Returns a helper computing (px, py, KE) over all particles and rigid bodies in a world."""

    def helper(world):
        px = sum(p.mass * p.velocity.x for p in world.particles)
        py = sum(p.mass * p.velocity.y for p in world.particles)
        ke = sum(0.5 * p.mass * (p.velocity.x**2 + p.velocity.y**2) for p in world.particles)
        for b in world.rigid_bodies:
            px += b.mass * b.velocity.x
            py += b.mass * b.velocity.y
            ke += 0.5 * b.mass * (b.velocity.x**2 + b.velocity.y**2)
            ke += 0.5 * b.moment_of_inertia * b.angular_velocity**2
        return px, py, ke

    return helper
