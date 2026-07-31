"""Tests that verify the PBD constraints (Distance, Hinge, Chain) converge to their geometric invariants and apply velocity corrections exactly once, per README.md Phase 3 claims."""

import pytest
from core.constraints import ChainConstraint, DistanceConstraint, HingeConstraint
from core.particles import Particle
from core.rigidbody import RigidBody
from core.vectors import Vector2D


def test_distance_constraint_converges_to_rest_length(world, particle_pair):
    """A stretched pair solved repeatedly must converge to the rest length — stiffness 0.5 halves the error per solve, so 100 solves make it vanish."""
    p1, p2 = particle_pair
    c = DistanceConstraint(p1, p2, (0, 0), (0, 0), rest_length=50.0, stiffness=0.5)
    for _ in range(100):
        c.solve()
    assert p1.position.distance(p2.position) == pytest.approx(50.0, rel=1e-6)


def test_distance_constraint_stiffness_zero_is_noop(world, particle_pair):
    """Zero stiffness means the constraint applies exactly no correction — positions must be bit-identical after solving."""
    p1, p2 = particle_pair
    c = DistanceConstraint(p1, p2, (0, 0), (0, 0), rest_length=50.0, stiffness=0.0)
    pos_a0 = (p1.position.x, p1.position.y)
    pos_b0 = (p2.position.x, p2.position.y)
    for _ in range(10):
        c.solve()
    assert p1.position.x == pytest.approx(pos_a0[0], abs=0)
    assert p1.position.y == pytest.approx(pos_a0[1], abs=0)
    assert p2.position.x == pytest.approx(pos_b0[0], abs=0)
    assert p2.position.y == pytest.approx(pos_b0[1], abs=0)


def test_distance_constraint_full_stiffness_corrects_in_one_step(world, particle_pair):
    """At stiffness 1.0 one solve must bring the pair exactly to rest length — mass-weighted correction closes the full error."""
    p1, p2 = particle_pair
    c = DistanceConstraint(p1, p2, (0, 0), (0, 0), rest_length=50.0, stiffness=1.0)
    c.solve()
    assert p1.position.distance(p2.position) == pytest.approx(50.0, abs=1e-9)


def test_pinned_particle_never_moves_under_constraint_or_gravity(world):
    """A pinned particle must be immovable by gravity, integration, and constraint solving alike — infinite effective mass."""
    anchor = Particle(400, 300, 1.0)
    anchor.pinned = True
    bob = Particle(450, 300, 1.0)
    world.add_particle(anchor)
    world.add_particle(bob)
    world.add_constraint(DistanceConstraint(anchor, bob, (0, 0), (0, 0), rest_length=50.0, stiffness=1.0))
    for _ in range(50):
        world.step(0.01)
    assert anchor.position.x == pytest.approx(400.0, abs=0)
    assert anchor.position.y == pytest.approx(300.0, abs=0)


def test_hinge_constraint_anchor_points_coincide_after_solve(world):
    """The hinge must pull its two world-space anchor points together — with zero moment of inertia the solve is purely translational and exact."""
    a = RigidBody(400, 400, 10.0, 0.0)
    b = RigidBody(400, 300, 1.0, 0.0)
    hinge = HingeConstraint(a, b, (0, 25), (0, -25))
    for _ in range(10):
        hinge.solve()
    wa = hinge._get_world_anchor(a, (0, 25))
    wb = hinge._get_world_anchor(b, (0, -25))
    dist = ((wa[0] - wb[0]) ** 2 + (wa[1] - wb[1]) ** 2) ** 0.5
    assert dist == pytest.approx(0.0, abs=1e-9)


def test_chain_constraint_creates_expected_link_count(world, box_body):
    """A chain of n_links must produce n_links - 1 Link nodes in the world and n_links DistanceConstraint segments, per README.md."""
    payload = RigidBody(400, 300, 2.0, 0.0)
    payload.set_shape([(-20, -20), (20, -20), (20, 20), (-20, 20)])
    chain = ChainConstraint(world, box_body, payload, (0, -10), (0, 20), n_links=8, stiffness=0.8, friction=0.2)
    assert len(chain.links) == 7
    assert len(world.links) == 7
    assert len(chain.segments) == 8


def test_distance_constraint_velocity_correction_applied_exactly_once(world, particle_pair):
    """Regression for the README-documented duplicate-application bug: the measured velocity delta must equal the PBD position correction exactly (a doubled application would be 2x), and a second solve at rest length must change nothing."""
    p1, p2 = particle_pair
    m1, m2 = p1.mass, p2.mass
    c = DistanceConstraint(p1, p2, (0, 0), (0, 0), rest_length=50.0, stiffness=1.0)
    # current length 20, rest 50: diff = (20-50)/20 = -1.5, correction_x = 20 * -1.5 * 1.0 = -30
    correction_x = -30.0
    vx0 = (p1.velocity.x, p2.velocity.x)
    c.solve()
    assert p1.velocity.x - vx0[0] == pytest.approx(correction_x * m2 / (m1 + m2), abs=1e-9)
    assert p2.velocity.x - vx0[1] == pytest.approx(-correction_x * m1 / (m1 + m2), abs=1e-9)
    vx_after = (p1.velocity.x, p2.velocity.x)
    c.solve()
    assert p1.velocity.x == pytest.approx(vx_after[0], abs=0)
    assert p2.velocity.x == pytest.approx(vx_after[1], abs=0)
