from core.rigidbody import RigidBody
from core.constraints import HingeConstraint
from simulation.world import World


def build(world: World) -> None:
    """Two rigid bodies connected by a hinge."""
    body_a = RigidBody(400, 400, 10.0, 0.0)
    body_a.set_shape([(-50, -25), (50, -25), (50, 25), (-50, 25)])

    body_b = RigidBody(400, 300, 1.0, 0.0)
    body_b.set_shape([(-50, -25), (50, -25), (50, 25), (-50, 25)])

    hinge = HingeConstraint(body_a, body_b, (0, -25), (0, 25))

    world.add_rigid_bodies(body_a)
    world.add_rigid_bodies(body_b)
    world.add_constraint(hinge)