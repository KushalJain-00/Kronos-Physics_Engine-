from core.rigidbody import RigidBody
from core.constraints import WeldConstraint
from simulation.world import World


def build(world: World) -> None:
    """Two rigid bodies welded together — fixed relative position and angle, effectively one rigid body."""
    body_a = RigidBody(400, 400, 10.0, 0.0)
    body_a.set_shape([(-50, -25), (50, -25), (50, 25), (-50, 25)])

    body_b = RigidBody(400, 320, 5.0, 0.0)
    body_b.set_shape([(-50, -25), (50, -25), (50, 25), (-50, 25)])

    weld = WeldConstraint(body_a, body_b, (0, 25), (0, -25))

    world.add_rigid_bodies(body_a)
    world.add_rigid_bodies(body_b)
    world.add_constraint(weld)
