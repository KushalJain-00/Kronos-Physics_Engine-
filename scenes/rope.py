from core.rigidbody import RigidBody
from core.constraints import ChainConstraint
from simulation.world import World

def build(world: World) -> None:
    anchor = RigidBody(400, 500, 100.0, 0.0)
    anchor.set_shape([(-30, -10), (30, -10), (30, 10), (-30, 10)])
    payload = RigidBody(400, 300, 2.0, 0.0)
    payload.set_shape([(-20, -20), (20, -20), (20, 20), (-20, 20)])

    rope = ChainConstraint(world, anchor, payload, (0, -10), (0, 20), visible_links=True, n_links=10, stiffness=0.8, friction=0.2)

    world.add_rigid_bodies(anchor)
    world.add_rigid_bodies(payload)
    world.add_constraint(rope)