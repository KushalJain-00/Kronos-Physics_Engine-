from core.rigidbody import RigidBody
from simulation.world import World


def build(world: World, count: int = 6) -> None:
    """Vertical box stack — the canonical stress test for solver stability.
    Use this to visually track jitter/penetration improvements in Phase 4."""
    box_size = 40
    for i in range(count):
        body = RigidBody(400, 500 - i * (box_size + 2), 1.0, 0.0)
        body.set_shape([
            (-box_size, -box_size), (box_size, -box_size),
            (box_size, box_size), (-box_size, box_size),
        ])
        world.add_rigid_bodies(body)