from core.particles import Particle
from core.constraints import DistanceConstraint
from simulation.world import World

def build(world: World) -> None:
    anchor = Particle(400, 500 , 1.0)
    anchor.pinned = True
    bob = Particle(500, 400, 1.0)
    constraint = DistanceConstraint(anchor, bob, (0, 0), (0, 0), 150, stiffness=1.0)
    world.add_particle(anchor)
    world.add_particle(bob)
    world.add_constraint(constraint)