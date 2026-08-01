from core.rigidbody import RigidBody
from core.constraints import HingeConstraint, MotorConstraint
from simulation.world import World


def build(world: World) -> None:
    """A pinned stator with a free rotor driven to a constant angular velocity by a motor."""
    stator = RigidBody(400, 400, 100.0, 0.0)
    stator.set_shape([(-30, -30), (30, -30), (30, 30), (-30, 30)])
    stator.pinned = True

    rotor = RigidBody(400, 400, 2.0, 0.0)
    rotor.set_shape([(-12, -12), (12, -12), (12, 12), (-12, 12)])

    hinge = HingeConstraint(stator, rotor, (0, 0), (0, 0))
    motor = MotorConstraint(stator, rotor, target_angular_velocity=2.0)

    world.add_rigid_bodies(stator)
    world.add_rigid_bodies(rotor)
    world.add_constraint(hinge)
    world.add_constraint(motor)
