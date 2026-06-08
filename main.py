from core.particles import Particle
from core.springs import Spring
from core.rigidbody import RigidBody
from core.constraints import DistanceConstraint
from visualization.renderer import Renderer
from simulation.world import World

world = World(800, 600)
renderer = Renderer(world)
p1 = Particle(400, 200, 1.0)
# p1.pinned = True
# p2 = Particle(400, 600, 1.0)
# p3 = Particle(500, 600, 1.0)
# p4 = Particle(600, 600, 1.0)
# p5 = Particle(700, 600, 1.0)
# spring = Spring(p1 , p2 , length=100, k=0.1, damp=0.5)
# constraint1 = DistanceConstraint(p1 , p2 , (0,0) , (0,0) , 100 , 0.3)
# constraint2 = DistanceConstraint(p2 , p3 , (1,0) , (0,0) , 100 , 0.3)
# constraint3 = DistanceConstraint(p3 , p4 , (1,0) , (0,0) , 100 , 0.3)
# constraint4 = DistanceConstraint(p4 , p5 , (1,0) , (0,0) , 100 , 0.3)
body = RigidBody(400, 300, 1.0, 0.0)
body.set_shape([(-50,-25),(50,-25),(50,25),(-50,25)])
body.angular_velocity = 1.0
body2 = RigidBody(450, 350, 1.0, 0.0)
body2.set_shape([(-50,-25),(50,-25),(50,25),(-50,25)])
body2.angular_velocity = 1.0 
constraint = DistanceConstraint(body , body2 , (50,25) , (50,25) , 100)
# world.add_particle(p1)
# world.add_particle(p2)
# world.add_particle(p3)
# world.add_particle(p4)
# world.add_particle(p5)
# world.add_spring(spring)
# world.add_constraint(constraint1)
# world.add_constraint(constraint2)
# world.add_constraint(constraint3)
# world.add_constraint(constraint4)
world.add_rigid_bodies(body)
world.add_rigid_bodies(body2)
world.add_constraint(constraint)

renderer.run()