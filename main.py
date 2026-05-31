from core.particles import Particle
from core.springs import Spring
from core.rigidbody import RigidBody
from visualization.renderer import Renderer
from simulation.world import World

world = World(800, 600)
renderer = Renderer(world)
# p1 = Particle(0, 100, 1.0)
# p2 = Particle(600, 800, 10.0)
# spring = Spring(p1 , p2 , length=100, k=0.1, damp=0.5)
body = RigidBody(400, 300, 1.0, 0.0)
body.set_shape([(-50,-25),(50,-25),(50,25),(-50,25)])
body.angular_velocity = 1.0
body2 = RigidBody(450, 350, 1.0, 0.0)
body2.set_shape([(-50,-25),(50,-25),(50,25),(-50,25)])
body2.angular_velocity = 1.0 
# world.add_particle(p1)
# world.add_particle(p2)
# world.add_spring(spring)
world.add_rigid_bodies(body)
world.add_rigid_bodies(body2)

renderer.run()