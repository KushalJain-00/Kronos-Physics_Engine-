from core.particles import Particle
from core.springs import Spring
from core.rigidbody import RigidBody
from core.constraints import DistanceConstraint , HingeConstraint , ChainConstraint
from visualization.renderer import Renderer
from simulation.world import World
from ui.control_panel import ControlPanel

world = World(800, 600)
panel = ControlPanel(world)
renderer = Renderer(world)

body = RigidBody(200, 100, 1.0, 0.0)
body.set_shape([(-50,-25),(50,-25),(50,25),(-50,25)])
body.angular_velocity = 1.0
body2 = RigidBody(450, 350, 1.0, 0.0)
body2.set_shape([(-50,-25),(50,-25),(50,25),(-50,25)])
# body2.angular_velocity = 1.0 
# body3 = RigidBody(500, 300, 1.0, 0.0)
# body3.set_shape([(-50,-25),(50,-25),(50,25),(-50,25)])
# body3.angular_velocity = 1.0

constraint = DistanceConstraint(body , body2 , (50,25) , (25,-25) , 100)
# constraint2 = ChainConstraint(world , body2 , body3 , (-50,25) , (-50,25) , visible_links=True , n_links=5)

world.add_rigid_bodies(body)
world.add_rigid_bodies(body2)
# world.add_rigid_bodies(body3)
world.add_constraint(constraint)
# world.add_constraint(constraint2)
panel.start()
renderer.run()