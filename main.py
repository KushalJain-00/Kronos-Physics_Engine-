from core.particles import Particle
from core.springs import Spring
from visualization.renderer import Renderer
from simulation.world import World

world = World(800, 600)
renderer = Renderer(world)
p1 = Particle(0, 100, 1.0)
p2 = Particle(600, 800, 10.0)
spring = Spring(p1 , p2 , length=100, k=0.1, damp=0.5)
world.add_particle(p1)
world.add_particle(p2)
world.add_spring(spring)
renderer.run()