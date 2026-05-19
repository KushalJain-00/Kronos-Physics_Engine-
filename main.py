from core.particles import Particle
from visualization.renderer import Renderer
from simulation.world import World

world = World(800, 600)
renderer = Renderer(world)
renderer.run()