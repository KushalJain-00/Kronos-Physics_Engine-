from core.particles import Particle
from visualization.renderer import Renderer

p = Particle(400, 500, 1)
r = Renderer(800, 600)
r.run()