from core.vectors import Vector2D
from core.particles import Particle

class World:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.particles = []
        self.gravity = Vector2D(0, -9.8)
        self.dt = 0.1

    def add_particle(self, particle):
        self.particles.append(particle)

    def step(self, dt=None):
        if dt is None:
            dt = self.dt
        for p in self.particles:
            p.apply_force(self.gravity)
            p.update(dt)
            self._handle_boundaries(p)

    def _handle_boundaries(self, p):
        if p.position.y <= 0:
            p.position.y = 0
            p.velocity.y *= -0.8
        if p.position.y >= self.height:
            p.position.y = self.height
            p.velocity.y *= -0.8
        if p.position.x <= 0:
            p.position.x = 0
            p.velocity.x *= -0.8
        if p.position.x >= self.width:
            p.position.x = self.width
            p.velocity.x *= -0.8