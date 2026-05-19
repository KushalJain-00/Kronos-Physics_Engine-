import pygame
import random
from core.vectors import Vector2D
from core.particles import Particle
from simulation.world import World

class Renderer:
    def __init__(self, world, title="Physics Engine"):
        self.world = world
        pygame.init()
        self.screen = pygame.display.set_mode((world.width, world.height))
        pygame.display.set_caption(title)
        self.clock = pygame.time.Clock()

    def to_screen(self, x, y):
        return int(x), int(self.world.height - y)

    def from_screen(self, sx, sy):
        return float(sx), float(self.world.height - sy)

    def draw_particle(self, particle):
        sx, sy = self.to_screen(particle.position.x, particle.position.y)
        pygame.draw.circle(self.screen, particle.color, (sx, sy), 10)

    def run(self):
        dt = 0.1
        running = True

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    mx, my = event.pos
                    wx, wy = self.from_screen(mx, my)
                    color = (
                        random.randint(0, 255),
                        random.randint(0, 255),
                        random.randint(0, 255),
                    )
                    particle = Particle(wx, wy, 1.0, color=color)
                    particle.velocity = Vector2D(random.uniform(-50, 50), random.uniform(0, 150))
                    self.world.add_particle(particle)

            self.world.step(dt)

            self.screen.fill((0, 0, 0))
            for p in self.world.particles:
                self.draw_particle(p)
            pygame.display.flip()
            self.clock.tick(60)

        pygame.quit()

        