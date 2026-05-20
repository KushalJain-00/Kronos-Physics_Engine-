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

    def draw_grid(self, spacing=50):
        grid_color = (30, 30, 30)  # dark grey, subtle
        font = pygame.font.SysFont("monospace", 10)
        # vertical lines
        for x in range(0, self.world.width, spacing):
            pygame.draw.line(self.screen, grid_color, (x, 0), (x, self.world.height))
            label = font.render(str(x), True, (60, 60, 60))
            self.screen.blit(label, (x + 2, self.world.height - 15))
        
        # horizontal lines
        for y in range(0, self.world.height, spacing):
            pygame.draw.line(self.screen, grid_color, (0, y), (self.world.width, y))
            label = font.render(str(y), True, (60, 60, 60))
            self.screen.blit(label, (2, self.world.height - y - 12))
            
    def draw_particle(self, particle):
        sx, sy = self.to_screen(particle.position.x, particle.position.y)
        pygame.draw.circle(self.screen, particle.color, (sx, sy), particle.radius)

    def run(self):
        dt = 0.1
        running = True

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:
                        mass = 1.0
                    elif event.button == 3:
                        mass = 5.0
                    mx, my = event.pos
                    wx, wy = self.from_screen(mx, my)
                    color = (
                        random.randint(0, 255),
                        random.randint(0, 255),
                        random.randint(0, 255),
                    )
                    particle = Particle(wx, wy, mass, color=color)
                    particle.velocity = Vector2D(random.uniform(-50, 50), random.uniform(0, 150))
                    self.world.add_particle(particle)

            self.world.step(dt)

            self.screen.fill((0, 0, 0))
            self.draw_grid()
            for p in self.world.particles:
                self.draw_particle(p)
            pygame.display.flip()
            self.clock.tick(60)

        pygame.quit()

    