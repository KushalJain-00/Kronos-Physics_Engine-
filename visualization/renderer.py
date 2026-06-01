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

    def draw_spring(self, spring):
        sx1, sy1 = self.to_screen(spring.p1.position.x, spring.p1.position.y)
        sx2, sy2 = self.to_screen(spring.p2.position.x, spring.p2.position.y)
        pygame.draw.line(self.screen, (100, 100, 255), (sx1, sy1), (sx2, sy2), 2)

    def draw_rigid_body(self , body):
        vertices = body.get_world_vertices()
        screen_vertices = [self.to_screen(x , y) for x , y in vertices]
        pygame.draw.polygon(self.screen , body.color , screen_vertices , 2)

    def draw_sat_debug(self, body):
        vertices = body.get_world_vertices()
        axes = body.get_axes()
        cx, cy = self.to_screen(body.position.x, body.position.y)
        
        for axis in axes:
            # normalize axis
            length = (axis[0]**2 + axis[1]**2) ** 0.5
            if length == 0:
                continue
            nx, ny = axis[0]/length, axis[1]/length
            
            # draw axis line through center
            scale = 60
            x1 = int(cx - nx * scale)
            y1 = int(cy + ny * scale)  # flip y
            x2 = int(cx + nx * scale)
            y2 = int(cy - ny * scale)  # flip y
            pygame.draw.line(self.screen, (255, 255, 0), (x1, y1), (x2, y2), 1)

    def run(self):
        accumulator = 0.0
        physics_dt = 0.008
        last_time = pygame.time.get_ticks() / 1000.0
        running = True
        while running:
            current_time = pygame.time.get_ticks() / 1000.0
            frame_time = min(current_time - last_time , 0.05)
            last_time = current_time
            accumulator += frame_time

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:
                        mass = 1.0
                    elif event.button == 3:
                        mass = 5.0
                    else:
                        continue
                    mx, my = event.pos
                    wx, wy = self.from_screen(mx, my)
                    color = (
                        random.randint(0, 255),
                        random.randint(0, 255),
                        random.randint(0, 255),
                    )
                    particle = Particle(wx, wy, mass, color=color)
                    # particle.velocity = Vector2D(random.uniform(-50, 50), random.uniform(0, 150))
                    self.world.add_particle(particle)
            
            while accumulator >= physics_dt:
                self.world.step(physics_dt)
                accumulator -= physics_dt

            self.screen.fill((0, 0, 0))
            self.draw_grid()
            for spring in self.world.springs:
                self.draw_spring(spring)
            for p in self.world.particles:
                self.draw_particle(p)
            for body in self.world.rigid_bodies:
                self.draw_rigid_body(body)
                self.draw_sat_debug(body)
            pygame.display.flip()
            self.clock.tick(60)

        pygame.quit()

    