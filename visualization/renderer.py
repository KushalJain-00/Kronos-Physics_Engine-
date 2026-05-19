import pygame
from core.vectors import Vector2D
from core.particles import Particle
import random

class Renderer:
    def __init__(self , width , height , title = "Physics Engine"):
        pygame.init()
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width , height))
        pygame.display.set_caption(title)
        self.clock = pygame.time.Clock()
    
    def to_screen(self , x , y):
        return int(x), int(self.height - y)
    
    def from_screen(self, sx , sy):
        return float(sx) , float(self.height - sy)
    
    def draw_particle(self, particle):
        sx, sy = self.to_screen(particle.position.x, particle.position.y)
        pygame.draw.circle(self.screen, particle.color, (sx, sy), 10)
    
    def run(self, particle = None):
        GRAVITY = Vector2D(0 , -9.8)
        dt = 0.1
        running = True

        if particle is None:
            particles = []
        elif isinstance(particle , list):
            particles = particle
        else:
            particles = [particle]

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN or event.type == pygame.K_SPACE:
                    mx , my = event.pos
                    wx , wy = self.from_screen(mx , my)
                    color = (
                        random.randint(0, 255),
                        random.randint(0, 255),
                        random.randint(0, 255),
                    )
                    p = Particle(wx , wy , 1.0, color=color)
                    p.velocity = Vector2D(random.uniform(-50, 50), random.uniform(0, 150))
                    particles.append(p)
            for p in particles:
                p.apply_force(GRAVITY)
                p.update(dt)

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

            self.screen.fill((0, 0, 0))
            for p in particles:
                self.draw_particle(p)
            pygame.display.flip()
            self.clock.tick(60)

        pygame.quit()

