from core.vectors import Vector2D
from core.particles import Particle

class World:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.particles = []
        self.gravity = Vector2D(0, -9.8)
        self.restitution = 0.9
        self.drag_coefficient = 0
        self.dt = 0.05

    def add_particle(self, particle):
        self.particles.append(particle)

    def step(self, dt=None):
        if dt is None:
            dt = self.dt
        for p in self.particles:
            drag_force = Vector2D(-self.drag_coefficient * p.velocity.x , -self.drag_coefficient * p.velocity.y) 
            p.apply_force(self.gravity)
            p.apply_force(drag_force)
            p.update(dt)
            self._handle_boundaries(p)
        self._handle_collisions()

    def _handle_boundaries(self, p):
        # Lower Boundry
        if p.position.y <= 0:
            p.position.y = 0
            p.velocity.y *= -self.restitution
        # Upper Boundry
        if p.position.y >= self.height:
            p.position.y = self.height
            p.velocity.y *= -self.restitution
        # Left Boundry
        if p.position.x <= 0:
            p.position.x = 0
            p.velocity.x *= -self.restitution
        # Right Boundry
        if p.position.x >= self.width:
            p.position.x = self.width
            p.velocity.x *= -self.restitution
    
    def _handle_collisions(self):
        for i in range(len(self.particles)):
            for j in range(i+1, len(self.particles)):
                p1 = self.particles[i]
                p2 = self.particles[j]
                
                dist = p1.position.distance(p2.position)
                min_dist = p1.radius + p2.radius
                
                if dist < min_dist and dist > 0:
                    # position correction - separate them
                    overlap = min_dist - dist
                    dx = (p2.position.x - p1.position.x) / dist
                    dy = (p2.position.y - p1.position.y) / dist
                    p1.position.x -= dx * overlap / 2
                    p1.position.y -= dy * overlap / 2
                    p2.position.x += dx * overlap / 2
                    p2.position.y += dy * overlap / 2
                    
                    # momentum conservation
                    m1, m2 = p1.mass, p2.mass
                    v1x, v1y = p1.velocity.x, p1.velocity.y
                    v2x, v2y = p2.velocity.x, p2.velocity.y
                    
                    p1.velocity.y = ((m1-m2)*v1y + 2*m2*v2y) / (m1+m2) * self.restitution
                    p2.velocity.x = ((m2-m1)*v2x + 2*m1*v1x) / (m1+m2) * self.restitution
                    p2.velocity.y = ((m2-m1)*v2y + 2*m1*v1y) / (m1+m2) * self.restitution
                    p1.velocity.x = ((m1-m2)*v1x + 2*m2*v2x) / (m1+m2) * self.restitution
    
