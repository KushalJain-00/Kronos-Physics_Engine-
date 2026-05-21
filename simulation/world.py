from core.vectors import Vector2D

class World:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.particles = []
        self.springs = []
        self.gravity = Vector2D(0, -9.8)
        self.restitution = 0.9
        self.drag_coefficient = 0
        self.dt = 0.016

    def add_particle(self, particle):
        self.particles.append(particle)

    def add_spring(self , spring):
        self.springs.append(spring)

    def step(self, dt=None):
        if dt is None:
            dt = self.dt
        
        for spring in self.springs:
            spring.apply_spring_force()
        
        for p in self.particles:
            drag_force = Vector2D(-self.drag_coefficient * p.velocity.x , -self.drag_coefficient * p.velocity.y) 
            p.apply_force(Vector2D(self.gravity.x * p.mass, self.gravity.y * p.mass))
            p.apply_force(drag_force)
            p.update(dt)
            self._handle_boundaries(p)
        
        self._handle_collisions()

    def _handle_boundaries(self, p):
        # Lower Boundry
        if p.position.y <= p.radius:
            p.position.y = p.radius
            p.velocity.y *= -self.restitution
        # Upper Boundry
        if p.position.y >= self.height - p.radius:
            p.position.y = self.height - p.radius
            p.velocity.y *= -self.restitution
        # Left Boundry
        if p.position.x <= p.radius:
            p.position.x = p.radius
            p.velocity.x *= -self.restitution
        # Right Boundry
        if p.position.x >= self.width - p.radius:
            p.position.x = self.width - p.radius
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
                    # collision normal
                    nx = dx
                    ny = dy
                    dvx = p1.velocity.x - p2.velocity.x  # relative velocity
                    dvy = p1.velocity.y - p2.velocity.y
                    # relative velocity along normal
                    dot = dvx * nx + dvy * ny
                    if dot <= 0:
                        continue
                    # impulse scalar
                    impulse = (-(1 + self.restitution) * dot) / (1/p1.mass + 1/p2.mass)

                    # apply impulse along normal only
                    p1.velocity.x += (impulse / p1.mass) * nx
                    p1.velocity.y += (impulse / p1.mass) * ny
                    p2.velocity.x -= (impulse / p2.mass) * nx
                    p2.velocity.y -= (impulse / p2.mass) * ny
                        
