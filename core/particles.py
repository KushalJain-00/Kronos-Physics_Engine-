from core.vectors import Vector2D

class Particle:
    def __init__(self , x: float , y: float , mass: float, color=(255, 255, 255)):
        self.mass = mass
        self.radius = max(5 , int(mass * 5))
        self.time = 0.0
        self.position = Vector2D(x , y)
        self.velocity = Vector2D(0 , 0)
        self.acceleration = Vector2D(0 , 0)
        self.old_acceleration = Vector2D(0 , 0)
        self.color = color
    
    def apply_force(self , force):
        self.acceleration.x += force.x / self.mass
        self.acceleration.y += force.y / self.mass
    
    def update(self , dt):
        self.old_acceleration = Vector2D(self.acceleration.x , self.acceleration.y)
        self.position.x += self.velocity.x * dt + 0.5 * self.acceleration.x * dt**2
        self.position.y += self.velocity.y * dt + 0.5 * self.acceleration.y * dt**2
        self.velocity.x += self.acceleration.x * dt
        self.velocity.y += self.acceleration.y * dt
        self.acceleration = Vector2D(0 , 0)
        self.time += dt

    def __repr__(self):
        return f"Particle(t={self.time:.2f}s, pos={self.position}, vel={self.velocity}, mass={self.mass})"
