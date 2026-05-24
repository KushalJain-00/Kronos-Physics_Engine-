from core.vectors import Vector2D
import math

class RigidBody:
    def __init__(self , x: float, y: float, mass: float , angle: float ,  color = (255 , 255 , 255)):
        if mass <= 0 :
            raise ValueError("mass must be > 0")  
        self.mass = mass
        self.time = 0.0
        self.position = Vector2D(x , y)
        self.velocity = Vector2D(0 , 0)
        self.acceleration = Vector2D(0 , 0)
        self.old_acceleration = Vector2D(0 , 0)
        self.vertices = []
        self.angle = angle
        self.angular_velocity = 0.0
        self.angular_acceleration = 0.0
        self.moment_of_inertia = 0.0
        self.color = color
    
    def set_shape(self , vertices):
        self.vertices = vertices
        self.moment_of_inertia = self._calculate_inertia()
    
    def _calculate_inertia(self):
        n = len(self.vertices)
        numerator = 0
        denominator = 0
        for i in range(n):
            p1 = self.vertices[i]
            p2 = self.vertices[(i + 1) % n]
            cross = abs(p1[0] * p2[1] - p1[1] * p2[0])
            numerator += cross * (p1[0]**2 + p1[0]*p2[0] + p2[0]**2 + p1[1]**2 + p1[1]*p2[1] + p2[1]**2)
            denominator += cross
        if denominator == 0:
            return 0
        return (self.mass / 6) * (numerator / denominator)
    
    def get_world_vertices(self):
        world_vertices = []
        cos_a = math.cos(self.angle)
        sin_a = math.sin(self.angle)
        for (lx , ly) in self.vertices:
            rx = lx * cos_a - ly * sin_a
            ry = lx * sin_a + ly * cos_a
            wx = self.position.x + rx
            wy = self.position.y + ry
            world_vertices.append((wx , wy))
        return world_vertices
    
    def apply_force(self , force):
        self.acceleration.x += force.x / self.mass
        self.acceleration.y += force.y / self.mass

    def update(self, dt):
        self.old_acceleration = Vector2D(self.acceleration.x, self.acceleration.y)
        # linear
        self.position.x += self.velocity.x * dt + 0.5 * self.acceleration.x * dt**2
        self.position.y += self.velocity.y * dt + 0.5 * self.acceleration.y * dt**2
        self.velocity.x += self.acceleration.x * dt
        self.velocity.y += self.acceleration.y * dt
        # rotational
        self.angle += self.angular_velocity * dt + 0.5 * self.angular_acceleration * dt**2
        self.angular_velocity += self.angular_acceleration * dt
        # reset
        self.acceleration = Vector2D(0, 0)
        self.angular_acceleration = 0.0
        self.time += dt

