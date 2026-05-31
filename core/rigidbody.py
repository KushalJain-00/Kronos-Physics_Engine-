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

    def get_axes(self):
        vertices = self.get_world_vertices()
        normals = []
        n = len(vertices)
        for i in range(n):
            p1 = vertices[i]
            p2 = vertices[(i+1) % n]
            edge = (p2[0] - p1[0], p2[1] - p1[1])
            normal = (-edge[1], edge[0])
            normals.append(normal)
        return normals

    def project_onto_axis(self, axis):
        vertices = self.get_world_vertices()
        projections = [v[0] * axis[0] + v[1] * axis[1] for v in vertices]
        return min(projections), max(projections)
    
    def sat_collision(self, other):
        axes = self.get_axes() + other.get_axes()
        min_overlap = float('inf')
        best_axis = None
        for axis in axes:
            min_a, max_a = self.project_onto_axis(axis)
            min_b, max_b = other.project_onto_axis(axis)
            if max_a < min_b or max_b < min_a:
                return None
            overlap = min(max_a, max_b) - max(min_a, min_b)
            if overlap < min_overlap:
                min_overlap = overlap
                best_axis = axis
        length = (best_axis[0]**2 + best_axis[1]**2) ** 0.5
        if length == 0:
            return None
        normal = (best_axis[0]/length, best_axis[1]/length)
        return {"normal": normal, "depth": min_overlap}

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

