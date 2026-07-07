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
            length = (edge[0]**2 + edge[1]**2) ** 0.5
            if length > 0:
                normal = (-edge[1] / length, edge[0] / length)
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
        axis_len = (best_axis[0]**2 + best_axis[1]**2) ** 0.5
        normal = (best_axis[0]/axis_len, best_axis[1]/axis_len)
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

    def find_contact_point(self, other):
        best_point = None
        best_depth = -float('inf')
        
        # check each vertex of other — is it inside self?
        axes = self.get_axes()
        for vertex in other.get_world_vertices():
            depth = float('inf')
            inside = True
            for i, axis in enumerate(axes):
                proj = vertex[0] * axis[0] + vertex[1] * axis[1]    # project vertex onto axis
                min_s, max_s = self.project_onto_axis(axis)     # project self onto axis
                if proj < min_s or proj > max_s:
                    inside = False
                    break
                # how deep is this vertex inside self along this axis
                d = min(proj - min_s, max_s - proj)
                depth = min(depth, d)
            if inside and depth > best_depth:
                best_depth = depth
                best_point = vertex
        return best_point
    
    def particle_collision(self, particle):
        vertices = self.get_world_vertices()
        n = len(vertices)
        min_dist = float('inf')
        best_normal = None
        best_depth = 0
        
        for i in range(n):
            ax, ay = vertices[i]
            bx, by = vertices[(i+1) % n]
            px, py = particle.position.x, particle.position.y
            
            # closest point on this edge to particle center
            dx, dy = bx - ax, by - ay
            t = ((px - ax) * dx + (py - ay) * dy) / (dx*dx + dy*dy + 1e-10)
            t = max(0.0, min(1.0, t))
            cx = ax + t * dx
            cy = ay + t * dy
            
            # distance from particle center to closest point
            dist = math.sqrt((px - cx)**2 + (py - cy)**2)
            
            if dist < min_dist:
                min_dist = dist
                if dist > 1e-10:
                    best_normal = ((px - cx) / dist, (py - cy) / dist)
                    best_depth = particle.radius - dist
                else:
                    best_normal = (0.0 , 1.0)
                    best_depth = particle.radius
        
        if min_dist < particle.radius:
            return {"normal": best_normal, "depth": best_depth}
        return None