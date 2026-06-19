from core.vectors import Vector2D
import threading

class World:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.particles = []
        self.springs = []
        self.rigid_bodies = []
        self.constraints = []
        self.gravity = Vector2D(0, -9.8)
        self.restitution = 0.9
        self.rigid_body_restitution = 0.5 
        self.drag_coefficient = 0
        self.mu = 0.1
        self.dt = 0.1
        self.paused = False
        self.lock = threading.Lock()
        self.selected = None

    def add_particle(self, particle):
        self.particles.append(particle)

    def add_spring(self , spring):
        self.springs.append(spring)

    def add_rigid_bodies(self , body):
        self.rigid_bodies.append(body)

    def add_constraint(self , constraint):
        self.constraints.append(constraint)

    def clear(self):
        self.particles = []
        self.springs = []
        self.rigid_bodies = []
        self.constraints = []
    
    def step(self, dt=None):
        with self.lock:
            if self.paused:
                return
            if dt is None:
                dt = self.dt
            
            for body in self.rigid_bodies:
                body.apply_force(Vector2D(self.gravity.x * body.mass, self.gravity.y * body.mass))
                body.update(dt)
                body.angular_velocity *= (0.99 ** (dt / 0.016))
                self._handle_rigid_body_boundries(body)
            
            for spring in self.springs:
                spring.apply_spring_force()
            
            for p in self.particles:
                if hasattr(p, 'pinned') and p.pinned:
                    p.acceleration = Vector2D(0, 0)
                    continue
                drag_force = Vector2D(-self.drag_coefficient * p.velocity.x , -self.drag_coefficient * p.velocity.y) 
                p.apply_force(Vector2D(self.gravity.x * p.mass, self.gravity.y * p.mass))
                p.apply_force(drag_force)
                p.update(dt)
                self._handle_boundaries(p)
            
            for i in range(len(self.rigid_bodies)):
                for j in range(i+1, len(self.rigid_bodies)):
                    b1 = self.rigid_bodies[i]
                    b2 = self.rigid_bodies[j]
                    result = b1.sat_collision(b2)
                    if result:
                        self._correct_rigid_body_positions(b1, b2, result)

            for _ in range(8):
                self._handle_collisions()
                self._handle_rigid_body_collisions()
                self._handle_particle_rigid_body_collisions()
            
            for _ in range(5):
                for constraint in self.constraints:
                    constraint.solve()
            
    def _handle_boundaries(self, p):
        # Lower Boundry for Particle
        if p.position.y <= p.radius:
            p.position.y = p.radius
            p.velocity.y *= -self.restitution
        # Upper Boundry for Particle
        if p.position.y >= self.height - p.radius:
            p.position.y = self.height - p.radius
            p.velocity.y *= -self.restitution
        # Left Boundry for Particle
        if p.position.x <= p.radius:
            p.position.x = p.radius
            p.velocity.x *= -self.restitution
        # Right Boundry for Particle
        if p.position.x >= self.width - p.radius:
            p.position.x = self.width - p.radius
            p.velocity.x *= -self.restitution
        
    def _handle_rigid_body_boundries(self , body):
        vertices = body.get_world_vertices()
        if not vertices:
            return
        x_coords = [vertice[0] for vertice in vertices]
        y_coords = [vertice[1] for vertice in vertices]
        
        # bottom
        if min(y_coords) <= 0:
            penetration = -min(y_coords)
            body.position.y += penetration
            body.velocity.y *= -self.restitution
            body.angular_velocity *= self.restitution
            if abs(body.acceleration.y) < 2.0:
                body.acceleration.y = 0
            if abs(body.acceleration.x) < 2.0:
                body.acceleration.x = 0
            if abs(body.angular_acceleration) < 0.1:
                body.angular_acceleration = 0
            if abs(body.velocity.y) < 2.0:
                body.velocity.y = 0
            if abs(body.velocity.x) < 2.0:
                body.velocity.x = 0
            if abs(body.angular_velocity) < 0.1:
                body.angular_velocity = 0
            
        
        # top
        if max(y_coords) >= self.height:
            penetration = max(y_coords) - self.height
            body.position.y -= penetration
            body.velocity.y *= -self.restitution
            body.angular_velocity *= self.restitution
        
        # left
        if min(x_coords) <= 0:
            penetration = -min(x_coords)
            body.position.x += penetration
            body.velocity.x *= -self.restitution
            body.angular_velocity *= self.restitution
        
        # right
        if max(x_coords) >= self.width:
            penetration = max(x_coords) - self.width
            body.position.x -= penetration
            body.velocity.x *= -self.restitution
            body.angular_velocity *= self.restitution
    
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

    def _resolve_rigid_body_collision(self, b1, b2, result):
        normal = list(result["normal"])
        depth = result["depth"]

        # relative velocity
        rel_vel_x = b1.velocity.x - b2.velocity.x
        rel_vel_y = b1.velocity.y - b2.velocity.y
        vel_along_normal = rel_vel_x * normal[0] + rel_vel_y * normal[1]

        # only resolve if approaching
        if vel_along_normal > 0:
            return

        # impulse
        impulse_scalar = -(1 + self.rigid_body_restitution) * vel_along_normal
        impulse_scalar /= (1/b1.mass + 1/b2.mass)

        b1.velocity.x += (impulse_scalar / b1.mass) * normal[0]
        b1.velocity.y += (impulse_scalar / b1.mass) * normal[1]
        b2.velocity.x -= (impulse_scalar / b2.mass) * normal[0]
        b2.velocity.y -= (impulse_scalar / b2.mass) * normal[1]

        tangent = (-normal[1], normal[0])
        vel_along_tangent = rel_vel_x * tangent[0] + rel_vel_y * tangent[1]
        friction_scalar = -vel_along_tangent / (1/b1.mass + 1/b2.mass)
        if abs(friction_scalar) > abs(impulse_scalar * self.mu):
            friction_scalar = -impulse_scalar * self.mu * (1 if vel_along_tangent > 0 else -1)
        b1.velocity.x += (friction_scalar / b1.mass) * tangent[0]
        b1.velocity.y += (friction_scalar / b1.mass) * tangent[1]
        b2.velocity.x -= (friction_scalar / b2.mass) * tangent[0]
        b2.velocity.y -= (friction_scalar / b2.mass) * tangent[1]

        # angular effect from friction
        cp = b1.find_contact_point(b2)
        if cp is None:
            cp = b2.find_contact_point(b1)

        if cp is not None:
            # vector from center of each body to contact point
            r1x = cp[0] - b1.position.x
            r1y = cp[1] - b1.position.y
            r2x = cp[0] - b2.position.x
            r2y = cp[1] - b2.position.y
        else:
            r1x = normal[0] * depth / 2
            r1y = normal[1] * depth / 2
            r2x = -normal[0] * depth / 2
            r2y = -normal[1] * depth / 2
            
        if b1.moment_of_inertia > 0:
            torque1 = r1x * impulse_scalar * normal[1] - r1y * impulse_scalar * normal[0]
            b1.angular_velocity += torque1 / b1.moment_of_inertia
        if b2.moment_of_inertia > 0:
            torque2 = r2x * impulse_scalar * normal[1] - r2y * impulse_scalar * normal[0]
            b2.angular_velocity -= torque2 / b2.moment_of_inertia

    def _handle_rigid_body_collisions(self):
        for i in range(len(self.rigid_bodies)):
            for j in range(i+1, len(self.rigid_bodies)):
                b1 = self.rigid_bodies[i]
                b2 = self.rigid_bodies[j]
                result = b1.sat_collision(b2)
                if result:
                    self._resolve_rigid_body_collision(b1, b2, result)
    
    def _correct_rigid_body_positions(self , b1 , b2 , result):
        normal = list(result["normal"])
        depth = result["depth"]
        total_mass = b1.mass + b2.mass
        slop = 0.5
        correction = 0.2
        corrected_depth = max(depth - slop, 0)
        direction_x = b1.position.x - b2.position.x
        direction_y = b1.position.y - b2.position.y
        dot = direction_x * normal[0] + direction_y * normal[1]
        if dot < 0:
            normal[0] = -normal[0]
            normal[1] = -normal[1]
        b1.position.x += normal[0] * corrected_depth * (b2.mass / total_mass) * correction
        b1.position.y += normal[1] * corrected_depth * (b2.mass / total_mass) * correction
        b2.position.x -= normal[0] * corrected_depth * (b1.mass / total_mass) * correction
        b2.position.y -= normal[1] * corrected_depth * (b1.mass / total_mass) * correction

    def _resolve_particle_rigid_body_collision(self, particle, body, result):
        normal = result["normal"]
        depth = result["depth"]

        # position correction
        particle.position.x += normal[0] * depth
        particle.position.y += normal[1] * depth

        # relative velocity
        rel_vel_x = particle.velocity.x - body.velocity.x
        rel_vel_y = particle.velocity.y - body.velocity.y
        vel_along_normal = rel_vel_x * normal[0] + rel_vel_y * normal[1]

        # only resolve if approaching
        if vel_along_normal > 0:
            return

        # impulse
        impulse_scalar = -(1 + self.restitution) * vel_along_normal
        impulse_scalar /= (1/particle.mass + 1/body.mass)

        particle.velocity.x += (impulse_scalar / particle.mass) * normal[0]
        particle.velocity.y += (impulse_scalar / particle.mass) * normal[1]
        body.velocity.x -= (impulse_scalar / body.mass) * normal[0]
        body.velocity.y -= (impulse_scalar / body.mass) * normal[1]

        # vector from body center to contact point
        # contact point is on particle surface toward body
        contact_x = particle.position.x - normal[0] * particle.radius
        contact_y = particle.position.y - normal[1] * particle.radius

        r_x = contact_x - body.position.x
        r_y = contact_y - body.position.y

        body_surface_velocity_x = body.velocity.x - body.angular_velocity * r_y
        body_surface_velocity_y = body.velocity.y + body.angular_velocity * r_x

        relative_vel_x = particle.velocity.x - body_surface_velocity_x
        relative_vel_y = particle.velocity.y - body_surface_velocity_y

        # torque = r cross impulse
        if body.moment_of_inertia > 0:
            torque = r_x * (-impulse_scalar * normal[1]) - r_y * (-impulse_scalar * normal[0])
            body.angular_velocity += torque / body.moment_of_inertia

    def _handle_particle_rigid_body_collisions(self):
        for body in self.rigid_bodies:
            for particle in self.particles:
                result = body.particle_collision(particle)
                if result:
                    self._resolve_particle_rigid_body_collision(particle, body, result)
