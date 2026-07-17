import math
from core.chains_and_ropes import Link

class DistanceConstraint:
    def __init__(self , body_a , body_b , anchor_a , anchor_b , rest_length , stiffness = 0.5):
        self.body_a = body_a
        self.body_b = body_b
        self.anchor_a = anchor_a
        self.anchor_b = anchor_b
        if rest_length <= 0:
            raise ValueError("rest_length must be > 0")
        self.rest_length = rest_length
        if stiffness < 0 or stiffness > 1:
            raise ValueError("stiffness must be between 0 and 1")
        self.stiffness = stiffness

    def _get_world_anchor(self, body, anchor):
        if hasattr(body, 'angle'):
            # rotate anchor by body angle
            cos_a = math.cos(body.angle)
            sin_a = math.sin(body.angle)
            rx = anchor[0] * cos_a - anchor[1] * sin_a
            ry = anchor[0] * sin_a + anchor[1] * cos_a
            return (body.position.x + rx, body.position.y + ry)
        else:
            return (body.position.x + anchor[0], body.position.y + anchor[1])
    
    def solve(self):
        world_anchor_a = self._get_world_anchor(self.body_a, self.anchor_a)
        world_anchor_b = self._get_world_anchor(self.body_b, self.anchor_b)
        
        delta_x = world_anchor_b[0] - world_anchor_a[0]
        delta_y = world_anchor_b[1] - world_anchor_a[1]
        current_length = (delta_x**2 + delta_y**2) ** 0.5
        
        if current_length == 0:
            return
        
        difference = (current_length - self.rest_length) / current_length
        correction_x = delta_x * difference * self.stiffness
        correction_y = delta_y * difference * self.stiffness

        a_pinned = getattr(self.body_a, 'pinned', False)
        b_pinned = getattr(self.body_b, 'pinned', False)

        if a_pinned and b_pinned:
            return

        if a_pinned:
            # only move b, full correction
            self.body_b.position.x -= correction_x
            self.body_b.position.y -= correction_y
        elif b_pinned:
            # only move a, full correction
            self.body_a.position.x += correction_x
            self.body_a.position.y += correction_y
        else:
            # normal mass weighted correction
            total_mass = self.body_a.mass + self.body_b.mass
            mass_ratio_a = self.body_b.mass / total_mass
            mass_ratio_b = self.body_a.mass / total_mass
            self.body_a.position.x += correction_x * mass_ratio_a
            self.body_a.position.y += correction_y * mass_ratio_a
            self.body_b.position.x -= correction_x * mass_ratio_b
            self.body_b.position.y -= correction_y * mass_ratio_b
        if not a_pinned and not b_pinned:
            self.body_a.velocity.x += correction_x * mass_ratio_a
            self.body_a.velocity.y += correction_y * mass_ratio_a
            self.body_b.velocity.x -= correction_x * mass_ratio_b
            self.body_b.velocity.y -= correction_y * mass_ratio_b
        elif a_pinned:
            self.body_b.velocity.x -= correction_x
            self.body_b.velocity.y -= correction_y
        elif b_pinned:
            self.body_a.velocity.x += correction_x
            self.body_a.velocity.y += correction_y

class HingeConstraint:
    def __init__(self, body_a , body_b , anchor_a , anchor_b):
        self.body_a = body_a
        self.body_b = body_b
        self.anchor_a = anchor_a
        self.anchor_b = anchor_b
    
    def _get_world_anchor(self, body, anchor):
        if hasattr(body, 'angle'):
            cos_a = math.cos(body.angle)
            sin_a = math.sin(body.angle)
            rx = anchor[0] * cos_a - anchor[1] * sin_a
            ry = anchor[0] * sin_a + anchor[1] * cos_a
            return (body.position.x + rx, body.position.y + ry)
        else:
            return (body.position.x + anchor[0], body.position.y + anchor[1])
    
    def solve(self):
        world_anchor_a = self._get_world_anchor(self.body_a, self.anchor_a)
        world_anchor_b = self._get_world_anchor(self.body_b, self.anchor_b)
        
        delta_x = world_anchor_b[0] - world_anchor_a[0]
        delta_y = world_anchor_b[1] - world_anchor_a[1]
        
        r_a = (world_anchor_a[0] - self.body_a.position.x, world_anchor_a[1] - self.body_a.position.y)
        r_b = (world_anchor_b[0] - self.body_b.position.x, world_anchor_b[1] - self.body_b.position.y)

        a_pinned = getattr(self.body_a, 'pinned', False)
        b_pinned = getattr(self.body_b, 'pinned', False)
        if a_pinned and b_pinned:
            return

        I_a = getattr(self.body_a, 'moment_of_inertia', 0)
        I_b = getattr(self.body_b, 'moment_of_inertia', 0)

        n = (delta_x, delta_y)
        n_length = (n[0]**2 + n[1]**2) ** 0.5
        if n_length == 0:
            return

        n_unit = (n[0] / n_length, n[1] / n_length)
        cross_a = r_a[0] * n_unit[1] - r_a[1] * n_unit[0]
        cross_b = r_b[0] * n_unit[1] - r_b[1] * n_unit[0]

        inv_mass_a = 0.0 if a_pinned else 1 / self.body_a.mass
        inv_mass_b = 0.0 if b_pinned else 1 / self.body_b.mass

        w_a = inv_mass_a + ((cross_a**2) / I_a if I_a > 0 and not a_pinned else 0)
        w_b = inv_mass_b + ((cross_b**2) / I_b if I_b > 0 and not b_pinned else 0)
        if w_a + w_b == 0:
            return

        constraint_error = n_length
        lambda_ = constraint_error / (w_a + w_b)

        if not a_pinned:
            self.body_a.position.x += n_unit[0] * (inv_mass_a * lambda_)
            self.body_a.position.y += n_unit[1] * (inv_mass_a * lambda_)
            self.body_a.velocity.x += n_unit[0] * (inv_mass_a * lambda_)
            self.body_a.velocity.y += n_unit[1] * (inv_mass_a * lambda_)
            if I_a > 0:
                self.body_a.angle += cross_a * lambda_ / I_a
                self.body_a.angular_velocity += cross_a * lambda_ / I_a

        if not b_pinned:
            self.body_b.position.x -= n_unit[0] * (inv_mass_b * lambda_)
            self.body_b.position.y -= n_unit[1] * (inv_mass_b * lambda_)
            self.body_b.velocity.x -= n_unit[0] * (inv_mass_b * lambda_)
            self.body_b.velocity.y -= n_unit[1] * (inv_mass_b * lambda_)
            if I_b > 0:
                self.body_b.angle += -cross_b * lambda_ / I_b
                self.body_b.angular_velocity += -cross_b * lambda_ / I_b

class ChainConstraint:
    def __init__(self, world, body_a, body_b, anchor_a, anchor_b, n_links, stiffness=0.5, friction=0.1, visible_links=True):        
        self.world = world
        self.body_a = body_a
        self.body_b = body_b
        self.anchor_a = anchor_a
        self.anchor_b = anchor_b
        self.n_links = n_links
        self.stiffness = stiffness
        self.friction = friction
        self.visible_links = visible_links
        self.links = []
        self.segments = []

        start = self._get_world_anchor(body_a, anchor_a)
        end = self._get_world_anchor(body_b, anchor_b)

        for i in range(n_links - 1):
            t = (i + 1) / n_links
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1])
            link = Link(x, y, mass=1.0, friction=friction)
            self.links.append(link)
            world.add_link(link)

        if n_links <= 0:
            raise ValueError("n_links must be > 0")

        total_dist = ((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2) ** 0.5
        total_dist = max(total_dist, 1e-6)
        link_length = total_dist / n_links

        nodes = [self.body_a] + self.links + [self.body_b]
        for i in range(len(nodes) - 1):
            node_a = nodes[i]
            node_b = nodes[i + 1]
            seg_anchor_a = (0,0) if isinstance(node_a, Link) else self.anchor_a if i == 0 else (0,0)
            seg_anchor_b = (0,0) if isinstance(node_b, Link) else self.anchor_b if i == len(nodes) - 2 else (0,0)
            constraint = DistanceConstraint(node_a, node_b, seg_anchor_a, seg_anchor_b, link_length, stiffness)
            self.segments.append(constraint)
    
    def _get_world_anchor(self, body, anchor):
        if hasattr(body, 'angle'):
            # rotate anchor by body angle
            cos_a = math.cos(body.angle)
            sin_a = math.sin(body.angle)
            rx = anchor[0] * cos_a - anchor[1] * sin_a
            ry = anchor[0] * sin_a + anchor[1] * cos_a
            return (body.position.x + rx, body.position.y + ry)
        else:
            return (body.position.x + anchor[0], body.position.y + anchor[1])    

    def solve(self):
        for _ in range(4):
            for segment in self.segments:
                segment.solve()
            self._apply_friction()
    
    def _apply_friction(self):
        for i in range(len(self.links) - 1):
            a = self.links[i]
            b = self.links[i + 1]
            dx = b.position.x - a.position.x
            dy = b.position.y - a.position.y
            length = (dx**2 + dy**2) ** 0.5
            if length == 0:
                continue
            # Normal direction to the segment
            nx = -dy / length
            ny = dx / length
            # Relative velocity in the normal direction
            rel_vx = b.velocity.x - a.velocity.x
            rel_vy = b.velocity.y - a.velocity.y
            rel_normal_velocity = rel_vx * nx + rel_vy * ny

            normal_force = a.mass * 9.81
            max_friction = self.friction * normal_force
            reduced_mass = 1/((1/a.mass) + (1/b.mass))
            friction_impulse = max(-max_friction, min(max_friction, -rel_normal_velocity * reduced_mass))
            a.velocity.x -= friction_impulse * nx
            a.velocity.y -= friction_impulse * ny
            b.velocity.x += friction_impulse * nx
            b.velocity.y += friction_impulse * ny
            friction_impulse = max(-max_friction, min(max_friction, -rel_tangential_velocity * reduced_mass))
            a.velocity.x -= friction_impulse * tx
            a.velocity.y -= friction_impulse * ty
            b.velocity.x += friction_impulse * tx
            b.velocity.y += friction_impulse * ty
    