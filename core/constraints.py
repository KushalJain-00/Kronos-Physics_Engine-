import math
from core.chains_and_ropes import Link

class DistanceConstraint:
    def __init__(self , body_a , body_b , anchor_a , anchor_b , rest_length , stiffness = 0.5):
        self.body_a = body_a
        self.body_b = body_b
        self.anchor_a = anchor_a
        self.anchor_b = anchor_b
        if rest_length < 0:
            raise ValueError("rest_length must be >= 0")
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

        a_pinned = getattr(self.body_a, 'pinned', False)
        b_pinned = getattr(self.body_b, 'pinned', False)
        if a_pinned and b_pinned:
            return

        inv_mass_a = 0.0 if a_pinned else 1 / self.body_a.mass
        inv_mass_b = 0.0 if b_pinned else 1 / self.body_b.mass

        n_unit = (delta_x / current_length, delta_y / current_length)
        r_a = (world_anchor_a[0] - self.body_a.position.x, world_anchor_a[1] - self.body_a.position.y)
        r_b = (world_anchor_b[0] - self.body_b.position.x, world_anchor_b[1] - self.body_b.position.y)
        I_a = getattr(self.body_a, 'moment_of_inertia', 0)
        I_b = getattr(self.body_b, 'moment_of_inertia', 0)
        cross_a = r_a[0] * n_unit[1] - r_a[1] * n_unit[0]
        cross_b = r_b[0] * n_unit[1] - r_b[1] * n_unit[0]

        # generalized inverse mass: linear + rotational (r x n)^2/I term moves the
        # anchor along n exactly like a translation of inv_mass — rigid bodies get torque
        w_a = inv_mass_a + (cross_a**2 / I_a if I_a > 0 and not a_pinned else 0)
        w_b = inv_mass_b + (cross_b**2 / I_b if I_b > 0 and not b_pinned else 0)
        if w_a + w_b == 0:
            return

        lambda_ = (current_length - self.rest_length) * self.stiffness / (w_a + w_b)

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

class HingeConstraint(DistanceConstraint):
    def __init__(self, body_a, body_b, anchor_a, anchor_b, stiffness=1.0):
        super().__init__(body_a, body_b, anchor_a, anchor_b, rest_length=0.0, stiffness=stiffness)

class ChainConstraint:
    def __init__(self, world, body_a, body_b, anchor_a, anchor_b, n_links, stiffness=0.5, friction=0.1, iterations=4, visible_links=True):        
        if n_links <= 0:
            raise ValueError("n_links must be > 0")
        self.world = world
        self.body_a = body_a
        self.body_b = body_b
        self.anchor_a = anchor_a
        self.anchor_b = anchor_b
        self.n_links = n_links
        self.stiffness = stiffness
        self.friction = friction
        self.iterations = iterations
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
        for _ in range(self.iterations):
            for segment in self.segments:
                segment.solve()
            self._apply_friction()

    def _apply_friction(self):
        # friction at the anchor joints too, not just link-to-link
        nodes = [self.body_a] + self.links + [self.body_b]
        for i in range(len(nodes) - 1):
            a = nodes[i]
            b = nodes[i + 1]
            if getattr(a, 'pinned', False) or getattr(b, 'pinned', False):
                continue
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

            # ponytail: friction cap assumes normal force ~ link weight in world gravity;
            # real rope tension would need per-segment force accumulation
            normal_force = a.mass * abs(self.world.gravity.y)
            max_friction = self.friction * normal_force
            reduced_mass = 1/((1/a.mass) + (1/b.mass))
            friction_impulse = max(-max_friction, min(max_friction, -rel_normal_velocity * reduced_mass))
            a.velocity.x -= friction_impulse * nx
            a.velocity.y -= friction_impulse * ny
            b.velocity.x += friction_impulse * nx
            b.velocity.y += friction_impulse * ny
    
class AngleConstraint:
    def __init__(self, body_a, body_b, max_angle, min_angle, stiffness = 0.5):
        if min_angle > max_angle:
            raise ValueError("min_angle must not be greater than max_angle")
        if stiffness < 0 or stiffness > 1:
            raise ValueError("stiffness must be between 0 and 1")
        self.body_a = body_a
        self.body_b = body_b
        # degrees at construction, radians internally (RigidBody.angle is radians)
        self.max_angle = math.radians(max_angle)
        self.min_angle = math.radians(min_angle)
        self.stiffness = stiffness

    def solve(self):
        a_pinned = getattr(self.body_a, 'pinned', False)
        b_pinned = getattr(self.body_b, 'pinned', False)
        i_a = getattr(self.body_a, 'moment_of_inertia', 0)
        i_b = getattr(self.body_b, 'moment_of_inertia', 0)
        inv_a = 0.0 if a_pinned else (1 / i_a if i_a > 0 else 0)
        inv_b = 0.0 if b_pinned else (1 / i_b if i_b > 0 else 0)
        w = inv_a + inv_b
        if w == 0:
            return
        # wrap relative angle to [-pi, pi]: an unwrapped difference spins the
        # constraint into huge corrections once either body passes a full turn
        relative_angle = (self.body_b.angle - self.body_a.angle) % (2 * math.pi)
        if relative_angle > math.pi:
            relative_angle -= 2 * math.pi
        if relative_angle > self.max_angle:
            correction = (relative_angle - self.max_angle) * self.stiffness
            self.body_b.angle -= correction * (inv_b / w)
            self.body_a.angle += correction * (inv_a / w)
            self.body_b.angular_velocity -= correction * (inv_b / w)
            self.body_a.angular_velocity += correction * (inv_a / w)
        elif relative_angle < self.min_angle:
            correction = (self.min_angle - relative_angle) * self.stiffness
            self.body_b.angle += correction * (inv_b / w)
            self.body_a.angle -= correction * (inv_a / w)
            self.body_b.angular_velocity += correction * (inv_b / w)
            self.body_a.angular_velocity -= correction * (inv_a / w)


class MotorConstraint:
    def __init__(self, body_a, body_b, target_angular_velocity=0.0, stiffness=1.0):
        if stiffness < 0 or stiffness > 1:
            raise ValueError("stiffness must be between 0 and 1")
        self.body_a = body_a
        self.body_b = body_b
        self.target_angular_velocity = target_angular_velocity
        self.stiffness = stiffness

    def solve(self):
        # pure velocity motor: drive relative angular velocity (omega_b - omega_a) to the target
        a_pinned = getattr(self.body_a, 'pinned', False)
        b_pinned = getattr(self.body_b, 'pinned', False)
        I_a = getattr(self.body_a, 'moment_of_inertia', 0)
        I_b = getattr(self.body_b, 'moment_of_inertia', 0)
        inv_a = 0.0 if a_pinned else (1 / I_a if I_a > 0 else 0)
        inv_b = 0.0 if b_pinned else (1 / I_b if I_b > 0 else 0)
        w = inv_a + inv_b
        if w == 0:
            return
        error = self.target_angular_velocity - (self.body_b.angular_velocity - self.body_a.angular_velocity)
        correction = error * self.stiffness
        if not a_pinned:
            self.body_a.angular_velocity -= correction * (inv_a / w)
        if not b_pinned:
            self.body_b.angular_velocity += correction * (inv_b / w)


class WeldConstraint:
    def __init__(self, body_a, body_b, anchor_a, anchor_b, stiffness=1.0):
        self.body_a = body_a
        self.body_b = body_b
        self.anchor_a = anchor_a
        self.anchor_b = anchor_b
        self._hinge = HingeConstraint(body_a, body_b, anchor_a, anchor_b, stiffness=stiffness)
        # lock relative angle to its current (wrapped) value — that's what "zero relative DOF" means
        rel = (body_b.angle - body_a.angle) % (2 * math.pi)
        if rel > math.pi:
            rel -= 2 * math.pi
        self._angle = AngleConstraint(body_a, body_b, max_angle=math.degrees(rel), min_angle=math.degrees(rel), stiffness=stiffness)

    def _get_world_anchor(self, body, anchor):
        return self._hinge._get_world_anchor(body, anchor)

    def solve(self):
        self._hinge.solve()
        self._angle.solve()