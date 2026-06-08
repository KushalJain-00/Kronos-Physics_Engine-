import math

class DistanceConstraint:
    def __init__(self , body_a , body_b , anchor_a , anchor_b , rest_length , stiffness = 0.5):
        self.body_a = body_a
        self.body_b = body_b
        self.anchor_a = anchor_a
        self.anchor_b = anchor_b
        self.rest_length = rest_length
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
        # update velocities to match position correction
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