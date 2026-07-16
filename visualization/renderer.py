import pygame
import random
from core.vectors import Vector2D
from core.particles import Particle
from simulation.world import World

class Renderer:
    def __init__(self, world, title="Physics Engine"):
        self.world = world
        pygame.init()
        self.screen = pygame.display.set_mode((world.width, world.height))
        pygame.display.set_caption(title)
        self.clock = pygame.time.Clock()

    def to_screen(self, x, y):
        return int(x), int(self.world.height - y)

    def from_screen(self, sx, sy):
        return float(sx), float(self.world.height - sy)

    def draw_grid(self, spacing=50):
        grid_color = (30, 30, 30)  # dark grey, subtle
        font = pygame.font.SysFont("monospace", 10)
        # vertical lines
        for x in range(0, self.world.width, spacing):
            pygame.draw.line(self.screen, grid_color, (x, 0), (x, self.world.height))
            label = font.render(str(x), True, (60, 60, 60))
            self.screen.blit(label, (x + 2, self.world.height - 15))
        
        # horizontal lines
        for y in range(0, self.world.height, spacing):
            pygame.draw.line(self.screen, grid_color, (0, y), (self.world.width, y))
            label = font.render(str(y), True, (60, 60, 60))
            self.screen.blit(label, (2, self.world.height - y - 12))
            
    def draw_particle(self, particle):
        sx, sy = self.to_screen(particle.position.x, particle.position.y)
        pygame.draw.circle(self.screen, particle.color, (sx, sy), particle.radius)

    def draw_spring(self, spring):
        sx1, sy1 = self.to_screen(spring.p1.position.x, spring.p1.position.y)
        sx2, sy2 = self.to_screen(spring.p2.position.x, spring.p2.position.y)
        pygame.draw.line(self.screen, (100, 100, 255), (sx1, sy1), (sx2, sy2), 2)

    def draw_rigid_body(self , body):
        vertices = body.get_world_vertices()
        screen_vertices = [self.to_screen(x , y) for x , y in vertices]
        pygame.draw.polygon(self.screen , body.color , screen_vertices , 2)

    def draw_sat_debug(self, body):
        vertices = body.get_world_vertices()
        axes = body.get_axes()
        cx, cy = self.to_screen(body.position.x, body.position.y)
        
        for axis in axes:
            # normalize axis
            length = (axis[0]**2 + axis[1]**2) ** 0.5
            if length == 0:
                continue
            nx, ny = axis[0]/length, axis[1]/length
            
            # draw axis line through center
            scale = 60
            x1 = int(cx - nx * scale)
            y1 = int(cy + ny * scale)  # flip y
            x2 = int(cx + nx * scale)
            y2 = int(cy - ny * scale)  # flip y
            pygame.draw.line(self.screen, (255, 255, 0), (x1, y1), (x2, y2), 1)

    def draw_constraint(self, constraint):
        from core.constraints import HingeConstraint, DistanceConstraint, ChainConstraint
        if isinstance(constraint, HingeConstraint):
            self._draw_hinge(constraint)
        elif isinstance(constraint, DistanceConstraint):
            self._draw_distance(constraint)
        elif isinstance(constraint, ChainConstraint):
            self._draw_chain(constraint)
    
    def _draw_hinge(self , constraint):
        anchor = constraint._get_world_anchor(constraint.body_a, constraint.anchor_a)
        sx ,sy = self.to_screen(anchor[0] , anchor[1])
        pygame.draw.circle(self.screen , (255 , 0 , 0) , (sx , sy) , 1)

    def _draw_distance(self , constraint):
        anchor_a = constraint._get_world_anchor(constraint.body_a, constraint.anchor_a)
        anchor_b = constraint._get_world_anchor(constraint.body_b, constraint.anchor_b)
        sx1 , sy1 = self.to_screen(anchor_a[0] , anchor_a[1])
        sx2 , sy2 = self.to_screen(anchor_b[0] , anchor_b[1])
        pygame.draw.line(self.screen , (255 , 0 , 0) , (sx1 , sy1) , (sx2 , sy2) , 2)

    def _draw_chain(self , constraint):
        for segment in constraint.segments:
            anchor_a = segment._get_world_anchor(segment.body_a, segment.anchor_a)
            anchor_b = segment._get_world_anchor(segment.body_b, segment.anchor_b)
            sx1 , sy1 = self.to_screen(anchor_a[0] , anchor_a[1])
            sx2 , sy2 = self.to_screen(anchor_b[0] , anchor_b[1])
            pygame.draw.line(self.screen , (255 , 0 , 0) , (sx1 , sy1) , (sx2 , sy2) , 2)
        sx ,sy = self.to_screen(constraint.position.x , constraint.position.y)
        pygame.draw.circle(self.screen , (255 , 0 , 0) , (sx , sy) , 1)

    def _draw_distance(self , constraint):
        sx1 , sy1 = self.to_screen(constraint.anchor_a.position.x , constraint.anchor_a.position.y)
        sx2 , sy2 = self.to_screen(constraint.anchor_b.position.x , constraint.anchor_b.position.y)
        pygame.draw.line(self.screen , (255 , 0 , 0) , (sx1 , sy1) , (sx2 , sy2) , 2)

    def _draw_chain(self , constraint):
        pass
    
    def _point_in_polygon(self, x, y, vertices):
        inside = False
        for i in range(len(vertices)):
            j = (i + 1) % len(vertices)
            xi, yi = vertices[i]
            xj, yj = vertices[j]
            intersect = ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi)
            if intersect:
                inside = not inside
        return inside

    def _try_select(self, wx, wy):
        selected = None
        with self.world.lock:
            for p in self.world.particles:
                dist = ((p.position.x - wx)**2 + (p.position.y - wy)**2) ** 0.5
                if dist <= p.radius:
                    selected = p
                    break
            if selected is None:
                for body in self.world.rigid_bodies:
                    vertices = body.get_world_vertices()
                    if vertices and self._point_in_polygon(wx, wy, vertices):
                        selected = body
                        break
            self.world.selected = selected

    def run(self):
        accumulator = 0.0
        physics_dt = 0.008
        last_time = pygame.time.get_ticks() / 1000.0
        running = True
        while running:
            current_time = pygame.time.get_ticks() / 1000.0
            frame_time = min(current_time - last_time , 0.05)
            last_time = current_time
            accumulator += frame_time

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    mx, my = event.pos
                    wx, wy = self.from_screen(mx, my)
                    if event.button == 1:
                        self._try_select(wx, wy)
                        continue
                    elif event.button == 3:
                        mass = 5.0
                    else:
                        continue

                    color = (
                        random.randint(0, 255),
                        random.randint(0, 255),
                        random.randint(0, 255),
                    )
                    particle = Particle(wx, wy, mass, color=color)
                    # particle.velocity = Vector2D(random.uniform(-50, 50), random.uniform(0, 150))
                    self.world.add_particle(particle)
            
            while accumulator >= physics_dt:
                self.world.step(physics_dt)
                accumulator -= physics_dt

            self.screen.fill((0, 0, 0))
            self.draw_grid()
            for spring in self.world.springs:
                self.draw_spring(spring)
            for p in self.world.particles:
                self.draw_particle(p)
            for body in self.world.rigid_bodies:
                self.draw_rigid_body(body)
                self.draw_sat_debug(body)
            for constraint in self.world.constraints:
                self.draw_constraint(constraint)
            pygame.display.flip()
            self.clock.tick(60)

        pygame.quit()

    