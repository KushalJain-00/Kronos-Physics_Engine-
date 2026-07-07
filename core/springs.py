from core.vectors import Vector2D

class Spring:
    def __init__(self , p1 , p2 , length: float , k: float = 0.1 , damp: float = 0.02):
        self.p1 = p1
        self.p2 = p2
        self.rest_length = length
        self.spring_const = k
        self.damping = damp
    
    def apply_spring_force(self , p1: Vector2D = None , p2: Vector2D = None):
        p1 = p1 or self.p1      
        p2 = p2 or self.p2
        current_length = p1.position.distance(p2.position)
        if current_length == 0:
            return
        spring_force = self.spring_const * (current_length - self.rest_length)
        dx = (p2.position.x - p1.position.x) / current_length
        dy = (p2.position.y - p1.position.y) / current_length
        # damping — opposes relative velocity along spring axis
        dvx = p2.velocity.x - p1.velocity.x
        dvy = p2.velocity.y - p1.velocity.y
        damp_force = self.damping * (dvx * dx + dvy * dy)
        total = spring_force + damp_force
        
        # apply to both particles — equal and opposite
        p1.apply_force(Vector2D(total * dx, total * dy))
        p2.apply_force(Vector2D(-total * dx, -total * dy))
        # print(f"length: {current_length}, force: {spring_force}")