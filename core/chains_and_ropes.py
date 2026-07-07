from core.particles import Particle

class Link(Particle):
    def __init__(self , x , y , mass , friction = 0.3 , radius = 5):
        super().__init__(x , y , mass)
        self.friction = friction
        self.radius = radius
    
    