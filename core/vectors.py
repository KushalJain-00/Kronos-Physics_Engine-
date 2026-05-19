import numpy as np 

class Vector2D:
    def __init__(self , x: float , y: float):
        self.x = x
        self.y = y
    
    def add(self , other):
        return Vector2D(self.x + other.x , self.y + other.y)
    
    def sub(self , other):
        return Vector2D(self.x - other.x , self.y - other.y)
    
    def multiply(self , scalar):
        return Vector2D(self.x * scalar , self.y * scalar)
    
    def magnitude(self):
        return np.sqrt(self.x**2 + self.y**2)
    
    def dot_product(self , other):
        return (self.x * other.x) + (self.y * other.y)
    
    def normalize(self):
        mag = self.magnitude()
        if mag == 0:
            return Vector2D(0 , 0)
        return Vector2D(self.x / mag , self.y / mag)
    
    def distance(self , other):
        return np.sqrt(((other.x - self.x)**2) + ((other.y - self.y)**2))
    
    def angle(self , other):
        mag_a = self.magnitude()
        mag_b = other.magnitude()
        cos_theta = self.dot_product(other) / (mag_a * mag_b)
        cos_theta = max(-1.0, min(1.0, cos_theta))
        return np.degrees(np.arccos(cos_theta))

    def __repr__(self):
        return f"Vector2D({self.x:.2f}, {self.y:.2f})"
    
    