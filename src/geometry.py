import math
from src.vector import Vec3
from src.ray import Ray

class Sphere:
    def __init__(self, center: Vec3, radius: float, color: Vec3):
        self.center = center
        self.radius = radius
        self.color = color

    def hit(self, ray: Ray):
        """
        Retorna la distancia 't' más cercana si hay impacto, 
        de lo contrario retorna None.
        """
        oc = ray.origin - self.center
        
        a = ray.direction.dot(ray.direction)
        b = 2.0 * ray.direction.dot(oc)
        c = oc.dot(oc) - self.radius**2
        
        discriminant = b**2 - 4*a*c
        
        if discriminant < 0:
            return None
        else:
            # Nos interesa la raíz más pequeña (la más cercana a la cámara)
            t = (-b - math.sqrt(discriminant)) / (2.0*a)
            if t > 0:
                return t
            return None