import math
from src.vector import Vec3
from src.ray import Ray

class HitRecord:
    def __init__(self, t, point, normal, color):
        self.t = t
        self.point = point
        self.normal = normal
        self.color = color

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

        t = (-b - math.sqrt(discriminant)) / (2.0*a)
        if t > 0:
            point = ray.point_at(t)
            # Calculamos la normal y la normalizamos
            normal = (point - self.center) / self.radius
            return HitRecord(t, point, normal, self.color)
        
        return None