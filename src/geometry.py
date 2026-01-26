import math
import random
from src.vector import Vec3
from src.ray import Ray


class HitRecord:
    def __init__(self, t, point, normal, color, emission):
        self.t = t
        self.point = point
        self.normal = normal
        self.color = color
        self.emission = emission


class Sphere:
    def __init__(self, center: Vec3, radius: float, color: Vec3, emission=None):
        self.center = center
        self.radius = radius
        self.color = color
        # Si no se define, la esfera no emite luz (Vec3(0,0,0))
        self.emission = emission if emission else Vec3(0, 0, 0)

    def hit(self, ray: Ray):
        """
        Retorna la distancia 't' más cercana si hay impacto,
        de lo contrario retorna None.
        """
        oc = ray.origin - self.center

        a = ray.direction.dot(ray.direction)
        b = 2.0 * ray.direction.dot(oc)
        c = oc.dot(oc) - self.radius**2

        discriminant = b**2 - 4 * a * c

        if discriminant < 0:
            return None

        t = (-b - math.sqrt(discriminant)) / (2.0 * a)
        if t > 0:
            point = ray.point_at(t)
            # Calculamos la normal y la normalizamos
            normal = (point - self.center) / self.radius
            return HitRecord(t, point, normal, self.color, self.emission)

        return None


    def random_point_on_surface(self):
        """
        Genera un punto aleatorio en la superficie de la esfera.
        """
        # Generamos una dirección aleatoria uniforme en una esfera
        theta = 2 * math.pi * random.random()
        phi = math.acos(2 * random.random() - 1)

        dx = math.sin(phi) * math.cos(theta)
        dy = math.sin(phi) * math.sin(theta)
        dz = math.cos(phi)

        direction = Vec3(dx, dy, dz)
        # El punto es: centro + (dirección_unitaria * radio)
        return self.center + direction * self.radius
