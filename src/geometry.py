import math
import random
from src.vector import Vec3
from src.ray import Ray


class HitRecord:
    def __init__(self, t, point, normal, color, emission, is_metal=False, fuzz=0.0):
        self.t = t
        self.point = point
        self.normal = normal
        self.color = color
        self.emission = emission
        self.is_metal = is_metal
        self.fuzz = fuzz


class Sphere:
    def __init__(self, center: Vec3, radius: float, color: Vec3, emission=None, is_metal=False, fuzz=0.0):
        self.center = center
        self.radius = radius
        self.color = color
        # Si no se define, la esfera no emite luz (Vec3(0,0,0))
        self.emission = emission if emission else Vec3(0, 0, 0)
        self.is_metal = is_metal
        self.fuzz = fuzz if fuzz <= 1.0 else 1.0 # Limitamos a 1.0

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
            return HitRecord(t, point, normal, self.color, self.emission, self.is_metal, self.fuzz)

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

    def sample_solid_angle(self, hit_point):
        """
        Genera una dirección aleatoria dentro del cono que subtiende 
        la esfera desde el punto de choque (Muestreo de Cono Sólido).
        """
        direction_to_center = self.center - hit_point
        dist_sq = direction_to_center.dot(direction_to_center)
        dist = math.sqrt(dist_sq)
        
        # Dirección normalizada hacia el centro de la luz
        z_axis = direction_to_center.normalize()
        
        # Calculamos el ángulo máximo del cono (sin(theta) = R/d)
        # Si estamos dentro de la luz, el radio es 1 (toda la esfera)
        sin_theta_max_sq = (self.radius * self.radius) / dist_sq
        cos_theta_max = math.sqrt(max(0, 1 - sin_theta_max_sq))
        
        # Muestreo aleatorio dentro del cono
        r1 = random.random()
        r2 = random.random()
        
        phi = 2 * math.pi * r1
        # cos_theta varía entre 1 (centro) y cos_theta_max (borde del cono)
        cos_theta = 1 - r2 * (1 - cos_theta_max)
        sin_theta = math.sqrt(max(0, 1 - cos_theta * cos_theta))
        
        # Dirección en espacio local del cono
        local_dir = Vec3(
            math.cos(phi) * sin_theta,
            math.sin(phi) * sin_theta,
            cos_theta
        )
        
        # Transformar a espacio global (creando una base ortonormal simple)
        # Buscamos un vector no paralelo a z_axis
        helper = Vec3(1, 0, 0) if abs(z_axis.x) < 0.8 else Vec3(0, 1, 0)
        x_axis = z_axis.cross(helper).normalize()
        y_axis = z_axis.cross(x_axis)
        
        world_dir = x_axis * local_dir.x + y_axis * local_dir.y + z_axis * local_dir.z
        
        # El Ángulo Sólido (Omega) es la "cantidad de cielo" que ocupa la luz
        solid_angle = 2 * math.pi * (1 - cos_theta_max)
        
        return world_dir.normalize(), solid_angle
