"""Esfera - primitiva geométrica básica para ray tracing."""

import math
import random
from typing import Optional, Tuple
from src.vector import Vec3
from src.ray import Ray
from src.geometry.hit_record import HitRecord
from src.geometry.aabb import AABB


class Sphere:
    """
    Esfera en el espacio 3D. Definida por centro, radio y material.

    Atributos:
        center (Vec3): Centro de la esfera
        radius (float): Radio de la esfera
        material (Material): Material de la esfera
    """

    def __init__(
        self,
        center: Vec3,
        radius: float,
        material: "Material",
    ):
        """
        Inicializa una esfera con su geometría y material.

        Args:
            center: Posición del centro de la esfera
            radius: Radio de la esfera
            material: Material de la esfera (Material ABC)

        Ejemplo:
            >>> from src.materials import DiffuseMaterial
            >>> mat = DiffuseMaterial(Vec3(0.8, 0.3, 0.3))
            >>> sphere = Sphere(Vec3(0, 0, 0), 1.0, mat)
        """
        self.center = center
        self.radius = radius
        self.material = material

    def hit(self, ray: Ray, t_min=0.001, t_max=float("inf")):
        """
        Intersección rayo-esfera mediante ecuación cuadrática.

        Resuelve (O + t*D - C)·(O + t*D - C) = r². Retorna la raíz menor válida
        en [t_min, t_max]; si no, la mayor; si ninguna, None.

        Args:
            ray (Ray): El rayo a probar
            t_min (float): Distancia mínima válida (0.001 evita shadow acne)
            t_max (float): Distancia máxima válida

        Returns:
            HitRecord o None: Información del impacto más cercano, o None
        """
        oc = ray.origin - self.center

        a = ray.direction.dot(ray.direction)
        b = 2.0 * ray.direction.dot(oc)
        c = oc.dot(oc) - self.radius**2

        discriminant = b**2 - 4 * a * c

        if discriminant < 0:
            return None

        sqrtd = math.sqrt(discriminant)

        # Encontramos la raíz más cercana que esté en el rango aceptable
        root = (-b - sqrtd) / (2.0 * a)
        if root <= t_min or t_max <= root:
            root = (-b + sqrtd) / (2.0 * a)
            if root <= t_min or t_max <= root:
                return None

        t = root
        point = ray.point_at(t)
        # Calculamos la normal y la normalizamos
        normal = (point - self.center) / self.radius

        return HitRecord(
            t=t,
            point=point,
            normal=normal,
            material=self.material,
            obj_ref=self,
        )

    def random_point_on_surface(self):
        """
        Genera un punto aleatorio uniformemente distribuido en la superficie de la esfera.

        Usa coordenadas esféricas con phi = arccos(2u - 1) para garantizar
        distribución uniforme en área (sin acumulación en los polos).

        Returns:
            Vec3: Punto aleatorio en la superficie
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
        Muestreo de ángulo sólido para importance sampling de luces esféricas.

        Genera una dirección dentro del cono que subtiende la esfera vista desde hit_point
        y calcula el ángulo sólido Ω = 2π(1 - cos(θ_max)).

        Args:
            hit_point (Vec3): Punto desde el cual se muestrea la luz

        Returns:
            tuple: (dirección: Vec3, ángulo_sólido: float) en estereorradianes
        """
        direction_to_center = self.center - hit_point
        dist_sq = direction_to_center.dot(direction_to_center)

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
            math.cos(phi) * sin_theta, math.sin(phi) * sin_theta, cos_theta
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

    def bounding_box(self):
        """
        Returns:
            AABB: Caja envolvente mínima de la esfera (centro ± radio en cada eje)
        """
        return AABB(
            self.center - Vec3(self.radius, self.radius, self.radius),
            self.center + Vec3(self.radius, self.radius, self.radius),
        )
