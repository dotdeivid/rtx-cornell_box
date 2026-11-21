"""Esfera - primitiva geométrica básica para ray tracing."""

import math
import random
from typing import Optional, Tuple, TYPE_CHECKING

from src.vector import Vec3
from src.ray import Ray
from src.geometry.base import Geometry
from src.geometry.hit_record import HitRecord
from src.geometry.aabb import AABB

if TYPE_CHECKING:
    from src.materials.base import Material


class Sphere(Geometry):
    """
    Esfera en el espacio 3D. Definida por centro, radio y material.
    """

    def __init__(
        self,
        center: Vec3,
        radius: float,
        material: "Material",
    ) -> None:
        if radius <= 0:
            raise ValueError("El radio de la esfera debe ser positivo")
        self.center = center
        self.radius = float(radius)
        self.material = material
        r_vec = Vec3(self.radius, self.radius, self.radius)
        self._bbox = AABB(self.center - r_vec, self.center + r_vec)

    def hit(
        self, ray: Ray, t_min: float = 0.001, t_max: float = float("inf")
    ) -> Optional[HitRecord]:
        """
        Intersección rayo-esfera mediante ecuación cuadrática optimizada (half_b).
        """
        oc = ray.origin - self.center

        a = ray.direction.length_squared()
        half_b = oc.dot(ray.direction)
        c = oc.length_squared() - self.radius * self.radius

        discriminant = half_b * half_b - a * c
        if discriminant < 0.0:
            return None

        sqrtd = math.sqrt(discriminant)

        root = (-half_b - sqrtd) / a
        if root <= t_min or t_max <= root:
            root = (-half_b + sqrtd) / a
            if root <= t_min or t_max <= root:
                return None

        t = root
        point = ray.point_at(t)
        outward_normal = (point - self.center) / self.radius

        rec = HitRecord(
            t=t,
            point=point,
            normal=outward_normal,
            material=self.material,
            obj_ref=self,
        )
        rec.set_face_normal(ray, outward_normal)
        return rec

    def random_point_on_surface(self) -> Vec3:
        """Genera un punto aleatorio uniformemente distribuido en la superficie de la esfera."""
        theta = 2.0 * math.pi * random.random()
        phi = math.acos(2.0 * random.random() - 1.0)

        dx = math.sin(phi) * math.cos(theta)
        dy = math.sin(phi) * math.sin(theta)
        dz = math.cos(phi)

        return self.center + Vec3(dx, dy, dz) * self.radius

    def sample_solid_angle(self, hit_point: Vec3) -> Tuple[Vec3, float]:
        """
        Muestreo de ángulo sólido para importance sampling de luces esféricas.

        Returns:
            Tupla ``(dirección_unitaria, ángulo_sólido)`` en estereorradianes.
        """
        direction_to_center = self.center - hit_point
        dist_sq = direction_to_center.length_squared()
        if dist_sq <= self.radius * self.radius:
            # Punto dentro o sobre la esfera: toda la esfera (4π sr) es visible
            theta = 2.0 * math.pi * random.random()
            phi = math.acos(2.0 * random.random() - 1.0)
            uniform_dir = Vec3(
                math.sin(phi) * math.cos(theta),
                math.sin(phi) * math.sin(theta),
                math.cos(phi),
            )
            return uniform_dir, 4.0 * math.pi

        z_axis = direction_to_center.normalize()
        sin_theta_max_sq = (self.radius * self.radius) / dist_sq
        cos_theta_max = math.sqrt(max(0.0, 1.0 - sin_theta_max_sq))

        r1 = random.random()
        r2 = random.random()

        phi = 2.0 * math.pi * r1
        cos_theta = 1.0 - r2 * (1.0 - cos_theta_max)
        sin_theta = math.sqrt(max(0.0, 1.0 - cos_theta * cos_theta))

        local_dir = Vec3(
            math.cos(phi) * sin_theta, math.sin(phi) * sin_theta, cos_theta
        )

        helper = Vec3(1.0, 0.0, 0.0) if abs(z_axis.x) < 0.8 else Vec3(0.0, 1.0, 0.0)
        x_axis = z_axis.cross(helper).normalize()
        y_axis = z_axis.cross(x_axis)

        world_dir = x_axis * local_dir.x + y_axis * local_dir.y + z_axis * local_dir.z
        solid_angle = 2.0 * math.pi * (1.0 - cos_theta_max)

        return world_dir.normalize(), solid_angle

    def bounding_box(self) -> AABB:
        """Retorna la AABB precalculada de la esfera."""
        return self._bbox
