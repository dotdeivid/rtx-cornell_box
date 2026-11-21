"""Material metálico."""

from typing import Optional, Tuple
from src.geometry.hit_record import HitRecord
from src.materials.base import Material
from src.vector import Vec3
from src.ray import Ray
from src.utils import random_in_unit_sphere


class MetalMaterial(Material):
    """
    Material metálico con reflexión especular y rugosidad opcional (``fuzz``).
    """

    def __init__(self, albedo: Vec3, fuzz: float = 0.0) -> None:
        """
        Args:
            albedo: Color del metal
            fuzz: Rugosidad del metal clampeada a ``[0.0, 1.0]``
        """
        self.albedo = albedo
        self.fuzz = max(0.0, min(float(fuzz), 1.0))

    def scatter(
        self, ray: Ray, hit_record: HitRecord
    ) -> Tuple[bool, Optional[Vec3], Optional[Ray]]:
        """Refleja el rayo respecto a la normal de superficie."""
        reflected = ray.direction.reflect(hit_record.normal)

        if self.fuzz > 0.0:
            reflected = reflected + random_in_unit_sphere() * self.fuzz

        if reflected.dot(hit_record.normal) > 0.0:
            scattered_ray = Ray(hit_record.point, reflected)
            return True, self.albedo, scattered_ray

        return False, None, None

    def emitted(self) -> Vec3:
        """No emite luz."""
        return Vec3(0.0, 0.0, 0.0)
