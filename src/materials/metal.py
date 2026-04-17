"""Material metálico."""

from typing import Optional, Tuple
from src.geometry import HitRecord
from src.materials.base import Material
from src.vector import Vec3
from src.ray import Ray
from src.utils import random_in_unit_sphere


class MetalMaterial(Material):
    """
    Material metálico con reflexión.

    Refleja rayos perfectamente (o con fuzziness).
    """

    def __init__(self, albedo: Vec3, fuzz: float = 0.0):
        """
        Args:
            albedo: Color del metal
            fuzz: "Rugosidad" del metal (0 = espejo perfecto, 1 = muy rugoso)
        """
        self.albedo = albedo
        self.fuzz = min(fuzz, 1.0)  # Clamp a [0, 1]

    def scatter(
        self, ray: Ray, hit_record: HitRecord
    ) -> Tuple[bool, Optional[Vec3], Optional[Ray]]:
        """Refleja el rayo."""
        reflected = ray.direction.reflect(hit_record.normal)

        # Agregar fuzziness
        if self.fuzz > 0:
            reflected = reflected + random_in_unit_sphere() * self.fuzz

        scattered_ray = Ray(hit_record.point, reflected)

        # Solo dispersa si el rayo reflejado va hacia afuera
        scattered = reflected.dot(hit_record.normal) > 0

        if scattered:
            return True, self.albedo, scattered_ray
        else:
            return False, None, None

    def emitted(self) -> Vec3:
        """No emite luz."""
        return Vec3(0, 0, 0)
