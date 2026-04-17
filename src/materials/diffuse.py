"""Material difuso (Lambertiano)."""

from typing import Optional, Tuple
from src.geometry import HitRecord
from src.materials.base import Material
from src.vector import Vec3
from src.ray import Ray
from src.utils import random_in_unit_sphere


class DiffuseMaterial(Material):
    """
    Material difuso simple.

    Refleja la luz en todas las direcciones de manera uniforme.
    """

    def __init__(self, albedo: Vec3):
        """
        Args:
            albedo: Color base del material (reflectancia)
        """
        self.albedo = albedo

    def scatter(
        self, ray: Ray, hit_record: HitRecord
    ) -> Tuple[bool, Optional[Vec3], Optional[Ray]]:
        """Dispersa el rayo de manera difusa."""
        # Dirección aleatoria en hemisferio
        scatter_direction = hit_record.normal + random_in_unit_sphere().normalize()

        # Prevenir dirección cero
        if abs(scatter_direction.length()) < 1e-8:
            scatter_direction = hit_record.normal

        scattered_ray = Ray(hit_record.point, scatter_direction)
        return True, self.albedo, scattered_ray

    def emitted(self) -> Vec3:
        """No emite luz."""
        return Vec3(0, 0, 0)
