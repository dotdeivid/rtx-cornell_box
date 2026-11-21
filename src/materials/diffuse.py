"""Material difuso (Lambertiano)."""

from typing import Optional, Tuple
from src.geometry.hit_record import HitRecord
from src.materials.base import Material
from src.vector import Vec3
from src.ray import Ray
from src.utils import random_in_unit_sphere


class DiffuseMaterial(Material):
    """
    Material difuso Lambertiano con muestreo coseno-ponderado en el hemisferio.
    """

    uses_nee: bool = True

    def __init__(self, albedo: Vec3) -> None:
        self.albedo = albedo

    def scatter(
        self, ray: Ray, hit_record: HitRecord
    ) -> Tuple[bool, Optional[Vec3], Optional[Ray]]:
        """Dispersa el rayo de manera difusa alrededor de la normal orientada."""
        scatter_direction = hit_record.normal + random_in_unit_sphere().normalize()

        if scatter_direction.length_squared() < 1e-16:
            scatter_direction = hit_record.normal

        scattered_ray = Ray(hit_record.point, scatter_direction)
        return True, self.albedo, scattered_ray

    def emitted(self) -> Vec3:
        """No emite luz."""
        return Vec3(0.0, 0.0, 0.0)
