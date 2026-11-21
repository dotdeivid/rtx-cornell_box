"""Material dieléctrico (vidrio, agua)."""

import math
import random
from typing import Optional, Tuple

from src.geometry.hit_record import HitRecord
from src.materials.base import Material
from src.vector import Vec3
from src.ray import Ray


class DielectricMaterial(Material):
    """
    Material dieléctrico transparente (refracción de Snell + Fresnel-Schlick).
    """

    def __init__(self, ior: float) -> None:
        """
        Args:
            ior: Índice de refracción (debe ser > 0; ej. 1.0 aire, 1.33 agua, 1.5 vidrio)
        """
        if ior <= 0.0:
            raise ValueError("El índice de refracción (ior) debe ser positivo")
        self.ior = float(ior)

    def scatter(
        self, ray: Ray, hit_record: HitRecord
    ) -> Tuple[bool, Optional[Vec3], Optional[Ray]]:
        """Refracta o refleja el rayo según las leyes de Snell y Fresnel."""
        attenuation = Vec3(1.0, 1.0, 1.0)
        unit_direction = ray.direction

        # Compatibilidad tanto con HitRecord orientado por set_face_normal()
        # como con HitRecord construido manualmente en tests.
        dot_dn = unit_direction.dot(hit_record.normal)
        if not hit_record.front_face:
            # Saliendo del material; normal ya apunta hacia adentro (contra el rayo)
            normal = hit_record.normal if dot_dn <= 0.0 else -hit_record.normal
            ri = self.ior
        elif dot_dn > 0.0:
            # HitRecord manual saliendo sin haber llamado set_face_normal
            normal = -hit_record.normal
            ri = self.ior
        else:
            # Entrando al material
            normal = hit_record.normal
            ri = 1.0 / self.ior

        cos_theta = min(-unit_direction.dot(normal), 1.0)
        sin_theta = math.sqrt(max(0.0, 1.0 - cos_theta * cos_theta))

        cannot_refract = ri * sin_theta > 1.0

        if cannot_refract or self._reflectance(cos_theta, ri) > random.random():
            direction = unit_direction.reflect(normal)
        else:
            direction = self._refract(unit_direction, normal, ri)

        scattered_ray = Ray(hit_record.point, direction)
        return True, attenuation, scattered_ray

    def emitted(self) -> Vec3:
        """No emite luz."""
        return Vec3(0.0, 0.0, 0.0)

    @staticmethod
    def _reflectance(cosine: float, ref_idx: float) -> float:
        """Aproximación de Schlick para reflectancia de Fresnel."""
        r0 = (1.0 - ref_idx) / (1.0 + ref_idx)
        r0 = r0 * r0
        return r0 + (1.0 - r0) * ((1.0 - cosine) ** 5)

    @staticmethod
    def _refract(uv: Vec3, n: Vec3, etai_over_etat: float) -> Vec3:
        """Calcula la dirección refractada usando la ley de Snell vectorial."""
        cos_theta = min(-uv.dot(n), 1.0)
        r_out_perp = (uv + n * cos_theta) * etai_over_etat
        r_out_parallel = n * (-math.sqrt(abs(1.0 - r_out_perp.length_squared())))
        return r_out_perp + r_out_parallel
