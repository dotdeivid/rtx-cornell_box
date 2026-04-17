"""Material dieléctrico (vidrio, agua)."""

import math
from typing import Optional, Tuple
from src.geometry import HitRecord
from src.materials.base import Material
from src.vector import Vec3
from src.ray import Ray
import random


class DielectricMaterial(Material):
    """
    Material dieléctrico (refracción).

    Simula materiales transparentes como vidrio o agua.
    """

    def __init__(self, ior: float):
        """
        Args:
            ior: Índice de refracción (1.0 = aire, 1.5 = vidrio, 1.33 = agua)
        """
        self.ior = ior

    def scatter(
        self, ray: Ray, hit_record: HitRecord
    ) -> Tuple[bool, Optional[Vec3], Optional[Ray]]:
        """Refracta o refleja el rayo según las leyes de Fresnel."""
        attenuation = Vec3(1.0, 1.0, 1.0)  # El vidrio no atenúa el color

        unit_direction = ray.direction  # ya normalizado por Ray.__init__

        # Determinar si entramos o salimos y orientar la normal
        if unit_direction.dot(hit_record.normal) > 0:
            # Saliendo (rayo y normal misma dirección): orientar normal hacia adentro
            normal = hit_record.normal * -1
            ri = self.ior
        else:
            # Entrando (rayo opuesto a normal): normal ya está correcta
            normal = hit_record.normal
            ri = 1.0 / self.ior

        # Calcular cos_theta (siempre positivo porque normal opone al rayo)
        cos_theta = min(-unit_direction.dot(normal), 1.0)
        sin_theta = math.sqrt(1.0 - cos_theta * cos_theta)

        # Verificar reflexión total interna
        cannot_refract = ri * sin_theta > 1.0

        if cannot_refract or self._reflectance(cos_theta, ri) > random.random():
            # Reflejar
            direction = unit_direction.reflect(normal)
        else:
            # Refractar
            direction = self._refract(unit_direction, normal, ri)

        scattered_ray = Ray(hit_record.point, direction)
        return True, attenuation, scattered_ray

    def emitted(self) -> Vec3:
        """No emite luz."""
        return Vec3(0, 0, 0)

    @staticmethod
    def _reflectance(cosine: float, ref_idx: float) -> float:
        """
        Aproximación de Schlick para reflectancia de Fresnel.

        Args:
            cosine: Coseno del ángulo de incidencia
            ref_idx: Índice de refracción

        Returns:
            Probabilidad de reflexión
        """
        r0 = (1 - ref_idx) / (1 + ref_idx)
        r0 = r0 * r0
        return r0 + (1 - r0) * pow((1 - cosine), 5)

    @staticmethod
    def _refract(uv: Vec3, n: Vec3, etai_over_etat: float) -> Vec3:
        """
        Calcula la dirección refractada usando ley de Snell vectorial.

        Args:
            uv: Dirección incidente (unitaria)
            n: Normal de superficie
            etai_over_etat: Ratio de índices de refracción

        Returns:
            Dirección refractada
        """
        cos_theta = min(-uv.dot(n), 1.0)
        r_out_perp = (uv + n * cos_theta) * etai_over_etat
        r_out_parallel = n * (-math.sqrt(abs(1.0 - r_out_perp.dot(r_out_perp))))
        return r_out_perp + r_out_parallel
