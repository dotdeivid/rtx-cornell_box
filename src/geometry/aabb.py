"""Axis-Aligned Bounding Box (AABB) para aceleración de ray tracing."""

from src.vector import Vec3
from src.ray import Ray


class AABB:
    """
    Axis-Aligned Bounding Box — caja envolvente alineada con los ejes XYZ.

    Usada para acelerar ray tracing y construir el BVH. Los tests de intersección
    son O(1) con el algoritmo de slabs (Kay-Kajiya 1986).

    Atributos:
        min (Vec3): Esquina con las coordenadas mínimas en cada eje
        max (Vec3): Esquina con las coordenadas máximas en cada eje
    """

    def __init__(self, min_pt: Vec3, max_pt: Vec3):
        """
        Inicializa una caja envolvente alineada con los ejes.

        Args:
            min_pt (Vec3): Esquina con las coordenadas mínimas en cada eje
            max_pt (Vec3): Esquina con las coordenadas máximas en cada eje

        Nota:
            Se asume que min_pt.x <= max_pt.x, min_pt.y <= max_pt.y, min_pt.z <= max_pt.z
        """
        self.min = min_pt
        self.max = max_pt

    def hit(self, ray: Ray, t_min, t_max):
        """
        Prueba si un rayo intersecta la caja usando el algoritmo de slabs (Kay-Kajiya 1986).

        Calcula los intervalos de intersección en cada eje y verifica que se solapan.

        Args:
            ray (Ray): El rayo a probar
            t_min (float): Distancia mínima válida
            t_max (float): Distancia máxima válida

        Returns:
            bool: True si hay intersección en [t_min, t_max]
        """
        origin_vals = (ray.origin.x, ray.origin.y, ray.origin.z)
        direction_vals = (ray.direction.x, ray.direction.y, ray.direction.z)
        min_vals = (self.min.x, self.min.y, self.min.z)
        max_vals = (self.max.x, self.max.y, self.max.z)

        for i in range(3):
            origin_i = origin_vals[i]
            direction_i = direction_vals[i]
            min_i = min_vals[i]
            max_i = max_vals[i]

            invD = 1.0 / direction_i
            t0 = (min_i - origin_i) * invD
            t1 = (max_i - origin_i) * invD

            if invD < 0:
                t0, t1 = t1, t0

            t_min = max(t0, t_min)
            t_max = min(t1, t_max)

            if t_max <= t_min:
                return False
        return True

    def union(self, other):
        """
        Retorna una nueva AABB que envuelve completamente esta caja y otra.

        Args:
            other (AABB): La otra caja envolvente

        Returns:
            AABB: Caja mínima que contiene ambas cajas
        """
        new_min = Vec3(
            min(self.min.x, other.min.x),
            min(self.min.y, other.min.y),
            min(self.min.z, other.min.z),
        )
        new_max = Vec3(
            max(self.max.x, other.max.x),
            max(self.max.y, other.max.y),
            max(self.max.z, other.max.z),
        )
        return AABB(new_min, new_max)
