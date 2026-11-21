"""Axis-Aligned Bounding Box (AABB) para aceleración de ray tracing."""

from __future__ import annotations

from src.vector import Vec3
from src.ray import Ray


class AABB:
    """
    Axis-Aligned Bounding Box — caja envolvente alineada con los ejes XYZ.

    Usada para acelerar ray tracing y construir el BVH. Los tests de intersección
    son O(1) con el algoritmo de slabs (Kay-Kajiya 1986).
    """

    __slots__ = ("min", "max")

    def __init__(self, min_pt: Vec3, max_pt: Vec3) -> None:
        self.min = min_pt
        self.max = max_pt

    def hit(self, ray: Ray, t_min: float, t_max: float) -> bool:
        """
        Prueba si un rayo intersecta la caja usando el algoritmo de slabs (Kay-Kajiya 1986).

        Maneja de forma segura direcciones paralelas a los ejes (``direction_i == 0``)
        sin producir ``ZeroDivisionError``, ``RuntimeWarning`` ni ``NaN``.
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

            if abs(direction_i) < 1e-12:
                # Rayo paralelo a este eje: solo hay intersección si el origen está dentro del slab
                if origin_i < min_i or origin_i > max_i:
                    return False
                continue

            inv_d = 1.0 / direction_i
            t0 = (min_i - origin_i) * inv_d
            t1 = (max_i - origin_i) * inv_d

            if inv_d < 0.0:
                t0, t1 = t1, t0

            if t0 > t_min:
                t_min = t0
            if t1 < t_max:
                t_max = t1

            if t_max <= t_min:
                return False
        return True

    def union(self, other: AABB) -> AABB:
        """Retorna una nueva AABB que envuelve completamente esta caja y ``other``."""
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

    def centroid(self) -> Vec3:
        """Retorna el centroide geométrico de la caja."""
        return Vec3(
            0.5 * (self.min.x + self.max.x),
            0.5 * (self.min.y + self.max.y),
            0.5 * (self.min.z + self.max.z),
        )

    def longest_axis(self) -> int:
        """Retorna el índice del eje de mayor extensión (0=X, 1=Y, 2=Z)."""
        dx = self.max.x - self.min.x
        dy = self.max.y - self.min.y
        dz = self.max.z - self.min.z
        if dx >= dy and dx >= dz:
            return 0
        if dy >= dz:
            return 1
        return 2
