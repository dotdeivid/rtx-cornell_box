"""Triángulo - primitiva para modelos 3D complejos."""

import math
from src.vector import Vec3
from src.ray import Ray
from src.geometry.hit_record import HitRecord
from src.geometry.aabb import AABB


class Triangle:
    """
    Triángulo en el espacio 3D. Primitiva básica para meshes 3D.

    La normal se precalcula como (v1-v0) × (v2-v0) normalizado.

    Atributos:
        v0, v1, v2 (Vec3): Vértices del triángulo
        material (Material): Material del triángulo
        normal (Vec3): Normal precalculada (regla mano derecha)
    """

    def __init__(
        self,
        v0: Vec3,
        v1: Vec3,
        v2: Vec3,
        material: "Material",
    ):
        """
        Args:
            v0, v1, v2: Vértices del triángulo (orden antihorario define la cara frontal)
            material: Material del triángulo (Material ABC)
        """
        self.v0 = v0
        self.v1 = v1
        self.v2 = v2
        self.material = material

        # Precalculamos la normal del triángulo
        edge1 = self.v1 - self.v0
        edge2 = self.v2 - self.v0
        self.normal = edge1.cross(edge2).normalize()

    def bounding_box(self):
        """
        Caja envolvente de los 3 vértices con margen de 0.001 para evitar espesor cero.

        Returns:
            AABB: Caja envolvente del triángulo
        """
        min_pt = Vec3(
            min(self.v0.x, self.v1.x, self.v2.x) - 0.001,
            min(self.v0.y, self.v1.y, self.v2.y) - 0.001,
            min(self.v0.z, self.v1.z, self.v2.z) - 0.001,
        )
        max_pt = Vec3(
            max(self.v0.x, self.v1.x, self.v2.x) + 0.001,
            max(self.v0.y, self.v1.y, self.v2.y) + 0.001,
            max(self.v0.z, self.v1.z, self.v2.z) + 0.001,
        )
        return AABB(min_pt, max_pt)

    def hit(self, ray: Ray, t_min=0.001, t_max=float("inf")):
        """
        Intersección rayo-triángulo usando el algoritmo Möller-Trumbore (1997).

        Resuelve O + t*D = v0 + u*edge1 + v*edge2 con la regla de Cramer.
        Válido si u ≥ 0, v ≥ 0, u+v ≤ 1 y t ∈ (t_min, t_max).

        Args:
            ray (Ray): El rayo a probar
            t_min (float): Distancia mínima válida
            t_max (float): Distancia máxima válida

        Returns:
            HitRecord o None: Información del impacto, o None si no hay intersección
        """
        # Algoritmo Möller-Trumbore
        edge1 = self.v1 - self.v0
        edge2 = self.v2 - self.v0
        h = ray.direction.cross(edge2)
        a = edge1.dot(h)

        # Si a es cercano a 0, el rayo es paralelo al triángulo
        if -1e-8 < a < 1e-8:
            return None

        f = 1.0 / a
        s = ray.origin - self.v0
        u = f * s.dot(h)

        if u < 0.0 or u > 1.0:
            return None

        q = s.cross(edge1)
        v = f * ray.direction.dot(q)

        if v < 0.0 or u + v > 1.0:
            return None

        # Calculamos t para ver dónde está la intersección en la línea del rayo
        t = f * edge2.dot(q)

        if t_min < t < t_max:
            intersection_point = ray.point_at(t)
            return HitRecord(
                t=t,
                point=intersection_point,
                normal=self.normal,
                material=self.material,
                obj_ref=self,
            )

        return None
