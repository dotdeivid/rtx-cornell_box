"""Triángulo - primitiva para modelos 3D complejos."""

from typing import Optional, TYPE_CHECKING

from src.vector import Vec3
from src.ray import Ray
from src.geometry.base import Geometry
from src.geometry.hit_record import HitRecord
from src.geometry.aabb import AABB

if TYPE_CHECKING:
    from src.materials.base import Material


class Triangle(Geometry):
    """
    Triángulo en el espacio 3D. Primitiva básica para meshes 3D.

    Precalcula aristas ``edge1``, ``edge2``, normal y AABB para acelerar
    el algoritmo de Möller-Trumbore en el hot path.
    """

    def __init__(
        self,
        v0: Vec3,
        v1: Vec3,
        v2: Vec3,
        material: "Material",
    ) -> None:
        self.v0 = v0
        self.v1 = v1
        self.v2 = v2
        self.material = material

        self.edge1: Vec3 = self.v1 - self.v0
        self.edge2: Vec3 = self.v2 - self.v0
        self.normal: Vec3 = self.edge1.cross(self.edge2).normalize()

        pad = 0.001
        min_pt = Vec3(
            min(self.v0.x, self.v1.x, self.v2.x) - pad,
            min(self.v0.y, self.v1.y, self.v2.y) - pad,
            min(self.v0.z, self.v1.z, self.v2.z) - pad,
        )
        max_pt = Vec3(
            max(self.v0.x, self.v1.x, self.v2.x) + pad,
            max(self.v0.y, self.v1.y, self.v2.y) + pad,
            max(self.v0.z, self.v1.z, self.v2.z) + pad,
        )
        self._bbox = AABB(min_pt, max_pt)

    def bounding_box(self) -> AABB:
        """Retorna la AABB precalculada del triángulo."""
        return self._bbox

    def hit(
        self, ray: Ray, t_min: float = 0.001, t_max: float = float("inf")
    ) -> Optional[HitRecord]:
        """Intersección rayo-triángulo usando Möller-Trumbore (1997)."""
        h = ray.direction.cross(self.edge2)
        a = self.edge1.dot(h)

        if -1e-8 < a < 1e-8:
            return None

        f = 1.0 / a
        s = ray.origin - self.v0
        u = f * s.dot(h)

        if u < 0.0 or u > 1.0:
            return None

        q = s.cross(self.edge1)
        v = f * ray.direction.dot(q)

        if v < 0.0 or u + v > 1.0:
            return None

        t = f * self.edge2.dot(q)

        if t_min < t < t_max:
            intersection_point = ray.point_at(t)
            rec = HitRecord(
                t=t,
                point=intersection_point,
                normal=self.normal,
                material=self.material,
                obj_ref=self,
            )
            rec.set_face_normal(ray, self.normal)
            return rec

        return None
