"""Cuadrilátero - superficie plana para paredes y luces de área."""

import math
import random
from typing import Optional, Tuple, TYPE_CHECKING

from src.vector import Vec3
from src.ray import Ray
from src.geometry.base import Geometry
from src.geometry.hit_record import HitRecord
from src.geometry.aabb import AABB

if TYPE_CHECKING:
    from src.materials.base import Material


class Quad(Geometry):
    """
    Cuadrilátero (paralelogramo) plano en el espacio 3D.

    Definido por un vértice origen ``Q`` y dos vectores de lado ``u``, ``v``.
    Cualquier punto en el quad es ``P = Q + s*u + t*v`` con ``s, t ∈ [0, 1]``.
    """

    def __init__(
        self,
        Q: Vec3,
        u: Vec3,
        v: Vec3,
        material: "Material",
    ) -> None:
        self.Q = Q
        self.u = u
        self.v = v
        self.material = material

        n = u.cross(v)
        n_len_sq = n.length_squared()
        if n_len_sq <= 1e-16:
            raise ValueError("Los vectores u y v de un Quad no pueden ser colineales ni nulos")

        self.area: float = math.sqrt(n_len_sq)
        self.normal: Vec3 = n / self.area
        self.D: float = self.normal.dot(self.Q)
        self.w: Vec3 = n / n_len_sq

        # Precalcular AABB con margen mínimo para evitar espesor cero
        p1 = self.Q
        p2 = self.Q + self.u
        p3 = self.Q + self.v
        p4 = self.Q + self.u + self.v
        pad = 0.0001
        min_pt = Vec3(
            min(p1.x, p2.x, p3.x, p4.x) - pad,
            min(p1.y, p2.y, p3.y, p4.y) - pad,
            min(p1.z, p2.z, p3.z, p4.z) - pad,
        )
        max_pt = Vec3(
            max(p1.x, p2.x, p3.x, p4.x) + pad,
            max(p1.y, p2.y, p3.y, p4.y) + pad,
            max(p1.z, p2.z, p3.z, p4.z) + pad,
        )
        self._bbox = AABB(min_pt, max_pt)

    @property
    def center(self) -> Vec3:
        """Centro geométrico del quad: ``Q + 0.5*u + 0.5*v``."""
        return self.Q + (self.u * 0.5) + (self.v * 0.5)

    def bounding_box(self) -> AABB:
        """Retorna la AABB precalculada del quad."""
        return self._bbox

    def hit(
        self, ray: Ray, t_min: float = 0.001, t_max: float = float("inf")
    ) -> Optional[HitRecord]:
        """Intersección rayo-quad con orientación de normal hacia el rayo incidente."""
        denom = self.normal.dot(ray.direction)
        if abs(denom) < 1e-8:
            return None

        t = (self.D - self.normal.dot(ray.origin)) / denom
        if t < t_min or t > t_max:
            return None

        intersection = ray.point_at(t)
        planar_hit_pt_vector = intersection - self.Q

        alpha = self.w.dot(planar_hit_pt_vector.cross(self.v))
        beta = self.w.dot(self.u.cross(planar_hit_pt_vector))

        if not (0.0 <= alpha <= 1.0 and 0.0 <= beta <= 1.0):
            return None

        rec = HitRecord(
            t=t,
            point=intersection,
            normal=self.normal,
            material=self.material,
            obj_ref=self,
        )
        rec.set_face_normal(ray, self.normal)
        return rec

    def sample_solid_angle(self, hit_point: Vec3) -> Tuple[Vec3, float]:
        """
        Muestreo de ángulo sólido para importance sampling de luces de área.

        Emite en la dirección de ``self.normal`` (luz de una sola cara), evitando
        iluminar por detrás del emisor y protegiendo contra ``distance == 0``.
        """
        random_point = self.Q + (self.u * random.random()) + (self.v * random.random())
        direction_to_light = random_point - hit_point
        distance_sq = direction_to_light.length_squared()

        if distance_sq <= 1e-12:
            return self.normal, 0.0

        distance = math.sqrt(distance_sq)
        direction = direction_to_light / distance

        # Coseno respecto a la cara emisora (hacia afuera: -direction apunta desde la luz al hit)
        cos_light = max(0.0, self.normal.dot(-direction))
        if cos_light <= 0.0:
            return direction, 0.0

        clamped_dist_sq = max(distance_sq, 1.0)
        solid_angle = (self.area * cos_light) / clamped_dist_sq
        return direction, solid_angle
