"""BVH (Bounding Volume Hierarchy) para aceleración espacial."""

from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

from src.ray import Ray
from src.geometry.base import Geometry
from src.geometry.aabb import AABB
from src.geometry.hit_record import HitRecord


class BVHNode(Geometry):
    """
    Árbol binario de jerarquía de volúmenes envolventes (BVH).

    Reduce la complejidad de intersección de O(n) a O(log n) mediante poda espacial.
    Divide cada nodo por el eje de mayor extensión ordenando por centroide de AABB.
    """

    __slots__ = ("left", "right", "box")

    def __init__(self, left: Geometry, right: Geometry, box: AABB) -> None:
        self.left = left
        self.right = right
        self.box = box

    @classmethod
    def create(
        cls,
        objects: Sequence[Geometry],
        start: int = 0,
        end: Optional[int] = None,
    ) -> Geometry:
        """
        Construye el árbol BVH recursivamente sin mutar la colección original.

        Args:
            objects: Secuencia no vacía de objetos geométricos.
            start: Índice inicial del segmento.
            end: Índice final (exclusivo). Por defecto ``len(objects)``.

        Returns:
            Raíz del subárbol construido (``BVHNode`` o primitiva única).

        Raises:
            ValueError: Si el segmento de objetos está vacío.
        """
        if end is None:
            end = len(objects)
        if end - start <= 0:
            raise ValueError("No se puede construir un BVH con una lista vacía de objetos")

        # Copiamos el segmento y precalculamos (obj, bbox, centroid) para no mutar la entrada
        items: List[Tuple[Geometry, AABB, Tuple[float, float, float]]] = []
        for obj in objects[start:end]:
            bbox = obj.bounding_box()
            c = bbox.centroid()
            items.append((obj, bbox, (c.x, c.y, c.z)))

        return cls._build(items, 0, len(items))

    @classmethod
    def _build(
        cls,
        items: List[Tuple[Geometry, AABB, Tuple[float, float, float]]],
        start: int,
        end: int,
    ) -> Geometry:
        span = end - start

        if span == 1:
            return items[start][0]

        if span == 2:
            left_obj, left_box, _ = items[start]
            right_obj, right_box, _ = items[start + 1]
            return cls(left_obj, right_obj, left_box.union(right_box))

        # Calcular bounding box total del segmento para elegir el eje más largo
        full_box = items[start][1]
        for i in range(start + 1, end):
            full_box = full_box.union(items[i][1])

        axis = full_box.longest_axis()

        segment = items[start:end]
        segment.sort(key=lambda item: item[2][axis])
        items[start:end] = segment

        mid = start + span // 2
        left = cls._build(items, start, mid)
        right = cls._build(items, mid, end)

        return cls(left, right, left.bounding_box().union(right.bounding_box()))

    def hit(
        self, ray: Ray, t_min: float = 0.001, t_max: float = float("inf")
    ) -> Optional[HitRecord]:
        """Intersección jerárquica con poda por AABB y reducción de ``t_max``."""
        if not self.box.hit(ray, t_min, t_max):
            return None

        hit_left = self.left.hit(ray, t_min, t_max)
        limit = hit_left.t if hit_left is not None else t_max
        hit_right = self.right.hit(ray, t_min, limit)

        return hit_right if hit_right is not None else hit_left

    def bounding_box(self) -> AABB:
        """Retorna la AABB precalculada que envuelve ambos hijos."""
        return self.box
