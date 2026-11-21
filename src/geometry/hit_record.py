"""Registro de intersección rayo-geometría."""

from typing import Optional, TYPE_CHECKING
from src.vector import Vec3

if TYPE_CHECKING:
    from src.materials.base import Material
    from src.ray import Ray


class HitRecord:
    """
    Almacena información sobre la intersección de un rayo con un objeto.

    Attributes:
        t: Parámetro del rayo en el punto de intersección
        point: Punto 3D de la intersección
        normal: Vector normal orientado siempre contra el rayo incidente
        material: Material del objeto intersectado (Material ABC)
        obj_ref: Referencia al objeto intersectado
        front_face: True si el rayo impactó la cara exterior de la superficie
    """

    __slots__ = ("t", "point", "normal", "material", "obj_ref", "front_face")

    def __init__(
        self,
        t: float,
        point: Vec3,
        normal: Vec3,
        material: Optional["Material"] = None,
        obj_ref: Optional[object] = None,
        front_face: bool = True,
    ) -> None:
        self.t = t
        self.point = point
        self.normal = normal
        self.material = material
        self.obj_ref = obj_ref
        self.front_face = front_face

    def set_face_normal(self, ray: "Ray", outward_normal: Vec3) -> None:
        """Orienta ``self.normal`` contra el rayo incidente y registra ``front_face``."""
        self.front_face = ray.direction.dot(outward_normal) < 0.0
        self.normal = outward_normal if self.front_face else -outward_normal

    def __repr__(self) -> str:
        return (
            f"HitRecord(t={self.t}, point={self.point}, normal={self.normal}, "
            f"front_face={self.front_face})"
        )
