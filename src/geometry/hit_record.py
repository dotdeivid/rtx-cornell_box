"""Registro de intersección rayo-geometría."""

from dataclasses import dataclass
from typing import Optional, TYPE_CHECKING
from src.vector import Vec3

if TYPE_CHECKING:
    from src.materials.base import Material


@dataclass
class HitRecord:
    """
    Almacena información sobre la intersección de un rayo con un objeto.

    Attributes:
        t: Parámetro del rayo en el punto de intersección
        point: Punto 3D de la intersección
        normal: Vector normal a la superficie en el punto de intersección
        material: Material del objeto intersectado (Material ABC)
        obj_ref: Referencia al objeto intersectado
    """

    t: float
    point: Vec3
    normal: Vec3
    material: Optional["Material"] = None
    obj_ref: Optional[object] = None
