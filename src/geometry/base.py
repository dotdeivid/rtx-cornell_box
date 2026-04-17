"""Base class abstracta para geometría."""

from abc import ABC, abstractmethod
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from src.ray import Ray
    from src.geometry.hit_record import HitRecord
    from src.geometry.aabb import AABB


class Geometry(ABC):
    """
    Clase base abstracta para todas las primitivas geométricas.

    Define la interfaz que deben implementar todos los objetos
    que pueden ser intersectados por rayos (esferas, quads, triángulos, etc).
    """

    @abstractmethod
    def hit(self, ray: "Ray", t_min: float, t_max: float) -> Optional["HitRecord"]:
        """
        Calcula la intersección del rayo con este objeto.

        Args:
            ray: Rayo a intersectar
            t_min: Distancia mínima válida
            t_max: Distancia máxima válida

        Returns:
            HitRecord con información de intersección, o None si no hay hit
        """
        pass

    @abstractmethod
    def bounding_box(self) -> "AABB":
        """
        Retorna la caja envolvente alineada con ejes (AABB) del objeto.

        Returns:
            AABB que contiene completamente este objeto
        """
        pass
