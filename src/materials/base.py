"""Sistema de materiales - Base abstracta."""

from abc import ABC, abstractmethod
from typing import Optional, Tuple, TYPE_CHECKING
from src.vector import Vec3
from src.ray import Ray

if TYPE_CHECKING:
    from src.geometry import HitRecord


class Material(ABC):
    """
    Clase base abstracta para materiales.

    Define la interfaz que todos los materiales deben implementar.
    """

    @abstractmethod
    def scatter(
        self, ray: Ray, hit_record: "HitRecord"
    ) -> Tuple[bool, Optional[Vec3], Optional[Ray]]:
        """
        Calcula cómo el rayo interactúa con el material.

        Args:
            ray: Rayo incidente
            hit_record: Información de la intersección

        Returns:
            Tuple de (scattered, attenuation, scattered_ray):
            - scattered: Si el rayo se dispersa
            - attenuation: Color de atenuación (albedo)
            - scattered_ray: Rayo dispersado (si hay)
        """
        pass

    @abstractmethod
    def emitted(self) -> Vec3:
        """
        Color emitido por el material (para luces).

        Returns:
            Vec3: Color emitido (negro para materiales no emisivos)
        """
        pass
