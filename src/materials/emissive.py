"""Material emisivo para luces de área."""

from typing import Optional, Tuple
from src.geometry.hit_record import HitRecord
from src.materials.base import Material
from src.vector import Vec3
from src.ray import Ray


class EmissiveMaterial(Material):
    """
    Material que emite luz (para luces de área).

    No dispersa rayos, solo emite radiancia.
    """

    def __init__(self, emission: Vec3) -> None:
        self.emission_color = emission

    def scatter(
        self, ray: Ray, hit_record: HitRecord
    ) -> Tuple[bool, Optional[Vec3], Optional[Ray]]:
        """Las luces no dispersan rayos, solo emiten."""
        return False, None, None

    def emitted(self) -> Vec3:
        """Retorna el color emitido."""
        return self.emission_color
