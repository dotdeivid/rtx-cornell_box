"""Configuración de parámetros de cámara."""

from dataclasses import dataclass
from typing import Optional
from src.vector import Vec3


@dataclass
class CameraConfig:
    """
    Configuración de la cámara virtual.

    Define la posición, orientación y propiedades ópticas de la cámara
    utilizada para renderizar la escena.

    Attributes:
        origin: Posición 3D de la cámara
        lookat: Punto 3D hacia donde mira la cámara
        vup: Vector "up" para orientación (típicamente (0,1,0))
        fov: Field of view vertical en grados
        aperture: Tamaño de apertura para depth of field (0 = todo enfocado)
        focus_distance: Distancia al plano focal (None = auto-calcular)

    Ejemplo:
        >>> # Cámara mirando hacia el centro de Cornell Box
        >>> config = CameraConfig(
        ...     origin=Vec3(278, 278, -800),
        ...     lookat=Vec3(278, 278, 0),
        ...     fov=40.0
        ... )
    """

    origin: Vec3 = Vec3(278, 278, -800)
    lookat: Vec3 = Vec3(278, 278, 278)
    vup: Vec3 = Vec3(0, 1, 0)
    fov: float = 40.0
    aperture: float = 20.0
    focus_distance: Optional[float] = None

    def __post_init__(self):
        """Valida parámetros después de inicialización."""
        if self.fov <= 0 or self.fov >= 180:
            raise ValueError("FOV debe estar entre 0 y 180 grados")
        if self.aperture < 0:
            raise ValueError("Aperture no puede ser negativo")
        if self.focus_distance is not None and self.focus_distance <= 0:
            raise ValueError("Focus distance debe ser positivo")
