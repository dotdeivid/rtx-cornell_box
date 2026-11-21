"""Configuración de parámetros de cámara."""

from dataclasses import dataclass, field
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
        vup: Vector "up" para orientación (típicamente (0, 1, 0))
        fov: Field of view vertical en grados
        aperture: Tamaño de apertura para depth of field (0 = todo enfocado)
        focus_distance: Distancia al plano focal (None = auto-calcular)
    """

    origin: Vec3 = field(default_factory=lambda: Vec3(278, 278, -800))
    lookat: Vec3 = field(default_factory=lambda: Vec3(278, 278, 278))
    vup: Vec3 = field(default_factory=lambda: Vec3(0, 1, 0))
    fov: float = 40.0
    aperture: float = 0.0
    focus_distance: Optional[float] = None

    def __post_init__(self) -> None:
        """Valida parámetros después de inicialización."""
        if self.fov <= 0 or self.fov >= 180:
            raise ValueError("FOV debe estar entre 0 y 180 grados")
        if self.aperture < 0:
            raise ValueError("Aperture no puede ser negativo")
        if self.focus_distance is not None and self.focus_distance <= 0:
            raise ValueError("Focus distance debe ser positivo")
        if self.origin == self.lookat:
            raise ValueError("origin y lookat no pueden ser el mismo punto")
        if self.vup.length_squared() == 0.0:
            raise ValueError("vup no puede ser el vector cero")
