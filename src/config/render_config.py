"""Configuración de parámetros de renderizado."""

from dataclasses import dataclass
from pathlib import Path


@dataclass
class RenderConfig:
    """
    Configuración centralizada para el proceso de renderizado.

    Esta clase encapsula todos los parámetros que controlan la calidad,
    rendimiento y salida del rendering.

    Attributes:
        width: Ancho de la imagen en píxeles
        height: Alto de la imagen en píxeles
        samples: Número de muestras por píxel (↑ = menos ruido, más tiempo)
        max_depth: Profundidad máxima de rebotes de rayos (↑ = más luz indirecta)
        use_parallel: Si True, usa rendering multi-core
        output_path: Ruta donde guardar la imagen renderizada
        gamma: Factor de corrección gamma para displays sRGB (típicamente 2.2)

    Ejemplo:
        >>> config = RenderConfig(width=800, height=600, samples=1000)
        >>> print(f"Resolución: {config.width}×{config.height}")
        Resolución: 800×600
    """

    width: int = 400
    height: int = 400
    samples: int = 400
    max_depth: int = 8
    use_parallel: bool = True
    output_path: Path = Path("output/result_render.png")
    gamma: float = 2.2

    @property
    def aspect_ratio(self) -> float:
        """Calcula aspect ratio de la imagen."""
        return self.width / self.height

    def __post_init__(self):
        """Valida parámetros después de inicialización."""
        if self.width <= 0 or self.height <= 0:
            raise ValueError("Width y height deben ser positivos")
        if self.samples <= 0:
            raise ValueError("Samples debe ser positivo")
        if self.max_depth < 0:
            raise ValueError("Max depth no puede ser negativo")
        if self.gamma <= 0:
            raise ValueError("Gamma debe ser positivo")

        # Convertir string a Path si es necesario
        if isinstance(self.output_path, str):
            self.output_path = Path(self.output_path)
