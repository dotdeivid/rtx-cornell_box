"""Sistema de cámara con profundidad de campo y proyección perspectiva."""

import math
from src.vector import Vec3
from src.ray import Ray
from src.config.camera_config import CameraConfig
from src.utils import random_in_unit_disk


class Camera:
    """
    Cámara virtual con proyección perspectiva y depth of field (DOF).

    Implementa un modelo de cámara thin-lens que simula depth of field
    realista con bokeh. La cámara se define por su posición, hacia dónde
    mira, y parámetros ópticos.

    Attributes:
        config: Configuración de la cámara
        aspect_ratio: Relación ancho/alto de la imagen
        origin: Posición de la cámara en el espacio
        lower_left: Esquina inferior izquierda del viewport
        horizontal: Vector horizontal del viewport
        vertical: Vector vertical del viewport
        u, v, w: Base ortonormal de la cámara
        lens_radius: Radio de la apertura para DOF
    """

    def __init__(self, config: CameraConfig, aspect_ratio: float):
        """
        Inicializa la cámara con la configuración dada.

        Args:
            config: Configuración de parámetros de cámara
            aspect_ratio: Relación ancho/alto (width/height)
        """
        self.config = config
        self.aspect_ratio = aspect_ratio
        self._setup_camera()

    def _setup_camera(self):
        """Calcula base ortonormal y viewport de la cámara."""
        # Calcular distancia focal
        self.focus_dist = self.config.focus_distance
        if self.focus_dist is None:
            # Auto-calcular basado en lookat
            self.focus_dist = (self.config.origin - self.config.lookat).length()

        # Construir base ortonormal {u, v, w}
        # w apunta hacia atrás (opposite de viewing direction)
        self.w = (self.config.origin - self.config.lookat).normalize()
        # u apunta a la derecha
        self.u = self.config.vup.cross(self.w).normalize()
        # v apunta hacia arriba
        self.v = self.w.cross(self.u)

        # Calcular dimensiones del viewport
        theta = math.radians(self.config.fov)
        h = math.tan(theta / 2)
        viewport_height = 2.0 * h
        viewport_width = self.aspect_ratio * viewport_height

        # Vectores del viewport escalados por distancia focal
        self.horizontal = self.u * viewport_width * self.focus_dist
        self.vertical = self.v * viewport_height * self.focus_dist

        # Esquina inferior izquierda del viewport
        self.lower_left = (
            self.config.origin
            - self.horizontal / 2
            - self.vertical / 2
            - self.w * self.focus_dist
        )

        # Radio del lente para DOF
        self.lens_radius = self.config.aperture / 2

    def get_ray(self, s: float, t: float) -> Ray:
        """
        Genera un rayo desde la cámara hacia el píxel (s, t).

        Implementa depth of field usando thin-lens camera model:
        - Rays se originan en puntos aleatorios dentro del disco de apertura
        - Todos convergen en el plano focal

        Args:
            s: Coordenada horizontal normalizada [0, 1]
            t: Coordenada vertical normalizada [0, 1]

        Returns:
            Ray desde la cámara hacia el punto (s,t) del viewport
        """
        # Offset aleatorio en el disco de apertura para DOF
        rd = random_in_unit_disk() * self.lens_radius
        offset = self.u * rd.x + self.v * rd.y

        # Punto de origen en el lente
        origin = self.config.origin + offset

        # Dirección hacia el punto en el viewport
        direction = (
            self.lower_left
            + self.horizontal * s
            + self.vertical * t
            - self.config.origin
            - offset
        )

        return Ray(origin, direction)
