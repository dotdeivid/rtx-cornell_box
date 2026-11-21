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
    mira y parámetros ópticos.
    """

    def __init__(self, config: CameraConfig, aspect_ratio: float) -> None:
        """
        Inicializa la cámara con la configuración dada.

        Args:
            config: Configuración de parámetros de cámara
            aspect_ratio: Relación ancho/alto (width/height)
        """
        if aspect_ratio <= 0:
            raise ValueError("aspect_ratio debe ser positivo")
        self.config = config
        self.aspect_ratio = aspect_ratio
        self._setup_camera()

    def _setup_camera(self) -> None:
        """Calcula base ortonormal y viewport de la cámara."""
        look_vec = self.config.origin - self.config.lookat
        auto_dist = look_vec.length()
        self.focus_dist: float = (
            self.config.focus_distance
            if self.config.focus_distance is not None
            else auto_dist
        )

        # Construir base ortonormal {u, v, w}
        # w apunta hacia atrás (opuesto a viewing direction)
        self.w = look_vec.normalize()

        # Si vup es casi paralelo a w (ej. vista cenital), usar un fallback ortogonal
        vup = self.config.vup
        cross_u = vup.cross(self.w)
        if cross_u.length_squared() < 1e-12:
            fallback_up = Vec3(0, 0, -1) if abs(self.w.z) < 0.9 else Vec3(1, 0, 0)
            cross_u = fallback_up.cross(self.w)

        self.u = cross_u.normalize()
        self.v = self.w.cross(self.u)

        # Calcular dimensiones del viewport
        theta = math.radians(self.config.fov)
        h = math.tan(theta / 2.0)
        viewport_height = 2.0 * h
        viewport_width = self.aspect_ratio * viewport_height

        # Vectores del viewport escalados por distancia focal
        self.horizontal = self.u * (viewport_width * self.focus_dist)
        self.vertical = self.v * (viewport_height * self.focus_dist)

        # Esquina inferior izquierda del viewport
        self.lower_left = (
            self.config.origin
            - self.horizontal / 2.0
            - self.vertical / 2.0
            - self.w * self.focus_dist
        )

        # Radio del lente para DOF
        self.lens_radius = self.config.aperture / 2.0

    def get_ray(self, s: float, t: float) -> Ray:
        """
        Genera un rayo desde la cámara hacia el píxel (s, t).

        Args:
            s: Coordenada horizontal normalizada [0, 1]
            t: Coordenada vertical normalizada [0, 1]

        Returns:
            Ray desde la cámara hacia el punto (s, t) del viewport
        """
        if self.lens_radius > 0.0:
            rd = random_in_unit_disk() * self.lens_radius
            offset = self.u * rd.x + self.v * rd.y
            origin = self.config.origin + offset
        else:
            offset = Vec3(0.0, 0.0, 0.0)
            origin = self.config.origin

        direction = (
            self.lower_left
            + self.horizontal * s
            + self.vertical * t
            - self.config.origin
            - offset
        )

        return Ray(origin, direction)
