"""Renderer - orquestador de rendering multi-core."""

import numpy as np
from PIL import Image
from typing import List
import multiprocessing
from functools import partial
import random

from src.vector import Vec3
from src.camera.camera import Camera
from src.geometry import BVHNode
from src.config.render_config import RenderConfig
from src.renderer.path_tracer import color_ray


class Renderer:
    """
    Orquestador principal del proceso de rendering.

    Coordinates multi-core rendering y guarda resultados.
    """

    def __init__(self, config: RenderConfig):
        """
        Args:
            config: Configuración de rendering
        """
        self.config = config

    def render(self, world: BVHNode, lights: List, camera: Camera):
        """
        Renderiza la escena completa.

        Args:
            world: Geometría de la escena (BVH)
            lights: Lista de fuentes de luz
            camera: Cámara virtual
        """
        print(f"Iniciando rendering {self.config.width}×{self.config.height}...")
        print(f"Samples: {self.config.samples}, Max depth: {self.config.max_depth}")
        print(f"Parallel: {self.config.use_parallel}")

        if self.config.use_parallel:
            image_data = self._render_parallel(world, lights, camera)
        else:
            image_data = self._render_sequential(world, lights, camera)

        # Guardar imagen
        self._save_image(image_data)
        print(f"✓ Imagen guardada en: {self.config.output_path}")

    def _render_parallel(self, world, lights, camera):
        """Rendering multi-core."""
        with multiprocessing.Pool() as pool:
            render_func = partial(
                self._render_row, world=world, lights=lights, camera=camera
            )
            rows = pool.map(render_func, range(self.config.height))

        return np.array(rows, dtype=np.uint8)

    def _render_sequential(self, world, lights, camera):
        """Rendering single-core."""
        rows = []
        for y in range(self.config.height):
            row = self._render_row(y, world, lights, camera)
            rows.append(row)
            if y % 50 == 0:
                print(f"  Progreso: {y}/{self.config.height}")

        return np.array(rows, dtype=np.uint8)

    def _render_row(self, y: int, world, lights, camera) -> np.ndarray:
        """
        Renderiza una fila de píxeles.

        Args:
            y: Índice de fila
            world, lights, camera: Parámetros de escena

        Returns:
            Array de píxeles RGB
        """
        row = []

        for x in range(self.config.width):
            pixel_color = Vec3(0, 0, 0)

            # Stratified sampling
            for _ in range(self.config.samples):
                u = (x + random.random()) / self.config.width
                v = (y + random.random()) / self.config.height

                ray = camera.get_ray(u, v)
                pixel_color += color_ray(
                    ray, world, lights, self.config.max_depth, self.config.max_depth
                )

            # Promedio y gamma correction
            pixel_color = pixel_color / self.config.samples
            pixel_color = Vec3(
                pixel_color.x ** (1.0 / self.config.gamma),
                pixel_color.y ** (1.0 / self.config.gamma),
                pixel_color.z ** (1.0 / self.config.gamma),
            )

            # Clamp y convertir a [0, 255]
            r = int(max(0, min(1, pixel_color.x)) * 255.99)
            g = int(max(0, min(1, pixel_color.y)) * 255.99)
            b = int(max(0, min(1, pixel_color.z)) * 255.99)

            row.append([r, g, b])

        return np.array(row, dtype=np.uint8)

    def _save_image(self, image_data: np.ndarray):
        """Guarda imagen a archivo."""
        # Flip vertical (OpenGL convention)
        image_data = np.flipud(image_data)

        img = Image.fromarray(image_data, "RGB")
        self.config.output_path.parent.mkdir(parents=True, exist_ok=True)
        img.save(self.config.output_path)
