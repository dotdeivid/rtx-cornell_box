"""Renderer - orquestador de rendering multi-core con stratified sampling."""

import math
import multiprocessing
import random
from typing import Optional, Sequence

import numpy as np
from PIL import Image

from src.vector import Vec3
from src.camera.camera import Camera
from src.geometry.base import Geometry
from src.config.render_config import RenderConfig
from src.renderer.path_tracer import AreaLight, color_ray


# Estado global por proceso worker (inicializado una única vez por proceso en Pool)
_WORKER_WORLD: Optional[Geometry] = None
_WORKER_LIGHTS: Sequence[AreaLight] = ()
_WORKER_CAMERA: Optional[Camera] = None
_WORKER_CONFIG: Optional[RenderConfig] = None


def _init_worker(
    world: Geometry,
    lights: Sequence[AreaLight],
    camera: Camera,
    config: RenderConfig,
) -> None:
    """Inicializa el estado inmutable de la escena una sola vez por proceso worker."""
    global _WORKER_WORLD, _WORKER_LIGHTS, _WORKER_CAMERA, _WORKER_CONFIG
    _WORKER_WORLD = world
    _WORKER_LIGHTS = lights
    _WORKER_CAMERA = camera
    _WORKER_CONFIG = config
    if config.seed is None:
        random.seed()


def _render_row_worker(y: int) -> np.ndarray:
    """Punto de entrada picklable para cada tarea de fila en ``multiprocessing.Pool``."""
    assert _WORKER_WORLD is not None
    assert _WORKER_CAMERA is not None
    assert _WORKER_CONFIG is not None
    return _render_row_impl(
        y, _WORKER_WORLD, _WORKER_LIGHTS, _WORKER_CAMERA, _WORKER_CONFIG
    )


def _render_row_impl(
    y: int,
    world: Geometry,
    lights: Sequence[AreaLight],
    camera: Camera,
    config: RenderConfig,
) -> np.ndarray:
    """
    Renderiza una fila completa de píxeles con verdadero stratified sampling.

    Si ``config.seed`` está definida, fija la semilla determinista por fila para
    que los modos secuencial y paralelo produzcan exactamente la misma imagen.
    """
    if config.seed is not None:
        random.seed(f"{config.seed}:{y}")

    width = config.width
    height = config.height
    samples = config.samples
    max_depth = config.max_depth
    max_sample = config.max_sample
    inv_gamma = 1.0 / config.gamma
    inv_samples = 1.0 / samples

    # Cuadrícula estratificada √samples × √samples + resto uniforme si no es cuadrado perfecto
    grid_n = math.isqrt(samples)
    stratified_count = grid_n * grid_n
    remainder = samples - stratified_count
    inv_grid = 1.0 / grid_n

    row = np.empty((width, 3), dtype=np.uint8)

    for x in range(width):
        r_acc = 0.0
        g_acc = 0.0
        b_acc = 0.0

        # Muestras estratificadas en sub-celdas (sx, sy)
        for sy in range(grid_n):
            for sx in range(grid_n):
                u = (x + (sx + random.random()) * inv_grid) / width
                v = (y + (sy + random.random()) * inv_grid) / height
                ray = camera.get_ray(u, v)
                sample = color_ray(ray, world, lights, max_depth, max_depth)

                if max_sample is not None:
                    r_acc += min(sample.x, max_sample)
                    g_acc += min(sample.y, max_sample)
                    b_acc += min(sample.z, max_sample)
                else:
                    r_acc += sample.x
                    g_acc += sample.y
                    b_acc += sample.z

        # Muestras sobrantes si samples no es un cuadrado perfecto
        for _ in range(remainder):
            u = (x + random.random()) / width
            v = (y + random.random()) / height
            ray = camera.get_ray(u, v)
            sample = color_ray(ray, world, lights, max_depth, max_depth)

            if max_sample is not None:
                r_acc += min(sample.x, max_sample)
                g_acc += min(sample.y, max_sample)
                b_acc += min(sample.z, max_sample)
            else:
                r_acc += sample.x
                g_acc += sample.y
                b_acc += sample.z

        # Promedio + corrección gamma segura (clamp >= 0)
        r_lin = max(0.0, r_acc * inv_samples)
        g_lin = max(0.0, g_acc * inv_samples)
        b_lin = max(0.0, b_acc * inv_samples)

        r_val = min(1.0, r_lin**inv_gamma)
        g_val = min(1.0, g_lin**inv_gamma)
        b_val = min(1.0, b_lin**inv_gamma)

        row[x, 0] = int(r_val * 255.99)
        row[x, 1] = int(g_val * 255.99)
        row[x, 2] = int(b_val * 255.99)

    return row


class Renderer:
    """
    Orquestador principal del proceso de rendering.

    Coordina rendering multi-core/secuencial y guarda la imagen resultante.
    """

    def __init__(self, config: RenderConfig) -> None:
        self.config = config

    def render(
        self,
        world: Geometry,
        lights: Sequence[AreaLight],
        camera: Camera,
    ) -> np.ndarray:
        """
        Renderiza la escena completa y guarda la imagen PNG en disco.

        Returns:
            Array RGB uint8 de forma ``(height, width, 3)`` (orientación final guardada).
        """
        print(f"Iniciando rendering {self.config.width}×{self.config.height}...")
        print(f"Samples: {self.config.samples}, Max depth: {self.config.max_depth}")
        print(f"Parallel: {self.config.use_parallel}")

        if self.config.use_parallel:
            image_data = self._render_parallel(world, lights, camera)
        else:
            image_data = self._render_sequential(world, lights, camera)

        flipped = self._save_image(image_data)
        print(f"✓ Imagen guardada en: {self.config.output_path}")
        return flipped

    def _render_parallel(
        self,
        world: Geometry,
        lights: Sequence[AreaLight],
        camera: Camera,
    ) -> np.ndarray:
        """Rendering multi-core pasando el BVH una sola vez por proceso worker."""
        with multiprocessing.Pool(
            initializer=_init_worker,
            initargs=(world, lights, camera, self.config),
        ) as pool:
            rows = pool.map(_render_row_worker, range(self.config.height))

        return np.array(rows, dtype=np.uint8)

    def _render_sequential(
        self,
        world: Geometry,
        lights: Sequence[AreaLight],
        camera: Camera,
    ) -> np.ndarray:
        """Rendering single-core."""
        rows = []
        for y in range(self.config.height):
            row = self._render_row(y, world, lights, camera)
            rows.append(row)
            if y % 50 == 0:
                print(f"  Progreso: {y}/{self.config.height}")

        return np.array(rows, dtype=np.uint8)

    def _render_row(
        self,
        y: int,
        world: Geometry,
        lights: Sequence[AreaLight],
        camera: Camera,
    ) -> np.ndarray:
        """Renderiza una fila de píxeles."""
        return _render_row_impl(y, world, lights, camera, self.config)

    def _save_image(self, image_data: np.ndarray) -> np.ndarray:
        """Voltea verticalmente (convención OpenGL -> imagen) y guarda a disco."""
        flipped = np.flipud(image_data)
        img = Image.fromarray(flipped, "RGB")
        self.config.output_path.parent.mkdir(parents=True, exist_ok=True)
        img.save(self.config.output_path)
        return flipped
