"""Utilidades de muestreo Monte Carlo y carga de modelos OBJ."""

from pathlib import Path
import random
from typing import List, Optional, TYPE_CHECKING, Union

from src.vector import Vec3
from src.geometry.triangle import Triangle

if TYPE_CHECKING:
    from src.materials.base import Material


def random_in_unit_sphere() -> Vec3:
    """
    Genera un vector aleatorio dentro de la esfera unitaria usando método de rechazo.

    Returns:
        Vec3: Vector con magnitud < 1, distribuido uniformemente en el volumen
    """
    while True:
        p = Vec3(
            random.uniform(-1.0, 1.0),
            random.uniform(-1.0, 1.0),
            random.uniform(-1.0, 1.0),
        )
        if p.length_squared() < 1.0:
            return p


def random_in_unit_disk() -> Vec3:
    """
    Genera un punto aleatorio dentro de un disco de radio 1 en el plano XY (método de rechazo).

    Usado para simular depth of field en el modelo de cámara thin-lens.

    Returns:
        Vec3: Punto con x, y ∈ [-1, 1], z = 0 y distancia al centro < 1
    """
    while True:
        p = Vec3(random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0), 0.0)
        if p.length_squared() < 1.0:
            return p


def load_obj(
    filename: Union[str, Path],
    scale: float = 1.0,
    offset: Optional[Vec3] = None,
    material: Optional["Material"] = None,
    ground_y: Optional[float] = None,
) -> List[Triangle]:
    """
    Carga un archivo ``.obj`` y lo convierte en lista de triángulos.

    Soporta vértices (``v``) y caras (``f``) con índices opcionales de textura/normal.
    Triangula polígonos de más de 3 lados usando fan desde el primer vértice.

    Args:
        filename: Ruta al archivo ``.obj``
        scale: Factor de escala uniforme aplicado a los vértices
        offset: Traslación aplicada después del escalado. Por defecto ``(0, 0, 0)``
        material: Material para todos los triángulos. Por defecto: ``DiffuseMaterial`` gris
        ground_y: Si se indica, traslada el modelo en Y para que su vértice más bajo
            quede exactamente en ``ground_y`` (útil para apoyarlo sobre el piso)

    Returns:
        Lista de triángulos del modelo

    Raises:
        OSError: Si el archivo no existe o no se puede leer
        ValueError: Si el archivo tiene un formato inválido o no contiene caras
    """
    vertices: List[Vec3] = []
    faces: List[List[int]] = []

    if offset is None:
        offset = Vec3(0.0, 0.0, 0.0)
    if material is None:
        from src.materials.diffuse import DiffuseMaterial

        material = DiffuseMaterial(Vec3(0.8, 0.8, 0.8))

    with open(filename, "r", encoding="utf-8") as f:
        for line_no, line in enumerate(f, start=1):
            try:
                if line.startswith("v "):
                    parts = line.split()
                    if len(parts) < 4:
                        continue
                    v = Vec3(float(parts[1]), float(parts[2]), float(parts[3]))
                    vertices.append(v * scale + offset)

                elif line.startswith("f "):
                    indices: List[int] = []
                    for p in line.split()[1:]:
                        idx = int(p.split("/")[0])
                        if idx == 0:
                            raise ValueError("índice OBJ 0 inválido (los índices comienzan en 1)")
                        resolved = idx - 1 if idx > 0 else len(vertices) + idx
                        if resolved < 0 or resolved >= len(vertices):
                            raise ValueError(f"índice de vértice fuera de rango: {idx}")
                        indices.append(resolved)
                    if len(indices) < 3:
                        raise ValueError("una cara OBJ debe tener al menos 3 vértices")
                    faces.append(indices)
            except ValueError as e:
                raise ValueError(f"{filename}:{line_no}: línea OBJ inválida: {e}") from e

    if not vertices or not faces:
        raise ValueError(f"{filename}: el archivo OBJ no contiene vértices o caras")

    if ground_y is not None:
        shift = Vec3(0.0, ground_y - min(v.y for v in vertices), 0.0)
        vertices = [v + shift for v in vertices]

    triangles: List[Triangle] = []
    for indices in faces:
        for i in range(1, len(indices) - 1):
            triangles.append(
                Triangle(
                    vertices[indices[0]],
                    vertices[indices[i]],
                    vertices[indices[i + 1]],
                    material,
                )
            )

    print(f"Modelo cargado: {len(triangles)} triángulos.")
    return triangles
