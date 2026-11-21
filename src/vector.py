"""Álgebra vectorial 3D para posiciones, direcciones y colores."""

from __future__ import annotations

import math
from typing import Union


class Vec3:
    """
    Vector tridimensional inmutable en la práctica, implementado con floats nativos
    y ``__slots__`` para mínimo overhead en el hot path del path tracer.
    """

    __slots__ = ("x", "y", "z")

    def __init__(self, x: float, y: float, z: float) -> None:
        self.x: float = float(x)
        self.y: float = float(y)
        self.z: float = float(z)

    # --- Operaciones Aritméticas Básicas ---

    def __add__(self, other: Vec3) -> Vec3:
        """Suma componente a componente."""
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: Vec3) -> Vec3:
        """Resta componente a componente."""
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __neg__(self) -> Vec3:
        """Negación componente a componente."""
        return Vec3(-self.x, -self.y, -self.z)

    def __mul__(self, other: Union[Vec3, float, int]) -> Vec3:
        """Multiplicación por escalar o por otro Vec3 (Hadamard)."""
        if isinstance(other, Vec3):
            return Vec3(self.x * other.x, self.y * other.y, self.z * other.z)
        return Vec3(self.x * other, self.y * other, self.z * other)

    def __rmul__(self, scalar: Union[float, int]) -> Vec3:
        """Multiplicación escalar por la izquierda: ``scalar * vec``."""
        return Vec3(self.x * scalar, self.y * scalar, self.z * scalar)

    def __truediv__(self, scalar: Union[float, int]) -> Vec3:
        """División por escalar componente a componente."""
        inv = 1.0 / scalar
        return Vec3(self.x * inv, self.y * inv, self.z * inv)

    def __eq__(self, other: object) -> bool:
        """Igualdad exacta por componentes."""
        if not isinstance(other, Vec3):
            return NotImplemented
        return self.x == other.x and self.y == other.y and self.z == other.z

    # --- Operaciones de Álgebra Lineal Clave ---

    def dot(self, other: Vec3) -> float:
        """Producto punto: ``v1·v2``."""
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other: Vec3) -> Vec3:
        """Producto cruz perpendicular a ``self`` y ``other``."""
        return Vec3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )

    def length_squared(self) -> float:
        """Magnitud euclidiana al cuadrado: ``x² + y² + z²``."""
        return self.x * self.x + self.y * self.y + self.z * self.z

    def length(self) -> float:
        """Magnitud euclidiana: ``sqrt(x² + y² + z²)``."""
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def normalize(self) -> Vec3:
        """Vector unitario con la misma dirección, o ``Vec3(0, 0, 0)`` si magnitud es 0."""
        mag = self.length()
        if mag == 0.0:
            return Vec3(0.0, 0.0, 0.0)
        inv = 1.0 / mag
        return Vec3(self.x * inv, self.y * inv, self.z * inv)

    def reflect(self, normal: Vec3) -> Vec3:
        """Reflexión especular: ``R = I - 2(I·N)N`` (con ``normal`` unitaria)."""
        factor = 2.0 * self.dot(normal)
        return Vec3(
            self.x - normal.x * factor,
            self.y - normal.y * factor,
            self.z - normal.z * factor,
        )

    def __repr__(self) -> str:
        return f"Vec3({self.x}, {self.y}, {self.z})"
