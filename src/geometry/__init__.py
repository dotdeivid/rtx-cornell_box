"""
Módulo de geometría - primitivas y estructuras de aceleración.

Este paquete contiene todas las primitivas geométricas y estructuras
de datos para ray tracing eficiente.

Primitivas:
    - Sphere: Esferas
    - Quad: Cuadril áteros (paredes, luces de área)
    - Triangle: Triángulos (para modelos 3D)

Estructuras:
    - AABB: Axis-Aligned Bounding Boxes
    - BVHNode: Bounding Volume Hierarchy
    - HitRecord: Información de intersecciones

Uso:
    from src.geometry import Sphere, Quad, BVHNode
"""

from src.geometry.base import Geometry
from src.geometry.hit_record import HitRecord
from src.geometry.aabb import AABB
from src.geometry.sphere import Sphere
from src.geometry.quad import Quad
from src.geometry.triangle import Triangle
from src.geometry.bvh import BVHNode

__all__ = [
    "Geometry",
    "HitRecord",
    "AABB",
    "Sphere",
    "Quad",
    "Triangle",
    "BVHNode",
]
