"""Paquete de materiales."""

from src.materials.base import Material
from src.materials.diffuse import DiffuseMaterial
from src.materials.metal import MetalMaterial
from src.materials.dielectric import DielectricMaterial
from src.materials.emissive import EmissiveMaterial

__all__ = [
    "Material",
    "DiffuseMaterial",
    "MetalMaterial",
    "DielectricMaterial",
    "EmissiveMaterial",
]
