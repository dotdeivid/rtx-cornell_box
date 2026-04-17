"""Scene builder - construcción de escenas Cornell Box."""

from enum import Enum
from typing import Tuple, List
from src.vector import Vec3
from src.geometry import Quad, Sphere, BVHNode
from src.materials import (
    DiffuseMaterial,
    MetalMaterial,
    DielectricMaterial,
    EmissiveMaterial,
)
from src.utils import load_obj


class SceneMode(Enum):
    """Modos de escena disponibles."""

    SPHERES = "spheres"
    BUNNY = "bunny"


def create_cornell_box_scene(mode: SceneMode = SceneMode.BUNNY) -> Tuple[BVHNode, List]:
    """
    Crea una escena Cornell Box con diferentes objetos centrales.

    Args:
        mode: Modo de escena (SPHERES o BUNNY)

    Returns:
        Tuple de (world: BVHNode, lights: List)
    """
    objects = []

    # Materiales de la Cornell Box
    red_diffuse = DiffuseMaterial(albedo=Vec3(0.65, 0.05, 0.05))
    white_diffuse = DiffuseMaterial(albedo=Vec3(0.73, 0.73, 0.73))
    green_diffuse = DiffuseMaterial(albedo=Vec3(0.12, 0.45, 0.15))
    light_material = EmissiveMaterial(emission=Vec3(15, 15, 15))

    # Paredes de Cornell Box
    # Pared izquierda (roja)
    objects.append(Quad(Vec3(555, 0, 0), Vec3(0, 555, 0), Vec3(0, 0, 555), red_diffuse))

    # Pared derecha (verde)
    objects.append(Quad(Vec3(0, 0, 0), Vec3(0, 555, 0), Vec3(0, 0, 555), green_diffuse))

    # Piso (blanco)
    objects.append(Quad(Vec3(0, 0, 0), Vec3(555, 0, 0), Vec3(0, 0, 555), white_diffuse))

    # Techo (blanco)
    objects.append(
        Quad(Vec3(555, 555, 555), Vec3(-555, 0, 0), Vec3(0, 0, -555), white_diffuse)
    )

    # Pared de fondo (blanca)
    objects.append(
        Quad(Vec3(0, 0, 555), Vec3(555, 0, 0), Vec3(0, 555, 0), white_diffuse)
    )

    # Luz de área (techo)
    light = Quad(
        Vec3(343, 554, 332),
        Vec3(-130, 0, 0),
        Vec3(0, 0, -105),
        light_material,
    )
    objects.append(light)

    # Objetos centrales según modo
    if mode == SceneMode.SPHERES:
        # Esfera de vidrio
        glass = DielectricMaterial(ior=1.5)
        objects.append(
            Sphere(
                center=Vec3(190, 90, 190),
                radius=90,
                material=glass,
            )
        )

        # Esfera metálica
        metal = MetalMaterial(albedo=Vec3(0.7, 0.6, 0.5), fuzz=0.05)
        objects.append(
            Sphere(
                center=Vec3(380, 90, 350),
                radius=90,
                material=metal,
            )
        )

    elif mode == SceneMode.BUNNY:
        # Cargar modelo 3D con material de vidrio
        glass = DielectricMaterial(ior=1.5)
        bunny_triangles = load_obj(
            "models/bunny.obj",
            scale=1400.0,
            offset=Vec3(278, 0, 278),
            material=glass,
        )
        objects.extend(bunny_triangles)

    # Construir BVH
    world = BVHNode.create(objects)

    # Las luces son objetos con EmissiveMaterial
    lights = [obj for obj in objects if hasattr(obj.material, "emission_color")]

    return world, lights
