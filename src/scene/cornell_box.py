"""Scene builder - construcción de escenas Cornell Box."""

from enum import Enum
from pathlib import Path
from typing import Callable, Dict, List, Tuple

from src.vector import Vec3
from src.geometry import Geometry, Quad, Sphere, BVHNode
from src.materials import (
    Material,
    DiffuseMaterial,
    MetalMaterial,
    DielectricMaterial,
    EmissiveMaterial,
)
from src.renderer.path_tracer import AreaLight
from src.utils import load_obj


# Directorio de modelos resuelto desde la raíz del proyecto (independiente del CWD)
MODELS_DIR = Path(__file__).resolve().parents[2] / "models"


class SceneMode(Enum):
    """Modos de escena disponibles."""

    SPHERES = "spheres"
    BUNNY = "bunny"
    BOXES = "boxes"


# ===========================================================================
# HELPERS PRIVADOS
# ===========================================================================


def _make_box(
    x0: float,
    x1: float,
    y0: float,
    y1: float,
    z0: float,
    z1: float,
    material: Material,
) -> List[Geometry]:
    """
    Construye una caja a partir de coordenadas mín/máx en cada eje.

    Genera 5 caras (sin base inferior, asumiendo que la caja descansa en el piso)
    con normales apuntando hacia el exterior de la caja.
    """
    dx = x1 - x0
    dy = y1 - y0
    dz = z1 - z0

    return [
        # Cara superior (normal +y)
        Quad(Vec3(x0, y1, z0), Vec3(0, 0, dz), Vec3(dx, 0, 0), material),
        # Cara frontal (z=z0, normal -z, hacia la cámara)
        Quad(Vec3(x0, y0, z0), Vec3(0, dy, 0), Vec3(dx, 0, 0), material),
        # Cara trasera (z=z1, normal +z)
        Quad(Vec3(x0, y0, z1), Vec3(dx, 0, 0), Vec3(0, dy, 0), material),
        # Cara derecha en cámara (x=x0, normal -x)
        Quad(Vec3(x0, y0, z0), Vec3(0, 0, dz), Vec3(0, dy, 0), material),
        # Cara izquierda en cámara (x=x1, normal +x)
        Quad(Vec3(x1, y0, z0), Vec3(0, dy, 0), Vec3(0, 0, dz), material),
    ]


# ===========================================================================
# CONSTRUCTORES DE OBJETOS CENTRALES
# ===========================================================================


def _get_spheres_objects() -> List[Geometry]:
    """
    Retorna los objetos centrales de la escena SPHERES:
    una esfera de vidrio (derecha en cámara) y una esfera metálica (izquierda en cámara).
    """
    objects: List[Geometry] = []

    # Esfera de vidrio (x=190 -> derecha en pantalla, más cerca de la cámara)
    glass = DielectricMaterial(ior=1.5)
    objects.append(Sphere(center=Vec3(190, 90, 190), radius=90, material=glass))

    # Esfera metálica (x=380 -> izquierda en pantalla, más al fondo)
    metal = MetalMaterial(albedo=Vec3(0.7, 0.6, 0.5), fuzz=0.05)
    objects.append(Sphere(center=Vec3(380, 90, 350), radius=90, material=metal))

    return objects


def _get_bunny_objects() -> List[Geometry]:
    """
    Retorna los objetos centrales de la escena BUNNY:
    el Stanford Bunny cargado desde OBJ, con material de vidrio, apoyado en el piso.
    """
    glass = DielectricMaterial(ior=1.5)
    return list(
        load_obj(
            MODELS_DIR / "bunny.obj",
            scale=1400.0,
            offset=Vec3(278, 0, 278),
            material=glass,
            ground_y=0.0,
        )
    )


def _get_boxes_objects() -> List[Geometry]:
    """
    Retorna los objetos centrales de la escena BOXES:
    las dos cajas rectangulares de la Cornell Box clásica.

    En el sistema de cámara (origin z=-800 mirando hacia +z), +x está a la izquierda
    y -x a la derecha:
    - Caja alta (y=330): atrás a la izquierda (x=[265, 430], z=[295, 460])
    - Caja baja (y=165): adelante a la derecha (x=[130, 295], z=[65, 230])
    """
    white_diffuse = DiffuseMaterial(albedo=Vec3(0.73, 0.73, 0.73))
    objects: List[Geometry] = []

    # Caja alta (atrás, izquierda en pantalla)
    objects.extend(_make_box(265, 430, 0, 330, 295, 460, white_diffuse))

    # Caja baja (adelante, derecha en pantalla)
    objects.extend(_make_box(130, 295, 0, 165, 65, 230, white_diffuse))

    return objects


# ===========================================================================
# CONSTRUCTOR PRINCIPAL DE LA ESCENA
# ===========================================================================


def create_cornell_box_scene(
    mode: SceneMode = SceneMode.SPHERES,
) -> Tuple[Geometry, List[AreaLight]]:
    """
    Crea una escena Cornell Box con los objetos centrales según el modo.

    Returns:
        Tupla ``(world, lights)`` listos para el renderer.
    """
    objects: List[Geometry] = []

    # --- Materiales de la Cornell Box ---
    red_diffuse = DiffuseMaterial(albedo=Vec3(0.65, 0.05, 0.05))
    white_diffuse = DiffuseMaterial(albedo=Vec3(0.73, 0.73, 0.73))
    green_diffuse = DiffuseMaterial(albedo=Vec3(0.12, 0.45, 0.15))
    light_material = EmissiveMaterial(emission=Vec3(15, 15, 15))

    # --- Paredes (normales apuntando hacia el interior) ---

    # Pared izquierda en cámara (roja, x=555): normal -x
    objects.append(Quad(Vec3(555, 0, 0), Vec3(0, 0, 555), Vec3(0, 555, 0), red_diffuse))

    # Pared derecha en cámara (verde, x=0): normal +x
    objects.append(Quad(Vec3(0, 0, 0), Vec3(0, 555, 0), Vec3(0, 0, 555), green_diffuse))

    # Piso (y=0): normal +y
    objects.append(Quad(Vec3(0, 0, 0), Vec3(0, 0, 555), Vec3(555, 0, 0), white_diffuse))

    # Techo (y=555): normal -y
    objects.append(
        Quad(Vec3(555, 555, 555), Vec3(-555, 0, 0), Vec3(0, 0, -555), white_diffuse)
    )

    # Pared de fondo (z=555): normal -z
    objects.append(Quad(Vec3(0, 0, 555), Vec3(0, 555, 0), Vec3(555, 0, 0), white_diffuse))

    # --- Luz de área (techo, normal -y apuntando hacia abajo) ---
    light = Quad(
        Vec3(343, 554, 332),
        Vec3(-130, 0, 0),
        Vec3(0, 0, -105),
        light_material,
    )
    objects.append(light)

    # --- Objetos centrales según modo ---
    scene_builders: Dict[SceneMode, Callable[[], List[Geometry]]] = {
        SceneMode.SPHERES: _get_spheres_objects,
        SceneMode.BUNNY: _get_bunny_objects,
        SceneMode.BOXES: _get_boxes_objects,
    }
    objects.extend(scene_builders[mode]())

    # --- Construir BVH y recolectar luces (cualquier primitiva AreaLight emisiva) ---
    world = BVHNode.create(objects)
    lights: List[AreaLight] = [
        obj
        for obj in objects
        if isinstance(obj, AreaLight) and isinstance(obj.material, EmissiveMaterial)
    ]

    return world, lights
