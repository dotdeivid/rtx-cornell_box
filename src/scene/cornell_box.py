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
    BUNNY   = "bunny"
    BOXES   = "boxes"


# ===========================================================================
# HELPERS PRIVADOS
# ===========================================================================

def _make_box(x0: float, x1: float, y0: float, y1: float,
              z0: float, z1: float, material) -> List[Quad]:
    """
    Construye una caja a partir de coordenadas mín/máx en cada eje.

    Genera 5 caras (sin base inferior, asumiendo que la caja
    descansa en el piso) con normales correctas apuntando hacia
    el interior de la escena.

    Args:
        x0, x1: Rango en X (x0 < x1)
        y0, y1: Rango en Y (y0 = suelo, y1 = techo de la caja)
        z0, z1: Rango en Z (z0 más cerca de la cámara)
        material: Material a aplicar a todas las caras

    Returns:
        Lista de 5 Quad que forman la caja
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
        # Cara izquierda (x=x0, normal -x)
        Quad(Vec3(x0, y0, z0), Vec3(0, 0, dz), Vec3(0, dy, 0), material),
        # Cara derecha (x=x1, normal +x)
        Quad(Vec3(x1, y0, z0), Vec3(0, dy, 0), Vec3(0, 0, dz), material),
    ]


# ===========================================================================
# CONSTRUCTORES DE OBJETOS CENTRALES
# ===========================================================================

def _get_spheres_objects() -> List:
    """
    Retorna los objetos centrales de la escena SPHERES:
    una esfera de vidrio y una esfera metálica.
    """
    objects = []

    # Esfera de vidrio (izquierda)
    glass = DielectricMaterial(ior=1.5)
    objects.append(Sphere(center=Vec3(190, 90, 190), radius=90, material=glass))

    # Esfera metálica (derecha)
    metal = MetalMaterial(albedo=Vec3(0.7, 0.6, 0.5), fuzz=0.05)
    objects.append(Sphere(center=Vec3(380, 90, 350), radius=90, material=metal))

    return objects


def _get_bunny_objects() -> List:
    """
    Retorna los objetos centrales de la escena BUNNY:
    el Stanford Bunny cargado desde OBJ, con material de vidrio.
    """
    glass = DielectricMaterial(ior=1.5)
    return load_obj(
        "models/bunny.obj",
        scale=1400.0,
        offset=Vec3(278, 0, 278),
        material=glass,
    )


def _get_boxes_objects() -> List:
    """
    Retorna los objetos centrales de la escena BOXES:
    las dos cajas rectangulares de la Cornell Box original (1984).

    - Caja alta (~330 unidades): a la izquierda
    - Caja baja (~165 unidades): a la derecha

    Dimensiones basadas en las especificaciones originales de
    Cornell University escaladas al sistema de coordenadas 555.
    """
    white_diffuse = DiffuseMaterial(albedo=Vec3(0.73, 0.73, 0.73))
    objects = []

    # Caja alta (izquierda)
    objects.extend(_make_box(130, 295, 0, 330, 65, 230, white_diffuse))

    # Caja baja (derecha)
    objects.extend(_make_box(265, 430, 0, 165, 295, 460, white_diffuse))

    return objects


# ===========================================================================
# CONSTRUCTOR PRINCIPAL DE LA ESCENA
# ===========================================================================

def create_cornell_box_scene(mode: SceneMode = SceneMode.SPHERES) -> Tuple[BVHNode, List]:
    """
    Crea una escena Cornell Box con los objetos centrales según el modo.

    Construye las paredes, el piso, el techo y la fuente de luz, luego
    delega la creación de los objetos centrales a la función
    correspondiente según el SceneMode recibido.

    Args:
        mode: SceneMode que determina qué objetos van en el interior.

    Returns:
        Tuple (world: BVHNode, lights: List) listos para el renderer.
    """
    objects = []

    # --- Materiales de la Cornell Box ---
    red_diffuse   = DiffuseMaterial(albedo=Vec3(0.65, 0.05, 0.05))
    white_diffuse = DiffuseMaterial(albedo=Vec3(0.73, 0.73, 0.73))
    green_diffuse = DiffuseMaterial(albedo=Vec3(0.12, 0.45, 0.15))
    light_material = EmissiveMaterial(emission=Vec3(15, 15, 15))

    # --- Paredes (normales apuntando hacia el interior) ---

    # Pared izquierda (roja, x=555): normal -x
    objects.append(Quad(Vec3(555, 0, 0), Vec3(0, 0, 555), Vec3(0, 555, 0), red_diffuse))

    # Pared derecha (verde, x=0): normal +x
    objects.append(Quad(Vec3(0, 0, 0), Vec3(0, 555, 0), Vec3(0, 0, 555), green_diffuse))

    # Piso (y=0): normal +y
    objects.append(Quad(Vec3(0, 0, 0), Vec3(0, 0, 555), Vec3(555, 0, 0), white_diffuse))

    # Techo (y=555): normal -y
    objects.append(Quad(Vec3(555, 555, 555), Vec3(-555, 0, 0), Vec3(0, 0, -555), white_diffuse))

    # Pared de fondo (z=555): normal -z
    objects.append(Quad(Vec3(0, 0, 555), Vec3(0, 555, 0), Vec3(555, 0, 0), white_diffuse))

    # --- Luz de área (techo) ---
    light = Quad(
        Vec3(343, 554, 332),
        Vec3(-130, 0, 0),
        Vec3(0, 0, -105),
        light_material,
    )
    objects.append(light)

    # --- Objetos centrales según modo ---
    scene_builders = {
        SceneMode.SPHERES: _get_spheres_objects,
        SceneMode.BUNNY:   _get_bunny_objects,
        SceneMode.BOXES:   _get_boxes_objects,
    }
    objects.extend(scene_builders[mode]())

    # --- Construir BVH y recolectar luces ---
    world = BVHNode.create(objects)
    lights = [obj for obj in objects if hasattr(obj.material, "emission_color")]

    return world, lights
