#!/usr/bin/env python3
"""
Path Tracer Cornell Box - Punto de entrada principal.

Ray tracer fotorealista con path tracing, NEE y BVH acceleration.
Renderiza la icónica Cornell Box con física de luz realista.

Uso:
    python main.py

Seleccionar preset:
    Cambiá la línea `ACTIVE_PRESET` al nombre del preset deseado.
    Podés agregar tus propios presets en el diccionario PRESETS.

Autor: Sandoval, Carlos David
GitHub: @dotdeivid
"""

from pathlib import Path
from dataclasses import dataclass
from typing import Optional

from src.config import RenderConfig, CameraConfig
from src.vector import Vec3
from src.camera import Camera
from src.renderer import Renderer
from src.scene import create_cornell_box_scene, SceneMode


# ===========================================================================
# SISTEMA DE PRESETS
# ===========================================================================

@dataclass
class Preset:
    """
    Configuración completa de render + cámara + escena en un solo objeto.

    Agregá presets nuevos al diccionario PRESETS con el mismo formato.
    """
    # Render
    width: int
    height: int
    samples: int
    max_depth: int
    output_path: str
    use_parallel: bool = True
    gamma: float = 2.2

    # Cámara
    fov: float = 40.0
    aperture: float = 0.0
    focus_distance: Optional[float] = None

    # Escena
    scene_mode: SceneMode = SceneMode.SPHERES


# ---------------------------------------------------------------------------
# Presets disponibles — agregá los tuyos acá
# ---------------------------------------------------------------------------
PRESETS: dict[str, Preset] = {

    "PREVIEW": Preset(
        width=200, height=200,
        samples=50, max_depth=4,
        output_path="output/preview.png",
        aperture=0.0,
    ),

    "MEDIUM": Preset(
        width=400, height=400,
        samples=400, max_depth=8,
        output_path="output/render_medium.png",
        aperture=0.0,
    ),

    "HIGH": Preset(
        width=800, height=800,
        samples=1000, max_depth=10,
        output_path="output/render_high.png",
        aperture=0.0,
    ),

    "ULTRA": Preset(
        width=800, height=800,
        samples=2000, max_depth=12,
        output_path="output/render_ultra.png",
        aperture=0.0,
    ),

    "DOF": Preset(
        width=400, height=400,
        samples=400, max_depth=8,
        output_path="output/render_dof.png",
        aperture=20.0,          # Depth of field activado
        focus_distance=None,    # Foco en lookat automático
    ),

    "BUNNY": Preset(
        width=400, height=400,
        samples=400, max_depth=8,
        output_path="output/render_bunny.png",
        aperture=0.0,
        scene_mode=SceneMode.BUNNY,
    ),

    "BOXES": Preset(
        width=400, height=400,
        samples=400, max_depth=8,
        output_path="output/render_boxes.png",
        aperture=0.0,
        scene_mode=SceneMode.BOXES,
    ),

    "TINY_PERFECT": Preset(
        width=200, height=200,
        samples=2000, max_depth=16,
        output_path="output/render_tiny_perfect.png",
        aperture=0.0,
    ),

}

# ===========================================================================
# SELECCIÓN ACTIVA — cambiá este valor para cambiar la calidad
# ===========================================================================
ACTIVE_PRESET = "TINY_PERFECT"


# ===========================================================================
# LÓGICA PRINCIPAL
# ===========================================================================

def apply_preset(name: str) -> tuple[RenderConfig, CameraConfig, SceneMode]:
    """
    Devuelve (RenderConfig, CameraConfig, SceneMode) para el preset dado.

    Args:
        name: Nombre del preset (debe existir en PRESETS)

    Returns:
        Tupla con las configuraciones listas para usar.

    Raises:
        KeyError: Si el nombre del preset no existe.
    """
    if name not in PRESETS:
        available = ", ".join(PRESETS.keys())
        raise KeyError(f"Preset '{name}' no existe. Disponibles: {available}")

    p = PRESETS[name]

    render_config = RenderConfig(
        width=p.width,
        height=p.height,
        samples=p.samples,
        max_depth=p.max_depth,
        use_parallel=p.use_parallel,
        gamma=p.gamma,
        output_path=Path(p.output_path),
    )

    camera_config = CameraConfig(
        fov=p.fov,
        aperture=p.aperture,
        focus_distance=p.focus_distance,
    )

    return render_config, camera_config, p.scene_mode


def main():
    """Función principal de rendering."""

    # ===== CARGAR PRESET =====
    render_config, camera_config, scene_mode = apply_preset(ACTIVE_PRESET)

    # ==== SETUP ====
    aspect_ratio = render_config.aspect_ratio
    camera = Camera(camera_config, aspect_ratio)
    renderer = Renderer(render_config)
    world, lights = create_cornell_box_scene(scene_mode)

    # ===== RENDER =====
    print("=" * 60)
    print(f"PATH TRACER - CORNELL BOX  [{ACTIVE_PRESET}]")
    print("=" * 60)
    print(f"Resolución : {render_config.width}×{render_config.height}")
    print(f"Samples    : {render_config.samples}")
    print(f"Max depth  : {render_config.max_depth}")
    print(f"Escena     : {scene_mode.value}")
    print(f"Output     : {render_config.output_path}")
    print("=" * 60)

    renderer.render(world, lights, camera)

    print("=" * 60)
    print("✓ Rendering completado exitosamente")
    print("=" * 60)


if __name__ == "__main__":
    main()
