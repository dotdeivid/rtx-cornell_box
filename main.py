#!/usr/bin/env python3
"""
Path Tracer Cornell Box - Punto de entrada principal.

Ray tracer fotorealista con path tracing, NEE y BVH acceleration.
Renderiza la icónica Cornell Box con física de luz realista.

Uso:
    python main.py

El código ha sido refactorizado siguiendo SOLID principles y
design patterns para mejor mantenibilidad.

Autor: Sandoval, Carlos David
GitHub: @dotdeivid
"""

from src.config import RenderConfig, CameraConfig
from src.camera import Camera
from src.renderer import Renderer
from src.scene import create_cornell_box_scene, SceneMode


def main():
    """Función principal de rendering."""

    # ===== CONFIGURACIÓN =====

    # Configuración de rendering
    render_config = RenderConfig(
        width=200,
        height=200,
        samples=200,  # Aumentar para mejor calidad (más lento)
        max_depth=8,  # Rebotes máximos de luz
        use_parallel=True,  # Multi-core rendering
        gamma=2.2,
    )

    # Configuración de cámara
    camera_config = CameraConfig(
        # origin y lookat ya tienen defaults para Cornell Box
        fov=40.0,
        aperture=20.0,  # Depth of field (0 = todo enfocado)
    )

    # ==== SETUP ====

    # Crear cámara
    aspect_ratio = render_config.aspect_ratio
    camera = Camera(camera_config, aspect_ratio)

    # Crear renderer
    renderer = Renderer(render_config)

    # Crear escena
    # Opciones: SceneMode.SPHERES o SceneMode.BUNNY
    world, lights = create_cornell_box_scene(SceneMode.SPHERES)

    # ===== RENDER =====

    print("=" * 60)
    print("PATH TRACER - CORNELL BOX")
    print("=" * 60)

    renderer.render(world, lights, camera)

    print("=" * 60)
    print("✓ Rendering completado exitosamente")
    print("=" * 60)


if __name__ == "__main__":
    main()
