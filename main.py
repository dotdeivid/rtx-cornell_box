#!/usr/bin/env python3
"""
Path Tracer Cornell Box - Punto de entrada principal con CLI y sistema de presets.

Ray tracer fotorrealista con path tracing, NEE, Russian Roulette y BVH acceleration.

Uso:
    python main.py
    python main.py --preset MEDIUM
    python main.py --preset PREVIEW --scene boxes --samples 100 --output output/boxes.png
    python main.py --list-presets

Autor: Sandoval, Carlos David
GitHub: @dotdeivid
"""

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Sequence, Tuple

from src.config import RenderConfig, CameraConfig
from src.camera import Camera
from src.renderer import Renderer
from src.scene import create_cornell_box_scene, SceneMode


# ===========================================================================
# SISTEMA DE PRESETS
# ===========================================================================


@dataclass
class Preset:
    """Configuración completa de render + cámara + escena en un solo objeto."""

    # Render
    width: int
    height: int
    samples: int
    max_depth: int
    output_path: str
    use_parallel: bool = True
    gamma: float = 2.2
    max_sample: Optional[float] = 10.0

    # Cámara
    fov: float = 40.0
    aperture: float = 0.0
    focus_distance: Optional[float] = None

    # Escena
    scene_mode: SceneMode = SceneMode.SPHERES


PRESETS: Dict[str, Preset] = {
    "PREVIEW": Preset(
        width=200,
        height=200,
        samples=50,
        max_depth=4,
        output_path="output/preview.png",
        aperture=0.0,
    ),
    "MEDIUM": Preset(
        width=400,
        height=400,
        samples=400,
        max_depth=8,
        output_path="output/render_medium.png",
        aperture=0.0,
    ),
    "HIGH": Preset(
        width=800,
        height=800,
        samples=1000,
        max_depth=10,
        output_path="output/render_high.png",
        aperture=0.0,
    ),
    "ULTRA": Preset(
        width=800,
        height=800,
        samples=2000,
        max_depth=12,
        output_path="output/render_ultra.png",
        aperture=0.0,
    ),
    "DOF": Preset(
        width=400,
        height=400,
        samples=400,
        max_depth=8,
        output_path="output/render_dof.png",
        aperture=20.0,
        focus_distance=None,
    ),
    "BUNNY": Preset(
        width=400,
        height=400,
        samples=400,
        max_depth=8,
        output_path="output/render_bunny.png",
        aperture=0.0,
        scene_mode=SceneMode.BUNNY,
    ),
    "BOXES": Preset(
        width=400,
        height=400,
        samples=400,
        max_depth=8,
        output_path="output/render_boxes.png",
        aperture=0.0,
        scene_mode=SceneMode.BOXES,
    ),
    "TINY_PERFECT": Preset(
        width=200,
        height=200,
        samples=2000,
        max_depth=16,
        output_path="output/render_tiny_perfect.png",
        aperture=0.0,
    ),
}

ACTIVE_PRESET = "TINY_PERFECT"


# ===========================================================================
# LÓGICA PRINCIPAL Y CLI
# ===========================================================================


def apply_preset(name: str) -> Tuple[RenderConfig, CameraConfig, SceneMode]:
    """
    Devuelve ``(RenderConfig, CameraConfig, SceneMode)`` para el preset dado.

    Raises:
        KeyError: Si el nombre del preset no existe.
    """
    key = name.upper()
    if key not in PRESETS:
        available = ", ".join(PRESETS.keys())
        raise KeyError(f"Preset '{name}' no existe. Disponibles: {available}")

    p = PRESETS[key]

    render_config = RenderConfig(
        width=p.width,
        height=p.height,
        samples=p.samples,
        max_depth=p.max_depth,
        use_parallel=p.use_parallel,
        gamma=p.gamma,
        max_sample=p.max_sample,
        output_path=Path(p.output_path),
    )

    camera_config = CameraConfig(
        fov=p.fov,
        aperture=p.aperture,
        focus_distance=p.focus_distance,
    )

    return render_config, camera_config, p.scene_mode


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """Parsea argumentos de línea de comandos."""
    parser = argparse.ArgumentParser(
        description="Path Tracer fotorrealista para la Cornell Box."
    )
    parser.add_argument(
        "--preset",
        default=ACTIVE_PRESET,
        type=str.upper,
        choices=list(PRESETS.keys()),
        help=f"Nombre del preset a usar (por defecto: {ACTIVE_PRESET})",
    )
    parser.add_argument(
        "--list-presets",
        action="store_true",
        help="Muestra los presets disponibles y termina.",
    )
    parser.add_argument(
        "--scene",
        choices=[m.value for m in SceneMode],
        help="Sobreescribe la escena del preset (spheres, bunny, boxes).",
    )
    parser.add_argument("--width", type=int, help="Sobreescribe el ancho en píxeles.")
    parser.add_argument("--height", type=int, help="Sobreescribe el alto en píxeles.")
    parser.add_argument("--samples", type=int, help="Sobreescribe samples por píxel.")
    parser.add_argument("--max-depth", type=int, help="Sobreescribe rebotes máximos.")
    parser.add_argument("--fov", type=float, help="Sobreescribe FOV vertical en grados.")
    parser.add_argument("--aperture", type=float, help="Sobreescribe apertura DOF.")
    parser.add_argument("--output", type=str, help="Sobreescribe ruta de salida PNG.")
    parser.add_argument("--seed", type=int, help="Semilla RNG para render reproducible.")
    parser.add_argument(
        "--sequential",
        action="store_true",
        help="Desactiva multiprocessing y renderiza en un solo núcleo.",
    )
    return parser.parse_args(argv)


def build_configs_from_args(
    args: argparse.Namespace,
) -> Tuple[RenderConfig, CameraConfig, SceneMode]:
    """Construye configuraciones finales aplicando overrides de CLI sobre el preset."""
    render_config, camera_config, scene_mode = apply_preset(args.preset)

    if args.scene is not None:
        scene_mode = SceneMode(args.scene)

    render_config = RenderConfig(
        width=args.width if args.width is not None else render_config.width,
        height=args.height if args.height is not None else render_config.height,
        samples=args.samples if args.samples is not None else render_config.samples,
        max_depth=(
            args.max_depth if args.max_depth is not None else render_config.max_depth
        ),
        use_parallel=False if args.sequential else render_config.use_parallel,
        gamma=render_config.gamma,
        max_sample=render_config.max_sample,
        seed=args.seed,
        output_path=(
            Path(args.output) if args.output is not None else render_config.output_path
        ),
    )

    camera_config = CameraConfig(
        origin=camera_config.origin,
        lookat=camera_config.lookat,
        vup=camera_config.vup,
        fov=args.fov if args.fov is not None else camera_config.fov,
        aperture=args.aperture if args.aperture is not None else camera_config.aperture,
        focus_distance=camera_config.focus_distance,
    )

    return render_config, camera_config, scene_mode


def main(argv: Optional[Sequence[str]] = None) -> None:
    """Función principal de rendering."""
    args = parse_args(argv)

    if args.list_presets:
        print("Presets disponibles:")
        for name, p in PRESETS.items():
            print(
                f"  {name:<14} {p.width}×{p.height}  samples={p.samples:<4} "
                f"depth={p.max_depth:<2} scene={p.scene_mode.value:<7} -> {p.output_path}"
            )
        return

    render_config, camera_config, scene_mode = build_configs_from_args(args)

    aspect_ratio = render_config.aspect_ratio
    camera = Camera(camera_config, aspect_ratio)
    renderer = Renderer(render_config)
    world, lights = create_cornell_box_scene(scene_mode)

    print("=" * 60)
    print(f"PATH TRACER - CORNELL BOX  [{args.preset.upper()}]")
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
