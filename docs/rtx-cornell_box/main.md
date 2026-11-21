# `main.py` — Punto de entrada y CLI

Archivo raíz del proyecto. Define el sistema de presets, el parser de argumentos CLI (`argparse`) y orquesta el proceso completo de renderizado.

---

## Presets (`PRESETS`)

`Preset` es una `dataclass` que agrupa resolución, `samples`, `max_depth`, ruta de salida, parámetros de cámara (`fov`, `aperture`, `focus_distance`) y `scene_mode` (`SPHERES`, `BUNNY`, `BOXES`).

Presets incluidos: `PREVIEW`, `MEDIUM`, `HIGH`, `ULTRA`, `DOF`, `BUNNY`, `BOXES`, `TINY_PERFECT`.

---

## Funciones principales

- **`apply_preset(name)`**: devuelve `(RenderConfig, CameraConfig, SceneMode)` para el preset indicado (insensible a mayúsculas/minúsculas).
- **`parse_args(argv=None)`**: procesa flags CLI (`--preset`, `--list-presets`, `--scene`, `--width`, `--height`, `--samples`, `--max-depth`, `--fov`, `--aperture`, `--output`, `--seed`, `--sequential`).
- **`build_configs_from_args(args)`**: combina el preset base con cualquier override pasado por línea de comandos.
- **`main(argv=None)`**: construye `Camera`, `Renderer` y `create_cornell_box_scene(scene_mode)` y ejecuta el renderizado.

---

## Ejemplos de uso

```bash
python main.py --list-presets
python main.py --preset PREVIEW
python main.py --preset BOXES --samples 200 --seed 7 --output output/boxes.png
```
