# Guía de Uso — RTX Cornell Box Path Tracer

Path tracer fotorrealista que renderiza la Cornell Box usando Monte Carlo Ray Tracing con BVH, Next Event Estimation, Russian Roulette y profundidad de campo.

---

## 1. Requisitos e instalación

**Python 3.9+** requerido.

```bash
pip install -r requirements.txt
# Para desarrollo / tests:
pip install -r requirements-dev.txt
```

| Paquete | Uso |
|---|---|
| `numpy` | Buffers de imagen RGB |
| `Pillow` | Guardar imagen PNG |

---

## 2. Uso básico y CLI

```bash
# Render con el preset activo (por defecto TINY_PERFECT)
python main.py

# Listar presets disponibles
python main.py --list-presets

# Ejecutar un preset específico
python main.py --preset PREVIEW
python main.py --preset MEDIUM
python main.py --preset BOXES
python main.py --preset BUNNY

# Sobreescribir parámetros desde consola sin editar código
python main.py --preset PREVIEW --scene boxes --samples 100 --seed 42 --output output/test_boxes.png
```

---

## 3. Configuración de rendering (`RenderConfig`)

```python
from pathlib import Path
from src.config import RenderConfig

render_config = RenderConfig(
    width=400,
    height=400,
    samples=400,
    max_depth=8,
    use_parallel=True,
    output_path=Path("output/mi_render.png"),
    gamma=2.2,
    max_sample=10.0,   # None desactiva el clamp anti-fireflies
    seed=None,         # Entero para renders deterministas
)
```

---

## 4. Configuración de cámara (`CameraConfig`)

```python
from src.config import CameraConfig
from src.vector import Vec3

camera_config = CameraConfig(
    origin=Vec3(278, 278, -800),   # Posición de la cámara
    lookat=Vec3(278, 278, 278),    # Punto hacia donde mira
    vup=Vec3(0, 1, 0),             # Vector "arriba"
    fov=40.0,                      # Campo de visión vertical en grados
    aperture=0.0,                  # 0 = pinhole (todo enfocado), >0 = DOF
    focus_distance=None,           # None = auto-calcular |origin - lookat|
)
```

> Nota: `Camera` maneja automáticamente vistas cenitales donde `origin - lookat` es paralelo a `vup` eligiendo un vector de referencia ortogonal de respaldo.

---

## 5. Modos de escena (`SceneMode`)

```python
from src.scene import create_cornell_box_scene, SceneMode

world, lights = create_cornell_box_scene(SceneMode.SPHERES)  # Vidrio + metal
world, lights = create_cornell_box_scene(SceneMode.BOXES)    # Dos cajas clásicas (alta atrás, baja adelante)
world, lights = create_cornell_box_scene(SceneMode.BUNNY)    # Stanford Bunny de vidrio (4,968 triángulos)
```

### Estructura fija de la Cornell Box

| Elemento | Material | Posición |
|---|---|---|
| Pared izquierda en cámara | Difuso rojo `(0.65, 0.05, 0.05)` | `x = 555` |
| Pared derecha en cámara | Difuso verde `(0.12, 0.45, 0.15)` | `x = 0` |
| Piso | Difuso blanco `(0.73, 0.73, 0.73)` | `y = 0` |
| Techo | Difuso blanco | `y = 555` |
| Pared de fondo | Difuso blanco | `z = 555` |
| Luz de área | Emisivo `(15, 15, 15)` | Techo (`y = 554`, orientada hacia abajo) |

---

## 6. Materiales disponibles

Todos heredan de `Material` (ABC):

- `DiffuseMaterial(albedo=Vec3(...))` — Lambertiano mate, usa NEE para luz directa.
- `MetalMaterial(albedo=Vec3(...), fuzz=0.0)` — Especular con rugosidad en `[0.0, 1.0]`.
- `DielectricMaterial(ior=1.5)` — Vidrio/agua con Snell + Fresnel-Schlick + TIR (`ior > 0`).
- `EmissiveMaterial(emission=Vec3(...))` — Fuente de luz de área (emisión frontal).

---

## 7. Cargar modelos 3D (`.obj`)

```python
from src.utils import load_obj
from src.vector import Vec3
from src.materials import DiffuseMaterial

triangles = load_obj(
    filename="models/bunny.obj",
    scale=1400.0,
    offset=Vec3(278, 0, 278),
    material=DiffuseMaterial(Vec3(0.8, 0.8, 0.8)),
    ground_y=0.0,  # opcional: apoya el vértice más bajo en y=0
)
```

`load_obj` lanza `OSError` si el archivo no existe y `ValueError` si el contenido es inválido (sin caras, índices fuera de rango o números mal formados).

---

## 8. Flujo interno del renderizado

1. `Renderer.render()` lanza un `multiprocessing.Pool` inicializando `(world, lights, camera, config)` **una sola vez por worker** (o ejecuta secuencialmente si `use_parallel=False`).
2. Por cada píxel, genera `samples` rayos con **stratified sampling** en una cuadrícula `√samples × √samples` (más cualquier resto uniforme).
3. `color_ray()` intersecta el BVH (`O(log N)`), evalúa emisión frontal, `material.scatter()`, **NEE** para materiales difusos y **Russian Roulette** a partir del tercer rebote.
4. Promedia las muestras, aplica corrección gamma `rgb^(1/gamma)` y guarda el PNG volteado verticalmente.
