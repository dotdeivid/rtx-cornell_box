# Guía de Uso — RTX Cornell Box Path Tracer

Path tracer fotorrealista que renderiza la Cornell Box usando Monte Carlo Ray Tracing con BVH, Next Event Estimation y profundidad de campo.

---

## Tabla de contenidos

1. [Requisitos e instalación](#1-requisitos-e-instalación)
2. [Uso básico](#2-uso-básico)
3. [Configuración de rendering](#3-configuración-de-rendering-renderconfig)
4. [Configuración de cámara](#4-configuración-de-cámara-cameraconfig)
5. [Modos de escena](#5-modos-de-escena)
6. [Materiales disponibles](#6-materiales-disponibles)
7. [Cargar modelos 3D (.obj)](#7-cargar-modelos-3d-obj)
8. [Calidad vs velocidad](#8-calidad-vs-velocidad)
9. [Flujo interno del renderizado](#9-flujo-interno-del-renderizado)

---

## 1. Requisitos e instalación

**Python 3.8+** requerido.

```bash
pip install -r requirements.txt
```

Dependencias principales:

| Paquete | Uso |
|---|---|
| `numpy` | Aritmética vectorial float64 |
| `Pillow` | Guardar imagen PNG |

---

## 2. Uso básico

```bash
python main.py
```

El resultado se guarda en `output/result_render.png`.

Todo el comportamiento se controla desde `main.py` editando los dos objetos de configuración:

```python
render_config = RenderConfig(...)   # calidad, resolución, salida
camera_config = CameraConfig(...)   # posición, lente, campo de visión
```

---

## 3. Configuración de rendering (`RenderConfig`)

```python
from src.config import RenderConfig
from pathlib import Path

render_config = RenderConfig(
    width=400,                              # Ancho de la imagen en píxeles
    height=400,                             # Alto de la imagen en píxeles
    samples=400,                            # Muestras por píxel (calidad)
    max_depth=8,                            # Rebotes máximos de luz
    use_parallel=True,                      # Multi-core (True recomendado)
    output_path=Path("output/mi_render.png"),
    gamma=2.2,                              # Corrección gamma para sRGB
)
```

### Parámetros explicados

#### `samples` — Muestras por píxel
Número de rayos lanzados por cada píxel. Más muestras = menos ruido, más tiempo.

```
samples=50   → Muy ruidoso, útil para pruebas rápidas (~5s)
samples=200  → Moderado, útil para iterar
samples=400  → Calidad por defecto (~2min)
samples=1000 → Alta calidad (~5min)
samples=4000 → Producción, casi sin ruido (~20min)
```

#### `max_depth` — Profundidad de rebotes
Cuántas veces puede rebotar un rayo antes de terminar. Afecta iluminación indirecta y caustics de vidrio.

```
max_depth=2  → Solo luz directa + un rebote
max_depth=8  → Por defecto, buen balance
max_depth=16 → Más caustics, cáusticas más pronunciadas en vidrio
```

#### `gamma` — Corrección gamma
`2.2` es el valor correcto para displays sRGB modernos (prácticamente todos). No cambiar salvo que se sepa lo que se hace.

#### `use_parallel`
Usa todos los núcleos de CPU disponibles distribuyendo filas entre procesos. Siempre `True` en producción. Usar `False` solo para depuración (los prints de progreso aparecen ordenados).

---

## 4. Configuración de cámara (`CameraConfig`)

```python
from src.config import CameraConfig
from src.vector import Vec3

camera_config = CameraConfig(
    origin=Vec3(278, 278, -800),   # Posición de la cámara
    lookat=Vec3(278, 278, 278),    # Punto hacia donde mira
    vup=Vec3(0, 1, 0),             # Vector "arriba" (no cambiar normalmente)
    fov=40.0,                      # Campo de visión vertical en grados
    aperture=20.0,                 # Apertura del lente (depth of field)
    focus_distance=None,           # None = auto-calcular desde lookat
)
```

### Parámetros explicados

#### `origin` y `lookat`
Definen dónde está la cámara y hacia dónde apunta. La Cornell Box ocupa el espacio `[0,555]³`, centrada en `(278, 278, 278)`.

```python
# Vista frontal estándar (por defecto)
origin = Vec3(278, 278, -800)
lookat = Vec3(278, 278, 278)

# Vista desde arriba (cenital)
origin = Vec3(278, 900, 278)
lookat = Vec3(278, 278, 278)

# Vista en diagonal
origin = Vec3(-200, 400, -400)
lookat = Vec3(278, 278, 278)
```

#### `fov` — Campo de visión
Ángulo vertical del viewport en grados. Valores menores = más zoom.

```
fov=20  → Muy teleobjetivo, poco distorsionado
fov=40  → Por defecto para Cornell Box
fov=60  → Gran angular ligero
fov=90  → Gran angular pronunciado
```

#### `aperture` — Profundidad de campo (DOF)
Simula el lente físico de una cámara real.

```
aperture=0    → Sin DOF, todo nítido (cámara pinhole ideal)
aperture=5    → DOF muy sutil
aperture=20   → DOF moderado (por defecto)
aperture=60   → DOF muy pronunciado, bokeh dramático
```

> El efecto de desenfoque se produce en objetos que estén fuera del **plano focal**. El plano focal está por defecto a la distancia desde `origin` hasta `lookat`.

#### `focus_distance` — Distancia focal manual
Por defecto (`None`) se calcula automáticamente como `|origin - lookat|`. Se puede fijar manualmente:

```python
focus_distance=1078.0   # Distancia exacta origin→lookat por defecto
focus_distance=600.0    # El plano nítido estará más cerca que lookat
```

---

## 5. Modos de escena

```python
from src.scene import create_cornell_box_scene, SceneMode

# Dos esferas: vidrio + metal
world, lights = create_cornell_box_scene(SceneMode.SPHERES)

# Stanford Bunny de vidrio (~69.000 triángulos, más lento)
world, lights = create_cornell_box_scene(SceneMode.BUNNY)
```

### `SceneMode.SPHERES`
La escena clásica con dos esferas:
- **Esfera izquierda**: vidrio (DielectricMaterial, ior=1.5) — refracción + reflexión interna total
- **Esfera derecha**: metal (MetalMaterial, fuzz=0.05) — reflexión especular con leve rugosidad

### `SceneMode.BUNNY`
Carga el modelo `models/bunny.obj` (Stanford Bunny) con material de vidrio. Requiere el archivo en `models/bunny.obj`. La construcción del BVH puede tardar unos segundos en la primera vez.

### Estructura fija de la Cornell Box
Independientemente del modo, la escena siempre incluye:

| Elemento | Material | Posición |
|---|---|---|
| Pared izquierda | Difuso rojo `(0.65, 0.05, 0.05)` | `x = 555` |
| Pared derecha | Difuso verde `(0.12, 0.45, 0.15)` | `x = 0` |
| Piso | Difuso blanco `(0.73, 0.73, 0.73)` | `y = 0` |
| Techo | Difuso blanco | `y = 555` |
| Pared de fondo | Difuso blanco | `z = 555` |
| Luz de área | Emisivo `(15, 15, 15)` | Techo, centrada |

---

## 6. Materiales disponibles

Todos los materiales heredan de `Material` (ABC). Se instancian y se pasan a las primitivas geométricas.

### `DiffuseMaterial` — Lambertiano (mate)

```python
from src.materials import DiffuseMaterial
from src.vector import Vec3

mat = DiffuseMaterial(albedo=Vec3(0.8, 0.3, 0.3))  # Rojo mate
```

Dispersa la luz uniformemente en el hemisferio. Recibe iluminación directa mediante NEE (Next Event Estimation). El color `albedo` va en `[0, 1]` por componente.

### `MetalMaterial` — Espejo / Metal

```python
from src.materials import MetalMaterial

mat = MetalMaterial(
    albedo=Vec3(0.7, 0.6, 0.5),   # Color del metal
    fuzz=0.0,                      # 0 = espejo perfecto, 1 = muy rugoso
)
```

Reflexión especular pura. El parámetro `fuzz` añade perturbación aleatoria a la dirección reflejada:

```
fuzz=0.0  → Espejo perfecto
fuzz=0.1  → Metal ligeramente cepillado
fuzz=0.5  → Metal rugoso / mate metalizado
fuzz=1.0  → Máxima rugosidad (casi difuso)
```

### `DielectricMaterial` — Vidrio / Agua

```python
from src.materials import DielectricMaterial

mat = DielectricMaterial(ior=1.5)   # Vidrio estándar
```

Implementa refracción (Ley de Snell) + Fresnel (aproximación de Schlick) + reflexión interna total. El índice de refracción `ior` controla cuánto dobla la luz:

```
ior=1.0   → Aire (sin refracción)
ior=1.33  → Agua
ior=1.5   → Vidrio (por defecto)
ior=2.4   → Diamante
```

### `EmissiveMaterial` — Fuente de luz

```python
from src.materials import EmissiveMaterial

mat = EmissiveMaterial(emission=Vec3(15, 15, 15))   # Luz blanca intensa
```

No dispersa rayos, solo emite luz. Los valores por encima de `1.0` representan intensidades mayores que blanco puro, lo que da luces realistas sin saturación inmediata. Para colores:

```python
EmissiveMaterial(emission=Vec3(20, 15, 8))    # Luz cálida/naranja
EmissiveMaterial(emission=Vec3(8, 12, 20))    # Luz fría/azul
EmissiveMaterial(emission=Vec3(15, 15, 15))   # Blanca neutra
```

---

## 7. Cargar modelos 3D (.obj)

```python
from src.utils import load_obj
from src.vector import Vec3
from src.materials import DiffuseMaterial

triangles = load_obj(
    filename="models/mi_modelo.obj",
    scale=1.0,          # Factor de escala uniforme
    offset=Vec3(0,0,0), # Traslación en el espacio
    material=DiffuseMaterial(Vec3(0.8, 0.8, 0.8)),
)
```

### Parámetros

| Parámetro | Tipo | Descripción |
|---|---|---|
| `filename` | `str` | Ruta al archivo `.obj` |
| `scale` | `float` | Escala uniforme. `1.0` = tamaño original |
| `offset` | `Vec3` | Desplaza el modelo en el espacio |
| `material` | `Material` | Material compartido por todos los triángulos |

### Flujo de carga

1. Lee cada línea `v x y z` → crea vértice, aplica `* scale + offset`
2. Lee cada línea `f v1 v2 v3` → crea triángulos (soporta polígonos n-lados con triangulación fan)
3. Ignora líneas `vt` (texturas) y `vn` (normales de vértice) — las normales se recalculan por triángulo
4. Retorna `list[Triangle]`

### Añadir el modelo a la escena

Después de cargar, envolver en BVH y añadir a la escena:

```python
from src.geometry import BVHNode

objects = []
objects.extend(triangles)          # Añadir triángulos a la escena
world = BVHNode.create(objects)    # El BVH maneja la aceleración
```

> Para modelos de más de 1000 triángulos, **siempre usar BVH**. Sin él, el render de un modelo de 69.000 triángulos tardaría horas en lugar de minutos.

---

## 8. Calidad vs velocidad

### Presets orientativos

| Preset | `samples` | `max_depth` | Resolución | Tiempo aprox. |
|---|---|---|---|---|
| Preview | 50 | 4 | 200×200 | ~10s |
| Desarrollo | 200 | 6 | 400×400 | ~45s |
| Estándar | 400 | 8 | 400×400 | ~2min |
| Alta calidad | 1000 | 10 | 600×600 | ~10min |
| Producción | 4000 | 16 | 800×800 | ~60min |

> Los tiempos varían según la CPU y el número de núcleos disponibles.

### Qué afecta más al tiempo

1. **`samples`** — Impacto lineal. Doblar samples = doble de tiempo.
2. **Resolución** — Impacto cuadrático. Pasar de 400 a 800 = 4× más píxeles.
3. **`max_depth`** — Impacto moderado. Relevante con materiales transparentes.
4. **Modo BUNNY** — La carga del modelo y el BVH añaden ~5s extra al inicio.

### Recomendaciones

- Para iterar rápido: `samples=50`, `width=200`, `height=200`, `use_parallel=True`
- Para resultado final: `samples≥1000`, resolución alta, `max_depth=10`
- El DOF (`aperture>0`) no añade costo computacional extra

---

## 9. Flujo interno del renderizado

Para entender qué sucede al llamar `renderer.render(world, lights, camera)`:

```
Renderer.render()
│
├── _render_parallel()  [si use_parallel=True]
│   └── Pool de procesos → un proceso por fila de píxeles
│
└── _render_row(y)  [por cada fila]
    └── Por cada píxel (x, y):
        ├── Generar N=samples rayos con muestreo estratificado
        │   └── u = (x + random) / width    # offset dentro del píxel
        │   └── v = (y + random) / height
        │
        ├── camera.get_ray(u, v)
        │   ├── Offset aleatorio en disco de apertura (DOF)
        │   └── Ray(origin + offset, focal_plane_point - origin - offset)
        │
        └── color_ray(ray, world, lights, depth=max_depth)
            │
            ├── world.hit(ray)  → BVHNode traversal O(log n)
            │   ├── AABB.hit()  → test rápido de bounding box
            │   └── geometry.hit()  → Sphere/Quad/Triangle intersección exacta
            │
            ├── Si hit es EmissiveMaterial → retornar emisión directamente
            │
            ├── material.scatter(ray, hit)
            │   ├── DiffuseMaterial → dirección aleatoria en hemisferio
            │   ├── MetalMaterial   → reflexión especular ± fuzz
            │   └── DielectricMaterial → refracción (Snell) o reflexión (Fresnel)
            │
            ├── Si DiffuseMaterial:
            │   ├── calculate_nee()  → shadow ray a cada luz
            │   │   └── solid_angle sampling → BRDF Lambertiano → luz directa
            │   └── color_ray(scattered_ray, depth-1)  → luz indirecta
            │
            └── Si Metal / Dielectric:
                └── color_ray(scattered_ray, depth-1)  → solo rebote especular

        Resultado: promedio de N samples → gamma correction → [0,255] RGB
```

### Conceptos clave del algoritmo

**Stratified sampling** — El offset de cada sample dentro del píxel es `(x + random(), y + random())` en lugar de exactamente `(x, y)`. Esto distribuye mejor las muestras y reduce el aliasing.

**BVH (Bounding Volume Hierarchy)** — Árbol binario que organiza la geometría espacialmente. Reduce las pruebas de intersección de O(n) a O(log n). Para el Bunny (69.000 triángulos) supone una aceleración de ~9.000×.

**NEE (Next Event Estimation)** — En lugar de esperar que un rayo difuso golpee la luz por azar, se samplea directamente una dirección hacia cada fuente de luz. Reduce drásticamente el ruido en materiales mate.

**`puede_ver_luz`** — Flag interno que evita contar la emisión de la luz dos veces: una vez por NEE (iluminación directa) y otra vez si el rayo rebotado la golpea directamente.
