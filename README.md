# Ray Tracer - Cornell Box

Un **path tracer fotorrealista** implementado en Python que simula la física de la luz para renderizar la icónica Cornell Box con materiales avanzados (vidrio, metal, difuso) y modelos 3D complejos.

![Python](https://img.shields.io/badge/Python-3.9+-blue.svg)
![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)
![Status](https://img.shields.io/badge/Status-Active-success.svg)

---

## Tabla de Contenidos

- [Características](#características)
- [¿Qué hace este proyecto?](#qué-hace-este-proyecto)
- [¿Cómo funciona?](#cómo-funciona)
- [Instalación](#instalación)
- [Uso y CLI](#uso-y-cli)
- [Presets y Configuraciones de Rendering](#presets-y-configuraciones-de-rendering)
- [Efecto de los Parámetros](#efecto-de-los-parámetros)
- [Estructura del Proyecto](#estructura-del-proyecto)
- [Documentación](#documentación)
- [Rendimiento](#rendimiento)
- [Troubleshooting](#troubleshooting)
- [Referencias](#referencias)

---

## Características

### Técnicas de Rendering
- **Path Tracing**: Integración Monte Carlo de la ecuación de rendering
- **Next Event Estimation (NEE)**: Muestreo directo de luces de área para reducir ruido 10-100×
- **Russian Roulette**: Terminación sin sesgo de caminos profundos de baja energía
- **Stratified Sampling**: Antialiasing con cuadrícula sub-píxel `√samples × √samples`
- **BVH (Bounding Volume Hierarchy)**: Aceleración `O(log N)` con división por eje más largo y centroide

### Materiales Físicamente Basados
- **Difusos (Lambertian)**: Superficies mates con reflexión coseno-ponderada
- **Metales**: Reflexión especular con rugosidad ajustable (`fuzz`)
- **Dieléctricos**: Vidrio/agua con refracción (Ley de Snell + Fresnel-Schlick + reflexión total interna)
- **Emisores**: Luces de área de una cara para sombras suaves con penumbra

### Cámara Avanzada
- **Profundidad de Campo (DOF)**: Modelo *thin-lens* con desenfoque bokeh físicamente correcto
- **Field of View ajustable**: Teleobjetivo a gran angular
- **Distancia focal configurable**: Control del plano de enfoque y manejo seguro de vistas cenitales

### Optimizaciones
- **Renderizado paralelo**: Multi-core con `multiprocessing.Pool` e inicialización única del BVH por worker
- **`Vec3` con floats nativos y `__slots__`**: Elimina el overhead de crear arrays NumPy de 3 elementos por operación
- **BVH Tree**: `O(log N)` intersecciones con AABBs y aristas precalculadas
- **Gamma Correction**: Corrección sRGB 2.2 para displays modernos

---

## ¿Qué hace este proyecto?

Este ray tracer simula cómo la luz real rebota en una escena 3D para generar imágenes fotorrealistas. A diferencia de la rasterización (OpenGL/DirectX), trazamos rayos desde la cámara hacia cada píxel, seguimos su trayectoria mientras rebotan en objetos, refractan a través de vidrio y finalmente llegan a fuentes de luz.

### Escenas Disponibles

- **Cornell Box con Esferas (`SceneMode.SPHERES`)**: Escena con esfera de vidrio y esfera metálica (color bleeding, cáusticas, reflexiones especulares, sombras suaves).
- **Cornell Box con Cajas (`SceneMode.BOXES`)**: Escena clásica de Cornell (1984) con caja alta trasera y caja baja delantera difusas.
- **Cornell Box con Bunny (`SceneMode.BUNNY`)**: Stanford Bunny de vidrio cargado desde `models/bunny.obj` (2,503 vértices, 4,968 triángulos).

---

## ¿Cómo funciona?

### Pipeline de Rendering

```
1. Configuración (CLI + Presets)
   ├─ RenderConfig: resolución, samples, profundidad, seed, max_sample, salida
   └─ CameraConfig: posición, FOV, apertura DOF, distancia focal

2. Por cada píxel
   └─ N muestras (stratified sampling en grilla √N × √N)
       └─ Por cada muestra:
           ├─ Rayo desde cámara (con offset DOF si aperture > 0)
           └─ color_ray() recursivo (hasta max_depth + Russian Roulette)
               ├─ Intersección con escena (BVH: O(log N))
               ├─ Emisor  → retornar emisión si es cara frontal y can_see_light
               ├─ Metal   → reflexión especular + fuzz
               ├─ Vidrio  → refracción/reflexión según Fresnel-Schlick
               └─ Difuso  → NEE (luz directa) + rebote aleatorio (luz indirecta)

3. Post-proceso
   ├─ Promedio de muestras (con clamping configurable anti-fireflies)
   ├─ Gamma correction (rgb^(1/2.2))
   └─ Guardar PNG
```

---

## Instalación

### Requisitos

- Python 3.9 o superior
- ~500 MB de RAM para renderizado estándar
- CPU multi-core recomendado

### Pasos

```bash
# 1. Clonar repositorio
git clone https://github.com/dotdeivid/rtx-cornell_box.git
cd rtx-cornell_box

# 2. Crear entorno virtual (recomendado)
python3 -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate

# 3. Instalar dependencias
pip install -r requirements.txt

# (Opcional) Dependencias de desarrollo y tests
pip install -r requirements-dev.txt
pytest -v
mypy src main.py
```

### Dependencias

```
numpy   # Buffers de imagen y conversión RGB
pillow  # Guardar imágenes PNG
```

---

## Uso y CLI

### Renderizado Básico

```bash
# Usa el preset activo definido en main.py (por defecto TINY_PERFECT)
python main.py

# Listar presets disponibles
python main.py --list-presets

# Elegir preset y escena por línea de comandos
python main.py --preset PREVIEW
python main.py --preset BOXES
python main.py --preset BUNNY --samples 200 --output output/mi_bunny.png

# Render reproducible con semilla fija y modo secuencial
python main.py --preset PREVIEW --scene spheres --seed 42 --sequential
```

### Flags disponibles en `main.py`

| Flag | Descripción |
|------|-------------|
| `--preset` | Nombre del preset (`PREVIEW`, `MEDIUM`, `HIGH`, `ULTRA`, `DOF`, `BUNNY`, `BOXES`, `TINY_PERFECT`) |
| `--list-presets` | Muestra todos los presets disponibles y termina |
| `--scene` | Sobreescribe la escena (`spheres`, `bunny`, `boxes`) |
| `--width`, `--height` | Sobreescribe resolución en píxeles |
| `--samples` | Sobreescribe muestras por píxel |
| `--max-depth` | Sobreescribe rebotes máximos de luz |
| `--fov`, `--aperture` | Sobreescribe parámetros ópticos de cámara |
| `--output` | Ruta del PNG de salida |
| `--seed` | Semilla RNG determinista (mismo resultado en modo secuencial y paralelo) |
| `--sequential` | Desactiva `multiprocessing` y renderiza en un solo núcleo |

---

## Presets y Configuraciones de Rendering

Los presets viven en `PRESETS` dentro de `main.py`:

| Preset | Resolución | Samples | Depth | Escena | Aperture | Error MC (`1/√N`) |
|--------|-----------|---------|-------|--------|----------|-------------------|
| **PREVIEW** | 200×200 | 50 | 4 | `spheres` | 0.0 | ~14.1% |
| **MEDIUM** | 400×400 | 400 | 8 | `spheres` | 0.0 | ~5.0% |
| **HIGH** | 800×800 | 1000 | 10 | `spheres` | 0.0 | ~3.2% |
| **ULTRA** | 800×800 | 2000 | 12 | `spheres` | 0.0 | ~2.2% |
| **DOF** | 400×400 | 400 | 8 | `spheres` | 20.0 | ~5.0% |
| **BUNNY** | 400×400 | 400 | 8 | `bunny` | 0.0 | ~5.0% |
| **BOXES** | 400×400 | 400 | 8 | `boxes` | 0.0 | ~5.0% |
| **TINY_PERFECT** | 200×200 | 2000 | 16 | `spheres` | 0.0 | ~2.2% |

---

## Efecto de los Parámetros

- **`samples`**: controla el ruido Monte Carlo (cae como `1/√samples`). Para reducir el ruido a la mitad se necesitan 4× más muestras.
- **`max_depth`**: limita rebotes máximos. Para escenas cerradas como la Cornell Box con vidrio, `8–16` ofrece iluminación indirecta y cáusticas ricas.
- **`width` / `height`**: impacto cuadrático en tiempo; el `aspect_ratio` se ajusta automáticamente.
- **`gamma`**: `2.2` para displays sRGB estándar.
- **`aperture` y `focus_distance`**: controlan la profundidad de campo (*bokeh*). Con `aperture=0` la cámara es *pinhole* (todo nítido).
- **`fov`**: `40°` encuadra la Cornell Box completa desde `z=-800`.
- **`max_sample`** *(en `RenderConfig`, por defecto `10.0`)*: recorta cada muestra por componente antes de promediar para suprimir *fireflies* (píxeles aislados muy brillantes). Introduce un **sesgo intencional** leve en cáusticas y reflejos muy energéticos: valores bajos (ej. `3.0`) limpian más rápido a pocos *samples*, mientras que `max_sample=None` desactiva el clamp para un estimador Monte Carlo sin sesgo.

---

## Estructura del Proyecto

```
rtx-cornell_box/
│
├── main.py                     # CLI y sistema de presets
├── bugs.md                     # Registro de auditoría y mejoras aplicadas
│
├── src/
│   ├── vector.py               # Vec3 (floats nativos + __slots__)
│   ├── ray.py                  # Ray: rayos paramétricos
│   ├── utils.py                # Sampling Monte Carlo, load_obj
│   │
│   ├── geometry/               # Primitivas geométricas (heredan de Geometry ABC)
│   │   ├── base.py             # Geometry ABC
│   │   ├── hit_record.py       # HitRecord con front_face y normal orientada
│   │   ├── sphere.py           # Esferas
│   │   ├── quad.py             # Cuadriláteros (paredes, cajas, luces)
│   │   ├── triangle.py         # Triángulos (Möller-Trumbore)
│   │   ├── aabb.py             # Bounding boxes (slabs seguro ante dir==0)
│   │   └── bvh.py              # BVH Tree (eje más largo + centroide)
│   │
│   ├── materials/              # Sistema de materiales (Material ABC)
│   │   ├── base.py             # Material ABC
│   │   ├── diffuse.py          # DiffuseMaterial (Lambertian)
│   │   ├── metal.py            # MetalMaterial (especular + fuzz)
│   │   ├── dielectric.py       # DielectricMaterial (Snell + Fresnel)
│   │   └── emissive.py         # EmissiveMaterial (luces de área)
│   │
│   ├── renderer/               # Motor de rendering
│   │   ├── path_tracer.py      # Path tracer + NEE + Russian Roulette
│   │   └── renderer.py         # Renderer paralelo/secuencial + stratified sampling
│   │
│   ├── camera/
│   │   └── camera.py           # Cámara thin-lens con DOF
│   │
│   ├── scene/
│   │   └── cornell_box.py      # Constructores de escenas (SPHERES, BOXES, BUNNY)
│   │
│   └── config/
│       ├── render_config.py    # RenderConfig (dataclass)
│       └── camera_config.py    # CameraConfig (dataclass)
│
├── models/
│   └── bunny.obj               # Stanford Bunny (2,503 vértices, 4,968 triángulos)
│
├── output/                     # Imágenes renderizadas
├── tests/                      # Suite de tests unitarios e integración (pytest)
├── docs/                       # Documentación técnica y conceptual
├── mypy.ini                    # Configuración de tipado estático
├── requirements.txt
└── requirements-dev.txt
```

---

## Documentación

- [docs/usage.md](docs/usage.md) — Guía de uso y configuración
- [docs/concepts.md](docs/concepts.md) — Conceptos fundamentales de Ray/Path Tracing
- [docs/foundations.md](docs/foundations.md) — Fundamentos matemáticos
- [docs/roadmap.md](docs/roadmap.md) — Roadmap de estudio
- [docs/faq.md](docs/faq.md) — Preguntas frecuentes
- [docs/rtx-cornell_box/](docs/rtx-cornell_box/) — Documentación por módulo

---

## Rendimiento

### Optimizaciones Implementadas

- **`Vec3` nativo con `__slots__`**: evita crear arrays NumPy de 3 elementos en cada operación vectorial.
- **BVH Tree con eje más largo y centroide**: reduce pruebas de intersección de `O(N)` a `O(log N)` (para el Bunny de 4,968 triángulos, ~12–15 tests por rayo).
- **Renderizado paralelo con `Pool(initializer=...)`**: envía el BVH una sola vez por proceso worker en lugar de re-serializarlo por cada fila.
- **Stratified Sampling real**: cuadrícula `√samples × √samples` por píxel para menor varianza a igual `samples`.
- **Russian Roulette**: corta caminos profundos de baja contribución sin introducir sesgo.

---

## Troubleshooting

- **"Modelo cargado: 0 triángulos"**: verifica que `models/bunny.obj` existe y que ejecutas el script desde la raíz del repositorio.
- **Renderizado lento**: prueba `python main.py --preset PREVIEW` o baja `--samples` / `--width`.
- **Imagen ruidosa**: sube `--samples` (el error decrece como `1/√samples`).
- **Imagen oscura**: comprueba que `gamma=2.2` y que `max_depth` sea al menos `6–8`.

---

## Referencias

- **"Physically Based Rendering"** — Pharr, Jakob, Humphreys
- **"Ray Tracing in One Weekend"** — Peter Shirley
- **"The Rendering Equation"** — James Kajiya (1986)
- **Möller-Trumbore (1997)** — Intersección rayo-triángulo
- **Fresnel-Schlick (1994)** — Aproximación de reflectancia

---

## Autor

**Sandoval, Carlos David**
- GitHub: [@dotdeivid](https://github.com/dotdeivid)
