# Ray Tracer - Cornell Box

Un **path tracer fotorrealista** implementado en Python que simula la física de la luz para renderizar la icónica Cornell Box con materiales avanzados (vidrio, metal, difuso) y modelos 3D complejos.

![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)
![License](https://img.shields.io/badge/License-MIT-green.svg)
![Status](https://img.shields.io/badge/Status-Active-success.svg)

---

## Tabla de Contenidos

- [Características](#características)
- [¿Qué hace este proyecto?](#qué-hace-este-proyecto)
- [¿Cómo funciona?](#cómo-funciona)
- [Instalación](#instalación)
- [Uso](#uso)
- [Configuraciones de Rendering](#configuraciones-de-rendering)
- [Efecto de los Parámetros](#efecto-de-los-parámetros)
- [Estructura del Proyecto](#estructura-del-proyecto)
- [Clases Principales](#clases-principales)
- [Documentación](#documentación)
- [Rendimiento](#rendimiento)
- [Referencias](#referencias)

---

## Características

### Técnicas de Rendering
- **Path Tracing**: Integración Monte Carlo de la ecuación de rendering
- **Next Event Estimation (NEE)**: Muestreo directo de luces para reducir ruido 10-100×
- **Stratified Sampling**: Antialiasing con cuadrícula sub-píxel
- **BVH (Bounding Volume Hierarchy)**: Aceleración O(log N) de intersecciones

### Materiales Físicamente Basados
- **Difusos (Lambertian)**: Superficies mates con reflexión coseno-ponderada
- **Metales**: Reflexión especular con rugosidad ajustable (`fuzz`)
- **Dieléctricos**: Vidrio/agua con refracción (Ley de Snell + Fresnel-Schlick)
- **Emisores**: Luces de área para sombras suaves con penumbra

### Cámara Avanzada
- **Profundidad de Campo (DOF)**: Desenfoque bokeh físicamente correcto
- **Field of View ajustable**: Teleobjetivo a gran angular
- **Distancia focal configurable**: Control artístico del plano de enfoque

### Optimizaciones
- **Renderizado paralelo**: Multi-core con `multiprocessing` (speedup ~lineal)
- **BVH Tree**: O(log N) intersecciones vs O(N) fuerza bruta
- **Gamma Correction**: Corrección sRGB 2.2 para displays modernos

---

## ¿Qué hace este proyecto?

Este ray tracer simula cómo la luz real rebota en una escena 3D para generar imágenes fotorrealistas. A diferencia de rasterización (OpenGL/DirectX), trazamos rayos desde la cámara hacia cada píxel, seguimos su trayectoria mientras rebotan en objetos, refractan a través de vidrio y finalmente llegan a fuentes de luz.

### Escenas Disponibles

**Cornell Box con Esferas** (`SceneMode.SPHERES`)
- Escena clásica de validación con esfera de vidrio y esfera metálica
- Efectos: color bleeding, caustics, reflexiones especulares, sombras suaves

**Cornell Box con Bunny** (`SceneMode.BUNNY`)
- Stanford Bunny de vidrio (~69,000 triángulos)
- Demuestra BVH rendering de modelos complejos

---

## ¿Cómo funciona?

### Pipeline de Rendering

```
1. Configuración
   ├─ RenderConfig: resolución, samples, profundidad, salida
   └─ CameraConfig: posición, FOV, apertura DOF

2. Por cada píxel
   └─ N muestras (stratified sampling)
       └─ Por cada muestra:
           ├─ Rayo desde cámara (con offset DOF en disco de apertura)
           └─ color_ray() recursivo (hasta max_depth)
               ├─ Intersección con escena (BVH: O(log N))
               ├─ Emisor  → retornar emisión directamente
               ├─ Metal   → reflexión especular + fuzz
               ├─ Vidrio  → refracción/reflexión según Fresnel
               └─ Difuso  → NEE (luz directa) + rebote aleatorio (luz indirecta)

3. Post-proceso
   ├─ Promedio de muestras
   ├─ Gamma correction (rgb^(1/2.2))
   └─ Guardar PNG
```

### Física Implementada

- **Ley de Snell**: `η₁ sin(θ₁) = η₂ sin(θ₂)` — refracción en dieléctricos
- **Fresnel-Schlick**: `R(θ) = R₀ + (1-R₀)(1-cosθ)⁵` — reflexión probabilística
- **BRDF Lambertiano**: `ρ/π` — materiales difusos físicamente correctos
- **Ley de Lambert**: intensidad proporcional a `cos(θ)` del ángulo de incidencia

---

## Instalación

### Requisitos

- Python 3.8 o superior
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

# 4. Renderizar
python main.py
```

### Dependencias

```
numpy>=1.21.0    # Álgebra vectorial y arrays
Pillow>=9.0.0    # Guardar imágenes PNG
```

---

## Uso

### Renderizado Básico

```bash
python main.py
# Imagen guardada en: output/result_render.png
```

### Cambiar Escena

Edita `main.py`:

```python
from src.scene import SceneMode, create_cornell_box_scene

# Cornell Box con esferas (vidrio + metal)
world, lights = create_cornell_box_scene(SceneMode.SPHERES)

# Cornell Box con Stanford Bunny de vidrio
world, lights = create_cornell_box_scene(SceneMode.BUNNY)
```

### Ajustar Calidad

```python
from src.config import RenderConfig
from pathlib import Path

render_config = RenderConfig(
    width=400,          # Ancho en píxeles
    height=400,         # Alto en píxeles
    samples=400,        # Muestras Monte Carlo por píxel
    max_depth=8,        # Máximo de rebotes de luz
    use_parallel=True,  # Renderizado multi-core
    gamma=2.2,          # Corrección gamma sRGB
    output_path=Path("output/result_render.png"),
)
```

### Configurar Cámara

```python
from src.config import CameraConfig

camera_config = CameraConfig(
    origin=Vec3(278, 278, -800),  # Posición de la cámara
    lookat=Vec3(278, 278, 278),   # Punto al que apunta
    vup=Vec3(0, 1, 0),            # Vector "arriba" del mundo
    fov=40.0,                      # Field of view en grados
    aperture=20.0,                 # Apertura de lente (DOF)
    focus_distance=None,           # None = auto (foco en lookat)
)
```

---

## Configuraciones de Rendering

Estas tres configuraciones cubren los casos de uso principales. Cópialas directamente en `main.py`.

### Baja Calidad — Vista Previa Rápida

Para verificar composición y cambios de escena sin esperar. El ruido es visible pero la estructura general es clara.

```python
render_config = RenderConfig(
    width=200,
    height=200,
    samples=50,
    max_depth=4,
    use_parallel=True,
    gamma=2.2,
    output_path=Path("output/preview.png"),
)

camera_config = CameraConfig(
    fov=40.0,
    aperture=0.0,   # Sin DOF: todo enfocado, más limpio a pocos samples
)
```

| Atributo | Valor | Impacto |
|----------|-------|---------|
| Resolución | 200×200 | 4× menos píxeles que 400×400 |
| Samples | 50 | Ruido visible (~20% de error MC) |
| Depth | 4 | Solo 4 rebotes — luz indirecta básica |
| Tiempo estimado | ~5-15 seg | Depende de CPU |

---

### Buena Calidad — Uso General

Balance entre calidad y tiempo. Adecuada para evaluar materiales, iluminación e iteraciones de diseño.

```python
render_config = RenderConfig(
    width=400,
    height=400,
    samples=400,
    max_depth=8,
    use_parallel=True,
    gamma=2.2,
    output_path=Path("output/render.png"),
)

camera_config = CameraConfig(
    fov=40.0,
    aperture=20.0,   # DOF moderado
    focus_distance=None,
)
```

| Atributo | Valor | Impacto |
|----------|-------|---------|
| Resolución | 400×400 | Detalle suficiente para evaluar |
| Samples | 400 | Ruido bajo (~5% de error MC) |
| Depth | 8 | Color bleeding visible, caustics básicas |
| Tiempo estimado | ~1.5-3 min | En CPU de 8 núcleos |

---

### Ultra Calidad — Producción

Para resultado final. Ruido casi imperceptible, color bleeding rico, sombras suaves definidas.

```python
render_config = RenderConfig(
    width=800,
    height=800,
    samples=2000,
    max_depth=12,
    use_parallel=True,
    gamma=2.2,
    output_path=Path("output/ultra.png"),
)

camera_config = CameraConfig(
    fov=40.0,
    aperture=20.0,
    focus_distance=None,
)
```

| Atributo | Valor | Impacto |
|----------|-------|---------|
| Resolución | 800×800 | 4× más detalle que 400×400 |
| Samples | 2000 | Ruido muy bajo (~2.2% de error MC) |
| Depth | 12 | Iluminación indirecta convergida, caustics completas |
| Tiempo estimado | ~30-60 min | En CPU de 8 núcleos |

---

### Tabla Comparativa

| Preset | Resolución | Samples | Depth | Tiempo* | Error MC |
|--------|-----------|---------|-------|---------|----------|
| **Baja** | 200×200 | 50 | 4 | ~10 seg | ~14% |
| **Buena** | 400×400 | 400 | 8 | ~2 min | ~5% |
| **Ultra** | 800×800 | 2000 | 12 | ~45 min | ~2.2% |

*Tiempos aproximados en CPU de 8 núcleos. El error MC sigue la fórmula `1/√samples`.

---

## Efecto de los Parámetros

Entender qué cambia cada parámetro permite combinarlos para predecir el resultado antes de renderizar.

### `samples` — Muestras por píxel

Controla cuántos rayos se promedian por píxel. Es el parámetro que más afecta la **limpieza de la imagen**.

```
samples=50   → Ruido granulado visible, manchas en sombras
samples=200  → Ruido moderado, estructura clara
samples=400  → Ruido bajo, aceptable para evaluación
samples=1000 → Ruido muy bajo, zonas oscuras limpias
samples=4000 → Casi sin ruido, resultado de producción
```

**Relación**: el error cae como `1/√samples`. Para reducir el ruido a la mitad, se necesitan **4× más samples**. Es el parámetro más costoso en tiempo.

* **Cuándo bajar**: en iteraciones de diseño donde se evalúa composición, no calidad final.
* **Cuándo subir**: en el render final, especialmente si hay zonas oscuras (interior de vidrio, sombras profundas).

---

### `max_depth` — Rebotes máximos de luz

Limita cuántas veces puede rebotar un rayo antes de retornar negro. Afecta la **riqueza de la iluminación indirecta**.

```
max_depth=2  → Solo luz directa + un rebote. Escena oscura, sin color bleeding.
max_depth=4  → Dos rebotes indirectos. Color bleeding incipiente.
max_depth=8  → Cornell Box converge bien aquí. Color bleeding visible, caustics básicas.
max_depth=12 → Caustics completas, tonos medios más ricos.
max_depth=16 → Beneficio marginal (<1% de diferencia visual).
```

**Regla práctica**:
- Escenas abiertas (cielo visible): `depth=4` es suficiente
- Cornell Box (espacio cerrado): `depth=8` mínimo, `depth=12` para producción
- Escenas con vidrio grueso (muchos rebotes internos): `depth=12+`

**Cuándo importa más**: en escenas cerradas, con vidrio, y cuando las zonas oscuras se ven demasiado negras. Si la escena ya está bien iluminada a depth=6, subir a 12 cambiará poco.

---

### `width` / `height` — Resolución

Determina el **tamaño y detalle** de la imagen de salida. Afecta el tiempo de forma cuadrática: duplicar la resolución cuadruplica los píxeles a renderizar.

```
200×200  →    40,000 píxeles (pruebas rápidas)
400×400  →   160,000 píxeles (evaluación)
800×800  →   640,000 píxeles (producción)
1920×1080 → 2,073,600 píxeles (muy costoso)
```

El aspecto ratio afecta la cámara: `aspect_ratio = width / height` se pasa automáticamente al viewport.

---

### `gamma` — Corrección gamma

Ajusta cómo se convierten los colores lineales del renderer al espacio de color del display.

```
gamma=1.0  → Sin corrección. Imagen visualmente oscura (incorrecta en monitores sRGB).
gamma=2.2  → Estándar sRGB. Correcto para la mayoría de monitores modernos.
gamma=1.8  → Estándar macOS antiguo (menos común hoy).
```

**No cambiar** salvo que se sepa con certeza que el display usa un espacio de color diferente. `2.2` es el correcto para prácticamente cualquier monitor moderno.

---

### `aperture` — Apertura de lente (DOF)

Simula el diafragma de una cámara real. A mayor apertura, mayor profundidad de campo y círculos de bokeh más grandes.

```
aperture=0    → Cámara pinhole. Todo enfocado. Sin bokeh.
aperture=5    → DOF muy sutil. Apenas perceptible.
aperture=20   → DOF moderado (default Cornell Box).
aperture=60   → Bokeh pronunciado. Solo el plano focal es nítido.
```

**Interacción con `focus_distance`**: el plano focal (zona nítida) está en la distancia `focus_distance` desde la cámara. Con `focus_distance=None` se enfoca automáticamente en el punto `lookat`.

Si `aperture=0`, `focus_distance` no tiene efecto visual.

---

### `fov` — Field of View (ángulo de visión)

Controla cuánto "ve" la cámara verticalmente. Afecta la perspectiva y distorsión de la imagen.

```
fov=20°  → Teleobjetivo. Objetos grandes, poca distorsión. Perspectiva comprimida.
fov=40°  → Normal para Cornell Box. Balance entre tamaño y distorsión.
fov=60°  → Gran angular suave. Más contexto visible.
fov=90°  → Gran angular pronunciado. Distorsión en bordes visible.
```

**Regla**: para la Cornell Box, `fov=40°` está calibrado para que la caja llene bien el frame desde la posición de cámara por defecto (`z=-800`). Cambiar el FOV sin mover la cámara cambia el encuadre.

---

### Combinando Parámetros — Predicción del Resultado

| Objetivo | Configuración |
|----------|--------------|
| Prueba rápida de composición | `samples=50`, `max_depth=4`, `aperture=0` |
| Evaluar materiales y colores | `samples=200`, `max_depth=8`, `aperture=0` |
| Evaluar profundidad de campo | `samples=400`, `max_depth=8`, `aperture=20-60` |
| Imagen final limpia | `samples=2000`, `max_depth=12`, todo habilitado |
| Escena con mucho vidrio | `max_depth=12+`, `samples` alto (caustics son ruidosas) |
| Escena oscura / sombras profundas | `samples` alto (zonas oscuras convergen lento) |
| Solo luz directa (prueba NEE) | `max_depth=1`, `samples=50` |

**Ejemplo de razonamiento**: si la imagen se ve bien iluminada pero granulada → subir `samples`. Si las sombras se ven demasiado negras → subir `max_depth`. Si el bokeh es muy agresivo y distrae → bajar `aperture`. Si la escena se ve aplanada → subir `max_depth` (falta iluminación indirecta).

---

## Estructura del Proyecto

```
rtx-cornell_box/
│
├── main.py                     # Punto de entrada principal
│
├── src/
│   ├── vector.py               # Vec3: álgebra vectorial 3D
│   ├── ray.py                  # Ray: rayos paramétricos
│   │
│   ├── geometry/               # Primitivas geométricas
│   │   ├── base.py             # Geometry ABC
│   │   ├── hit_record.py       # HitRecord (dataclass)
│   │   ├── sphere.py           # Esferas
│   │   ├── quad.py             # Cuadriláteros (paredes, luces)
│   │   ├── triangle.py         # Triángulos (modelos .obj)
│   │   ├── aabb.py             # Bounding boxes
│   │   └── bvh.py              # BVH Tree
│   │
│   ├── materials/              # Sistema de materiales (ABC)
│   │   ├── base.py             # Material ABC
│   │   ├── diffuse.py          # DiffuseMaterial (Lambertian)
│   │   ├── metal.py            # MetalMaterial (especular)
│   │   ├── dielectric.py       # DielectricMaterial (refracción)
│   │   └── emissive.py         # EmissiveMaterial (luces de área)
│   │
│   ├── renderer/               # Motor de rendering
│   │   ├── path_tracer.py      # Path tracer + NEE
│   │   └── renderer.py         # Renderer principal
│   │
│   ├── camera/
│   │   └── camera.py           # Cámara thin-lens con DOF
│   │
│   ├── scene/
│   │   └── cornell_box.py      # Constructores de escena
│   │
│   ├── config/
│   │   ├── render_config.py    # RenderConfig (dataclass)
│   │   └── camera_config.py    # CameraConfig (dataclass)
│   │
│   └── utils.py                # Sampling Monte Carlo, load_obj
│
├── models/
│   └── bunny.obj               # Stanford Bunny (~69K triángulos)
│
├── output/                     # Imágenes renderizadas
├── tests/                      # Tests unitarios
├── docs_nuevo/                 # Documentación completa
├── requirements.txt
└── Dockerfile
```

---

## Rendimiento

### Optimizaciones Implementadas

**BVH Tree**
- Sin BVH: O(N) — 69,000 tests por rayo (Stanford Bunny)
- Con BVH: O(log N) — ~17 tests por rayo
- Speedup: ~4,000×

**Renderizado Paralelo**
- `multiprocessing.Pool` — una fila por worker, todos los núcleos disponibles
- Speedup ~lineal: 8 cores → ~7× más rápido

**Stratified Sampling**
- Divide cada píxel en cuadrícula √samples × √samples
- 30-50% menos ruido que muestreo aleatorio puro para el mismo N

### Benchmarks (referencia: 8 núcleos @ 3.5 GHz)

| Escena | Resolución | Samples | Depth | Tiempo |
|--------|-----------|---------|-------|--------|
| Spheres | 400×400 | 400 | 8 | ~1.5 min |
| Bunny | 400×400 | 400 | 8 | ~2.3 min |
| Spheres (ultra) | 800×800 | 2000 | 12 | ~45 min |

---

## Troubleshooting

**"Modelo cargado: 0 triángulos"**
Verifica que `models/bunny.obj` existe. El modelo Stanford Bunny se puede descargar de http://www.graphics.stanford.edu/data/3Dscanrep/

**Renderizado muy lento**
Baja `samples` (50-100) y `max_depth` (4) para pruebas. Asegúrate de que `use_parallel=True`.

**Imagen muy ruidosa**
Sube `samples`. El ruido es inherente al Monte Carlo — se reduce como `1/√samples`. Ver sección [Configuraciones de Rendering](#configuraciones-de-rendering).

**Imagen oscura**
Comprueba que `gamma=2.2` (sin corrección gamma la imagen aparece oscura). Si el problema persiste, sube `max_depth` — puede que los rebotes estén siendo cortados antes de alcanzar la luz.

**Bokeh demasiado agresivo**
Baja `aperture` o ponlo a `0` para deshabilitar el DOF completamente.

---

## Referencias

### Papers y Libros
- **"Physically Based Rendering"** — Pharr, Jakob, Humphreys (PBR Bible)
- **"Ray Tracing in One Weekend"** — Peter Shirley (tutorial base)
- **"The Rendering Equation"** — James Kajiya (1986)

### Técnicas Implementadas
- Path Tracing (Kajiya 1986)
- Next Event Estimation — muestreo directo de fuentes de luz
- BVH con partición por eje aleatorio
- Ley de Snell vectorial
- Aproximación de Fresnel-Schlick (Christophe Schlick, 1994)
- Möller-Trumbore (1997) — intersección rayo-triángulo
- Cornell Box original (1984) — Program of Computer Graphics, Cornell University

---

## Autor

**Sandoval, Carlos David**
- GitHub: [@dotdeivid](https://github.com/dotdeivid)
