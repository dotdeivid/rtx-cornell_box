# `src/renderer/renderer.py` — Clase `Renderer`

Orquestador del proceso de rendering. Gestiona la distribución del trabajo entre núcleos de CPU, acumula los samples por píxel, aplica correcciones de imagen y guarda el resultado.

---

## Clase `Renderer`

### Constructor

```python
Renderer(config: RenderConfig)
```

Guarda la configuración para usarla en todos los métodos. No realiza ningún cálculo en la construcción.

---

## Métodos

### `render(world, lights, camera)`

Punto de entrada principal. Coordina el proceso completo de renderizado.

**Parámetros:**

| Parámetro | Tipo | Descripción |
|---|---|---|
| `world` | `BVHNode` | Árbol BVH con toda la geometría de la escena |
| `lights` | `List` | Lista de objetos con `EmissiveMaterial` para NEE |
| `camera` | `Camera` | Cámara que genera los rayos |

**Flujo:**
```
1. Imprimir info de configuración
2. Según config.use_parallel:
   → True:  _render_parallel()
   → False: _render_sequential()
3. _save_image(image_data)
```

---

### `_render_parallel(world, lights, camera) → np.ndarray`

Rendering multi-core usando `multiprocessing.Pool`.

```python
with multiprocessing.Pool() as pool:
    render_func = partial(self._render_row, world=world, lights=lights, camera=camera)
    rows = pool.map(render_func, range(self.config.height))
```

- `Pool()` sin argumentos usa todos los núcleos disponibles.
- `partial` congela los argumentos `world`, `lights`, `camera` para que `pool.map` solo necesite pasar el índice de fila `y`.
- `pool.map` distribuye las filas entre los procesos y recoge los resultados en orden.

**Por qué `multiprocessing` y no `threading`:**
El GIL (Global Interpreter Lock) de Python impide que los threads ejecuten código Python en paralelo real. `multiprocessing` crea procesos separados con memoria propia, evitando el GIL. El costo es que los datos (`world`, `lights`, `camera`) se serializan (pickle) y se copian a cada proceso.

**Retorna:** `np.ndarray` de forma `(height, width, 3)`, dtype `uint8`.

---

### `_render_sequential(world, lights, camera) → np.ndarray`

Rendering single-core. Renderiza fila a fila con progreso impreso cada 50 filas.

```python
for y in range(self.config.height):
    row = self._render_row(y, world, lights, camera)
    rows.append(row)
    if y % 50 == 0:
        print(f"Progreso: {y}/{self.config.height}")
```

Útil para depuración: los prints aparecen en orden y sin interferencia entre procesos.

**Retorna:** `np.ndarray` de forma `(height, width, 3)`, dtype `uint8`.

---

### `_render_row(y, world, lights, camera) → np.ndarray`

Renderiza una fila completa de píxeles. Es la unidad de trabajo distribuida entre los procesos.

**Para cada píxel `x` en la fila `y`:**

**1. Muestreo estratificado (stratified sampling):**
```python
for _ in range(self.config.samples):
    u = (x + random.random()) / self.config.width
    v = (y + random.random()) / self.config.height
    ray = camera.get_ray(u, v)
    pixel_color += color_ray(ray, world, lights, max_depth, max_depth)
```

El offset `random.random()` dentro del píxel es el muestreo estratificado: en lugar de siempre muestrear el centro exacto del píxel, se muestrea un punto aleatorio dentro del área del píxel. Reduce el aliasing sin costo adicional.

**2. Promedio de samples:**
```python
pixel_color = pixel_color / self.config.samples
```

**3. Gamma correction:**
```python
pixel_color = Vec3(
    pixel_color.x ** (1.0 / self.config.gamma),
    pixel_color.y ** (1.0 / self.config.gamma),
    pixel_color.z ** (1.0 / self.config.gamma),
)
```

Los monitores sRGB aplican una curva gamma de 2.2 a la señal. Para compensar, se aplica la gamma inversa (`1/2.2 ≈ 0.4545`) antes de guardar. Sin esto, la imagen aparecería demasiado oscura.

**4. Clamp y conversión a uint8:**
```python
r = int(max(0, min(1, pixel_color.x)) * 255.99)
g = int(max(0, min(1, pixel_color.y)) * 255.99)
b = int(max(0, min(1, pixel_color.z)) * 255.99)
```

El clamp previene valores fuera de `[0,1]` (posibles con emisiones muy intensas). El `* 255.99` en lugar de `* 256` evita que el valor 1.0 exacto produzca 256 (fuera del rango uint8).

**Retorna:** `np.ndarray` de forma `(width, 3)`, dtype `uint8`.

---

### `_save_image(image_data: np.ndarray)`

Guarda la imagen renderizada en disco.

```python
image_data = np.flipud(image_data)   # Flip vertical
img = Image.fromarray(image_data, "RGB")
self.config.output_path.parent.mkdir(parents=True, exist_ok=True)
img.save(self.config.output_path)
```

**Flip vertical:** el renderer genera las filas de abajo hacia arriba (coordenada `v=0` en el fondo), pero el formato PNG almacena de arriba hacia abajo. `np.flipud` corrige esta inversión.

`mkdir(parents=True, exist_ok=True)` crea el directorio `output/` si no existe, sin error si ya existe.
