# `src/config/render_config.py` — Dataclass `RenderConfig`

Configuración centralizada de todos los parámetros que controlan la calidad, rendimiento y salida del proceso de renderizado.

---

## Clase `RenderConfig` (dataclass)

### Campos

| Campo | Tipo | Default | Descripción |
|---|---|---|---|
| `width` | `int` | `400` | Ancho de la imagen en píxeles |
| `height` | `int` | `400` | Alto de la imagen en píxeles |
| `samples` | `int` | `400` | Muestras Monte Carlo por píxel |
| `max_depth` | `int` | `8` | Máximo de rebotes de luz por rayo |
| `use_parallel` | `bool` | `True` | Usar rendering multi-core |
| `output_path` | `Path` | `Path("output/result_render.png")` | Ruta de salida |
| `gamma` | `float` | `2.2` | Factor de corrección gamma |

---

## Propiedad

### `aspect_ratio → float`

```python
@property
def aspect_ratio(self) -> float:
    return self.width / self.height
```

Relación ancho/alto. Se pasa al constructor de `Camera` para que el viewport tenga las proporciones correctas.

---

## Método de validación

### `__post_init__()`

Ejecutado automáticamente por `@dataclass` después del `__init__`. Valida que los parámetros sean físicamente coherentes:

```python
if self.width <= 0 or self.height <= 0:
    raise ValueError("Width y height deben ser positivos")

if self.samples <= 0:
    raise ValueError("Samples debe ser positivo")

if self.max_depth < 0:
    raise ValueError("Max depth no puede ser negativo")

if self.gamma <= 0:
    raise ValueError("Gamma debe ser positivo")

# Convertir string a Path si es necesario
if isinstance(self.output_path, str):
    self.output_path = Path(self.output_path)
```

La conversión de `str` a `Path` permite pasar la ruta como string sin necesidad de importar `Path` en el llamador.

---

## Impacto de cada parámetro

### `samples`
Relación directa con calidad y tiempo. El error del estimador Monte Carlo cae como `1/√samples`.

```
samples=50   → ~10% de error, útil para preview (~5s)
samples=400  → ~5% de error, calidad estándar (~2min)
samples=1600 → ~2.5% de error, alta calidad (~8min)
```

### `max_depth`
Limita la recursión de `color_ray`. Profundidades bajas cortan caminos de luz legítimos (especialmente en vidrio, donde los fotones rebotan múltiples veces internamente).

```
max_depth=2  → Solo luz directa + un rebote indirecto
max_depth=8  → Buen balance para Cornell Box (default)
max_depth=16 → Necesario para caustics complejas
```

Si `depth ≤ 0`, `color_ray` retorna `Vec3(0,0,0)`, terminando la recursión.

### `gamma`
`2.2` es el valor estándar sRGB. La corrección se aplica como `color^(1/gamma)` en el renderer antes de convertir a uint8. Sin esta corrección, la imagen aparecería más oscura en monitores modernos.

### `use_parallel`
`True` usa `multiprocessing.Pool` con todos los núcleos disponibles. La aceleración es casi lineal con el número de núcleos (el trabajo por fila es independiente).

`False` es útil para depuración porque el output de `print` en `_render_sequential` aparece ordenado y sin mezclas entre procesos.
