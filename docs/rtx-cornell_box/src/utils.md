# `src/utils.py` — Utilidades

Funciones auxiliares de muestreo aleatorio y carga de modelos 3D. No contiene clases.

---

## `random_in_unit_sphere() → Vec3`

Genera un punto aleatorio uniformemente distribuido **dentro** de la esfera unitaria (radio < 1).

### Algoritmo

Método de rechazo (rejection sampling):
1. Generar punto aleatorio en el cubo `[-1, 1]³`
2. Si su longitud es `< 1.0` → está dentro de la esfera, retornar
3. Si no → descartar y repetir

### Por qué este método

Usar coordenadas esféricas directas (`r·sin(φ)·cos(θ), ...`) con `r = random()` produce distribuciones no uniformes (más puntos cerca del centro). El método de rechazo garantiza uniformidad.

- Eficiencia: `π/6 ≈ 52.4%` de los puntos generados se aceptan
- Iteraciones esperadas: `6/π ≈ 1.91`

### Usos en el proyecto

- `DiffuseMaterial.scatter()`: dirección de rebote difuso
- `MetalMaterial.scatter()`: perturbación de fuzziness
- `random_in_unit_disk()`: base del algoritmo

---

## `generar_direccion_aleatoria(normal: Vec3) → Vec3`

Genera un vector **unitario** aleatorio en el hemisferio orientado por `normal`.

### Algoritmo

1. Llamar a `random_in_unit_sphere().normalize()` → punto en la superficie de la esfera
2. Test de hemisferio: si `random_dir.dot(normal) > 0` → mismo lado que la normal
3. Si no → invertir (`* -1`)

### Por qué se necesita

Un vector aleatorio en la esfera completa tiene 50% de probabilidad de apuntar "hacia adentro" de la superficie (hemisferio equivocado). Esta función garantiza que el rayo rebotado salga hacia el exterior.

### Usos

Scattering difuso: la dirección de rebote de un material Lambertiano debe estar siempre en el hemisferio de la normal.

---

## `random_in_unit_disk() → Vec3`

Genera un punto aleatorio uniformemente distribuido dentro del disco unitario (círculo 2D en el plano XY, `z=0`).

### Algoritmo

Método de rechazo en 2D:
1. Generar punto en cuadrado `[-1,1]² × {0}`
2. Si su longitud es `< 1.0` → retornar
3. Si no → repetir

- Eficiencia: `π/4 ≈ 78.5%`
- Iteraciones esperadas: `4/π ≈ 1.27`

### Uso

Exclusivamente en `Camera.get_ray()` para simular profundidad de campo (DOF). El punto resultante escala por `lens_radius` y se usa como offset del origen del rayo en el plano de la lente.

---

## `load_obj(filename, scale, offset, material) → list[Triangle]`

Carga un archivo Wavefront OBJ y retorna una lista de `Triangle` listos para ser añadidos a la escena.

### Parámetros

| Parámetro | Tipo | Default | Descripción |
|---|---|---|---|
| `filename` | `str` | — | Ruta al archivo `.obj` |
| `scale` | `float` | `1.0` | Factor de escala uniforme |
| `offset` | `Vec3` | `Vec3(0,0,0)` | Traslación aplicada a todos los vértices |
| `material` | `Material` | `DiffuseMaterial(gris)` | Material compartido por todos los triángulos |

### Procesamiento línea a línea

**Líneas `v x y z` (vértices):**
```python
v = Vec3(x, y, z)
vertices.append(v * scale + offset)
```
La escala y traslación se aplican en la carga, no en runtime.

**Líneas `f ...` (caras):**
Soporta tres formatos OBJ:
- `f v1 v2 v3` — simple
- `f v1/vt1 v2/vt2 v3/vt3` — con coordenadas UV (se ignoran)
- `f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3` — con normales OBJ (se ignoran, se recalculan)

Los índices OBJ empiezan en 1; se convierten a base-0:
```python
idx - 1 if idx > 0 else len(vertices) + idx   # Soporta índices negativos
```

**Polígonos n-lados (triangulación fan):**
```
Cara con vértices [v0, v1, v2, v3, v4]:
  → Triángulo [v0, v1, v2]
  → Triángulo [v0, v2, v3]
  → Triángulo [v0, v3, v4]
```

**Líneas ignoradas:** `vt` (texturas), `vn` (normales OBJ), `#` (comentarios), `g`, `o`, `mtl`.

### Retorno

Lista de `Triangle`. Si hay error (archivo no encontrado, formato inválido), imprime el error y retorna lista vacía. No lanza excepciones.

### Uso típico

```python
triangles = load_obj("models/bunny.obj", scale=1400.0, offset=Vec3(278, 0, 278), material=glass)
objects.extend(triangles)
world = BVHNode.create(objects)   # Siempre envolver en BVH para modelos grandes
```
