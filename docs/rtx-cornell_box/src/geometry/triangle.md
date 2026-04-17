# `src/geometry/triangle.py` — Clase `Triangle`

Primitiva geométrica triángulo. Bloque fundamental de los modelos 3D cargados desde archivos OBJ. La Cornell Box con el Stanford Bunny usa ~69.000 instancias de esta clase.

---

## Clase `Triangle`

### Constructor

```python
Triangle(v0: Vec3, v1: Vec3, v2: Vec3, material: Material)
```

| Parámetro | Tipo | Descripción |
|---|---|---|
| `v0` | `Vec3` | Primer vértice |
| `v1` | `Vec3` | Segundo vértice |
| `v2` | `Vec3` | Tercer vértice |
| `material` | `Material` | Material compartido (todos los triángulos de un mismo modelo usan el mismo) |

**Normal precalculada en `__init__`:**
```python
edge1  = v1 - v0
edge2  = v2 - v0
normal = edge1.cross(edge2).normalize()
```

La normal sigue la **regla de la mano derecha**: si los vértices están en sentido antihorario vistos desde el frente, la normal apunta hacia el observador. El orden de los vértices en el OBJ determina qué lado es el "frente".

Las normales OBJ (`vn`) se ignoran y se recalculan por triángulo. Esto significa que el modelo usa **flat shading** (normal constante por triángulo), no smooth shading (normal interpolada por vértice).

---

## Métodos

### `hit(ray, t_min, t_max) → HitRecord | None`

Implementa el **algoritmo de Möller-Trumbore (1997)**, el estándar de la industria para intersección rayo-triángulo.

**Idea:** resolver el sistema lineal que iguala el punto en el rayo con el punto en el triángulo expresado en coordenadas baricéntricas:

```
O + t·D = v0 + u·(v1-v0) + v·(v2-v0)
```

Esto es un sistema 3×3. En lugar de invertir una matriz, se usa la regla de Cramer con productos cruzados, lo que evita divisiones hasta el final:

```python
h = ray.direction.cross(edge2)
a = edge1.dot(h)               # Si |a| < 1e-8 → rayo paralelo al triángulo

f = 1.0 / a
s = ray.origin - v0
u = f * s.dot(h)               # Si u < 0 o u > 1 → fuera del triángulo

q = s.cross(edge1)
v = f * ray.direction.dot(q)   # Si v < 0 o u+v > 1 → fuera del triángulo

t = f * edge2.dot(q)           # Distancia a lo largo del rayo
```

**Condiciones de validez:**
1. `|a| > 1e-8` — rayo no paralelo
2. `0 ≤ u ≤ 1` — dentro del triángulo (primera coordenada baricéntrica)
3. `v ≥ 0` y `u + v ≤ 1` — dentro del triángulo (segunda coordenada baricéntrica)
4. `t_min < t < t_max` — en el rango válido del rayo

Si todas se cumplen: **retorna** `HitRecord(t, point, normal, material, obj_ref=self)`.

**Ventaja sobre otros métodos:** no precalcula el plano del triángulo, trabaja directamente con los vértices. Complejidad O(1) con número fijo de operaciones.

---

### `bounding_box() → AABB`

AABB a partir de los valores extremos de los tres vértices, con margen `0.001`:

```python
min_pt = Vec3(min(v0.x, v1.x, v2.x) - 0.001, ...)
max_pt = Vec3(max(v0.x, v1.x, v2.x) + 0.001, ...)
```

El margen previene cajas de espesor cero para triángulos perfectamente alineados con un plano coordenado (ej. un triángulo horizontal en `y=0` tendría `min.y = max.y` sin el margen).

---

## Coordenadas baricéntricas

Los parámetros `u` y `v` del algoritmo son coordenadas baricéntricas del punto de impacto:

```
P = v0 + u·(v1-v0) + v·(v2-v0)
  = (1-u-v)·v0 + u·v1 + v·v2
```

- `u=0, v=0` → punto en `v0`
- `u=1, v=0` → punto en `v1`
- `u=0, v=1` → punto en `v2`
- `u+v=1` → borde entre `v1` y `v2`

La condición `u ≥ 0, v ≥ 0, u+v ≤ 1` garantiza que el punto está dentro del triángulo.
