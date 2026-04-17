# `src/geometry/quad.py` — Clase `Quad`

Cuadrilátero plano definido por un punto origen y dos vectores de lado. Usado para las paredes, el techo, el piso y la fuente de luz de área de la Cornell Box.

---

## Clase `Quad`

### Constructor

```python
Quad(Q: Vec3, u: Vec3, v: Vec3, material: Material)
```

| Parámetro | Tipo | Descripción |
|---|---|---|
| `Q` | `Vec3` | Esquina origen del cuadrilátero |
| `u` | `Vec3` | Vector del primer lado |
| `v` | `Vec3` | Vector del segundo lado (no requiere ser perpendicular a u) |
| `material` | `Material` | Material del quad |

Los cuatro vértices son: `Q`, `Q+u`, `Q+v`, `Q+u+v`.

**Valores precalculados en `__init__`:**
```python
n      = u.cross(v)
normal = n.normalize()      # Normal del plano
D      = normal.dot(Q)      # Constante de la ecuación del plano: n·P = D
w      = n / n.dot(n)       # Vector auxiliar para coordenadas baricéntricas
```

`w` se precalcula para evitar recalcularlo en cada test de intersección.

---

## Propiedades

### `center → Vec3`

Centro geométrico del quad:
```python
return Q + (u * 0.5) + (v * 0.5)
```
Corresponde a los parámetros `s=0.5, t=0.5` en la parametrización.

---

## Métodos

### `hit(ray, t_min, t_max) → HitRecord | None`

Intersección en dos fases:

**Fase 1 — Rayo-Plano:**
```
denom = normal · ray.direction
```
Si `|denom| < 1e-8`, el rayo es paralelo al plano → retorna `None`.

```
t = (D - normal · ray.origin) / denom
```
Si `t` está fuera de `[t_min, t_max]` → retorna `None`.

**Fase 2 — Test de contención en el quad:**

Se proyecta el punto de impacto a coordenadas locales del quad:
```python
planar_vec = intersection - Q
alpha = w · (planar_vec × v)
beta  = w · (u × planar_vec)
```

Si `0 ≤ alpha ≤ 1` y `0 ≤ beta ≤ 1` → el punto está dentro del quad.

**Retorna:** `HitRecord(t, intersection, normal, material, obj_ref=self)` o `None`.

---

### `sample_solid_angle(hit_point: Vec3) → tuple[Vec3, float]`

Importance sampling de la superficie del quad para NEE (Next Event Estimation).

**Paso 1 — Punto aleatorio en el quad:**
```python
random_point = Q + (u * random()) + (v * random())
```

**Paso 2 — Dirección y distancia:**
```python
direction_to_light = random_point - hit_point
distance = |direction_to_light|
direction = direction_to_light / distance
```

**Paso 3 — Ángulo sólido subtendido:**
```
area       = |u × v|               (área del quad)
cos_light  = |normal · direction|  (abs para emitir por ambas caras)
solid_angle = area · cos_light / distance²
```

La fórmula convierte área en ángulo sólido: a mayor distancia o menor ángulo entre la normal de la luz y la dirección, menor ángulo sólido, menor contribución.

**Retorna:** `(direction: Vec3, solid_angle: float)`

---

### `bounding_box() → AABB`

Calcula la AABB a partir de las 4 esquinas del quad con un margen de `0.0001` en cada dirección. El margen es necesario porque un quad perfectamente alineado con un plano coordenado tendría espesor cero en un eje, lo que causaría problemas numéricos en el test AABB.
