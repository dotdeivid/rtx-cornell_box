# `src/geometry/aabb.py` — Clase `AABB`

Axis-Aligned Bounding Box: caja rectangular alineada con los ejes coordenados. Es la estructura de rechazo rápido que hace posible el BVH. Un test AABB es órdenes de magnitud más barato que un test de intersección exacta con la geometría real.

---

## Clase `AABB`

### Constructor

```python
AABB(min_pt: Vec3, max_pt: Vec3)
```

| Parámetro | Descripción |
|---|---|
| `min_pt` | Esquina con las coordenadas mínimas en X, Y, Z |
| `max_pt` | Esquina con las coordenadas máximas en X, Y, Z |

Se asume `min_pt ≤ max_pt` componente a componente.

### Atributos

| Atributo | Tipo |
|---|---|
| `min` | `Vec3` |
| `max` | `Vec3` |

---

## Métodos

### `hit(ray, t_min, t_max) → bool`

Test de intersección usando el **algoritmo de slabs** (Kay y Kajiya, 1986).

**Idea:** una AABB es la intersección de 3 pares de planos paralelos (uno por eje). Para que el rayo golpee la caja, debe atravesar los 3 pares simultáneamente.

Para cada eje `i ∈ {X, Y, Z}`:
```python
invD = 1.0 / direction_i         # Inversa precalculada

t0 = (min_i - origin_i) * invD  # Entrada al slab
t1 = (max_i - origin_i) * invD  # Salida del slab

if invD < 0:
    t0, t1 = t1, t0              # Intercambiar si dirección negativa

t_min = max(t_min, t0)           # Entrada más tardía
t_max = min(t_max, t1)           # Salida más temprana

if t_max <= t_min:
    return False                 # Early exit: no hay solapamiento
```

Después de los 3 ejes, si `t_max > t_min` → el rayo atraviesa la caja.

**Por qué `1/direction` en lugar de división directa:**
Las tuplas de componentes se construyen fuera del bucle para evitar crear 4 listas temporales por iteración.

**Nota sobre división por cero:**
Si `direction_i = 0`, `invD = ±inf`. Los productos `(min_i - origin_i) * inf` producen `±inf` o `nan`. En IEEE 754, el algoritmo de slabs sigue siendo correcto con infinitos, pero Python emite un `RuntimeWarning` que es esperado y no representa un error.

**Retorna:** `True` si el rayo intersecta la caja en `[t_min, t_max]`, `False` si no.

---

### `union(other: AABB) → AABB`

Retorna la AABB mínima que contiene **completamente** a `self` y a `other`.

```python
new_min = Vec3(min(self.min.x, other.min.x), ...)
new_max = Vec3(max(self.max.x, other.max.x), ...)
return AABB(new_min, new_max)
```

Usado exclusivamente en `BVHNode.create()` para calcular la caja padre que envuelve a los dos hijos:

```python
full_box = left.bounding_box().union(right.bounding_box())
```

**Propiedades:**
- Conmutativa: `a.union(b) == b.union(a)`
- El volumen resultado es siempre ≥ max(vol_a, vol_b)

---

## Rol en el BVH

La AABB es el "portero" del BVH. Antes de probar un rayo contra cualquier geometría real (costosa), se prueba contra la AABB (barata). Si el rayo no golpea la caja, se descarta todo el subárbol sin más trabajo.

```
AABB.hit() → O(1) → 6 divisiones + comparaciones
Sphere.hit() → O(1) → cuadrática, sqrt
Triangle.hit() → O(1) → Möller-Trumbore, 9 productos cruzados
```

Para 69.000 triángulos, el BVH con AABBs permite descartar ~68.980 triángulos con tests baratos antes de llegar a los ~16 triángulos que hay que probar exactamente.
