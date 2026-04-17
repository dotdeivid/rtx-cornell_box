# `src/geometry/bvh.py` — Clase `BVHNode`

Bounding Volume Hierarchy: árbol binario de aceleración espacial. Organiza todos los objetos de la escena para reducir el número de tests de intersección de O(n) a O(log n).

---

## Clase `BVHNode`

Cada nodo del árbol contiene:
- **Dos hijos** (`left`, `right`): pueden ser `BVHNode` u objetos geométricos directos (`Sphere`, `Quad`, `Triangle`)
- **Una AABB** (`box`): la caja envolvente que contiene todos los objetos del subárbol

### Constructor (uso interno)

```python
BVHNode(left, right, box: AABB)
```

No se llama directamente. Usar el método de fábrica `BVHNode.create()`.

---

## Métodos

### `create(objects, start=0, end=None) → BVHNode | Geometry` (classmethod)

Construye el árbol BVH recursivamente a partir de una lista de objetos geométricos.

**Algoritmo:**

```
create(objects[start:end]):

  Elegir eje aleatorio: axis ∈ {0=X, 1=Y, 2=Z}

  CASO BASE 1 — 1 objeto:
    → Retornar el objeto directamente (evita nodo innecesario)

  CASO BASE 2 — 2 objetos:
    → Ordenarlos por posición mínima en el eje elegido
    → Crear BVHNode(left=obj_a, right=obj_b, box=union(bbox_a, bbox_b))

  CASO RECURSIVO — N > 2 objetos:
    → Ordenar objects[start:end] por posición mínima en el eje elegido
    → mid = start + (end - start) // 2
    → left  = create(objects, start, mid)
    → right = create(objects, mid, end)
    → box   = left.bounding_box().union(right.bounding_box())
    → Retornar BVHNode(left, right, box)
```

**Comparación para ordenar:**
```python
def box_compare(obj):
    return obj.bounding_box().min.[x|y|z][axis]
```

Los objetos se ordenan por la coordenada mínima de su bounding box en el eje seleccionado, luego se dividen a la mitad.

**Por qué eje aleatorio:**
Elegir siempre el mismo eje puede producir árboles degenerados (largos y delgados) para escenas con objetos alineados. La elección aleatoria produce árboles más balanceados en promedio.

**Retorno:** la raíz del árbol, que puede ser un `BVHNode` (si hay ≥ 2 objetos) o directamente un objeto geométrico (si solo hay 1).

---

### `hit(ray, t_min, t_max) → HitRecord | None`

Traversal recursivo del árbol para encontrar el objeto más cercano que el rayo intersecta.

```python
# 1. Test de rechazo rápido
if not self.box.hit(ray, t_min, t_max):
    return None

# 2. Probar hijo izquierdo
hit_left = self.left.hit(ray, t_min, t_max)

# 3. Probar hijo derecho con rango reducido
limit = hit_left.t if hit_left else t_max
hit_right = self.right.hit(ray, t_min, limit)

# 4. Retornar el más cercano
return hit_right if hit_right else hit_left
```

**La optimización clave** está en el paso 3: si el hijo izquierdo encontró un hit en `t = hit_left.t`, no tiene sentido buscar hits en el hijo derecho más allá de esa distancia. El rango se reduce de `[t_min, t_max]` a `[t_min, hit_left.t]`, podando subárboles enteros.

**Polimorfismo:** `self.left` y `self.right` pueden ser cualquier objeto con método `hit()` — otro `BVHNode` (nodo interno) o una geometría (hoja). El código es idéntico en ambos casos.

---

### `bounding_box() → AABB`

Retorna `self.box`, la AABB precalculada durante la construcción. O(1).

---

## Complejidad

| Operación | Sin BVH | Con BVH |
|---|---|---|
| Construcción | — | O(n log n) — una sola vez al inicio |
| Intersección por rayo | O(n) | O(log n) promedio |
| Escena con 69.000 triángulos | 69.000 tests | ~17 tests |

El BVH se construye una sola vez al crear la escena y se reutiliza para todos los rayos del render.

---

## Árbol resultante (ejemplo simplificado)

```
             BVHNode [toda la escena]
            /                        \
    BVHNode [paredes]         BVHNode [objetos]
     /        \                /          \
  Quad      Quad          Sphere        Sphere
 (roja)    (verde)       (vidrio)      (metal)
```

Cuando un rayo entra, primero prueba la caja raíz. Si golpea, prueba ambas ramas. En cada nivel, las ramas que no golpean se descartan completamente.
