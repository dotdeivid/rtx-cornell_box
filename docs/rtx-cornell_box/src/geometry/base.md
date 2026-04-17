# `src/geometry/base.py` — Clase base `Geometry`

Define la interfaz abstracta que toda primitiva geométrica del proyecto debe implementar.

---

## Clase `Geometry` (ABC)

Clase base abstracta. No se puede instanciar directamente. Sirve como contrato: cualquier objeto que quiera existir en la escena y ser intersectado por rayos debe heredar de `Geometry` e implementar sus dos métodos.

### Por qué usar ABC

Sin una interfaz común, el BVH y el path tracer tendrían que saber qué tipo concreto es cada objeto para llamar la función correcta. Con `Geometry`, todo objeto se trata de forma uniforme — el mismo código funciona para `Sphere`, `Quad`, `Triangle` y `BVHNode`.

---

## Métodos abstractos

### `hit(ray, t_min, t_max) → Optional[HitRecord]`

Calcula si el rayo intersecta este objeto en el intervalo `[t_min, t_max]`.

| Parámetro | Tipo | Descripción |
|---|---|---|
| `ray` | `Ray` | El rayo a probar |
| `t_min` | `float` | Distancia mínima válida (evita auto-intersección) |
| `t_max` | `float` | Distancia máxima válida |

**Retorna:** `HitRecord` con los datos del impacto si hay intersección válida, `None` si no.

Cada subclase implementa la matemática específica de su geometría:
- `Sphere` → ecuación cuadrática
- `Quad` → intersección rayo-plano + test baricéntrico
- `Triangle` → algoritmo Möller-Trumbore
- `BVHNode` → prueba AABB primero, luego traversal recursivo

### `bounding_box() → AABB`

Retorna la caja envolvente alineada con los ejes (AABB) mínima que contiene completamente al objeto.

El BVH llama a este método durante su construcción para calcular la jerarquía. Todos los objetos deben poder calcular su propia bounding box.

---

## Subclases en el proyecto

| Clase | Archivo |
|---|---|
| `Sphere` | `src/geometry/sphere.py` |
| `Quad` | `src/geometry/quad.py` |
| `Triangle` | `src/geometry/triangle.py` |
| `BVHNode` | `src/geometry/bvh.py` |

`BVHNode` también implementa `hit()` y `bounding_box()`, lo que permite anidarlo dentro de otros `BVHNode` de forma recursiva.
