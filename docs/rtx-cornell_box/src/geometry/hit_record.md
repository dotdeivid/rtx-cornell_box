# `src/geometry/hit_record.py` — Dataclass `HitRecord`

Estructura de datos que encapsula toda la información relevante sobre una intersección rayo-geometría. Es el objeto que los métodos `hit()` retornan cuando hay un impacto.

---

## Clase `HitRecord` (dataclass)

```python
@dataclass
class HitRecord:
    t:        float
    point:    Vec3
    normal:   Vec3
    material: Optional[Material] = None
    obj_ref:  Optional[object]   = None
```

### Campos

| Campo | Tipo | Descripción |
|---|---|---|
| `t` | `float` | Parámetro del rayo en el punto de impacto. Representa la distancia real desde el origen del rayo (ya que la dirección está normalizada). |
| `point` | `Vec3` | Coordenadas 3D exactas del punto de intersección. Calculado como `ray.point_at(t)`. |
| `normal` | `Vec3` | Vector normal a la superficie en el punto de impacto. Siempre apunta hacia fuera de la superficie (exterior). |
| `material` | `Material` | Referencia al material del objeto golpeado. Determina cómo se comporta la luz en ese punto (difuso, metal, vidrio, emisivo). |
| `obj_ref` | `object` | Referencia al objeto geométrico golpeado. Usado en NEE para verificar si el shadow ray golpeó la luz correcta: `shadow_hit.obj_ref == light`. |

---

## Por qué existe este objeto

Centraliza en un solo lugar toda la información que el path tracer necesita después de una intersección. Sin él, `hit()` tendría que retornar múltiples valores separados o una tupla, lo que haría el código más frágil y difícil de leer.

El uso de `@dataclass` genera automáticamente `__init__`, `__repr__` y `__eq__` sin código boilerplate.

---

## Cómo se construye (ejemplo en Sphere)

```python
return HitRecord(
    t=root,
    point=ray.point_at(root),
    normal=(point - self.center) / self.radius,
    material=self.material,
    obj_ref=self,
)
```

Cada geometría es responsable de calcular su propia normal y pasarse a sí misma como `obj_ref`.

---

## Cómo se consume (en path_tracer)

```python
hit = world.hit(ray, 0.001, float("inf"))
if hit:
    emission = hit.material.emitted()          # ¿Emite luz?
    scattered, att, new_ray = hit.material.scatter(ray, hit)  # ¿Cómo rebota?
    brdf = hit.material.albedo / math.pi       # BRDF del material
    is_light = shadow_hit.obj_ref == light     # ¿Es la luz que buscamos?
```
