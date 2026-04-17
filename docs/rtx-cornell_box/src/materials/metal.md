# `src/materials/metal.py` — Clase `MetalMaterial`

Material metálico con reflexión especular. Simula metales pulidos (espejos perfectos con `fuzz=0`) o metales cepillados/rugosos (con `fuzz > 0`).

---

## Clase `MetalMaterial`

### Constructor

```python
MetalMaterial(albedo: Vec3, fuzz: float = 0.0)
```

| Parámetro | Tipo | Descripción |
|---|---|---|
| `albedo` | `Vec3` | Color del metal. Colorea la reflexión (el oro refleja más en amarillo). |
| `fuzz` | `float` | Rugosidad: 0 = espejo perfecto, 1 = máxima dispersión. Se clampea a [0,1]. |

```python
MetalMaterial(Vec3(0.7, 0.6, 0.5), fuzz=0.05)  # Metal casi pulido
MetalMaterial(Vec3(1.0, 0.8, 0.0), fuzz=0.0)   # Oro espejo
MetalMaterial(Vec3(0.8, 0.8, 0.9), fuzz=0.3)   # Aluminio cepillado
```

---

## Métodos

### `scatter(ray, hit_record) → tuple[bool, Vec3 | None, Ray | None]`

Calcula la reflexión especular del rayo.

**Dirección reflejada:**
```python
reflected = ray.direction.reflect(hit_record.normal)
```

La fórmula de reflexión es: `R = I - 2·(I·N)·N`

Donde `I = ray.direction` (vector incidente, normalizado) y `N = hit_record.normal`.

**Adición de fuzziness:**
```python
if self.fuzz > 0:
    reflected = reflected + random_in_unit_sphere() * self.fuzz
```

Se perturba la dirección reflejada con un offset aleatorio dentro de una esfera de radio `fuzz`. Esto simula microsuperficies irregulares que hacen que la reflexión no sea perfectamente especular.

**Test de validez:**
```python
scattered = reflected.dot(hit_record.normal) > 0
```

Si el rayo reflejado apunta hacia el interior de la superficie (puede ocurrir con `fuzz` alto y ángulos rasantes), el rayo se descarta. Retorna `(False, None, None)`.

**Retorna:** `(True, self.albedo, Ray(hit_point, reflected))` si válido, `(False, None, None)` si no.

---

### `emitted() → Vec3`

Retorna `Vec3(0, 0, 0)`. Los metales no emiten luz.

---

## Rol en el path tracer

`MetalMaterial` usa la **rama especular** del path tracer (sin NEE):

```python
else:
    # Metal y Dielectric: solo rebote especular
    indirect = color_ray(scattered_ray, world, lights, depth-1, max_depth, True)
    return attenuation * indirect
```

El parámetro `puede_ver_luz=True` permite que el rayo especular vea la emisión directa de la fuente de luz si la golpea. Esto es correcto porque la reflexión especular no muestrea la luz por separado como hace NEE.

---

## Diferencia visual con `fuzz`

```
fuzz = 0.0  → Reflejo nítido, espejo perfecto
fuzz = 0.1  → Reflejo ligeramente borroso (metal pulido)
fuzz = 0.3  → Reflejo claramente difuso (metal cepillado)
fuzz = 1.0  → Reflejo muy disperso, casi como material difuso
```

A mayor `fuzz`, más rayos se descartan por el test `reflected.dot(normal) > 0`, lo que oscurece ligeramente el metal en ángulos rasantes — un comportamiento físicamente plausible.
