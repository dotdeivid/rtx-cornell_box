# `src/materials/dielectric.py` — Clase `DielectricMaterial`

Material dieléctrico (transparente). Simula vidrio, agua, diamante y cualquier material que refracte la luz. Implementa la Ley de Snell vectorial y la aproximación de Fresnel de Schlick.

---

## Clase `DielectricMaterial`

### Constructor

```python
DielectricMaterial(ior: float)
```

| Parámetro | Tipo | Descripción |
|---|---|---|
| `ior` | `float` | Índice de refracción del material |

Valores comunes de `ior`:

| Material | IOR |
|---|---|
| Aire | 1.0 |
| Agua | 1.33 |
| Vidrio | 1.5 |
| Zafiro | 1.77 |
| Diamante | 2.4 |

---

## Métodos

### `scatter(ray, hit_record) → tuple[bool, Vec3, Ray]`

Decide si el rayo refracta o refleja en el punto de impacto, usando física de Fresnel.

**Paso 1 — Determinar si el rayo entra o sale del material:**

```python
if unit_direction.dot(hit_record.normal) > 0:
    # Rayo y normal en la misma dirección → saliendo del material
    normal = hit_record.normal * -1   # Invertir normal para que se oponga al rayo
    ri = self.ior                     # η_dentro / η_afuera = ior / 1.0
else:
    # Rayo opuesto a normal → entrando al material
    normal = hit_record.normal        # Normal ya apunta hacia afuera (opuesta al rayo)
    ri = 1.0 / self.ior               # η_dentro / η_afuera = 1.0 / ior
```

El cociente `ri` es siempre η₁/η₂ (medio de entrada / medio de salida).

**Paso 2 — Calcular ángulo de incidencia:**

```python
cos_theta = min(-unit_direction.dot(normal), 1.0)
sin_theta = sqrt(1.0 - cos_theta²)
```

**Paso 3 — Verificar reflexión interna total:**

```python
cannot_refract = ri * sin_theta > 1.0
```

Si `ri · sin(θ) > 1`, la Ley de Snell no tiene solución: toda la luz debe reflejarse. Ocurre cuando la luz intenta salir de un medio denso (vidrio) hacia uno menos denso (aire) con un ángulo suficientemente grande.

**Paso 4 — Decidir reflexión o refracción (Fresnel):**

```python
if cannot_refract or self._reflectance(cos_theta, ri) > random():
    direction = unit_direction.reflect(normal)     # Reflexión
else:
    direction = self._refract(unit_direction, normal, ri)  # Refracción
```

La decisión es **probabilística**: con probabilidad `R(θ)` (Fresnel) se refleja, con `1 - R(θ)` se refracta. Promediando muchas muestras, se obtiene el efecto Fresnel correcto sin necesidad de lanzar dos rayos.

**Atenuación:** siempre `Vec3(1, 1, 1)` — el vidrio ideal no absorbe ningún color.

**Retorna:** siempre `(True, Vec3(1,1,1), Ray(hit_point, direction))`.

---

### `_reflectance(cosine, ref_idx) → float` (staticmethod)

Aproximación de Schlick para la reflectancia de Fresnel:

```python
r0 = ((1 - ref_idx) / (1 + ref_idx))²
R(θ) = r0 + (1 - r0) · (1 - cos θ)⁵
```

`R(θ)` es la fracción de luz reflejada en función del ángulo de incidencia.

- `θ = 0` (incidencia normal) → `R = r0` (mínima reflexión)
- `θ = 90°` (incidencia rasante) → `R = 1.0` (reflexión total)

Para vidrio (`ior=1.5`): `r0 = ((1-1.5)/(1+1.5))² = 0.04` — solo un 4% de reflexión en incidencia normal.

---

### `_refract(uv, n, etai_over_etat) → Vec3` (staticmethod)

Calcula la dirección refractada usando la forma vectorial de la Ley de Snell.

La dirección refractada se descompone en dos componentes respecto a la normal:

```python
# Componente perpendicular a la normal (en el plano de incidencia)
r_out_perp = (uv + n * cos_theta) * etai_over_etat

# Componente paralela a la normal
r_out_parallel = n * (-sqrt(abs(1.0 - r_out_perp.length()²)))

return r_out_perp + r_out_parallel
```

El `abs()` previene raíces de números negativos por errores numéricos de punto flotante. En la práctica, `_refract` solo se llama cuando la reflexión total ya fue descartada, por lo que el valor bajo la raíz es siempre ≥ 0.

---

## Rol en el path tracer

`DielectricMaterial` usa la rama especular (sin NEE), igual que el metal:

```python
else:
    # Especular: metal y dielectric
    indirect = color_ray(scattered_ray, ..., puede_ver_luz=True)
    return attenuation * indirect
```

`puede_ver_luz=True` permite que el rayo refractado/reflejado detecte la emisión directa de la fuente de luz si la alcanza.
