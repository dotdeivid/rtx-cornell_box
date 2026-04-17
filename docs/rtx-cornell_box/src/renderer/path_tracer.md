# `src/renderer/path_tracer.py` — Path Tracer

Núcleo matemático del renderizado. Implementa el algoritmo de path tracing recursivo con Next Event Estimation (NEE). Es donde se evalúa la Rendering Equation de Kajiya.

---

## Función `calculate_nee(rec, world, lights) → Vec3`

**Next Event Estimation** — calcula la contribución de iluminación directa en un punto difuso.

### Parámetros

| Parámetro | Tipo | Descripción |
|---|---|---|
| `rec` | `HitRecord` | Punto difuso desde el que se calcula la iluminación |
| `world` | `BVHNode` | Geometría completa de la escena (para shadow rays) |
| `lights` | `List` | Lista de objetos con `EmissiveMaterial` |

### Algoritmo

Para cada fuente de luz:

**1. Muestrear dirección hacia la luz:**
```python
l_dir, solid_angle = light.sample_solid_angle(rec.point)
```
`sample_solid_angle` retorna una dirección aleatoria hacia la luz y el ángulo sólido que subtiende (para corregir el peso de la muestra).

**2. Calcular coseno del ángulo de incidencia:**
```python
cos_theta_surface = max(0, l_dir.dot(rec.normal))
```
Si `cos_theta_surface ≤ 0`, la luz está por debajo del horizonte de la superficie → no contribuye.

**3. Shadow ray:**
```python
shadow_ray = Ray(rec.point + rec.normal * 0.001, l_dir)
shadow_hit = world.hit(shadow_ray, 0.001, float("inf"))
```
El rayo de sombra se lanza con un pequeño offset en la dirección de la normal para evitar auto-intersección (shadow acne).

**4. Verificar visibilidad:**
```python
if shadow_hit and shadow_hit.obj_ref == light:
```
El shadow ray debe golpear **exactamente** la fuente de luz (no otro objeto que esté entre medias). `obj_ref` es la referencia al objeto geométrico, no solo al material.

**5. Calcular contribución:**
```python
brdf = rec.material.albedo / math.pi      # BRDF Lambertiana
direct_light += light.material.emitted() * brdf * cos_theta_surface * solid_angle
```

La fórmula implementa la integral de la Rendering Equation para iluminación directa:
```
L_direct = emission · (albedo/π) · cos(θ) · Ω
```

### Retorna

`Vec3` con el color de la iluminación directa acumulada de todas las fuentes de luz.

---

## Función `color_ray(ray, world, lights, depth, max_depth, puede_ver_luz=True) → Vec3`

Estimador Monte Carlo de la Rendering Equation. Traza un rayo recursivamente hasta alcanzar la profundidad máxima o una fuente de luz.

### Parámetros

| Parámetro | Tipo | Descripción |
|---|---|---|
| `ray` | `Ray` | Rayo a trazar |
| `world` | `BVHNode` | Escena completa |
| `lights` | `List` | Fuentes de luz para NEE |
| `depth` | `int` | Profundidad actual de recursión (decrece en cada rebote) |
| `max_depth` | `int` | Límite máximo de rebotes |
| `puede_ver_luz` | `bool` | Si `False`, ignora emisión directa (evita doble conteo con NEE) |

### Flujo de ejecución

```
1. ¿depth ≤ 0?
   → Retornar Vec3(0,0,0)  [límite alcanzado, absorber rayo]

2. world.hit(ray, 0.001, inf)
   → Si no hay hit: retornar color de fondo

3. ¿Emissive?
   → Si emission.length() > 0:
     - puede_ver_luz=True  → retornar emission
     - puede_ver_luz=False → retornar Vec3(0,0,0)  [ya contado por NEE]

4. material.scatter(ray, hit)
   → Si scattered=False: retornar Vec3(0,0,0)  [absorbido]

5. ¿DiffuseMaterial?
   → SÍ:
     nee = calculate_nee(hit, world, lights)          [iluminación directa]
     indirect = color_ray(scattered_ray, ..., depth-1, puede_ver_luz=False)
     return nee + attenuation * indirect

   → NO (metal o dielectric):
     indirect = color_ray(scattered_ray, ..., depth-1, puede_ver_luz=True)
     return attenuation * indirect
```

### Color de fondo

```python
if ray.direction.y > 0.8:
    return Vec3(1.5, 1.5, 1.5)   # Cielo brillante (rayos que apuntan muy arriba)
return Vec3(0.05, 0.05, 0.05)    # Suelo oscuro
```

En la Cornell Box cerrada, muy pocos rayos escapan sin golpear una pared. Este fondo actúa como color de escape para esos casos.

### El flag `puede_ver_luz`

Previene el **doble conteo** de la emisión de la luz:

- Los materiales difusos calculan iluminación directa mediante NEE (shadow ray explícito).
- El rayo indirecto difuso tiene `puede_ver_luz=False`: si golpea la luz, retorna negro (ya se contó en NEE).
- Los materiales especulares (metal/vidrio) no usan NEE, por lo que tienen `puede_ver_luz=True`: si su rayo alcanza la luz, cuenta normalmente.

---

## Implementación de la Rendering Equation

La función `color_ray` evalúa:

```
Lo(x, ωo) = Le(x, ωo) + ∫Ω fr(x, ωi, ωo) · Li(x, ωi) · cos(θi) dωi
```

- `Le` → `hit.material.emitted()` (emisión)
- `fr · cos(θ)` → BRDF del material multiplicado por el coseno
- `∫ Li dωi` → estimado por Monte Carlo: un solo rayo aleatorio + NEE para difusos

La separación NEE + indirecto es una técnica de **variance reduction**: en lugar de esperar que el rayo difuso encuentre la luz por azar, se muestrea directamente (NEE) y se combina con la contribución indirecta.
