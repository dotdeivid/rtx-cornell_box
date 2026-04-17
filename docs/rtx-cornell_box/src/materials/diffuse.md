# `src/materials/diffuse.py` — Clase `DiffuseMaterial`

Material Lambertiano. Dispersa la luz de forma uniforme en el hemisferio. Representa superficies mate: paredes, papel, madera sin barniz, yeso.

---

## Clase `DiffuseMaterial`

### Constructor

```python
DiffuseMaterial(albedo: Vec3)
```

| Parámetro | Tipo | Descripción |
|---|---|---|
| `albedo` | `Vec3` | Color del material en RGB [0,1]. Fracción de luz reflejada por componente. |

```python
DiffuseMaterial(Vec3(0.65, 0.05, 0.05))  # Pared roja
DiffuseMaterial(Vec3(0.73, 0.73, 0.73))  # Pared blanca
DiffuseMaterial(Vec3(0.12, 0.45, 0.15))  # Pared verde
```

---

## Métodos

### `scatter(ray, hit_record) → tuple[bool, Vec3, Ray]`

Calcula el rayo difuso rebotado.

**Dirección de dispersión:**
```python
scatter_direction = hit_record.normal + random_in_unit_sphere().normalize()
```

Esto es la suma de la normal y un vector unitario aleatorio, lo que produce una distribución aproximadamente coseno-ponderada alrededor de la normal — ligeramente más muestras cerca de la normal que en los bordes del hemisferio. Es una buena aproximación del scattering Lambertiano ideal.

**Caso degenerado:**
```python
if abs(scatter_direction.length()) < 1e-8:
    scatter_direction = hit_record.normal
```

Si el vector aleatorio cancela exactamente la normal (probabilidad extremadamente baja pero posible), `scatter_direction` sería el vector cero, lo que produciría un rayo inválido. Se usa la normal como fallback.

**Retorna:** `(True, self.albedo, Ray(hit_point, scatter_direction))`

---

### `emitted() → Vec3`

Retorna `Vec3(0, 0, 0)`. Los materiales difusos no emiten luz.

---

## Rol en el path tracer

`DiffuseMaterial` es el único material que usa **Next Event Estimation (NEE)**. El path tracer lo detecta con:

```python
if isinstance(hit.material, DiffuseMaterial):
    nee_contribution = calculate_nee(hit, world, lights)
    ...
```

Esto es correcto porque la NEE aplica el BRDF Lambertiano (`albedo / π`), que solo es válido para materiales difusos. Los metales y dieléctricos usan reflexión/refracción especular y no pueden muestrearse de la misma manera.

---

## BRDF Lambertiana

La función de distribución de reflectancia bidireccional (BRDF) para un material Lambertiano es constante:

```
fr(ωi, ωo) = albedo / π
```

El factor `π` normaliza la integral sobre el hemisferio para conservar energía. En `calculate_nee`:

```python
brdf = rec.material.albedo / math.pi
```
