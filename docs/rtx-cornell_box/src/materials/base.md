# `src/materials/base.py` — Clase base `Material`

Define la interfaz abstracta que todo material del proyecto debe implementar. Es el contrato entre la geometría (que detecta impactos) y el path tracer (que decide cómo evoluciona el rayo).

---

## Clase `Material` (ABC)

Clase base abstracta. No se puede instanciar directamente.

### Por qué existe

Permite que el path tracer trate todos los materiales de forma uniforme:

```python
# Funciona igual para Diffuse, Metal, Dielectric, Emissive
scattered, attenuation, ray = hit.material.scatter(ray, hit)
emission = hit.material.emitted()
```

Sin esta interfaz, el path tracer necesitaría un bloque `if/elif` por tipo de material, rompiendo el principio Open/Closed.

---

## Métodos abstractos

### `scatter(ray, hit_record) → tuple[bool, Vec3 | None, Ray | None]`

El método central. Determina cómo interactúa el rayo con el material.

**Parámetros:**

| Parámetro | Tipo | Descripción |
|---|---|---|
| `ray` | `Ray` | Rayo incidente |
| `hit_record` | `HitRecord` | Datos del punto de impacto (posición, normal, etc.) |

**Retorna una tupla de tres valores:**

| Valor | Tipo | Descripción |
|---|---|---|
| `scattered` | `bool` | `True` si el rayo se dispersa; `False` si el material lo absorbe |
| `attenuation` | `Vec3 \| None` | Color de atenuación (albedo). Multiplica el color del rayo rebotado. `None` si `scattered=False` |
| `scattered_ray` | `Ray \| None` | El nuevo rayo tras la interacción. `None` si `scattered=False` |

**Comportamiento por tipo:**

| Material | `scattered` | `attenuation` | Dirección del nuevo rayo |
|---|---|---|---|
| `DiffuseMaterial` | `True` | albedo | Aleatoria en hemisferio |
| `MetalMaterial` | `True` o `False` | albedo | Reflexión especular ± fuzz |
| `DielectricMaterial` | `True` | `(1,1,1)` | Refracción o reflexión (Fresnel) |
| `EmissiveMaterial` | `False` | `None` | — (no dispersa) |

---

### `emitted() → Vec3`

Color de luz emitida por el material. Solo `EmissiveMaterial` retorna algo distinto de negro.

**Retorna:** `Vec3` con la radiancia emitida. Para materiales no emisivos, `Vec3(0, 0, 0)`.

Este método permite que el path tracer pregunte a cualquier material "¿emites luz?" sin saber de antemano de qué tipo es.

---

## Subclases en el proyecto

| Clase | Archivo | Descripción |
|---|---|---|
| `DiffuseMaterial` | `src/materials/diffuse.py` | Lambertiano, rebote difuso |
| `MetalMaterial` | `src/materials/metal.py` | Reflexión especular |
| `DielectricMaterial` | `src/materials/dielectric.py` | Refracción + Fresnel |
| `EmissiveMaterial` | `src/materials/emissive.py` | Fuente de luz |
