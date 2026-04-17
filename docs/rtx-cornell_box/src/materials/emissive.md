# `src/materials/emissive.py` — Clase `EmissiveMaterial`

Material emisivo. Representa fuentes de luz de área. No dispersa rayos: solo emite luz. Es el material de la fuente de luz rectangular en el techo de la Cornell Box.

---

## Clase `EmissiveMaterial`

### Constructor

```python
EmissiveMaterial(emission: Vec3)
```

| Parámetro | Tipo | Descripción |
|---|---|---|
| `emission` | `Vec3` | Color e intensidad de la luz emitida. Puede superar 1.0 para simular fuentes brillantes. |

```python
EmissiveMaterial(Vec3(15, 15, 15))    # Luz blanca neutra intensa
EmissiveMaterial(Vec3(20, 15, 8))     # Luz cálida (naranja/amarillo)
EmissiveMaterial(Vec3(8, 12, 20))     # Luz fría (azul)
EmissiveMaterial(Vec3(5, 5, 5))       # Luz suave
```

Los valores por encima de `1.0` son necesarios para que la iluminación indirecta (rebotes en paredes) sea visible sin que las superficies directamente iluminadas se saturen completamente. En la Cornell Box se usa `15` de intensidad.

---

## Métodos

### `scatter(ray, hit_record) → tuple[bool, None, None]`

Las fuentes de luz no dispersan rayos. Siempre retorna:

```python
return False, None, None
```

El path tracer verifica `scattered=False` y detiene el camino del rayo en ese punto, usando solo la emisión como contribución de color.

---

### `emitted() → Vec3`

Retorna el color emitido:

```python
return self.emission_color
```

Es el único material que retorna un valor distinto de `Vec3(0,0,0)` en este método.

---

## Rol en el path tracer

El path tracer maneja `EmissiveMaterial` con un early return antes de cualquier operación de scattering:

```python
emission = hit.material.emitted()
if emission.length() > 0:
    return emission if puede_ver_luz else Vec3(0, 0, 0)
```

El flag `puede_ver_luz` evita contar la emisión dos veces:

- Cuando el rayo viene de NEE (iluminación directa ya contabilizada), `puede_ver_luz=False` → retorna negro.
- Cuando el rayo viene de una reflexión especular (metal/vidrio), `puede_ver_luz=True` → retorna la emisión real.

---

## Identificación de luces en la escena

Al construir la lista de luces para NEE, se identifica qué objetos tienen `EmissiveMaterial`:

```python
lights = [obj for obj in objects if hasattr(obj.material, "emission_color")]
```

El atributo `emission_color` es exclusivo de `EmissiveMaterial`, lo que lo hace un discriminador confiable sin necesidad de `isinstance`.
