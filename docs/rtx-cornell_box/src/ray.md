# `src/ray.py` — Clase Ray

Representa un rayo en el espacio 3D: una semirrecta con origen fijo y dirección normalizada. Es la estructura que viaja por la escena en cada paso del path tracing.

---

## Clase `Ray`

### Constructor

```python
Ray(origin: Vec3, direction: Vec3)
```

| Parámetro | Tipo | Descripción |
|---|---|---|
| `origin` | `Vec3` | Punto de inicio del rayo |
| `direction` | `Vec3` | Dirección de viaje (se normaliza automáticamente) |

**La dirección se normaliza siempre en el constructor.** Esto garantiza que el parámetro `t` en `point_at(t)` represente distancia real en unidades de escena. Sin normalización, `t` sería una distancia escalada por la magnitud de la dirección original, rompiendo las comparaciones de profundidad.

```python
# Ambos producen la misma dirección normalizada
Ray(Vec3(0,0,0), Vec3(1, 0, 0))
Ray(Vec3(0,0,0), Vec3(100, 0, 0))
```

### Atributos

| Atributo | Tipo | Descripción |
|---|---|---|
| `origin` | `Vec3` | Punto de inicio (sin modificar) |
| `direction` | `Vec3` | Dirección unitaria (normalizada) |

---

## Métodos

### `point_at(t: float) → Vec3`

Evalúa la ecuación paramétrica del rayo:

```
P(t) = origin + t · direction
```

Retorna el punto 3D en el rayo a distancia `t` desde el origen.

- `t = 0` → el origen exacto
- `t > 0` → puntos adelante (dirección del rayo)
- `t < 0` → puntos atrás (no usados en path tracing)

**Uso principal:** una vez que un algoritmo de intersección devuelve el parámetro `t`, se usa `point_at(t)` para obtener las coordenadas 3D del punto de impacto.

```python
# El rayo golpea la esfera en t=5.3
hit_point = ray.point_at(5.3)   # Vec3 con el punto exacto
```

---

## Tipos de rayos en el proyecto

| Tipo | Origen | Dirección |
|---|---|---|
| **Rayo primario** | Posición de la cámara | Hacia el píxel en el viewport |
| **Rayo de sombra** | Punto de impacto + offset de normal | Hacia la fuente de luz |
| **Rayo difuso** | Punto de impacto | Aleatoria en el hemisferio |
| **Rayo reflejado** | Punto de impacto | Reflexión especular |
| **Rayo refractado** | Punto de impacto | Refracción (Ley de Snell) |

---

## Nota sobre el offset de shadow rays

Al crear rayos de sombra (o rayos rebotados), el origen se desplaza ligeramente en la dirección de la normal:

```python
shadow_origin = hit_point + normal * 0.001
```

Esto evita que el rayo re-intersecte inmediatamente la misma superficie en `t ≈ 0` por errores de punto flotante, fenómeno conocido como **shadow acne**. El parámetro `t_min=0.001` en `world.hit()` tiene el mismo propósito.
