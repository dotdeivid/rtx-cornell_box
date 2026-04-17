# `src/camera/camera.py` — Clase `Camera`

Cámara virtual con modelo thin-lens. Genera rayos con perspectiva correcta y simula profundidad de campo (depth of field / bokeh) física.

---

## Clase `Camera`

### Constructor

```python
Camera(config: CameraConfig, aspect_ratio: float)
```

| Parámetro | Tipo | Descripción |
|---|---|---|
| `config` | `CameraConfig` | Parámetros ópticos y de posicionamiento |
| `aspect_ratio` | `float` | Relación ancho/alto (`width / height`) |

Llama a `_setup_camera()` al construirse.

---

## Métodos

### `_setup_camera()`

Precalcula todos los vectores y escalares necesarios para generar rayos. Se ejecuta una sola vez en la construcción.

**1. Distancia focal:**
```python
focus_dist = config.focus_distance or (config.origin - config.lookat).length()
```
Si `focus_distance` es `None`, se auto-calcula como la distancia entre la cámara y el punto `lookat`. Esto hace que los objetos en `lookat` queden perfectamente enfocados.

**2. Base ortonormal de la cámara (`u`, `v`, `w`):**

La cámara se describe con tres vectores perpendiculares entre sí:

```python
w = (config.origin - config.lookat).normalize()   # Apunta HACIA la cámara (atrás)
u = config.vup.cross(w).normalize()               # Apunta a la derecha
v = w.cross(u)                                     # Apunta arriba (recalculado)
```

`vup` es el vector "arriba" del mundo (normalmente `(0,1,0)`). El vector `v` se recalcula mediante el producto cruzado en lugar de usar `vup` directamente, garantizando que sea exactamente perpendicular a `w`.

**3. Dimensiones del viewport:**

```python
theta = radians(config.fov)
h = tan(theta / 2)
viewport_height = 2.0 * h
viewport_width  = aspect_ratio * viewport_height
```

**4. Vectores del viewport escalados por distancia focal:**

```python
horizontal = u * viewport_width  * focus_dist
vertical   = v * viewport_height * focus_dist
```

Al escalar por `focus_dist`, los píxeles del borde del viewport corresponden exactamente al plano focal. Esto es lo que hace que los objetos a `focus_dist` del ojo aparezcan nítidos.

**5. Esquina inferior izquierda del viewport:**

```python
lower_left = config.origin
           - horizontal / 2
           - vertical   / 2
           - w * focus_dist
```

**6. Radio del lente:**

```python
lens_radius = config.aperture / 2
```

---

### `get_ray(s: float, t: float) → Ray`

Genera un rayo desde la cámara hacia el píxel con coordenadas normalizadas `(s, t) ∈ [0,1]²`.

**1. Offset aleatorio en el disco de apertura (DOF):**

```python
rd     = random_in_unit_disk() * self.lens_radius
offset = self.u * rd.x + self.v * rd.y
```

El offset simula que el rayo no sale exactamente del centro de la lente, sino de un punto aleatorio en el área de apertura. A mayor `lens_radius`, mayor dispersión.

**2. Origen del rayo (punto en la lente):**

```python
origin = self.config.origin + offset
```

**3. Dirección hacia el punto en el plano focal:**

```python
focal_point = self.lower_left + self.horizontal * s + self.vertical * t

direction = focal_point - self.config.origin - offset
```

Equivalente a: dirección = punto en el plano focal − origen del rayo en la lente.

Todos los rayos que pasan por el mismo `(s, t)` convergen en el mismo punto del plano focal, sin importar el offset de apertura. Esto produce enfoque nítido en ese plano y desenfoque (bokeh) en todo lo demás.

**Retorna:** `Ray(origin, direction)` — el constructor de `Ray` normaliza la dirección automáticamente.

---

## Modelo thin-lens: resumen visual

```
           Objeto en foco          Objeto fuera de foco
               |                         |
               |                         |
   ·-----------·-----------·   ·---------·---------·
   |           |           |   |         |         |
   |           |           |   |         |         |
   Lente    Plano focal     |   |         |         |
   (apertura)               |   Círculo de confusión
                            Sensor/imagen
```

Con `aperture=0`, todos los rayos salen del mismo punto (cámara pinhole). Con `aperture>0`, los rayos de un mismo píxel salen de diferentes puntos de la lente pero convergen en el plano focal.
