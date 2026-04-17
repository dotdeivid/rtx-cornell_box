# `src/config/camera_config.py` — Dataclass `CameraConfig`

Configuración de los parámetros ópticos y de posicionamiento de la cámara virtual.

---

## Clase `CameraConfig` (dataclass)

### Campos

| Campo | Tipo | Default | Descripción |
|---|---|---|---|
| `origin` | `Vec3` | `Vec3(278, 278, -800)` | Posición de la cámara en el espacio 3D |
| `lookat` | `Vec3` | `Vec3(278, 278, 278)` | Punto hacia el que apunta la cámara |
| `vup` | `Vec3` | `Vec3(0, 1, 0)` | Vector "arriba" del mundo |
| `fov` | `float` | `40.0` | Campo de visión vertical en grados |
| `aperture` | `float` | `20.0` | Apertura del lente (controla DOF) |
| `focus_distance` | `Optional[float]` | `None` | Distancia al plano focal. `None` = auto |

---

## Método de validación

### `__post_init__()`

Valida los parámetros ópticos:

```python
if self.fov <= 0 or self.fov >= 180:
    raise ValueError("FOV debe estar entre 0 y 180 grados")

if self.aperture < 0:
    raise ValueError("Aperture no puede ser negativo")

if self.focus_distance is not None and self.focus_distance <= 0:
    raise ValueError("Focus distance debe ser positivo")
```

Un FOV de 0° sería un viewport de ancho cero; un FOV de 180° o más es indefinido matemáticamente (la tangente diverge). `aperture < 0` no tiene sentido físico.

---

## Parámetros explicados

### `origin` y `lookat`

Definen la posición y orientación de la cámara. La dirección de visión es `lookat - origin`.

Los defaults apuntan al centro de la Cornell Box (`278, 278, 278`) desde una posición frontal exterior (`z = -800`). La caja ocupa `[0, 555]³`.

### `vup`

Vector "arriba" de referencia para construir la base ortonormal de la cámara. Normalmente `(0, 1, 0)`. Solo debe cambiarse para efectos de cámara inclinada (dutch angle).

El vector `v` real de la cámara (arriba en pantalla) se recalcula en `Camera._setup_camera()` como `w.cross(u)`, garantizando ortogonalidad exacta incluso si `vup` no es perpendicular a la dirección de visión.

### `fov`

Ángulo vertical del viewport. Determina cuánto "ve" la cámara.

```
fov=20° → Teleobjetivo: poco FOV, objetos grandes, poca distorsión
fov=40° → Normal (default para Cornell Box)
fov=60° → Gran angular suave
fov=90° → Gran angular pronunciado, distorsión visible en bordes
```

Internamente se convierte a radianes y se usa: `h = tan(fov/2)`, `viewport_height = 2·h`.

### `aperture`

Diámetro del círculo de apertura de la lente. Controla la intensidad del efecto de profundidad de campo.

```
aperture=0    → Sin DOF (cámara pinhole ideal), todo en foco
aperture=5    → DOF muy sutil
aperture=20   → DOF moderado (default)
aperture=60   → DOF muy pronunciado, bokeh dramático
```

Internamente se usa `lens_radius = aperture / 2`. Cada rayo por píxel se origina en un punto aleatorio del disco de radio `lens_radius`.

### `focus_distance`

Distancia desde la cámara al plano focal (el plano donde los objetos aparecen nítidos).

- `None` (default): se auto-calcula como `|origin - lookat|`. Los objetos en `lookat` están en foco.
- Valor manual: permite desplazar el plano focal sin mover la cámara.

```python
focus_distance=1000.0   # Enfoca a 1000 unidades de la cámara
focus_distance=600.0    # Enfoca más cerca que lookat
```

Solo tiene efecto visible cuando `aperture > 0`.
