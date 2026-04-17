# `main.py` — Punto de entrada

Archivo raíz del proyecto. Configura y lanza el proceso completo de renderizado.

---

## Función `main()`

Única función del archivo. Orquesta los tres pasos de setup antes de llamar al renderer.

### Paso 1 — Configuración de rendering

```python
render_config = RenderConfig(
    width=200, height=200,
    samples=200, max_depth=8,
    use_parallel=True, gamma=2.2,
)
```

Crea un `RenderConfig` con los parámetros de calidad y salida. Ver `src/config/render_config.py` para el detalle de cada campo.

### Paso 2 — Configuración de cámara

```python
camera_config = CameraConfig(fov=40.0, aperture=20.0)
```

Usa los valores por defecto de posición (`origin`, `lookat`) que apuntan al centro de la Cornell Box. Solo se sobreescribe el FOV y la apertura para depth of field.

### Paso 3 — Setup de objetos

```python
aspect_ratio = render_config.aspect_ratio   # width / height
camera  = Camera(camera_config, aspect_ratio)
renderer = Renderer(render_config)
world, lights = create_cornell_box_scene(SceneMode.SPHERES)
```

- `Camera` precalcula la base ortonormal y el viewport a partir de la configuración.
- `Renderer` guarda la configuración para usarla al renderizar.
- `create_cornell_box_scene` construye toda la geometría, materiales y el árbol BVH.

### Paso 4 — Render

```python
renderer.render(world, lights, camera)
```

Bloquea hasta que la imagen esté completa. La imagen se guarda automáticamente en `output_path`.

---

## Cómo cambiar el modo de escena

```python
# Dos esferas (vidrio + metal)
world, lights = create_cornell_box_scene(SceneMode.SPHERES)

# Stanford Bunny de vidrio
world, lights = create_cornell_box_scene(SceneMode.BUNNY)
```

---

## Dependencias importadas

| Símbolo | Origen | Rol |
|---|---|---|
| `RenderConfig` | `src.config` | Parámetros de calidad y salida |
| `CameraConfig` | `src.config` | Parámetros ópticos de la cámara |
| `Camera` | `src.camera` | Genera rayos por píxel |
| `Renderer` | `src.renderer` | Orquesta el render multi-core |
| `create_cornell_box_scene` | `src.scene` | Factory de la escena |
| `SceneMode` | `src.scene` | Enum SPHERES / BUNNY |
