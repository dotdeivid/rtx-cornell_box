# `src/scene/cornell_box.py` — Constructores de escena

Factory de la Cornell Box. Construye toda la geometría, asigna materiales, arma el BVH y devuelve la lista de luces.

---

## Enum `SceneMode`

```python
class SceneMode(Enum):
    SPHERES = "spheres"
    BUNNY   = "bunny"
```

Selector del contenido central de la escena. Pasado como argumento a `create_cornell_box_scene()`.

---

## Función `create_cornell_box_scene(mode: SceneMode) → tuple[BVHNode, List]`

Construye la escena completa y retorna el mundo listo para renderizar.

### Materiales

```python
red_diffuse   = DiffuseMaterial(albedo=Vec3(0.65, 0.05, 0.05))
white_diffuse = DiffuseMaterial(albedo=Vec3(0.73, 0.73, 0.73))
green_diffuse = DiffuseMaterial(albedo=Vec3(0.12, 0.45, 0.15))
light_material = EmissiveMaterial(emission=Vec3(15, 15, 15))
```

Los valores de albedo están basados en las especificaciones físicas medidas de la Cornell Box original de la Cornell University.

### Geometría de la caja

La Cornell Box ocupa el cubo `[0, 555]³`. Las 5 superficies fijas son:

| Objeto | Tipo | Q | u | v | Material |
|---|---|---|---|---|---|
| Pared izquierda | Quad | (555, 0, 0) | (0, 555, 0) | (0, 0, 555) | Roja |
| Pared derecha | Quad | (0, 0, 0) | (0, 555, 0) | (0, 0, 555) | Verde |
| Piso | Quad | (0, 0, 0) | (555, 0, 0) | (0, 0, 555) | Blanca |
| Techo | Quad | (555, 555, 555) | (-555, 0, 0) | (0, 0, -555) | Blanca |
| Fondo | Quad | (0, 0, 555) | (555, 0, 0) | (0, 555, 0) | Blanca |

Los vectores `u` y `v` negativos en el techo y la pared del fondo invierten la normal, asegurando que apunte hacia el interior de la caja.

### Fuente de luz

```python
light = Quad(
    Q=Vec3(343, 554, 332),
    u=Vec3(-130, 0, 0),
    v=Vec3(0, 0, -105),
    material=light_material,
)
```

Quad rectangular en el techo (y≈554), ligeramente descentrado. Sus dimensiones son 130×105 unidades. La intensidad `15` permite iluminación indirecta visible sin sobreexponer las superficies directas.

### Objetos centrales según `mode`

**`SceneMode.SPHERES`:**
```python
glass = DielectricMaterial(ior=1.5)
Sphere(center=Vec3(190, 90, 190), radius=90, material=glass)   # Izquierda: vidrio

metal = MetalMaterial(albedo=Vec3(0.7, 0.6, 0.5), fuzz=0.05)
Sphere(center=Vec3(380, 90, 350), radius=90, material=metal)   # Derecha: metal
```

**`SceneMode.BUNNY`:**
```python
glass = DielectricMaterial(ior=1.5)
bunny_triangles = load_obj(
    "models/bunny.obj",
    scale=1400.0,
    offset=Vec3(278, 0, 278),
    material=glass,
)
objects.extend(bunny_triangles)   # ~69.000 triángulos
```

El scale de `1400.0` adapta el Bunny (que originalmente tiene dimensiones ~0.3 unidades) al espacio de la Cornell Box (555 unidades). El offset centra el modelo en XZ y lo coloca sobre el piso.

### Construcción del BVH

```python
world = BVHNode.create(objects)
```

Todos los objetos (paredes + luz + objetos centrales) se pasan juntos al BVH. Este ordena y particiona recursivamente por ejes hasta construir el árbol completo.

### Extracción de luces

```python
lights = [obj for obj in objects if hasattr(obj.material, "emission_color")]
```

Filtra de la lista de objetos los que tienen `EmissiveMaterial` (identificado por el atributo `emission_color`). En la escena estándar, solo el `Quad` de luz satisface esta condición.

Esta lista se pasa al path tracer para que `calculate_nee()` sepa a qué objetos lanzar shadow rays.

### Retorno

```python
return world, lights
```

- `world`: `BVHNode` raíz que contiene toda la escena, listo para `world.hit(ray, ...)`
- `lights`: lista de objetos emisivos para NEE
