# Guía Completa de ray.py: El Rayo - Fundamento del Ray Tracing

## Tabla de Contenidos

1. [Introducción](#introducción)
2. [Definición Matemática del Rayo](#definición-matemática-del-rayo)
3. [La Clase Ray](#la-clase-ray)
4. [Normalización de la Dirección](#normalización-de-la-dirección)
5. [El Método point_at](#el-método-point_at)
6. [Tipos de Rayos en Ray Tracing](#tipos-de-rayos-en-ray-tracing)
7. [Ejemplos Numéricos Completos](#ejemplos-numéricos-completos)
8. [Aplicaciones Prácticas](#aplicaciones-prácticas)
9. [Optimizaciones y Consideraciones](#optimizaciones-y-consideraciones)

---

## Introducción

### ¿Qué es un Rayo?

En el contexto de ray tracing (trazado de rayos), un **rayo** es una **semirecta** en el espacio 3D: una línea que tiene un punto de inicio pero se extiende infinitamente en una dirección.

**Analogía del mundo real**:
- Imagina el haz de una linterna:
  - **Origen**: La bombilla de la linterna
  - **Dirección**: Hacia donde apuntas la linterna
  - **Semirecta**: La luz sale de la bombilla y continúa en esa dirección indefinidamente

### ¿Por qué son fundamentales los rayos?

El **Ray Tracing** simula cómo viaja la luz simulando el recorrido de millones de rayos:

```
                       Monitor/Imagen Final
                              |
                              | Cada píxel recibe un rayo
                              ↓
                        +------------+
                        |  Cámara    |
                        +------------+
                              |
          +-------------------+-------------------+
          |                   |                   |
       Rayo 1              Rayo 2              Rayo 3
          ↓                   ↓                   ↓
      Esfera              Pared               Vacío
       (Rojo)             (Blanca)          (Cielo azul)
```

Cada rayo:
1. Parte de la cámara
2. Viaja por la escena
3. Puede intersectar objetos
4. Puede rebotar (reflexión/refracción)
5. Acumula color que se retorna al píxel

**Sin rayos = Sin ray tracing**. El rayo es literalmente LA estructura fundamental.

---

## Definición Matemática del Rayo

### Ecuación Paramétrica

Un rayo se define mediante la **ecuación paramétrica**:

```
P(t) = O + t*D
```

**Componentes**:

| Símbolo | Nombre | Tipo | Descripción |
|---------|--------|------|-------------|
| P(t) | Punto | Vec3 | Punto en el rayo a distancia t |
| O | Origen (Origin) | Vec3 | Punto de inicio del rayo |
| D | Dirección (Direction) | Vec3 | Vector unitario (magnitud = 1) |
| t | Parámetro | float | Distancia desde el origen |

### Interpretación Geométrica

Visualmente:

```
                              P(t=5)
                                 •
                                /
                               /
                              /
                   P(t=2)    /
                      •     /
                     /     /
                    /     /
                   /     /
        P(t=0)    /     /
  Origen •-------/-----/
         O      /     /
               /     /  Dirección D
              /     /    (vector unitario)
             /     /
            /     /
```

**Descripción**:
- Empiezas en el **origen** O
- Te mueves en la **dirección** D
- Avanzas una **distancia** t
- Llegas al **punto** P(t)

### Valores del Parámetro t

| Valor de t | Ubicación | Uso típico |
|------------|-----------|------------|
| t = 0 | En el origen O | Punto de inicio |
| t > 0 | Adelante del origen | Ray tracing normal |
| t = ∞ | Infinitamente lejos | Cielo/fondo |
| t < 0 | Atrás del origen | Generalmente NO usado |

**Rango válido en ray tracing**:
- **t_min** (típicamente 0.001): Distancia mínima para evitar shadow acne
- **t_max** (típicamente ∞): Distancia máxima de búsqueda

---

## La Clase Ray

### Código Completo

```python
class Ray:
    def __init__(self, origin: Vec3, direction: Vec3):
        self.origin = origin
        self.direction = direction.normalize()  # ¡CRÍTICO!
    
    def point_at(self, t: float) -> Vec3:
        return self.origin + self.direction * t
```

### Atributos

#### 1. `origin` (Vec3)

**Propósito**: Punto de inicio del rayo en coordenadas 3D.

**Valores típicos**:
```python
# Rayo desde la cámara
origin = Vec3(0, 0, -10)  # Cámara detrás de la escena

# Rayo de rebote desde una superficie
origin = Vec3(5.23, 1.45, 8.92)  # Punto de impacto

# Rayo de sombra desde superficie hacia luz
origin = hit_point + 0.001 * normal  # Ligeramente elevado
```

**Propiedades**:
- Coordenadas absolutas en espacio mundo
- NO cambia después de la creación del rayo
- Cada rebote crea un NUEVO rayo con NUEVO origen

#### 2. `direction` (Vec3)

**Propósito**: Vector unitario que indica la dirección del rayo.

**CRUCIAL**: SIEMPRE normalizado (magnitud = 1)

**Valores típicos**:
```python
# Hacia la derecha
direction = Vec3(1, 0, 0)  # length = 1 ✓

# Diagonal (después de normalizar)
direction = Vec3(1, 1, 0).normalize()  
# = Vec3(0.707, 0.707, 0) con length = 1 ✓

# Hacia una luz específica
direction = (light_pos - hit_point).normalize()
```

**Propiedades**:
- Magnitud SIEMPRE = 1 (vector unitario)
- Define la "pendiente" del rayo
- Se normaliza automáticamente en `__init__`

---

## Normalización de la Dirección

### ¿Qué es Normalizar?

**Normalizar** un vector significa ajustar su magnitud a 1 manteniendo su dirección:

```
Vector original: V = (3, 4, 0)
Magnitud: |V| = √(3² + 4²) = √25 = 5

Vector normalizado: V_norm = V / |V|
                           = (3, 4, 0) / 5
                           = (0.6, 0.8, 0)

Magnitud normalizada: |V_norm| = √(0.6² + 0.8²) = √1 = 1 ✓
```

### ¿Por Qué es CRÍTICO Normalizar?

#### Problema: t no representa distancia real

**Sin normalización**:

```python
direction = Vec3(10, 0, 0)  # Magnitud = 10, NO normalizado

ray = Ray(Vec3(0,0,0), direction)  # Imaginemos que NO normalizamos

# ¿A qué distancia está el punto?
point = ray.point_at(1)
# = Vec3(0,0,0) + 1 * Vec3(10,0,0)
# = Vec3(10, 0, 0)

PROBLEMA: t=1 pero avanzamos 10 unidades ❌
t NO representa la distancia real!
```

**Con normalización** (CORRECTO):

```python
direction = Vec3(10, 0, 0).normalize()  # = Vec3(1, 0, 0)

ray = Ray(Vec3(0,0,0), direction)

point = ray.point_at(1)
# = Vec3(0,0,0) + 1 * Vec3(1,0,0)
# = Vec3(1, 0, 0)

✓ t=1 y avanzamos 1 unidad
t SÍ representa la distancia real!
```

### Consecuencias de NO Normalizar

| Aspecto | Sin Normalizar | Con Normalizar |
|---------|----------------|----------------|
| **Distancias** | t inconsistente | t = distancia real |
| **Atenuación luz** | 1/d² INCORRECTO | 1/d² correcto |
| **Comparación hits** | No se puede determinar cuál es más cercano | t menor = más cercano |
| **Física materiales** | Probabilidades erróneas | Probabilidades correctas |
| **Debugging** | t = ??? | t interpretable |

### Ejemplo Numérico Completo

```python
# Vector diagonal NO normalizado
v = Vec3(3, 4, 0)
print(f"Magnitud: {v.length()}")  # 5.0

# Normalizar
v_norm = v.normalize()
print(f"Normalizado: {v_norm}")  # Vec3(0.6, 0.8, 0)
print(f"Magnitud: {v_norm.length()}")  # 1.0 ✓

# Crear rayo
ray = Ray(Vec3(0, 0, 0), v)
# Internamente: direction = Vec3(3,4,0).normalize() = Vec3(0.6, 0.8, 0)

# Calcular punto a distancia t=5
point = ray.point_at(5)
# = Vec3(0,0,0) + 5 * Vec3(0.6, 0.8, 0)
# = Vec3(3, 4, 0)

# Verificar distancia
dist = (point - ray.origin).length()
print(f"Distancia real: {dist}")  # 5.0 ✓

# t = 5 → distancia = 5 → ✓CORRECTO
```

### Cálculo Matemático de Normalización

Fórmula:

```
V_normalizado = V / |V|

donde |V| = √(x² + y² + z²)
```

**Paso a paso**:

```python
# Vector original
v = Vec3(2, 3, 6)

# Paso 1: Calcular magnitud
mag = sqrt(2² + 3² + 6²)
    = sqrt(4 + 9 + 36)
    = sqrt(49)
    = 7

# Paso 2: Dividir cada componente
v_norm = Vec3(2/7, 3/7, 6/7)
       = Vec3(0.286, 0.429, 0.857)

# Verificación
mag_norm = sqrt(0.286² + 0.429² + 0.857²)
         = sqrt(0.082 + 0.184 + 0.734)
         = sqrt(1.0)
         = 1.0 ✓
```

---

## El Método point_at

### Definición

```python
def point_at(self, t: float) -> Vec3:
    return self.origin + self.direction * t
```

Implementa directamente la ecuación paramétrica: **P(t) = O + t*D**

### Desglose de la Operación

```python
# Dado:
ray.origin = Vec3(1, 2, 3)
ray.direction = Vec3(0.6, 0, 0.8)  # Normalizado, length=1
t = 5

# Cálculo:
result = ray.origin + ray.direction * t

# Paso 1: Multiplicación escalar (t * D)
temp = 5 * Vec3(0.6, 0, 0.8)
     = Vec3(5*0.6, 5*0, 5*0.8)
     = Vec3(3, 0, 4)

# Paso 2: Suma de vectores (O + temp)
result = Vec3(1, 2, 3) + Vec3(3, 0, 4)
       = Vec3(1+3, 2+0, 3+4)
       = Vec3(4, 2, 7)

# Retorna: Vec3(4, 2, 7)
```

### Casos de Uso

#### Caso 1: t = 0 (En el origen)

```python
ray = Ray(Vec3(5, 10, 15), Vec3(1, 0, 0))

point = ray.point_at(0)
# = Vec3(5,10,15) + 0 * Vec3(1,0,0)
# = Vec3(5,10,15) + Vec3(0,0,0)
# = Vec3(5, 10, 15)  # Mismo que origin ✓
```

#### Caso 2: t positivo (Adelante)

```python
ray = Ray(Vec3(0, 0, 0), Vec3(1, 0, 0))

# A distancia 10
point = ray.point_at(10)
# = Vec3(0,0,0) + 10 * Vec3(1,0,0)
# = Vec3(10, 0, 0)  # 10 unidades en X ✓

# A distancia 2.5
point = ray.point_at(2.5)
# = Vec3(0,0,0) + 2.5 * Vec3(1,0,0)
# = Vec3(2.5, 0, 0)  # 2.5 unidades en X ✓
```

#### Caso 3: t negativo (Atrás - raro)

```python
ray = Ray(Vec3(10, 0, 0), Vec3(1, 0, 0))

# Atrás del origen (poco común)
point = ray.point_at(-5)
# = Vec3(10,0,0) + (-5) * Vec3(1,0,0)
# = Vec3(10,0,0) + Vec3(-5,0,0)
# = Vec3(5, 0, 0)  # ¡Atrás del origen!

# En ray tracing esto generalmente NO se usa
```

### Verificación de Distancia

**Importante**: Si la dirección está normalizada, la distancia desde el origen hasta `point_at(t)` es exactamente **t**.

```python
ray = Ray(Vec3(0, 0, 0), Vec3(0.6, 0.8, 0))

point = ray.point_at(10)  # t = 10
# = Vec3(0,0,0) + 10 * Vec3(0.6, 0.8, 0)
# = Vec3(6, 8, 0)

# Verificar distancia
distance = (point - ray.origin).length()
         = Vec3(6, 8, 0).length()
         = sqrt(6² + 8²)
         = sqrt(36 + 64)
         = sqrt(100)
         = 10  ✓ Exactamente igual a t!
```

---

## Tipos de Rayos en Ray Tracing

### 1. Rayos Primarios (Primary Rays)

**Propósito**: Rayo inicial desde la cámara a través de cada píxel.

**Origen**: Posición de la cámara

**Dirección**: Desde cámara hacia píxel en el plano de imagen

```python
# Configuración de cámara
camera_pos = Vec3(0, 0, -10)  # Cámara atrás
viewport_width = 2.0
viewport_height = 1.5

# Para píxel (x=400, y=300) en imagen 800×600
u = (400 / 800) * viewport_width - viewport_width/2
v = (300 / 600) * viewport_height - viewport_height/2

# Dirección hacia ese píxel
pixel_pos = Vec3(u, v, 0)  # Plano de imagen en z=0
direction = (pixel_pos - camera_pos).normalize()

# Rayo primario
primary_ray = Ray(camera_pos, direction)
```

**Cantidad**: 1 por píxel (o más con antialiasing)

### 2. Rayos de Sombra (Shadow Rays)

**Propósito**: Verificar si un punto está en sombra respecto a una luz.

**Origen**: Punto de impacto (ligeramente elevado por shadow acne)

**Dirección**: Hacia la fuente de luz

```python
# Hit en superficie
hit_point = Vec3(5, 2, 3)
normal = Vec3(0, 1, 0)  # Apunta arriba

# Luz
light_pos = Vec3(10, 10, 5)

# Rayo de sombra (origen ligeramente elevado)
shadow_origin = hit_point + 0.001 * normal
shadow_direction = (light_pos - shadow_origin).normalize()
shadow_ray = Ray(shadow_origin, shadow_direction)

# Si shadow_ray intersecta algo ANTES de llegar a la luz
# → punto está en sombra
# Si NO intersecta nada
# → punto está iluminado
```

**Cantidad**: 1 (o más) por luz por intersección

### 3. Rayos de Reflexión (Reflection Rays)

**Propósito**: Simular rebote especular (espejos, metales).

**Origen**: Punto de impacto

**Dirección**: Dirección reflejada según la normal

**Fórmula de reflexión**:
```
R = I - 2(I·N)N

donde:
I = dirección incidente (entrante)
N = normal de la superficie
R = dirección reflejada
```

```python
# Hit en espejo
hit_point = Vec3(3, 0, 0)
normal = Vec3(-1, 0, 0)  # Apunta a la izquierda
incident = Vec3(1, -1, 0).normalize()  # Rayo que llegó

# Calcular reflexión
dot = incident.dot(normal)  # 1*(-1) + (-1)*0 + 0*0 = -1
reflected = incident - 2 * dot * normal
# = Vec3(0.707, -0.707, 0) - 2*(-1)*Vec3(-1,0,0)
# = Vec3(0.707, -0.707, 0) + Vec3(-2, 0, 0)
# = Vec3(-1.293, -0.707, 0)
# ... normalizar

# Rayo reflejado
reflection_ray = Ray(hit_point, reflected.normalize())
```

**Cantidad**: 1 por rebote especular

### 4. Rayos de Refracción (Refraction Rays)

**Propósito**: Simular luz que atraviesa materiales transparentes (vidrio, agua).

**Origen**: Punto de impacto

**Direction**: Dirección refractada según Ley de Snell

**Ley de Snell**:
```
n₁ sin(θ₁) = n₂ sin(θ₂)
```

```python
# Hit en vidrio (IOR = 1.5)
hit_point = Vec3(0, 0, 5)
normal = Vec3(0, 0, -1)  # Apunta hacia nosotros
incident = Vec3(0, 0, 1).normalize()  # Perpendicular

# Refracción (aire n=1.0 → vidrio n=1.5)
refracted = refract(incident, normal, 1.0 / 1.5)

# Rayo refractado
refraction_ray = Ray(hit_point, refracted)
```

**Cantidad**: 1 por rebote refractivo

### 5. Rayos Difusos (Diffuse/Scatter Rays)

**Propósito**: Simular rebote difuso en materiales mate.

**Origen**: Punto de impacto

**Dirección**: Aleatoria en hemisferio orientado por la normal

```python
# Hit en superficie mate
hit_point = Vec3(2, 0, 3)
normal = Vec3(0, 1, 0)  # Apunta arriba

# Dirección aleatoria en hemisferio
random_dir = random_in_hemisphere(normal)

# Rayo difuso
diffuse_ray = Ray(hit_point, random_dir)
```

**Cantidad**: N por rebote difuso (Monte Carlo sampling)

### Comparación de Tipos

| Tipo | Origen | Dirección | Cantidad | Propósito |
|------|--------|-----------|----------|-----------|
| Primario | Cámara | Hacia píxel | 1/píxel | Visión inicial |
| Sombra | Superficie | Hacia luz | 1/luz | Iluminación directa |
| Reflexión | Superficie | Reflejada | 1/rebote | Espejos, metales |
| Refracción | Superficie | Refractada | 1/rebote | Vidrio, agua |
| Difuso | Superficie | Aleatoria | N/rebote | Materiales mate |

---

## Ejemplos Numéricos Completos

### Ejemplo 1: Rayo Horizontal Simple

```python
# Crear rayo que va hacia la derecha
origin = Vec3(0, 0, 0)
direction = Vec3(1, 0, 0)  # Ya normalizado, length=1

ray = Ray(origin, direction)
print(f"Origin: {ray.origin}")      # Vec3(0, 0, 0)
print(f"Direction: {ray.direction}")  # Vec3(1, 0, 0)

# Puntos en el rayo
print(ray.point_at(0))    # Vec3(0, 0, 0) - origen
print(ray.point_at(5))    # Vec3(5, 0, 0) - 5 unidades
print(ray.point_at(10))   # Vec3(10, 0, 0) - 10 unidades
print(ray.point_at(2.5))  # Vec3(2.5, 0, 0) - 2.5 unidades
```

### Ejemplo 2: Rayo Diagonal (Normalización Automática)

```python
# Vector NO normalizado
origin = Vec3(0, 0, 0)
direction = Vec3(3, 4, 0)  # length = 5, NO normalizado

# Crear rayo (normalización automática)
ray = Ray(origin, direction)

# Verificar normalización
print(f"Direction: {ray.direction}")
# Vec3(0.6, 0.8, 0)  ← normalizado!

print(f"Length: {ray.direction.length()}")
# 1.0 ✓

# Punto a distancia 10
point = ray.point_at(10)
# = Vec3(0,0,0) + 10 * Vec3(0.6, 0.8, 0)
# = Vec3(6, 8, 0)

print(f"Point: {point}")
# Vec3(6, 8, 0)

# Verificar distancia
dist = (point - origin).length()
print(f"Distance: {dist}")
# 10.0 ✓ (no 50 que sería sin normalizar)
```

### Ejemplo 3: Rayo de Cámara a Píxel

```python
# Configuración
camera_pos = Vec3(0, 0, -10)
image_width = 800
image_height = 600
viewport_width = 2.0
viewport_height = 1.5

# Píxel específico (centro de la imagen)
pixel_x = 400
pixel_y = 300

# Convertir a coordenadas viewport
u = (pixel_x / image_width) * viewport_width - viewport_width/2
v = (pixel_y / image_height) * viewport_height - viewport_height/2

# u = (400/800) * 2.0 - 1.0 = 1.0 - 1.0 = 0.0
# v = (300/600) * 1.5 - 0.75 = 0.75 - 0.75 = 0.0

# Píxel en el plano de imagen (z=0)
pixel_pos = Vec3(u, v, 0)  # Vec3(0, 0, 0) - centro

# Dirección: de cámara a píxel
direction_vec = pixel_pos - camera_pos
# = Vec3(0,0,0) - Vec3(0,0,-10)
# = Vec3(0, 0, 10)

# Crear rayo (normaliza automáticamente)
camera_ray = Ray(camera_pos, direction_vec)

print(f"Origin: {camera_ray.origin}")
# Vec3(0, 0, -10)

print(f"Direction: {camera_ray.direction}")
# Vec3(0, 0, 1)  ← normalizado de Vec3(0,0,10)

# Este rayo va directo hacia adelante (eje Z)
```

### Ejemplo 4: Intersección con Esfera

```python
# Rayo
ray = Ray(Vec3(0, 0, 0), Vec3(1, 0, 0))

# Esfera (centro y radio)
sphere_center = Vec3(5, 0, 0)
sphere_radius = 2

# (Algoritmo de intersección retornaría t ≈ 3)
t_intersection = 3.0

# Calcular punto de impacto
hit_point = ray.point_at(t_intersection)
print(f"Hit point: {hit_point}")
# Vec3(3, 0, 0)

# Calcular normal en el punto
normal = (hit_point - sphere_center).normalize()
# = (Vec3(3,0,0) - Vec3(5,0,0)).normalize()
# = Vec3(-2, 0, 0).normalize()
# = Vec3(-1, 0, 0)  # Apunta hacia el origen

print(f"Normal: {normal}")
# Vec3(-1, 0, 0)

# Verificar que está en la superficie
distance_to_center = (hit_point - sphere_center).length()
print(f"Distance to center: {distance_to_center}")
# 2.0 ✓ (igual al radio)
```

### Ejemplo 5: Rayo de Sombra

```python
# Superficie golpeada
hit_point = Vec3(3, 1, 2)
normal = Vec3(0, 1, 0)  # Apunta arriba

# Luz puntual
light_pos = Vec3(10, 10, 8)

# Calcular dirección hacia la luz
to_light = light_pos - hit_point
# = Vec3(10,10,8) - Vec3(3,1,2)
# = Vec3(7, 9, 6)

distance_to_light = to_light.length()
# = sqrt(7² + 9² + 6²)
# = sqrt(49 + 81 + 36)
# = sqrt(166)
# ≈ 12.88 unidades

# Rayo de sombra (origen ligeramente elevado)
shadow_origin = hit_point + 0.001 * normal
# = Vec3(3, 1, 2) + 0.001 * Vec3(0,1,0)
# = Vec3(3, 1.001, 2)

shadow_ray = Ray(shadow_origin, to_light)

print(f"Shadow ray origin: {shadow_ray.origin}")
# Vec3(3, 1.001, 2)

print(f"Shadow ray direction: {shadow_ray.direction}")
# Vec3(7,9,6).normalize() = Vec3(0.543, 0.699, 0.466)

# Ahora buscaríamos intersecciones en rango [t_min, distance_to_light]
# Si hay hit antes de llegar a la luz → sombra
# Si no hay hit → iluminado
```

---

## Aplicaciones Prácticas

### 1. Generación de Rayos de Cámara

```python
def generate_camera_ray(pixel_x, pixel_y, camera, viewport):
    """Genera rayo desde cámara a través de un píxel."""
    
    # Convertir píxel a coordenadas viewport
    u = (pixel_x / viewport.width) * viewport.aspect_ratio
    v = (pixel_y / viewport.height)
    
    # Punto en el plano de imagen
    pixel_world = camera.lower_left_corner + u*camera.horizontal + v*camera.vertical
    
    # Dirección: de cámara a píxel
    direction = pixel_world - camera.origin
    
    # Crear rayo (normaliza automáticamente)
    return Ray(camera.origin, direction)
```

### 2. Test de Visibilidad (Rayos de Sombra)

```python
def is_in_shadow(hit_point, normal, light_pos, scene):
    """Verifica si un punto está en sombra respecto a una luz."""
    
    # Elevar ligeramente para evitar shadow acne
    origin = hit_point + 0.001 * normal
    
    # Dirección hacia la luz
    to_light = light_pos - origin
    distance = to_light.length()
    
    # Rayo de sombra
    shadow_ray = Ray(origin to_light)
    
    # Buscar intersecciones
    hit = scene.hit(shadow_ray, 0.001, distance)
    
    # Si hay hit antes de la luz → en sombra
    return hit is not None
```

### 3. Path Tracing Recursivo

```python
def trace_ray(ray, scene, depth):
    """Traza un rayo recursivamente (path tracing simplificado)."""
    
    # Caso base: profundidad máxima
    if depth <= 0:
        return Vec3(0, 0, 0)  # Negro
    
    # Buscar intersección
    hit = scene.hit(ray, 0.001, float('inf'))
    
    if hit is None:
        # No golpeó nada → retornar color de cielo
        return sky_color(ray.direction)
    
    # Luz emitida por el objeto
    emitted = hit.emission
    
    # Si es fuente de luz, retornar emisión directamente
    if emitted.length() > 0:
        return emitted
    
    # Generar rayo de rebote (difuso, especular, o refracción)
    bounce_ray = scatter(ray, hit)
    
    # Recursión: trazar rayo de rebote
    incoming_light = trace_ray(bounce_ray, scene, depth - 1)
    
    # Aplicar BRDF y retornar
    return emitted + hit.color * incoming_light
```

### 4. Antialiasing con Múltiples Rayos

```python
def render_pixel_with_aa(pixel_x, pixel_y, samples_per_pixel):
    """Renderiza un píxel con antialiasing."""
    
    color_sum = Vec3(0, 0, 0)
    
    # Múltiples muestras por píxel
    for _ in range(samples_per_pixel):
        # Offset aleatorio dentro del píxel
        offset_x = pixel_x + random.random()
        offset_y = pixel_y + random.random()
        
        # Generar rayo
        ray = generate_camera_ray(offset_x, offset_y)
        
        # Trazar rayo
        color = trace_ray(ray, scene, max_depth=10)
        
        # Acumular
        color_sum += color
    
    # Promediar
    return color_sum / samples_per_pixel
```

---

## Optimizaciones y Consideraciones

### 1. Costo Computacional

**`point_at(t)` es extremadamente barato**:

```python
# Operaciones por llamada:
# - 3 multiplicaciones (t * direction.x/y/z)
# - 3 sumas (origin.x/y/z + temp.x/y/z)
# Total: 6 operaciones aritméticas básicas

# Comparación:
# - Normalización: ~15 operaciones (sqrt, 3 divisiones, etc.)
# - Intersección esfera: ~30 operaciones
# - Intersección triángulo: ~40 operaciones

# point_at es ≈5× más rápido que normalizar
```

**Optimización**: Como `point_at` es tan barato, NO hay necesidad de cachear resultados.

### 2. Normalización: Una Vez por Rayo

La dirección se normaliza **una sola vez** en `__init__`:

```python
ray = Ray(origin, direction)
# ↑ Normaliza UNA vez

# Luego puedes llamar point_at millones de veces
for t in range(1000000):
    point = ray.point_at(t)  # Sin costo de normalización
```

**Beneficio**: Pagas el costo de normalización una vez, obtienes beneficios siempre.

### 3. Precisión Numérica

**Errores de punto flotante** pueden acumularse:

```python
# Después de muchas operaciones
hit_point = ray.point_at(1234567.89)
# Puede tener pequeños errores (~10⁻⁶)

# Solución: usar t_min para evitar shadow acne
shadow_ray = Ray(hit_point + 0.001*normal, direction)
```

### 4. Vectores Cero

**PRECAUCIÓN**: No crear rayos con dirección cero:

```python
# ¡MAL!
zero_dir = Vec3(0, 0, 0)
ray = Ray(origin, zero_dir)
# Normalizar (0,0,0) → división por cero ❌

# SOLUCIÓN: Validar antes
if direction.length() > 1e-8:
    ray = Ray(origin, direction)  # ✓
else:
    # Manejar error
    raise ValueError("Direction cannot be zero")
```

### 5. Memoria y Cache

**Tamaño de un Ray**:
```
origin: Vec3 = 3 floats × 4 bytes = 12 bytes
direction: Vec3 = 3 floats × 4 bytes = 12 bytes
Total: 24 bytes por rayo
```

**Para 1 millón de rayos**: 24 MB (muy compacto)

**Localidad de cache**: Los rayos suelen accederse secuencialmente → buen rendimiento de cache.

### 6. Inmutabilidad

Los rayos en este sistema son **inmutables** (origin y direction no cambian después de creación):

**Ventajas**:
- Thread-safe (pueden compartirse entre hilos)
- Predictibilidad (no hay efectos secundarios)
- Debugging más fácil

**Crear nuevo rayo para cada rebote**:
```python
# Rebote
new_origin = hit_point
new_direction = reflected_dir
bounce_ray = Ray(new_origin, new_direction)  # Nuevo objeto
```

---

## Resumen

### Puntos Clave

1. **Ecuación paramétrica**: `P(t) = O + t*D` es la base de todo
2. **Normalización**: CRITICA para que t represente distancia real
3. **point_at(t)**: Método fundamental, muy eficiente (O(1))
4. **Tipos de rayos**: Primarios, sombra, reflexión, refracción, difusos
5. **Inmutabilidad**: Cada rebote crea un NUEVO rayo

### Valores Importantes

| Parámetro | Rango Típico | Propósito |
|-----------|--------------|-----------|
| t | 0.001 - ∞ | Distancia a lo largo del rayo |
| t_min | 0.001 | Evitar shadow acne |
| t_max | ∞ | Límite de búsqueda |
| direction.length() | 1.0 | SIEMPRE normalizado |

### Efecto de Valores

**t (parámetro de distancia)**:
- `t ↑` → Punto más lejos del origen
- `t ↓` → Punto más cerca del origen
- `t < t_min` → Ignorado (shadow acne)

**Normalización**:
- `SIN normalizar` → t NO representa distancia (❌ MALO)
- `CON normalizar` → t = distancia real (✓ CORRECTO)

### Relación con Otros Componentes

```
Ray + Geometry → Intersecciones (geometry.py)
                     ↓
                 HitRecord
                     ↓
              Cálculo de color
                     ↓
            Nuevos rayos (rebotes)
                     ↓
              Recursión (path tracing)
```

El **Ray** es el "mensajero" que conecta TODOS los componentes del ray tracer.

---

## Conclusión

A pesar de ser una clase muy simple (solo ~100 líneas con documentación), el `Ray` es **absolutamente fundamental** para ray tracing:

- **Sin Ray → Sin Ray Tracing**
- Es la abstracción matemática de "un rayo de luz viajando"
- Usado MILLONES de veces por frame
- Extremadamente optimizado (24 bytes, operaciones O(1))

**Comparable a**:
- Un píxel en rasterización (unidad fundamental)
- Un fotón en simulación física
- Un paquete en networking

La simplicidad del `Ray` es su fortaleza: hace UNA cosa (representar una semirecta), y la hace PERFECTAMENTE.
