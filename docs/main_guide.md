# Guía Completa de main.py: Path Tracer Cornell Box

## Tabla de Contenidos

1. [Introducción](#introducción)
2. [Arquitectura del System

a](#arquitectura-del-sistema)
3. [Física de la Luz - Ley de Snell y Fresnel](#física-de-la-luz)
4. [Next Event Estimation (NEE)](#next-event-estimation-nee)
5. [Path Tracing Recursivo](#path-tracing-recursivo)
6. [Materiales](#materiales)
7. [Sistema de Cámara](#sistema-de-cámara)
8. [Renderizado Paralelo](#renderizado-paralelo)
9. [Cornell Box - La Escena](#cornell-box)
10. [Optimización y Configuración](#optimización-y-configuración)

---

## Introducción

### ¿Qué es Path Tracing?

**Path Tracing** es un algoritmo de renderizado que simula el transporte de luz mediante Monte Carlo integration para resolver la **ecuación de renderizado**:

```
L_out(x, ω_out) = L_e(x, ω_out) + ∫_Ω f_r(x, ω_in, ω_out) L_in(x, ω_in) cos(θ) dω_in
```

**Componentes**:
- **L_out**: Luz saliente (lo que vemos)
- **L_e**: Luz emitida (si el objeto es una luz)
- **f_r**: BRDF (función de reflexión bidireccional del material)
- **L_in**: Luz entrante (recursión)
- **Ω**: Hemisferio sobre la superficie
- **θ**: Ángulo entre dirección entrante y normal

Esta integral es **imposible resolver analíticamente** para escenas complejas.

### Solución: Monte Carlo

Aproximamos la integral con muestras aleatorias:

```
L_out ≈ (1/N) Σ [f_r(x, ω_i, ω_out) × L_in(x, ω_i) × cos(θ_i)]
```

donde ω_i son direcciones aleatorias en el hemisferio.

**Características**:
- ✅ Físicamente correcto (dado suficientes muestras)
- ✅ Todos los efectos de iluminación (caustics, color bleeding, soft shadows)
- ❌ Convergencia lenta (Error ∝ 1/√N)
- ❌ Requiere muchas muestras (100-10,000 por píxel)

### Técnicas Implementadas en este Renderer

| Técnica | Propósito | Beneficio |
|---------|-----------|-----------|
| **Path Tracing** | Resolver ecuación de renderizado | Físicamente correcto |
| **NEE** | Muestrear luces directament

e | 10-100× menos ruido |
| **BVH** | Acelerar intersecciones | 1000-10,000× más rápido |
| **Stratified Sampling** | Antialiasing mejorado | Menos ruido |
| **Fresnel-Schlick** | Reflexión/refracción realista | Vidrio/metales correctos |
| **Depth of Field** | Desenfoquenatural | Bokeh realista |
| **Parallel Rendering** | Usar todos los CPU cores | 6-8× más rápido |

---

## Arquitectura del Sistema

### Flujo General

```
main.py
  ↓
render()  ← Función principal
  ├─ Configurar cámara (FOV, DOF, posición)
  ├─ Cargar escena: render_obj() → BVH + lights
  ├─ render_row() en paralelo para cada fila
  │   └─ Para cada píxel:
  │       └─ Para cada muestra:
  │           └─ color_ray() recursivo
  │               ├─ Encontrar intersección (BVH)
  │               ├─ Si luz: retornar emisión
  │               ├─ Si metal: reflejar
  │               ├─ Si vidrio: refractar/reflejar (Fresnel)
  │               └─ Si difuso:
  │                   ├─ NEE: calculate_nee()
  │                   └─ Rebote: recursión
  └─ Guardar imagen (gamma correction, flip)
```

### Módulos y Dependencias

```python
# Importaciones clave
from src.vector import Vec3        # Vectores 3D
from src.ray import Ray            # Rayos
from src.geometry import (
    Sphere, Quad, BVHNode          # Geometría
)
from src.utils import (
    generar_direccion_aleatoria,   # Hemisferio
    load_obj,                       # Modelos 3D
    random_in_unit_disk            # DOF
)
import multiprocessing             # Paralelo
```

### Estructura de Archivos

```
rtx-cornell_box/
  ├─ main.py                    ← ESTE ARCHIVO
  ├─ src/
  │   ├─ vector.py             # Vec3
  │   ├─ ray.py                # Ray
  │   ├─ geometry.py           # Sphere, Quad, Triangle, BVH
  │   └─ utils.py              # Utilidades
  ├─ models/
  │   └─ bunny.obj             # Modelo 3D
  └─ output/
      └─ bokeh.png             # Resultado
```

---

## Física de la Luz

### Ley de Snell - Refracción

Cuando la luz pasa de un medio a otro (aire ↔ vidrio), se **desvía**:

```
       Aire (η₁ = 1.0)
    -------------------  Interfaz
       Vidrio (η₂ = 1.5)

        Normal ↑
        |  
        |   θ₁ (incidencia)
        | ↗
Rayo ---|
        | ↘
        |   θ₂ (refracción)
```

**Ley de Snell**:
```
η₁ sin(θ₁) = η₂ sin(θ₂)
```

**Casos**:

1. **Mayor IOR** (aire → vidrio, η₁ < η₂):
   ```
   θ₂ < θ₁  (ángulo se REDUCE, se acerca a normal)
   
   Ejemplo:
   θ₁ = 45° en aire (η₁=1.0) → vidrio (η₂=1.5)
   sin(θ₂) = (1.0/1.5) × sin(45°) = 0.471
   θ₂ = 28.1°  ← Más cerrado ✓
   ```

2. **Menor IOR** (vidrio → aire, η₁ > η₂):
   ```
   θ₂ > θ₁  (ángulo AUMENTA, se aleja de normal)
   
   Ejemplo:
   θ₁ = 30° en vidrio (η₁=1.5) → aire (η₂=1.0)
   sin(θ₂) = (1.5/1.0) × sin(30°) = 0.75
   θ₂ = 48.6°  ← Más abierto ✓
   ```

3. **Reflexión Total Interna** (η₁ > η₂, θ₁ grande):
   ```
   Si η₁ sin(θ₁) > η₂ → sin(θ₂) > 1  ← IMPOSIBLE
   
   → No hay refracción, solo REFLEXIÓN
   
   Ejemplo:
   θ₁ = 50° en vidrio (η₁=1.5) → aire (η₂=1.0)
   sin(θ₂) = 1.5 × sin(50°) = 1.15 > 1  ❌
   
   → Reflexión total interna ✓
   ```

**Ángulo crítico**:
```
θ_crítico = arcsin(η₂ / η₁)

Para vidrio→aire:
θ_crítico = arcsin(1.0 / 1.5) = 41.8°

θ > 41.8° → reflexión total
θ < 41.8° → refracción posible
```

### Implementación Vectorial - `refract()`

```python
def refract(uv, n, etai_over_etat):
    # etai_over_etat = η₁/η₂
    
    cos_theta = min((uv * -1).dot(n), 1.0)
    
    # Componente perpendicular
    r_perp = (uv + n * cos_theta) * etai_over_etat
    
    # Componente paralela
    r_parallel = n * -sqrt(1.0 - r_perp.length()²)
    
    return r_perp + r_parallel
```

**Derivación**:

```
Vector refractado r = r_perp + r_parallel

Paso 1: Descomponer en paralelo y perpendicular a normal

uv = uv_perp + uv_parallel

Paso 2: Aplicar Ley de Snell a componente perpendicular

r_perp = (uv + n × cos(θ₁)) × (η₁/η₂)

Paso 3: Calcular componente paralela

|r_perp|² + |r_parallel|² = 1  (rayo normalizado)

|r_parallel| = sqrt(1 - |r_perp|²)
```

**Ejemplo Numérico**:
```python
# Rayo 45° entrando al vidrio
uv = Vec3(0.707, -0.707, 0).normalize()
n = Vec3(0, 1, 0)
ratio = 1.0 / 1.5  # aire → vidrio

refracted = refract(uv, n, ratio)
# ≈ Vec3(0.471, -0.882, 0)

# Verificar ángulo:
cos_theta = abs(refracted.dot(n))
theta = acos(cos_theta)
# ≈ 28.1° ✓ (menos que 45°, correcto)
```

### Fresnel - ¿Reflejar o Refractar?

En la realidad, **AMBOS** ocurren simultáneamente:
- Parte de la luz se **refleja**
- Parte se **refracta**

**Proporción** depende del ángulo (Ecuaciones de Fresnel).

**Aproximación de Schlick** - Mucho más simple:

```
R(θ) = R₀ + (1 - R₀)(1 - cos(θ))⁵

donde:
R₀ = ((η₁ - η₂)/(η₁ + η₂))²  (reflectancia normal)
```

**Implementación - `reflectance()`**:

```python
def reflectance(cosine, ref_idx):
    r0 = (1 - ref_idx) / (1 + ref_idx)
    r0 = r0 * r0
    return r0 + (1 - r0) * pow(1 - cosine, 5)
```

**Casos Numéricos**:

```python
# Vidrio (IOR 1.5) desde aire
ref_idx = 1.0 / 1.5 = 0.667

# 1. Perpendicular (θ=0°, cos=1.0):
R₀ = ((1-0.667)/(1+0.667))² = (0.333/1.667)² ≈ 0.04
R(0°) = 0.04 + 0 = 0.04
→ 4% reflectado, 96% refractado ✓

# 2. 45° (cos≈0.707):
R(45°) = 0.04 + (1-0.04)(1-0.707)⁵
       = 0.04 + 0.96 × 0.0025
       ≈ 0.042
→ 4.2% reflectado ✓

# 3. Rasante (θ=90°, cos=0):
R(90°) = 0.04 + (1-0.04)(1-0)⁵
       = 0.04 + 0.96 × 1
       = 1.0
→ 100% reflectado ✓ (Brewster)
```

**Uso en Path Tracing**:

```python
# Probabilísticamente elegir
R = reflectance(cos_theta, ratio)

if random() < R:
    # Reflejar (probabilidad R)
    direction = ray.reflect(normal)
else:
    # Refractar (probabilidad 1-R)
    direction = refract(ray, normal, ratio)
```

**Efecto Visual**:

```
Vidrio visto:
- Perpendicular: ~96% transparente (casi invisible)
- 45°: ~96% transparente
- Rasante (mirando de lado): ~0% transparente (espejo)

Esto simula cómo el vidrio se ve más reflectivo en los bordes ✓
```

---

## Next Event Estimation (NEE)

### El Problema: Path Tracing Naïve

**Sin NEE** (path tracing puro):

```
Para cada rebote:
    1. Dirección aleatoria en hemisferio
    2. Lanzar rayo
    3. Si golpea luz → CONTRIBUYE
    4. Si no → recursión

Problema: Probabilidad de golpear luz = Ω_luz / (2π)

Cornell Box:
- Luz: 130×105 = 13,650 unidades²
- Hemisferio: 2π ster

P(golpear) = 13,650 / (área proyectada total)
           ≈ 1/500 a 1/1000

→ ~999 de cada 1000 rayos NO encuentran la luz
→ Imagen MUY RUIDOSA
```

**Visualización**:

```
Path Tracing Puro (100 muestras):
████████▓▓▓▓▓▒▒▒▒▒░░░░░  ← MUY ruidoso
Ruido visible, pixelado

Con NEE (100 muestras):
████████▓▓▓▓▒▒░░  ← Más limpio
Ruido reducido ~10×
```

### La Solución: NEE

**Con NEE**:

```
Para cada rebote difuso:
    A. NEE - Luz Directa:
        1. Muestrear DIRECTAMENTE punto en cada luz
        2. Shadow ray → verificar visibilidad
        3. Si visible → CONTRIBUYE (SIEMPRE si no ocluida)
    
    B. Rebote Difuso - Luz Indirecta:
        1. Dirección aleatoria
        2. Recursión (NO sumar luz si golpea)
```

**Ventaja**:
- Luz directa: ~100% muestras contribuyen (si no ocluida)
- vs Puro: ~0.1% contribuyen

**Reducción de ruido**: ~10-100× para mismas muestras.

### Implementación - `calculate_nee()`

```python
def calculate_nee(rec, world, lights):
    direct_light = Vec3(0, 0, 0)
    
    for light in lights:
        # 1. Muestrear punto en luz
        l_dir, solid_angle = light.sample_solid_angle(rec.point)
        
        # 2. Shadow ray
        shadow_ray = Ray(rec.point + rec.normal * 0.001, l_dir)
        distancia = (light.center - rec.point).length()
        
        # 3. Test de visibilidad
        h = world.hit(shadow_ray, 0.001, distancia - 0.001)
        
        # 4. Si visible Y es la luz
        if h and h.obj_ref == light:
            cos_theta = max(0, rec.normal.dot(l_dir))
            
            # 5. Contribución
            direct_light += (
                light.emission * rec.color * 
                (cos_theta * solid_angle / π)
            )
    
    return direct_light
```

### Fórmula de Rendering

```
L_direct = Σ_luces [ E_luz × ρ_superficie × cos(θ) × (Ω / π) ]

donde:
- E_luz: Emisión de la luz (RGB)
- ρ_superficie: Albedo/color superficie
- cos(θ): normal · dirección_luz (Lambert)
- Ω: Ángulo sólido subtendido por la luz
- π: Normalización BRDF Lambertiano
```

**Ángulo Sólido**:

```
Ω = (Área_luz × cos(θ_luz)) / distancia²

donde:
- Área_luz: Área física de la luz
- cos(θ_luz): Cuánto "mira" la luz al punto
- distancia: Entre superficie y luz
```

**Ejemplo Numérico Completo**:

```python
# Configuración
superficie_point = Vec3(278, 100, 278)  # Piso
superficie_normal = Vec3(0, 1, 0)       # Arriba
superficie_albedo = Vec3(0.73, 0.73, 0.73)  # Blanco

luz_center = Vec3(278, 554, 278)  # Techo
luz_emission = Vec3(40, 40, 40)
luz_area = 130 × 105 = 13,650

# Cálculos
distancia = (luz_center - superficie_point).length()
         = (0, 454, 0).length()
         = 454

direccion = (luz_center - superficie_point).normalize()
         = Vec3(0, 1, 0)

cos_theta_superficie = superficie_normal.dot(direccion)
                     = 1.0  (perpendicular perfecto)

cos_theta_luz = 1.0  (luz apunta abajo)

solid_angle = (13,650 × 1.0) / (454²)
            = 13,650 / 206,116
            ≈ 0.0662 steradianes

# Contribución final
L_direct = Vec3(40,40,40) × Vec3(0.73,0.73,0.73) × 1.0 × (0.0662 / π)
         = Vec3(29.2,29.2,29.2) × 0.0211
         ≈ Vec3(0.616, 0.616, 0.616)
```

### Shadow Acne Prevention

**Problema**:

```
Offset = 0:
    shadow_ray.origin = rec.point (EXACTAMENTE en superficie)
    
    → BVH puede encontrar la MISMA superficie  
    → "Sombra" en sí mismo (shadow acne)
```

**Solución**:

```python
shadow_origin = rec.point + rec.normal * 0.001

Offset pequeño (~0.001) en dirección de normal
→ Rayo comienza FUERA de la superficie ✓
```

**Valores típicos**:
- 0.001: Cornell Box (555 unidades)
- 0.0001: Escenas pequeñas (<10 unidades)
-0.01: Escenas grandes (>10,000 unidades)

---

## Path Tracing Recursivo

### Algoritmo - `color_ray()`

```python
def color_ray(ray, world, lights, depth, puede_ver_luz=True):
    # Caso base 1: Profundidad máxima
    if depth <= 0:
        return Vec3(0, 0, 0)  # Negro (terminar)
    
    # Encontrar intersección más cercana (BVH)
    hit = world.hit(ray, 0.001, inf)
    
    # Caso base 2: No hit
    if not hit:
        return sky_color()  # Ambiente/cielo
    
    # CASO 1: Emisor (luz)
    if hit.emission.length() > 0:
        return hit.emission if puede_ver_luz else Vec3(0,0,0)
    
    # CASO 2: Metal
    if hit.is_metal:
        reflected = ray.direction.reflect(hit.normal)
        fuzzed = reflected + fuzz * random_in_unit_sphere()
        
        if fuzzed.dot(hit.normal) > 0:  # No dentro
            scattered = Ray(hit.point + 0.001*hit.normal, fuzzed)
            return color_ray(scattered, ..., depth-1, True) * color
        else:
            return Vec3(0,0,0)  # Absorción
    
    # CASO 3: Vidrio
    elif hit.is_dielectric:
        # Determinar entrada/salida
        front_face = ray.direction.dot(hit.normal) < 0
        ratio = (1.0/IOR) if front_face else IOR
        normal = hit.normal if front_face else -hit.normal
        
        # Fresnel
        cos_theta = min((-ray.direction).dot(normal), 1.0)
        sin_theta = sqrt(1 - cos_theta²)
        
        cannot_refract = ratio * sin_theta > 1.0
        
        if cannot_refract or reflectance(cos_theta, ratio) > random():
            direction = ray.direction.reflect(normal)  # Reflejar
        else:
            direction = refractrayo.direction, normal, ratio)  # Refractar
        
        scattered = Ray(hit.point + 0.001*direction, direction)
        return color_ray(scattered, ..., depth-1, True) * color
    
    # CASO 4: Difuso (Lambertian)
    else:
        # A. NEE - Luz directa
        direct = calculate_nee(hit, world, lights)
        
        # B. Rebote - Luz indirecta
        random_dir = generar_direccion_aleatoria(hit.normal)
        bounced = Ray(hit.point + 0.001*hit.normal, random_dir)
        indirect = color_ray(bounced, ..., depth-1, False) * hit.color
        
        return direct + indirect
```

### Árbol de Recursión - Ejemplo

```
Cámara → Rayo1 (depth=4)
           ↓
      Esfera ROJA difusa
           ↓
       NEE: Vec3(0.5, 0, 0)  ← Luz directa roja
           ↓
       Rebote: Rayo2 (depth=3) dirección aleatoria
           ↓
      Pared BLANCA difusa
           ↓
       NEE: Vec3(0.3, 0.3, 0.3)  ← Luz directa blanca
           ↓
       Rebote: Rayo3 (depth=2)
           ↓
      Pared VERDE difusa
           ↓
       NEE: Vec3(0, 0.2, 0)  ← Luz directa verde
           ↓
       Rebote: Rayo4 (depth=1)
           ↓
      Techo
           ↓
       NEE: Vec3(0.1, 0.1, 0.1)
           ↓
       Rebote: Rayo5 (depth=0)
           ↓
       return Vec3(0,0,0)  ← CASO BASE

# Desenrollar recursión:
R4 color = Vec3(0.1,0.1,0.1) + Vec3(0,0,0)*blanco = Vec3(0.1,0.1,0.1)

R3 color = Vec3(0,0.2,0) + Vec3(0.1,0.1,0.1)*verde
         = Vec3(0,0.2,0) + Vec3(0.012,0.045,0.015)
         = Vec3(0.012, 0.245, 0.015)

R2 color = Vec3(0.3,0.3,0.3) + Vec3(0.012,0.245,0.015)*blanco
         = Vec3(0.3,0.3,0.3) + Vec3(0.009,0.179,0.011)
         = Vec3(0.309, 0.479, 0.311)

R1 color = Vec3(0.5,0,0) + Vec3(0.309,0.479,0.311)*rojo
         = Vec3(0.5,0,0) + Vec3(0.201,0.031,0.020)
         = Vec3(0.701, 0.031, 0.020)  ← FINAL

Pixel final ≈ ROJO con tinte de iluminación indirecta ✓
```

### Parámetro `puede_ver_luz`

**Propósito**: Evitar double-counting de luces.

```
puede_ver_luz = True:
    - Rayos especulares (metales, vidrio)
    - Rayos primarios (cámara)
    → Pueden "ver" las luces directamente
    if hit.emission > 0: return emission  ✓

puede_ver_luz = False:
    - Rayos difusos (después de NEE)
    → NO pueden ver luces directamente
    if hit.emission > 0: return Vec3(0,0,0)  ✓
    
    ¿Por qué? Ya contamos luz con NEE
    Contarla de nuevo → 2× brillo (INCORRECTO)
```

**Ejemplo sin `puede_ver_luz`**:

```
# SIN flag (INCORRECTO):
Superficie difusa:
    L = NEE(luz) + Rebote_que_golpea_luz(emisión)
      = 1.0      + 0.001 × 40
      = 1.0      + 0.04
      = 1.04  ← Luz contada 2 veces ❌

# CON flag (CORRECTO):
Superficie difusa:
    L = NEE(luz) + Rebote(0, ya que puede_ver_luz=False)
      = 1.0      + 0
      = 1.0  ✓
```

---

## Materiales

### 1. Difuso (Lambertian)

**BRDF**:
```
f_r = ρ / π

donde:
- ρ: Albedo (color) de la superficie
- π: Normalización (integral sobre hemisferio = 1)
```

**Scatter**: Dirección aleatoria en hemisferio.

**Código**:
```python
# NEE
direct = calculate_nee(hit, world, lights)

# Rebote
random_dir = generar_direccion_aleatoria(hit.normal)
bounced = Ray(hit.point + 0.001*hit.normal, random_dir)
indirect = color_ray(bounced, ..., depth-1, False) * hit.color

return direct + indirect
```

**Propiedades**:
- Mate (no brillante)
- Color bleeding (refleja color a objetos cercanos)
- Iluminación indirecta importante

**Ejemplos visuales**:
- Paredes Cornell Box (rojo, verde, blanco)
- Yeso, papel, tela

### 2. Metal (Reflexión Especular)

**BRDF**: Casi delta (direccional pura).

**Reflexión**:
```
R = I - 2(I · N)N

donde:
- I: Dirección incidente
- N: Normal
- R: Dirección reflejada
```

**Rugosidad (fuzz)**:

```
perfect_reflection = I.reflect(N)

fuzzed_reflection = perfect_reflection + fuzz * random_in_unit_sphere()

fuzz = 0.0: Espejo perfect (sin perturbación)
fuzz = 0.1: Ligeramente difuso
fuzz = 0.5: Muy difuso (casi Lambertian)
```

**Código**:
```python
reflected = ray.direction.reflect(hit.normal)
perturb = generar_direccion_aleatoria(hit.normal) * fuzz
final = (reflected + perturb).normalize()

if final.dot(hit.normal) > 0:  # Válido
    scattered = Ray(hit.point + 0.001*normal, final)
    return color_ray(scattered, ..., depth-1, True) * color
else:
    return Vec3(0,0,0)  # Absorbido
```

**Ejemplos visuales**:
- fuzz=0.0: Espejo, cromo pulido
- fuzz=0.2: Acero inoxidable
- fuzz=0.5: Aluminio opaco

### 3. Dieléctrico (Vidrio)

**Fenómenos**:
1. **Refracción**: Ley de Snell
2. **Reflexión**: Fresnel (proporcional al ángulo)
3. **Reflexión Total Interna**: Ángulos grandes

**Código** (simplificado):
```python
# Determinar si entra o sale
front_face = ray.dot(normal) < 0
ratio = (1.0/IOR) if front_face else IOR
actual_normal = normal if front_face else -normal

# Fresnel
cos_theta = min((-ray).dot(actual_normal), 1.0)
sin_theta = sqrt(1 - cos_theta²)
cannot_refract = ratio * sin_theta > 1.0

# Decidir
if cannot_refract or reflectance(cos_theta, ratio) > random():
    direction = ray.reflect(actual_normal)  # Reflejar
else:
    direction = refract(ray, actual_normal, ratio)  # Refractar

# Continuar
scattered = Ray(hit.point + 0.001*direction, direction)
return color_ray(scattered,..., depth-1, True) * color
```

**IOR típicos**:
- Aire: 1.0
- Agua: 1.33
- Vidrio: 1.5-1.6
- Diamante: 2.42

**Efectos visuales**:
- Caustics (luz focalizada)
- Distorsión (objetos detrás)
- Reflexión en bordes (Fresnel)

---

## Sistema de Cámara

### Proyección Perspectiva

**Configuración**:

```python
camera_origin = Vec3(278, 278, -800)  # Posición cámara
lookat = Vec3(278, 278, 278)          # Punto objetivo
vup = Vec3(0, 1, 0)                   # Vector "arriba" mundial
fov = 40.0                             # Field of view (grados)
```

**Base Ortonormal** {u, v, w}:

```python
# w: Atrás (opuesto a viewing direction)
w = (origin - lookat).normalize()

# u: Derecha (perpendicular a w y vup)
u = vup.cross(w).normalize()

# v: Arriba (perpendicular a u y w, recomputado)
v = w.cross(u)

Resultado:
      v ↑
        |
  w ←---• camera
       /
      u →
```

**Viewport**:

```python
# Dimensiones basadas en FOV
theta = radians(fov)
h = tan(theta / 2)
viewport_height = 2.0 * h
viewport_width = aspect_ratio * viewport_height

# Escalado por distancia de enfoque
dist_to_focus = (origin - lookat).length()
horizontal = u * viewport_width * dist_to_focus
vertical = v * viewport_height * dist_to_focus

# Esquina inferior izquierda
lower_left = origin - horizontal/2 - vertical/2 - w*dist_to_focus
```

**Visualización**:

```
Cámara en (0,0,-800), mirando +Z:

       Viewport (plano en z=dist_to_focus)
        +-------+-------+
        |       |       |  viewport_height
        |   •   |   •   |  
        +-------+-------+
        lower_left   → horizontal
        ↑
        vertical
        
                •  camera (origin)
```

### Generación de Rayos

**Sin DOF** (pinhole):

```python
# Coordenadas normalizadas [0,1]
s = (x + 0.5) / width   # Horizontal
t = (y + 0.5) / height  # Vertical

# Punto en viewport
viewport_point = lower_left + horizontal*s + vertical*t

# Dirección
direction = (viewport_point - origin).normalize()

# Rayo
ray = Ray(origin, direction)
```

**Con DOF** (lente con apertura):

```python
# 1. Offset aleatorio en lente
lens_sample = random_in_unit_disk() * lens_radius
offset = u * lens_sample.x + v * lens_sample.y

# 2. Nuevo origen (punto en lente)
new_origin = origin + offset

# 3. Dirección hacia mismo punto focal
viewport_point = lower_left + horizontal*s + vertical*t
direction = (viewport_point - new_origin).normalize()

# 4. Rayo con DOF
ray = Ray(new_origin, direction)
```

**Efecto de Lens Radius**:

```
lens_radius = 0.0:
    Todos los rayos desde mismo origen
    → Todo enfocado (pinhole)

lens_radius = 10.0:
    Rayos desde diferentes puntos en lente
    → Objetos en plano focal: nítidos
    → Objetos fuera: borrosos (bokeh)

Regla:
    blur_amount ∝ lens_radius × |distance - focal_distance|
```

### Field of View (FOV)

**Efecto del FOV**:

```
FOV pequeño (20-40°):
    - Vista "telefoto"
    - Objetos grandes, poco contexto
    - Menos distorsión en bordes

FOV normal (50-70°):
    - Similar a ojo humano
    - Balance contexto/detalle

FOV grande (90-120°):
    - Vista "gran angular"
    - Mucho contexto, objetos pequeños
    - Distorsión en bordes (fishye a >120°)
```

**Cálculo viewport**:

```
fov = 90°:
    theta = π/2
    h = tan(π/4) = 1.0
    viewport_height = 2.0
    
fov = 40°:
    theta = 0.698 rad
    h = tan(0.349) = 0.364
    viewport_height = 0.728  ← Más pequeño
```

---

## Renderizado Paralelo

### Arquitectura

```python
if USE_PARALLEL:
    # Usar multiprocessing.Pool
    num_cores = cpu_count()  # Ej: 8
    
    worker_func = partial(render_row, width=..., height=..., ...)
    
    with Pool(processes=num_cores) as pool:
        results = pool.map(worker_func, range(height))
    
    # results[i] contiene fila i
    data = np.array(results, dtype=uint8)
else:
    # Secuencial (debugging)
    for y in range(height):
        row = render_row(y, ...)
```

### Speedup

**Ley de Amdahl**:

```
Speedup = 1 / (Parallel_fraction/N + Serial_fraction)

Path tracing: ~99% paralelizable

N=8 cores:
    Speedup = 1 / (0.99/8 + 0.01)
            = 1 / (0.124 + 0.01)
            = 1 / 0.134
            ≈ 7.46×  (de 8× teórico)
```

**Overhead**:
- Comunicación inter-proceso: ~5-10%
- Creación de procesos: Amortizado (una vez)
- Serialización de datos: Mínimo (solo resultados)

**Scaling**:

| Cores | Speedup Teórico | Speedup Real | Eficiencia |
|-------|-----------------|--------------|------------|
| 1 | 1.0× | 1.0× | 100% |
| 2 | 2.0× | 1.9× | 95% |
| 4 | 4.0× | 3.7× | 93% |
| 8 | 8.0× | 7.2× | 90% |
| 16 | 16.0× | 13.5× | 84% |

### Granularidad

**Una fila entera** por trabajo:

```
Ventajas:
- Suficiente trabajo para amortizar overhead
- Balanceo automático (no todas las filas tardan igual)

Desventajas:
- Si height pequeño (ej: 100), puede desequilibrarse al final
```

**Alternativas**:

```python
# Tiles (bloques 64×64):
for tile_y in range(0, height, 64):
    for tile_x in range(0, width, 64):
        render_tile(tile_x, tile_y, 64, 64)

Ventaja: Mejor balanceo
Desventaja: Más overhead
```

---

## Cornell Box

### Historia

Creada en **1984** por Cornell Program of Computer Graphics para:
- Validar algoritmos de iluminación global
- Comparar renders con fotografías reales
- Benchmark estándar de la industria

Ha permanecido como referencia por **40 años**.

### Especificación Original

```
Dimensiones:  552×549×550 mm (real)
              555×555×555 unidades (digital)

Paredes:
- Izquierda: Verde difuso
- Derecha: Rojo difuso
- Resto: Blanco difuso (reflectancia ~75%)

Luz:
- Techo, centrada
- 130×105 mm
- Temperatura color: ~5000K (blanco cálido)

Objetos:
- Original: 2 cajas (short box, tall box)
- Esta versión: 2 esferas (vidrio, metal)
```

### Fenómenos Visibles

1. **Color Bleeding**:
   ```
   Pared blanca cerca de pared roja
   → Luz  rebota desde rojo → blanca
   → Pared blanca se ve ligeramente roja ✓
   ```

2. **Soft Shadows**:
   ```
   Luz de área (no puntual)
   → Sombras con penumbra (bordes suaves)
   → Umbra (completamente oscuro) + Penumbra (parcial)
   ```

3. **Caustics** (con vidrio):
   ```
   Luz refractada por esfera de vidrio
   → Focalizada en puntos
   → Patrones brillantes en superficie ✓
   ```

4. **Specular Reflections** (espejo):
   ```
   Esfera metálica refleja:
   - Paredes coloreadas
   - Otra esfera
   - Luz del techo
   ```

### Implementación - `render_obj()`

```python
def render_obj(mode="cornell"):
    # Colores
    rojo = Vec3(0.65, 0.05, 0.05)
    verde = Vec3(0.12, 0.45, 0.15)
    blanco = Vec3(0.73, 0.73, 0.73)
    luz_emision = Vec3(40, 40, 40)
    
    # Paredes (Quads)
    lista = [
        # Izquierda (Verde) - normal hacia +X
        Quad(Vec3(555,0,0), Vec3(0,555,0), Vec3(0,0,555), verde),
        
        # Derecha (Rojo) - normal hacia -X
        Quad(Vec3(0,0,0), Vec3(0,0,555), Vec3(0,555,0), rojo),
        
        # Piso (Blanco) - normal hacia +Y
        Quad(Vec3(0,0,0), Vec3(0,0,555), Vec3(555,0,0), blanco),
        
        # Techo (Blanco) - normal hacia -Y
        Quad(Vec3(555,555,555), Vec3(0,0,-555), Vec3(-555,0,0), blanco),
        
        # Fondo (Blanco) - normal hacia -Z
        Quad(Vec3(0,0,555), Vec3(555,0,0), Vec3(0,555,0), blanco),
        
        # Luz (techo)
        Quad(Vec3(213,554.9,227), Vec3(130,0,0), Vec3(0,0,105),
             Vec3(0,0,0), emission=luz_emision)
    ]
    
    # Modelo 3D (bunny de vidrio)
    bunny_triangles = load_obj(
        "models/bunny.obj",
        color=Vec3(0.9,0.9,0.9),
        offset=Vec3(278, 0, 278),  # Centrado
        scale=3000.0,
material_params={'is_dielectric': True, 'ior': 1.5}
    )
    lista.extend(bunny_triangles)
    
    # BVH + extraer luces
    world = BVHNode.create(lista)
    lights = [obj for obj in lista if obj.emission.length() > 0]
    
    return world, lights
```

---

## Optimización y Configuración

### Parámetros de Calidad

**Tabla de Configuraciones**:

| Preset | Width×Height | Samples | Depth | Tiempo (8-core) | Calidad |
|--------|--------------|---------|-------|-----------------|---------|
| **Preview** | 200×200 | 100 | 4 | ~10 seg | Muy ruidoso |
| **Draft** | 400×400 | 100 | 6 | ~30 seg | Ruidoso |
| **Normal** | 400×400 | 400 | 8 | ~2 min | Aceptable |
| **Good** | 800×800 | 400 | 10 | ~10 min | Limpio |
| **Production** | 1920×1080 | 1000 | 12 | ~1 hora | Muy limpio |
| **Final** | 1920×1080 | 4000 | 16 | ~4 horas | Casi sin ruido |

### Tiempo de Render (Estimación)

```
T = (W × H × S × D × C) / (N_cores × Speedup)

donde:
- W, H: Resolución
- S: Samples per pixel
- D: Profundidad recursión
- C: Constante (~0.00001 seg/operación)
- N_cores: Núcleos CPU
- Speedup: ~0.9 × N_cores (90% eficiencia)

Ejemplo (400×400, 400 samples, depth 8, 8 cores):
T = (400 × 400 × 400 × 8 × 0.00001) / (8 × 0.9 × 8)
  = 512 / 57.6
  ≈ 8.9 minutos
```

### Trade-offs

**Samples vs Ruido**:
```
100 samples: Error ~ 10% (visible)
400 samples: Error ~ 5%  (aceptable)
1000 samples: Error ~ 3% (bueno)
4000 samples: Error ~ 1.5% (excelente)

Reducir error a la mitad → 4× más samples
```

**Depth vs Iluminación Indirecta**:
```
depth = 2: Solo 2 rebotes (oscuro, sin color bleeding)
depth = 4: Aceptable (luz indirecta básica)
depth = 8: Bueno (color bleeding evidente)
depth = 12: Excelente (convergencia casi completa)
depth = 16+: Marginal benefit (<1% diferencia)

Cornell Box: depth=8 suficiente
Escenas abiertas: depth=4 suficiente (menos rebotes)
```

### Gamma Correction

**¿Por qué?**

```
Displays NO son lineales:
    Entrada 0.5 → Brillo percibido ~0.22 (no 0.5)

Espacio lineal (renderizado):
    0.5 luz = mitad fotones

Espacio gamma (display):
    0.5 input ≠ mitad brillo visual
    
Solución: rgb_display = rgb_linear^(1/2.2)
```

**Implementación**:

```python
# Linear space (acumulado en rendering)
pixel_color = col / samples  # Ej: Vec3(0.5, 0.3, 0.1)

# Gamma correction (2.2)
r_gamma = pow(pixel_color.x, 1/2.2)  # 0.5^0.45 ≈ 0.73
g_gamma = pow(pixel_color.y, 1/2.2)  # 0.3^0.45 ≈ 0.58
b_gamma = pow(pixel_color.z, 1/2.2)  # 0.1^0.45 ≈ 0.35

# Quantize [0,1] → [0,255]
r = int(255.99 * r_gamma)  # 186
g = int(255.99 * g_gamma)  # 148
b = int(255.99 * b_gamma)  # 89
```

**Comparación**:

```
Sin gamma:
    Imagen muy oscura
    Sombras casi negras
    Aspecto "apagado"

Con gamma 2.2:
    Imagen correcta ✓
    Tonos medios preservados
    Aspecto natural
```

### Stratified Sampling

**vs Random Puro**:

```
Random Puro (100 samples):
    Algunas regiones: 20 muestras
    Otras regiones: 2 muestras
    → Clustering → Ruido irregular

Stratified (10×10 = 100 samples):
    Cada celda: exactamente 1 muestra
    → Distribución uniforme
    → 30-50% menos ruido para mismo N
```

**Implementación**:

```python
s_side = int(sqrt(samples))  # 10 para samples=100

for i in range(s_side):
    for j in range(s_side):
        # Offset dentro de celda (i,j)
        u_offset = (i + random()) / s_side  # [i/10, (i+1)/10]
        v_offset = (j + random()) / s_side
        
        s = (x + u_offset) / width
        t = (y + v_offset) / height
        
        # Trazar rayo con (s, t)
```

---

## Resumen

### Pipeline Completo

```
1. render()
    ├─ Configurar cámara (FOV, DOF, posición)
    ├─ Cargar escena: render_obj()
    │   ├─ Crear geometría (Quads, modelo OBJ)
    │   └─ BVH + extraer luces
    │
    ├─ Renderizado paralelo:
    │   ├─ Pool de workers (1 por core)
    │   └─ Para cada fila Y:
    │       └─ render_row(Y)
    │           └─ Para cada pixel X:
    │               └─ Stratified samples (cuadrícula):
    │                   ├─ DOF: offset en lente
    │                   ├─ Generar rayo
    │                   └─ color_ray() recursivo:
    │                       ├─ BVH hit
    │                       ├─ Si luz → emisión
    │                       ├─ Si metal → reflejar
    │                       ├─ Si vidrio → Fresnel
    │                       └─ Si difuso:
    │                           ├─ NEE (directo)
    │                           └─ Rebote (indirecto)
    │
    └─ Post-proceso:
        ├─ Gamma correction
        ├─ Flip vertical
        └─ Guardar PNG
```

### Técnicas Clave

| Técnica | Líneas Código | Impacto |
|---------|---------------|---------|
| **BVH** | ~200 (geometry.py) | 1000-10,000× speedup |
| **NEE** | ~30 | 10-100× menos ruido |
| **Stratified Sampling** | ~10 | 30-50% menos ruido |
| **Paralelo** | ~20 | 6-8× speedup |
| **Fresnel-Schlick** | ~5 | Vidrio realista |

### Ecuaciones Fundamentales

1. **Rendering**:
   ```
   L_out = L_e + ∫ f_r L_in cos(θ) dω
   ```

2. **Snell**:
   ```
   η₁ sin(θ₁) = η₂ sin(θ₂)
   ```

3. **Schlick**:
   ```
   R(θ) = R₀ + (1-R₀)(1-cos θ)⁵
   ```

4. **NEE**:
   ```
   L_direct = E × ρ × cos(θ) × (Ω/π)
   ```

5. **Gamma**:
   ```
   rgb_out = rgb_linear^(1/2.2)
   ```

### Relación con Tecnologías Profesionales

Este path tracer implementa las mismas técnicas fundamentales que:

- **Disney Hyperion** (Frozen, Moana, Encanto)
- **Pixar RenderMan** (Toy Story, Soul)
- **Blender Cycles** (open source)
- **V-Ray** (industria VFX)
- **Arnold** (industria cine/animación)
- **OptiX** (NVIDIA GPU ray tracing)

La **diferencia** está en:
- Optimizaciones avanzadas (Russian roulette, MIS, Bidirectional PT)
- Aceleración GPU
- Características adicionales (volumes, SSS, spectral rendering)
- Calidad de producción (denoising, adaptive sampling)

Pero los **fundamentos** son **idénticos**.

---

## Referencias

1. **"Physically Based Rendering: From Theory to Implementation"**  
   Pharr, Jakob, Humphreys (2016)  
   Biblia del rendering físico

2. **"Ray Tracing in One Weekend"**  
   Peter Shirley (2016)  
   Tutorial origen de este código

3. **Cornell Box Original**  
   Cornell Program of Computer Graphics (1984)  
   http://www.graphics.cornell.edu/online/box/

4. **"The Rendering Equation"**  
   James Kajiya (1986)  
   Paper fundacional de path tracing

5. **Fresnel Equations**  
   Augustin-Jean Fresnel (1823)  
   Base física de reflexión/refracción
