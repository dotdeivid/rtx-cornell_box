# Guía Completa de vector.py: Matemáticas Vectoriales y Álgebra Lineal

## Tabla de Contenidos

1. [Introducción](#introducción)
2. [Sistema de Coordenadas 3D](#sistema-de-coordenadas-3d)
3. [La Clase Vec3](#la-clase-vec3)
4. [Operaciones Aritméticas](#operaciones-aritméticas)
5. [Producto Punto (Dot Product)](#producto-punto-dot-product)
6. [Producto Cruz (Cross Product)](#producto-cruz-cross-product)
7. [Magnitud y Normalización](#magnitud-y-normalización)
8. [Reflexión Especular](#reflexión-especular)
9. [Aplicaciones en Ray Tracing](#aplicaciones-en-ray-tracing)
10. [Optimizaciones y Consideraciones](#optimizaciones-y-consideraciones)

---

## Introducción

### ¿Qué es un Vector?

Un **vector** es una entidad matemática con magnitud (tamaño) y dirección.

**Representación**:
```
v = (x, y, z)

x = componente en eje X
y = componente en eje Y  
z = componente en eje Z
```

**Visualización 3D**:
```
      Z
      ↑
      |      • (x,y,z)
      |     /|
      |    / |
      |   /  |
      |  /   |y
      | /    |
      |/     |
      O------•--→ Y
     /      x
    /
   X
```

**Diferencia: Vector vs Punto**:

| Aspecto | Punto | Vector |
|---------|-------|--------|
| Representa | Ubicación en espacio | Dirección y magnitud |
| Origen | Implícito | Explícito (desde origen) |
| Traducción | Cambia ubicación | NO afecta |
| Ejemplo | Posición de esfera | Dirección de luz |

### ¿Por qué Vec3 es Fundamental?

En ray tracing, Vec3 representa **TODO**:

1. **Posiciones** en el espacio:
   ```python
   sphere_center = Vec3(0, 5, -10)
   camera_position = Vec3(0, 2, 0)
   ```

2. **Direcciones** (normalizadas):
   ```python
   ray_direction = Vec3(1, 0, 0).normalize()
   surface_normal = Vec3(0, 1, 0)
   ```

3. **Colores** RGB:
   ```python
   red = Vec3(1.0, 0.0, 0.0)
   white = Vec3(1.0, 1.0, 1.0)
   dark_gray = Vec3(0.2, 0.2, 0.2)
   ```

4. **Offsets** y desplazamientos:
   ```python
   offset = Vec3(0.001, 0, 0)  # Shadow acne prevention
   ```

**Estadística de Uso**:
En un ray tracer típico:
- ~80% de operaciones son manipulación de Vec3
- Cada píxel genera decenas de Vec3 (rayos, hits, colores)
- Imagen 1920×1080 → ~2M píxeles → ~50M+ Vec3 operaciones

---

## Sistema de Coordenadas 3D

### Convención: Mano Derecha

El sistema de coordenadas usado es **mano derecha** (estándar en OpenGL, gráficos):

```
Regla Mano Derecha:
1. Apunta dedos en dirección +X
2. Dobla hacia +Y
3. El pulgar apunta +Z

        Y (arriba/up)
        ↑
        |
        |_____ X (derecha/right)
       /
      /
     Z (hacia ti/forward)
```

**Ejes Principales**:
```python
X_AXIS = Vec3(1, 0, 0)  # Derecha
Y_AXIS = Vec3(0, 1, 0)  # Arriba
Z_AXIS = Vec3(0, 0, 1)  # Adelante (fuera pantalla)
```

### Orientación en Escena Típica

```
              Y (cielo)
              ↑
              |
    Objeto    |    Cámara
       •      |      👁
        \     |     /
         \    |    /
          \   |   /
           \  |  /
   -X ←-----\-|-/-----→ +X
             \|/
              • Origen (0,0,0)
             /
            /
           Z (viewingdirection)
```

**Convención de escena**:
- Cámara mira hacia **+Z** (o -Z dependiendo de implementación)
- Objetos en frente: Z positivo
- Suelo: Y = 0 (plano XZ)
- Luz desde arriba: Y alto

---

## La Clase Vec3

### Estructura Interna

```python
class Vec3:
    def __init__(self, x, y, z):
        self.components = np.array([x, y, z], dtype=np.float64)
```

**Decisión de Diseño: NumPy Array**

| Aspecto | Lista Python | NumPy Array (elegido) |
|---------|-------------|---------------------|
| Precisión | float (variable) | float64 (fijo) |
| Velocidad | Lenta | Rápida (SIMD) |
| Operaciones | Manual | Vectorizadas |
| Memoria | ~56 bytes | ~24 bytes |
| Interop | Limitada | scipy/matplotlib |

**¿Por qué float64?**

```
float32 (32 bits):
- Precisión: ~7 dígitos decimales
- Rango: ±3.4 × 10³⁸
- Problema: Acumulación de error en miles de rebotes

float64 (64 bits):
- Precisión: ~15 dígitos decimales
- Rango: ±1.7 × 10³⁰⁸
- Solución: Errores mínimos incluso después de muchos rebotes

Overhead memory: 24 bytes vs 12 bytes (insignificante)
Precisión ganada: CRÍTICA
```

### Propiedades de Solo Lectura

```python
@property
def x(self):
    return self.components[0]
```

**¿Por qué properties?**

```python
# SIN properties (acceso directo):
v = Vec3(1, 2, 3)
v.components[0] = 999  # ❌ Mutable (peligroso)

# CON properties:
v = Vec3(1, 2, 3)
print(v.x)  # ✓ Lectura OK
v.x = 999   # ❌ Error: can't set attribute (inmutable) ✓
```

**Beneficios**:
- Thread-safe (múltiples hilos pueden leer simultáneamente)
- Debugging más fácil (valores no cambian inesperadamente)
- Semántica clara (x, y, z son coordenadas, no modificables)

### Inmutabilidad

**CRÍTICO**: Todas las operaciones retornan NUEVO Vec3:

```python
v1 = Vec3(1, 2, 3)
v2 = v1 +Vec3(1, 1, 1)

print(v1)  # Vec3(1, 2, 3)  ← SIN CAMBIAR ✓
print(v2)  # Vec3(2, 3, 4)  ← NUEVO
```

**Comparación con estilo mutable**:

```python
# Estilo mutable (C++, algunos lenguajes):
v1.add(Vec3(1,1,1))  # v1 se modifica ❌
print(v1)  # Vec3(2,3,4)  ← CAMBIÓ

# Estilo inmutable (Python, esta implementación):
v2 = v1 + Vec3(1,1,1)  # v1 intacto ✓
print(v1)  # Vec3(1,2,3)  ← SIN CAMBIO
```

**Ventajas inmutabilidad**:
- Predecible (no hay efectos secundarios)
- Thread-safe (compartir entre hilos sin locks)
- Debugging (valores históricos preservados)

**Desventaja**:
- Overhead memory (más GC)
- Solución: Python GC es muy eficiente, overhead despreciable

---

## Operaciones Aritméticas

### Suma de Vectores

**Fórmula**:
```
v1 + v2 = (v1.x + v2.x, v1.y + v2.y, v1.z + v2.z)
```

**Interpretación Geométrica**:

Coloca el inicio de v2 en el "final" (punta) de v1:

```
       v2
      ↗
     • final v1+v2
    ↗
   • v1
  ↗
 O origen
```

**Regla del Paralelogramo**:

```
      • v1+v2
     /|
   v2 |
   /  |v1
  •---•
 O
```

El punto final es la diagonal del paralelogramo.

**Ejemplo Numérico**:

```python
# Trasladar objeto
position = Vec3(5, 2, 0)
offset = Vec3(-3, 1, 2)
new_position = position + offset

# new_position = Vec3(5-3, 2+1, 0+2)
#              = Vec3(2, 3, 2)
```

**Propiedades**:

```
Conmutativa:  v1 + v2 = v2 + v1
Asociativa:   (v1 + v2) + v3 = v1 + (v2 + v3)
Identidad:    v + Vec3(0,0,0) = v
Inverso:      v + (-v) = Vec3(0,0,0)
```

### Resta de Vectores

**Fórmula**:
```
v1 - v2 = (v1.x - v2.x, v1.y - v2.y, v1.z - v2.z)
```

**Interpretación Geométrica**:

Vector que va DESDE v2 HASTA v1:

```
  v1 •
    ↗ \
   /   ↘ v1-v2
  /     \
 O       • v2
```

**Aplicación Común**: Calcular dirección entre dos puntos:

```python
pointA = Vec3(0, 0, 0)
pointB = Vec3(10, 5, 3)

# Dirección de A hacia B
direction = pointB - pointA
# direction = Vec3(10, 5, 3)

# Normalizar para rayo
ray_direction = direction.normalize()
```

**Ejemplo Numérico**:

```python
target = Vec3(15, 20, 10)
origin = Vec3(5, 10, 0)

vector_to_target = target - origin
# = Vec3(15-5, 20-10, 10-0)
# = Vec3(10, 10, 10)

distance = vector_to_target.length()
# = √(10² + 10² + 10²)
# = √300 ≈ 17.32 unidades
```

### Multiplicación por Escalar

**Fórmula**:
```
k * v = (k*v.x, k*v.y, k*v.z)
```

**Interpretación Geométrica**:

Escala el vector (cambia magnitud, mantiene dirección):

```
k = 2:   v → 2v (doble largo)
        ↗
       ↗
      ↗ v
     ↗
    O

k = 0.5: v → 0.5v (mitad)
      ↗ 0.5v
     ↗
    O

k = -1:  v → -v (invertido)
    O
     ↘
      ↘ -v
```

**Casos Especiales**:

```
k > 1:     Vector más largo (misma dirección)
k = 1:     Sin cambio
0 < k < 1: Vector más corto (misma dirección)
k = 0:     Vector cero
k < 0:     Vector inverso y escalado
```

**Ejemplo Numérico**:

```python
v = Vec3(3, 4, 0)
print(v.length())  # 5.0

scaled = v * 2
# = Vec3(6, 8, 0)
print(scaled.length())  # 10.0 (doble) ✓

shrunk = v * 0.5
# = Vec3(1.5, 2.0, 0)
print(shrunk.length())  # 2.5 (mitad) ✓

inverted = v * -1
# = Vec3(-3, -4, 0)
# |inverted| = 5.0 (misma magnitud, dirección opuesta)
```

### Multiplicación Componente a Componente

**Fórmula (Hadamard)**:
```
v1 * v2 = (v1.x*v2.x, v1.y*v2.y, v1.z*v2.z)
```

**Visualización**:

```
v1 = (2, 3, 4)
v2 = (1, 0.5, 2)
     ×   ×   ×
v1*v2 = (2, 1.5, 8)
```

**Aplicación Principal: Colores**

```python
# Luz blanca
light_color = Vec3(1.0, 1.0, 1.0)

# Superficie roja (refleja solo rojo)
surface_albedo = Vec3(0.8, 0.1, 0.1)

# Color resultante
reflected_light = light_color * surface_albedo
# = Vec3(1*0.8, 1*0.1, 1*0.1)
# = Vec3(0.8, 0.1, 0.1)  ← Luz roja ✓
```

**Otro Ejemplo - Modulación**:

```python
# Objetos con diferente visibilidad
object_pos = Vec3(10, 20, 30)
mask = Vec3(1, 0, 1)  # Ocultar componente Y

masked_pos = object_pos * mask
# = Vec3(10, 0, 30)  ← Y eliminado
```

### División por Escalar

**Fórmula**:
```
v / k = (v.x/k, v.y/k, v.z/k)
```

**Equivale a**: `v * (1/k)`

**Aplicación Principal: Promedios**

```python
# Promedio de 3 muestras de color
sample1 = Vec3(1.0, 0.5, 0.3)
sample2 = Vec3(0.8, 0.6, 0.4)
sample3 = Vec3(0.9, 0.55, 0.35)

average = (sample1 + sample2 + sample3) / 3
# = Vec3((1+0.8+0.9)/3, (0.5+0.6+0.55)/3, (0.3+0.4+0.35)/3)
# = Vec3(0.9, 0.55, 0.35)
```

**Ejemplo Numérico**:

```python
v = Vec3(12, 18, 24)

halved = v / 2
# = Vec3(6, 9, 12)

third = v / 3
# = Vec3(4, 6, 8)
```

---

## Producto Punto (Dot Product)

### Definición Matemática

**Fórmula Algebraica**:
```
v1 · v2 = v1.x*v2.x + v1.y*v2.y + v1.z*v2.z
```

**Fórmula Geométrica**:
```
v1 · v2 = |v1| |v2| cos(θ)

donde θ = ángulo entre los vectores
```

**Retorna**: Escalar (número), NO vector.

### Interpretación Geométrica

**Proyección**:

El producto punto mide la "proyección" de un vector sobre otro:

```
       v1
      ↗
     /
    /  θ
   /___↘_________→ v2
       |
       proyección de v1 sobre v2

proyección = |v1| cos(θ)

v1 · v2 = |v1| |v2| cos(θ)
        = (proyección) * |v2|
```

### Casos Especiales

#### Para Vectores Unitarios (|v1| = |v2| = 1):

```
v1 · v2 = cos(θ)

θ = 0°   → v1 · v2 = cos(0) = 1      (paralelos, misma dirección)
θ = 45°  → v1 · v2 = cos(45) ≈ 0.707
θ = 90°  → v1 · v2 = cos(90) = 0     (perpendiculares)
θ = 135° → v1 · v2 = cos(135) ≈ -0.707
θ = 180° → v1 · v2 = cos(180) = -1   (opuestos)
```

#### Significado del Signo:

```
v1 · v2 > 0  ⟹  θ < 90°  (ángulo agudo, misma dirección general)
v1 · v2 = 0  ⟹  θ = 90°  (PERPENDICULARES)
v1 · v2 < 0  ⟹  θ > 90°  (ángulo obtuso, direcciones opuestas)
```

### Ejemplo Numérico Completo

```python
# Ejemplo 1: Paralelos
v1 = Vec3(1, 0, 0)
v2 = Vec3(2, 0, 0)
dot = v1.dot(v2)
# = 1*2 + 0*0 + 0*0
# = 2

# Verificar con fórmula geométrica:
# |v1| = 1, |v2| = 2, θ = 0°
# v1 · v2 = 1 * 2 * cos(0) = 2 ✓

# Ejemplo 2: Perpendiculares
v1 = Vec3(1, 0, 0)  # Eje X
v2 = Vec3(0, 1, 0)  # Eje Y
dot = v1.dot(v2)
# = 1*0 + 0*1 + 0*0
# = 0  ← Perpendiculares ✓

# Ejemplo 3: Ángulo 45°
v1 = Vec3(1, 0, 0).normalize()     # (1, 0, 0)
v2 = Vec3(1, 1, 0).normalize()     # (0.707, 0.707, 0)

dot = v1.dot(v2)
# = 1*0.707 + 0*0.707 + 0*0
# = 0.707
# = cos(45°) ✓

# Recuperar ángulo:
angle = math.acos(dot)
# = arccos(0.707)
# = 45° (π/4 rad) ✓
```

### Aplicaciones en Ray Tracing

#### 1. Iluminación Difusa (Ley de Lambert)

```python
# Cuanto más alineado el rayo de luz con la normal,
# más iluminada está la superficie

normal = Vec3(0, 1, 0)  # Superficie horizontal
light_dir = Vec3(0.6, 0.8, 0).normalize()

# Intensidad proporcional a cos(θ)
brightness = max(0, normal.dot(light_dir))
# = max(0, 0*0.6 + 1*0.8 + 0*0)
# = 0.8
# = 80% de intensidad máxima ✓

# Si luz desde abajo (dot < 0), clamp a 0
```

#### 2. Test de Hemisferio

```python
# ¿Está la dirección en el mismo lado que la normal?

normal = Vec3(0, 1, 0)  # Apunta arriba
direction = Vec3(0.5, 0.7, 0.3).normalize()

if direction.dot(normal) > 0:
    print("Mismo hemisferio - válido")  # ✓
else:
    print("Hemisferio opuesto - invertir")
```

#### 3. Cálculo de Ángulos

```python
v1 = camera_forward.normalize()
v2 = object_direction.normalize()

dot_product = v1.dot(v2)
angle_rad = math.acos(dot_product)
angle_deg = math.degrees(angle_rad)

# Si angle < 90° → objeto en campo de visión
if dot_product > 0:
    print(f"Objeto visible ({angle_deg}° desde centro)")
```

### Propiedades Matemáticas

```
1. Conmutativa: v1 · v2 = v2 · v1

2. Distributiva: v1 · (v2 + v3) = v1·v2 + v1·v3

3. Asociativa con escalar: (kv1) · v2 = k(v1 · v2)

4. v · v = |v|²  (magnitud al cuadrado)

5. Cauchy-Schwarz: |v1 · v2| ≤ |v1||v2|
```

---

## Producto Cruz (Cross Product)

### Definición Matemática

**Fórmula**:
```
v1 × v2 = (v1.y*v2.z - v1.z*v2.y,
          v1.z*v2.x - v1.x*v2.z,
          v1.x*v2.y - v1.y*v2.x)
```

**Retorna**: Vector (NO escalar).

**Propiedades Únicas**:
- Resultado es PERPENDICULAR a ambos v1 y v2
- Magnitud = área del paralelogramo formado por v1 y v2

### Interpretación Geométrica

```
       v1×v2     
         ↑   (perpendicular a plano)
         |
         •----→ v2
        /
       v1
```

El vector resultante apunta **fuera del plano** formado por v1 y v2.

**Regla Mano Derecha**:

1. Apunta dedos en dirección de v1
2. Dobla dedos hacia v2
3. Pulgar apunta en dirección de v1 × v2

```
    Pulgar (v1×v2)
       ↑
       |
       •----→ Dedos doblados (v2)
      /
     Dedos (v1)
```

### Magnitud del Producto Cruz

```
|v1 × v2| = |v1| |v2| sin(θ)

donde θ = ángulo entre vectores
```

**Área del Paralelogramo**:

```
      v2
     ↗----•
    /    / |   Altura = |v1|sin(θ)
v1 /    /  |
  /    /   |
 •----/----•
      Base = |v2|

Área = base × altura
     = |v2| × |v1|sin(θ)
     = |v1 × v2|  ✓
```

### Casos Especiales

```
v1 × v2 = 0  ⟺  v1 y v2 son paralelos (o uno es cero)

v1 × v2 = -(v2 × v1)  (ANTICONMUTATIVO)

v × v = 0  (vector consigo mismo)

|v1 × v2| = máximo cuando v1 ⟂ v2 (perpendiculares)
```

### Ejes Canónicos

```python
x = Vec3(1, 0, 0)
y = Vec3(0, 1, 0)
z = Vec3(0, 0, 1)

# Regla mano derecha:
x.cross(y) = z   # Vec3(0, 0, 1)  ✓
y.cross(z) = x   # Vec3(1, 0, 0)  ✓
z.cross(x) = y   # Vec3(0, 1, 0)  ✓

# Anticonmutativo:
y.cross(x) = -z  # Vec3(0, 0, -1)  ✓
```

### Ejemplo Numérico Completo

```python
# Vectores en plano XY
v1 = Vec3(3, 0, 0)  # A lo largo de X
v2 = Vec3(0, 4, 0)  # A lo largo de Y

cross = v1.cross(v2)
# x: 0*0 - 0*4 = 0
# y: 0*0 - 3*0 = 0
# z: 3*4 - 0*0 = 12
# = Vec3(0, 0, 12)

# Perpendicular al plano XY ✓
# Apunta en +Z (regla mano derecha) ✓

# Magnitude:
mag = cross.length()  # 12

# Verificar con fórmula:
# |v1| = 3, |v2| = 4, θ = 90°
# |v1 × v2| = 3 * 4 * sin(90°) = 12 ✓

# Área del rectángulo:
area = mag  # 12 unidades² ✓
```

### Aplicaciones en Ray Tracing

#### 1. Calcular Normal de Triángulo

```python
# Triángulo con vértices v0, v1, v2
edge1 = v1 - v0
edge2 = v2 - v0

# Normal (perpendicular al plano del triángulo)
normal = edge1.cross(edge2).normalize()

# Ejemplo:
v0 = Vec3(0, 0, 0)
v1 = Vec3(1, 0, 0)
v2 = Vec3(0, 1, 0)

edge1 = Vec3(1, 0, 0)
edge2 = Vec3(0, 1, 0)

normal = edge1.cross(edge2).normalize()
# = Vec3(0, 0, 1).normalize()
# = Vec3(0, 0, 1)  ← Apunta en +Z ✓
```

#### 2. Construir Sistema de Coordenadas

```python
# Dado un vector "forward", construir right y up

forward = Vec3(0.6, 0.8, 0).normalize()
world_up = Vec3(0, 1, 0)

# Right perpendicular a forward y  world_up
right = forward.cross(world_up).normalize()

# Up perpendicular a right y forward
up = right.cross(forward).normalize()

# Ahora tenemos base ortonormal (perpendiculares entre sí)
```

#### 3. Test de Orientación

```python
# ¿Está el punto P del mismo lado del plano AB que el punto C?

AB = B - A
AC = C - A
AP = P - A

# Normal al plano formado por AB y AC
normal = AB.cross(AC)

# Proyección de AP sobre normal
test = normal.dot(AP)

if test > 0:
    print("P del mismo lado que C")
else:
    print("P del lado opuesto")
```

---

## Magnitud y Normalización

### Magnitud (Length)

**Fórmula (Norma Euclidiana)**:
```
|v| = √(x² + y² + z²)
```

**Teorema de Pitágoras en 3D**:

```
2D (plano XY):
|v| = √(x² + y²)

    v = (3, 4)
    |\
  5 | \ 4
    |  \
    |___\
      3
|v| = √(9 + 16) = 5

3D:
|v| = √(x² + y² + z²)

Imagina:
- Diagonal del rectángulo XY: d_xy = √(x² + y²)
- Luego desde d_xy hasta z: |v| = √(d_xy² + z²)
```

**Ejemplo Numérico**:

```python
v = Vec3(2, 3, 6)

length = v.length()
# = √(2² + 3² + 6²)
# = √(4 + 9 + 36)
# = √49
# = 7.0 ✓

# Verificar:
print(v.dot(v))  # 49 = |v|² ✓
print(math.sqrt(v.dot(v)))  # 7.0 ✓
```

**Propiedades**:

```
|v| = 0  ⟺  v = Vec3(0, 0, 0)

|kv| = |k| |v|  (escalar absoluto)

|v1 + v2| ≤ |v1| + |v2|  (desigualdad triangular)

v · v = |v|²  (relacionado con producto punto)
```

### Normalización

**Definición**:

Convertir vector en **unitario** (magnitud = 1) manteniendo dirección:

```
v̂ = v / |v|

donde:
v̂ = vector normalizado (unitopuño)
|v| = magnitud de v
```

**Propiedades del Vector Unitario**:
```
|v̂| = 1  (magnitud)
v̂ = (v.x/|v|, v.y/|v|, v.z/|v|)
```

**Visualización**:

```
v = (3, 4, 0)     |v| = 5
         •
        /|
       / | 4
    5 /  |
     /   |
    •----+
       3

v̂ = (3/5, 4/5, 0) = (0.6, 0.8, 0)
     •   |v̂| = 1
    /|
   / | 0.8
1 /  |
 /   |
•----+
  0.6

Misma dirección, magnitud = 1
```

**Ejemplo Numérico**:

```python
v = Vec3(5, 12, 0)
print(v.length())  # 13.0

v_norm = v.normalize()
# = v / 13
# = Vec3(5/13, 12/13, 0)
# = Vec3(0.3846..., 0.9231..., 0)

print(v_norm.length())  # 1.0 ✓

# Verificar dirección mantenida:
# v y v_norm son paralelos
dot = v.normalize().dot(v.normalize())
# = 1.0 (mismo dirección) ✓
```

**Caso Especial: Vector Cero**

```python
zero = Vec3(0, 0, 0)
zero_norm = zero.normalize()
# if mag == 0: return Vec3(0,0,0)
print(zero_norm)  # Vec3(0, 0, 0)

# No lanza error, retorna vector cero
# DEBES verificar externamente si es crítico
```

### ¿Por Qué Normalizar?

#### 1. Direcciones de Rayos

```python
# Ray.point_at(t) asume direction normalizada
# Para que t represente distancia REAL

direction = Vec3(10, 0, 0)  # NO normalizado
ray = Ray(origin, direction)

point = ray.point_at(1)
# Si direction NO normalizada:
# = origin + 1 * Vec3(10, 0, 0)
# = origin + 10 unidades  ← ¡t=1 pero avanzó 10!

# Con normalización:
direction_norm = Vec3(10, 0, 0).normalize()
# = Vec3(1, 0, 0)
ray = Ray(origin, direction_norm)

point = ray.point_at(1)
# = origin + 1 * Vec3(1, 0, 0)
# = origin + 1 unidad  ← t=1 y avanza 1 ✓
```

#### 2. Normales de Superficie

```python
# Iluminación requiere normales unitarias

normal = (hit_point - sphere_center).normalize()

brightness = max(0, normal.dot(light_dir))

# Si normal NO unitaria, brightness INCORRECTO
```

#### 3. Cálculos de Ángulos

```python
# dot product entre unitarios = cos(θ)

v1 = some_vector1.normalize()
v2 = some_vector2.normalize()

cos_theta = v1.dot(v2)
theta = math.acos(cos_theta)

# Si NO normalizados:
# v1.dot(v2) = |v1||v2|cos(θ)  ← NO solo cos(θ)
```

### Optimización: Length vs Length Squared

**Si solo necesitas COMPARAR** distancias:

```python
# LENTO (con sqrt):
if (point - center).length() < radius:
    # ...

# RÁPIDO (sin sqrt):
if (point - center).dot(point - center) < radius * radius:
    # ...

# length() hace sqrt (costoso)
# dot() no (solo multiplicaciones y sumas)
```

**Speedup**: ~5-10× más rápido.

---

## Reflexión Especular

### Fórmula de Reflexión

```
R = I - 2(I · N)N

donde:
I = rayo incidente
N = normal (DEBE ser unitaria)
R = rayo reflejado
```

### Derivación Matemática

Objetivo: Rebotar rayo I sobre superficie con normal N.

```
       N (normal)
       ↑
       |       R (reflejado)
       |      ↗
-------•----------  Superficie
       |    ↘
       |     I (incidente)
```

**Paso 1**: Proyección de I sobre N:

```
proj = (I · N) * N

Magnitud de proyección = I · N (producto punto)
Dirección = N
Vector proyección = magnitud × dirección
```

**Paso 2**: Componente perpendicular:

```
perp = I - proj

Es la parte de I que es perpendicular a N
```

**Paso 3**: Reflexión:

```
R = perp - proj
  = (I - proj) - proj
  = I - 2*proj
  = I - 2(I · N)N  ✓
```

**Visualización**:

```
       N
       ↑
       |
   ----•----  Superficie
   R   |   I
    ↖  |  ↗
      \|/
       proj

perp = parte horizontal de I
proj = parte vertical de I

Para reflejar:
- Mantener perp (horizontal)
- Invertir proj (vertical)

R = perp - proj
```

### Ejemplo Numérico

```python
# Rayo vertical hacia abajo
incident = Vec3(0, -1, 0)
normal = Vec3(0, 1, 0)  # Horizonte apunta arriba

# Paso por paso:
dot = incident.dot(normal)
# = 0*0 + (-1)*1 + 0*0
# = -1

reflection = incident - normal * (2 * dot)
# = Vec3(0,-1,0) - Vec3(0,1,0) * (2 * -1)
# = Vec3(0,-1,0) - Vec3(0,1,0) * (-2)
# = Vec3(0,-1,0) + Vec3(0,2,0)
# = Vec3(0, 1, 0)  ← Hacia arriba ✓

print(reflection)  # Vec3(0, 1, 0)
```

**Ejemplo 2: Ángulo 45°**

```python
# Rayo diagonal
incident = Vec3(1, -1, 0).normalize()
# = Vec3(0.707, -0.707, 0)

normal = Vec3(0, 1, 0)

dot = incident.dot(normal)
# = 0.707*0 + (-0.707)*1 + 0*0
# = -0.707

reflected = incident - normal * (2 * dot)
# = Vec3(0.707, -0.707, 0) - Vec3(0, 1, 0) * (-1.414)
# = Vec3(0.707, -0.707, 0) + Vec3(0, 1.414, 0)
# = Vec3(0.707, 0.707, 0)  ← Diagonal hacia arriba ✓

# Verificar simetría:
# Ángulo entrada = ángulo salida (desde normal)
angle_in = math.acos(abs(incident.dot(normal)))
angle_out = math.acos(abs(reflected.dot(normal)))
# Ambos = 45° ✓
```

### Ley de Reflexión

**Física**: Ángulo de incidencia = Ángulo de reflexión.

```
       N
       |
   θ_r |  θ_i    θ_i = θ_r
    ↖  |  ↗
   R   |   I
-------•-------
```

**Verificación Matemática**:

```python
# Para cualquier I y N:
I_norm = incident.normalize()
N = normal.normalize()
R = I_norm.reflect(N)

# Ángulo incidencia:
cos_theta_i = abs(I_norm.dot(N))
theta_i = math.acos(cos_theta_i)

# Ángulo reflexión:
cos_theta_r = abs(R.dot(N))
theta_r = math.acos(cos_theta_r)

# theta_i == theta_r  ✓ (ley de reflexión)
```

### Aplicaciones en Ray Tracing

#### 1. Espejos Perfectos

```python
# Hit en espejo
hit_record = mirror.hit(ray, 0.001, inf)

# Direccion reflejada
reflected_dir = ray.direction.reflect(hit_record.normal)

# Nuevo rayo
reflected_ray = Ray(hit_record.point, reflected_dir)

# Trazar recursivamente
color = trace_ray(reflected_ray, depth - 1)
```

#### 2. Metales con Rugosidad (Fuzzy Reflection)

```python
# Metal rugoso
reflected = ray.direction.reflect(normal)

# Añadir aleatoriedad proporcional a fuzz
fuzzed = reflected + fuzz * random_in_unit_sphere()

scattered_ray = Ray(hit_point, fuzzed.normalize())
```

#### 3. Fresnel (Reflexión Parcial)

```python
# En dieléctricos, parte se refleja, parte se refracta
reflectance = fresnel_schlick(cos_theta, ior)

if random() < reflectance:
    # Reflejar
    direction = ray.direction.reflect(normal)
else:
    # Refractar
    direction = refract(ray.direction, normal, ior)
```

---

## Aplicaciones en Ray Tracing

### Resumen de Uso por Operación

| Operación | Uso Principal | Frecuencia |
|-----------|---------------|------------|
| `+` (suma) | Trasladar puntos, combinar offsets | Muy alta |
| `-` (resta) | Calcular direcciones,distancias | Muy alta |
| `* escalar` | Escalar rayos, normalizar | Alta |
| `* Vec3` | Modular colores RGB | Alta |
| `dot()` | Iluminación, test hemisferio, ángulos | Muy alta |
| `cross()` | Normales triángulos, sistemas coord. | Media |
| `normalize()` | Rayos, normales, direcciones | Muy alta |
| `reflect()` | Espejos, metales | Alta |

### Flujo Típico de Ray Tracing

```python
# 1. Generar rayo desde cámara
pixel_pos = viewport_lower_left + u*horizontal + v*vertical
direction = (pixel_pos - camera_pos).normalize()  # normalize
ray = Ray(camera_pos, direction)

# 2. Calcular intersección
offset = ray.origin - sphere.center  # resta (-)
a = ray.direction.dot(ray.direction)  # dot
b = 2.0 * offset.dot(ray.direction)   # dot
c = offset.dot(offset) - radius*radius  # dot

# 3. Calcular punto de hit
hit_point = ray.point_at(t)  # suma (+), mult escalar (*)

# 4. Calcular normal
normal = (hit_point - sphere.center).normalize()  # resta, normalize

# 5. Iluminación
light_dir = (light_pos - hit_point).normalize()  # resta, normalize
brightness = max(0, normal.dot(light_dir))  # dot

# 6. Reflexión (si metal)
reflected = ray.direction.reflect(normal)  # reflect

# 7. Color final
final_color = albedo * brightness  # mult Vec3 (*)
```

---

## Optimizaciones y Consideraciones

### Rendimiento

**Operaciones por costo** (orden aproximado):

| Operación | Costo Relativo | Operaciones |
|-----------|----------------|-------------|
| `+, -, *` | 1× (baseline) | 3 ops |
| `dot()` | 1× | 3 mult + 2 sum |
| `/` | 2× | 3 div |
| `cross()` | 2× | 6 mult + 3 sub |
| `length()` | 5× | 3 mult + 2 sum + sqrt |
| `normalize()` | 6× | length + división |

**Optimización**: Evitar normalize() innecesarios.

```python
# LENTO (normaliza 3 veces):
v1 = vector1.normalize()
v2 = vector2.normalize()
v3 = vector3.normalize()

# RÁPIDO (normaliza 1 vez si posible):
combined = vector1 + vector2 + vector3
combined_norm = combined.normalize()
```

### Precisión Numérica

**float64 vs float32**:

```python
# Errores acumulativos en 1000 rebotes:
v = Vec3(0.1, 0.2, 0.3)

# Con float32:
for _ in range(1000):
    v = v.normalize() * 0.99
# Error acumulado: ~10⁻⁴ (0.01%)

# Con float64:
for _ in range(1000):
    v = v.normalize() * 0.99
# Error acumulado: ~10⁻¹⁴ (despreciable)
```

### Inmutabilidad y GC

**Overhead de memoria**:

```python
# Cada operación crea NUEVO Vec3
v1 = Vec3(1, 2, 3)
v2 = v1 + Vec3(1, 1, 1)  # v1 no cambia, v2 es nuevo
v3 = v2 * 2              # v2 no cambia, v3 es nuevo
# ... miles de operaciones

# Python GC (Garbage Collector) limpia automáticamente
# Overhead despreciable en práctica
```

**Alternativa mutable** (NO recomendada):

```python
# Si performance ES crítica en bucle interno:
components = np.array([1, 2, 3], dtype=np.float64)
components += 1  # In-place (sin crear nuevo)
components *= 2  # In-place

# Sacrifica limpieza por velocidad
# Solo para hotspots demostrados por profiling
```

### Properties vs Acceso Directo

**Costo de properties**:

```python
# Property:
x = v.x  # Llamada función __getattribute__ + property

# Acceso directo:
x = v.components[0]  # Indexación directa

# Diferencia: ~5-10 nanosegundos (despreciable)
# Beneficio de properties: Inmutabilidad + API limpia
```

---

## Resumen

### Conceptos Clave

1. **Vec3** = representación universal (posiciones, direcciones, colores)
2. **float64** = precisión crítica para ray tracing
3. **Inmutabilidad** = operaciones retornan nuevo Vec3
4. **Producto punto** = proyección, ángulos, iluminación
5. **Producto cruz** = perpendiculares, normales, sistemas coord.
6. **Normalización** = vectores unitarios para consistencia
7. **Reflexión** = ley reflexión para espejos/metales

### Fórmulas Esenciales

```
Suma:          v1 + v2 = (x1+x2, y1+y2, z1+z2)
Resta:         v1 - v2 = (x1-x2, y1-y2, z1-z2)
Escalar:       k*v = (k*x, k*y, k*z)
Dot:           v1 · v2 = x1*x2 + y1*y2 + z1*z2 = |v1||v2|cos(θ)
Cross:         v1 × v2 = perpendicular a ambos, |result| = |v1||v2|sin(θ)
Magnitud:      |v| = √(x² + y² + z²)
Normalize:     v̂ = v / |v|
Reflect:       R = I - 2(I · N)N
```

### Tabla de Referencia Rápida

| Necesitas | Usa | Ejemplo |
|-----------|-----|---------|
| Mover punto | `+` | `new_pos = pos + offset` |
| Dirección entre puntos | `-` | `dir = target - origin` |
| Escalar vector | `*` | `v2 = v * 2` |
| Modular color | `*` | `reflected = light * albedo` |
| Iluminación | `dot` | `brightness = normal.dot(light)` |
| Normal triángulo | `cross` | `normal = edge1.cross(edge2)` |
| Distancia | `length` | `dist = (p2 - p1).length()` |
| Rayo válido | `normalize` | `ray_dir = dir.normalize()` |
| Espejo | `reflect` | `refl = incident.reflect(normal)` |

### Relación con Tecnologías

Vec3 y operaciones vectoriales son IDÉNTICAS en:
- **GLSL** (shaders OpenGL)
- **HLSL** (shaders DirectX)
- **CUDA** (NVIDIA GPU programming)
- **Metal** (Apple GPU)
- **GLM** (OpenGL Mathematics library C++)
- **numpy** (Python científico)

Los conceptos son **universales** en gráficos por computadora.
