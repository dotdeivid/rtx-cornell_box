# Foundations: Lógica y Matemáticas de Cada Elemento

Guía pedagógica profunda sobre cada componente del ray tracer. Explica el *por qué* y el *cómo* de cada elemento, con diagramas, ejemplos numéricos y derivaciones paso a paso.

---

## Tabla de Contenidos

1. [Vec3 — El Vector 3D](#1-vec3--el-vector-3d)
2. [Ray — El Rayo](#2-ray--el-rayo)
3. [AABB — Caja Envolvente](#3-aabb--caja-envolvente)
4. [BVH — Jerarquía de Volúmenes](#4-bvh--jerarquía-de-volúmenes)
5. [HitRecord — Registro de Intersección](#5-hitrecord--registro-de-intersección)
6. [Sphere — Geometría Esférica](#6-sphere--geometría-esférica)
7. [Quad — Cuadriláteros Planos](#7-quad--cuadriláteros-planos)
8. [Triangle — Geometría Triangular](#8-triangle--geometría-triangular)
9. [Muestreo Monte Carlo (utils)](#9-muestreo-monte-carlo-utils)
10. [Path Tracing — El Pipeline Completo](#10-path-tracing--el-pipeline-completo)

---

## 1. Vec3 — El Vector 3D

### ¿Qué es un Vector?

Un **vector** es una entidad matemática con magnitud (tamaño) y dirección. En este proyecto, `Vec3` representa **todo**: posiciones, direcciones, colores y offsets.

```
v = (x, y, z)

      Z
      ↑
      |      • (x,y,z)
      |     /|
      |    / |
      |   /  |y
      |  /   |
      | /    |
      |/     |
      O------•--→ Y
     /      x
    /
   X
```

**Usos de Vec3 en ray tracing**:

| Uso | Ejemplo |
|-----|---------|
| Posición en espacio | `Vec3(278, 278, -800)` — posición de cámara |
| Dirección (normalizada) | `Vec3(0, 1, 0)` — apunta arriba |
| Color RGB | `Vec3(0.65, 0.05, 0.05)` — rojo |
| Offset (shadow acne) | `hit_point + normal * 0.001` |

### Sistema de Coordenadas: Mano Derecha

```
        Y (arriba)
        ↑
        |_____ X (derecha)
       /
      Z (hacia ti)
```

### Decisión de Diseño: NumPy float64

| Aspecto | Lista Python | NumPy float64 (elegido) |
|---------|-------------|---------------------|
| Precisión | ~7 dígitos | ~15 dígitos |
| Velocidad | Lenta | Rápida (SIMD) |
| Memoria | ~56 bytes | ~24 bytes |

Con float32, después de miles de rebotes de luz, el error acumulado puede llegar a ~10⁻⁴ (0.01%). Con float64: ~10⁻¹⁴ (despreciable). En ray tracing, la precisión es crítica.

### Inmutabilidad

**TODAS las operaciones retornan un NUEVO Vec3** — el original no cambia:

```python
v1 = Vec3(1, 2, 3)
v2 = v1 + Vec3(1, 1, 1)

print(v1)  # Vec3(1, 2, 3)  ← SIN CAMBIAR
print(v2)  # Vec3(2, 3, 4)  ← NUEVO objeto
```

Esto garantiza thread-safety en el renderizado paralelo con `multiprocessing.Pool`.

---

### Operaciones Aritméticas

#### Suma: `v1 + v2`

```
       v2
      ↗
     • final (v1+v2)
    ↗
   • v1
  ↗
 O origen
```

**Ejemplo — trasladar un punto**:
```python
position = Vec3(5, 2, 0)
offset   = Vec3(-3, 1, 2)
new_pos  = position + offset
# = Vec3(5-3, 2+1, 0+2) = Vec3(2, 3, 2)
```

#### Resta: `v1 - v2`

Vector que va **desde** v2 **hasta** v1. Aplicación principal: calcular dirección entre dos puntos.

```python
target    = Vec3(15, 20, 10)
origin    = Vec3(5, 10, 0)
direction = target - origin
# = Vec3(10, 10, 10)
distance  = direction.length()  # √300 ≈ 17.32
```

#### Multiplicación componente a componente (Hadamard)

Uso principal: modular colores RGB.

```python
light_color    = Vec3(1.0, 1.0, 1.0)  # Luz blanca
surface_albedo = Vec3(0.8, 0.1, 0.1)  # Superficie roja
reflected      = light_color * surface_albedo
# = Vec3(0.8, 0.1, 0.1)  ← Luz roja ✓
```

---

### Producto Punto (Dot Product)

**Fórmula algebraica**:
```
v1 · v2 = v1.x*v2.x + v1.y*v2.y + v1.z*v2.z
```

**Fórmula geométrica**:
```
v1 · v2 = |v1| |v2| cos(θ)
```

Retorna un **escalar**. Para vectores unitarios: `v1 · v2 = cos(θ)`.

#### Casos especiales

```
θ = 0°   → dot = 1      (paralelos, misma dirección)
θ = 90°  → dot = 0      (PERPENDICULARES)
θ = 180° → dot = -1     (opuestos)

dot > 0  ⟹  ángulo agudo  (mismo lado general)
dot = 0  ⟹  perpendiculares
dot < 0  ⟹  ángulo obtuso (lados opuestos)
```

#### Aplicaciones en ray tracing

**1. Iluminación difusa (Ley de Lambert)**:
```python
normal    = Vec3(0, 1, 0)            # Superficie horizontal
light_dir = Vec3(0.6, 0.8, 0).normalize()
brightness = max(0, normal.dot(light_dir))
# = 0.8  → 80% de intensidad máxima
```

**2. Test de hemisferio** — ¿está la dirección del mismo lado que la normal?
```python
if direction.dot(normal) > 0:
    # Mismo hemisferio ✓ (válido para rebote)
else:
    # Hemisferio opuesto — invertir
```

**3. Cálculo de ángulos**:
```python
cos_theta = v1.normalize().dot(v2.normalize())
theta     = math.acos(cos_theta)  # ángulo real en radianes
```

---

### Producto Cruz (Cross Product)

**Fórmula**:
```
v1 × v2 = (v1.y*v2.z - v1.z*v2.y,
           v1.z*v2.x - v1.x*v2.z,
           v1.x*v2.y - v1.y*v2.x)
```

Retorna un **vector** perpendicular a ambos operandos. Su magnitud es el área del paralelogramo formado por v1 y v2.

```
       v1×v2
         ↑   (perpendicular al plano)
         |
         •----→ v2
        /
       v1
```

**Regla mano derecha**: apunta dedos en v1, dobla hacia v2, el pulgar indica v1 × v2.

**Anti-conmutativo**: `v1 × v2 = -(v2 × v1)`

#### Ejes canónicos
```python
x.cross(y) = z   # Vec3(0, 0, 1) ✓
y.cross(z) = x   # Vec3(1, 0, 0) ✓
z.cross(x) = y   # Vec3(0, 1, 0) ✓
```

#### Aplicaciones

**1. Normal de un triángulo**:
```python
edge1 = v1 - v0
edge2 = v2 - v0
normal = edge1.cross(edge2).normalize()
# Perpendicular al plano del triángulo ✓
```

**2. Construir base ortonormal de la cámara**:
```python
w = (origin - lookat).normalize()  # Atrás
u = vup.cross(w).normalize()       # Derecha
v = w.cross(u)                     # Arriba (recalculado, exactamente perpendicular)
```

---

### Magnitud y Normalización

**Magnitud (norma euclidiana)**:
```
|v| = √(x² + y² + z²)
```

Ejemplo: `Vec3(2, 3, 6)` → `√(4+9+36)` = `√49` = **7.0**

**Normalización** — convertir a vector unitario (|v̂| = 1):
```
v̂ = v / |v|
```

Ejemplo:
```python
v      = Vec3(5, 12, 0)   # |v| = 13
v_norm = v.normalize()    # Vec3(5/13, 12/13, 0) ≈ Vec3(0.385, 0.923, 0)
v_norm.length()           # 1.0 ✓
```

**¿Por qué es crítico normalizar?** Porque `ray.point_at(t)` usa la ecuación `P(t) = O + t*D`. Si D no es unitario, `t` no representa distancia real — el rayo avanza `|D| * t` unidades en lugar de `t` unidades. Todos los cálculos de distancia, oclusión y atenuación serían incorrectos.

**Optimización**: para *comparar* distancias, evitar `length()` (que hace sqrt):
```python
# LENTO
if (point - center).length() < radius: ...

# RÁPIDO (~5-10×)
if (point - center).dot(point - center) < radius * radius: ...
```

---

### Reflexión Especular

**Fórmula**:
```
R = I - 2(I · N)N
```

donde I = incidente (normalizado), N = normal de superficie (normalizada).

**Derivación paso a paso**:

```
       N (normal)
       ↑
       |       R (reflejado)
       |      ↗
-------•----------  Superficie
            ↘
             I (incidente)

Paso 1: proyección de I sobre N
  proj = (I · N) * N

Paso 2: componente perpendicular
  perp = I - proj

Paso 3: reflexión (invertir proj, mantener perp)
  R = perp - proj = I - 2*proj = I - 2(I · N)N  ✓
```

**Ejemplo numérico — rayo 45°**:
```python
incident  = Vec3(1, -1, 0).normalize()  # Vec3(0.707, -0.707, 0)
normal    = Vec3(0, 1, 0)
dot       = incident.dot(normal)        # -0.707
reflected = incident - normal * (2 * dot)
# = Vec3(0.707, -0.707, 0) + Vec3(0, 1.414, 0)
# = Vec3(0.707, 0.707, 0)  ← diagonal hacia arriba ✓
# Ángulo entrada = ángulo salida (Ley de reflexión) ✓
```

---

## 2. Ray — El Rayo

### Definición matemática

Un rayo es una **semirecta** definida por la ecuación paramétrica:

```
P(t) = O + t * D

O = origen (Vec3)
D = dirección NORMALIZADA (Vec3, |D| = 1)
t = parámetro (distancia desde el origen)
```

```
                          P(t=5)
                             •
                            /
                P(t=2)     /
                   •      /
                  /      /
      Origen     /      /
        O-------/------/  → dirección D
```

| Valor de t | Significado |
|------------|-------------|
| t = 0 | El propio origen |
| t > 0 | Adelante del origen (ray tracing normal) |
| t < t_min (0.001) | Ignorado (shadow acne prevention) |
| t = ∞ | Cielo/fondo |

### Tipos de rayos

| Tipo | Origen | Dirección | Propósito |
|------|--------|-----------|-----------|
| Primario | Cámara | Hacia píxel | Visión inicial |
| Sombra | Superficie + offset | Hacia luz | Iluminación directa (NEE) |
| Reflexión | Superficie | Reflejada (`reflect()`) | Metales, espejos |
| Refracción | Superficie | Ley de Snell | Vidrio, agua |
| Difuso | Superficie | Aleatoria en hemisferio | Materiales Lambertian |

### Shadow acne: por qué t_min = 0.001

Cuando un rayo rebota en una superficie, puede re-intersectarse con la **misma** superficie por errores de punto flotante:

```
Superficie en y=0 (exacta)

Rayo rebota en (x=5, y=0.0000001, z=3)  ← Error numérico
              ↑
        [Re-impacta la misma superficie → punto negro]
```

Solución: ignorar intersecciones con `t < 0.001`. El rayo comienza "flotando" ligeramente sobre la superficie.

### Normalización automática en `__init__`

`Ray.__init__` llama `direction.normalize()` una sola vez. Después, `point_at(t)` puede llamarse millones de veces sin costo adicional de normalización:

```python
ray = Ray(origin, direction)   # normaliza UNA vez
for t in range(1_000_000):
    pt = ray.point_at(t)       # 6 operaciones básicas, sin sqrt
```

---

## 3. AABB — Caja Envolvente

### ¿Qué es?

Una **AABB** (Axis-Aligned Bounding Box) es el volumen rectangular mínimo alineado con los ejes X, Y, Z que contiene completamente un objeto.

```
        max (x_max, y_max, z_max)
          +-------------------------+
         /|                        /|
        / |                       / |
       /  |                      /  |
      +-------------------------+   |
      |   |     OBJETO 3D       |   |
      |   +---------------------|---+
      |  /                      |  /
      | /                       | /
      |/                        |/
      +-------------------------+
    min (x_min, y_min, z_min)
```

**¿Por qué "alineada con los ejes"?** Las caras son siempre perpendiculares a X, Y, Z — no puede rotar. Esto hace que la intersección rayo-caja sea muy rápida (solo comparaciones).

Comparación con OBB (caja orientada):
```
Objeto diagonal en AABB:        Mismo objeto en OBB:
    max                              /-------\
    +----------+                    /  #####  \
    | ####  ###|                   /  #######  \
    |  ########|                  min #######  max
    |   ######  |                  \  #######  /
    |    ####   |                   \  #####  /
    min--------+                    \-------/

AABB: ~40% espacio vacío      OBB: ~10% espacio vacío
AABB: intersección O(1)       OBB: intersección costosa
```

La AABB sacrifica ajuste exacto por velocidad de intersección. La diferencia de rendimiento es enorme en escenas con millones de rayos.

### Algoritmo de intersección: Slabs (Kay & Kajiya, 1986)

Una caja 3D es la intersección de 3 pares de planos paralelos (slabs):
- Slab X: entre x_min y x_max
- Slab Y: entre y_min y y_max
- Slab Z: entre z_min y z_max

Para cada eje, el rayo entra (t0) y sale (t1) del slab. La caja se intersecta solo si todos los intervalos se solapan:

```
t_entrada = max(t0_x, t0_y, t0_z)
t_salida  = min(t1_x, t1_y, t1_z)
intersecta si t_entrada < t_salida
```

**Ejemplo numérico completo**:
```
ray.origin    = (0, 0, 0)
ray.direction = (1, 1, 1) / √3  # Diagonal normalizada
box.min       = (2, 1, 3)
box.max       = (5, 4, 6)

Eje X: t0_x = 2√3 ≈ 3.46,  t1_x = 5√3 ≈ 8.66
Eje Y: t0_y = 1√3 ≈ 1.73,  t1_y = 4√3 ≈ 6.93
Eje Z: t0_z = 3√3 ≈ 5.20,  t1_z = 6√3 ≈ 10.39

t_entrada = max(3.46, 1.73, 5.20) = 5.20  ← entra por z=3
t_salida  = min(8.66, 6.93, 10.39) = 6.93  ← sale por y=4

Intervalo válido [5.20, 6.93] → HAY INTERSECCIÓN ✓
```

### Operación Union

Combina dos cajas en una que las contiene a ambas:
```python
# min(min1, min2) componente a componente
# max(max1, max2) componente a componente
box1 = AABB(Vec3(0,0,0), Vec3(2,2,2))
box2 = AABB(Vec3(1,1,1), Vec3(4,3,5))
union = AABB(Vec3(0,0,0), Vec3(4,3,5))  # min de mínimos, max de máximos
```

---

## 4. BVH — Jerarquía de Volúmenes

### Analogía: Biblioteca de libros

Imagina que tienes 1,000 libros y quieres encontrar uno específico:

- **Sin organización O(N)**: Revisas los 1,000 uno por uno
- **Con estanterías (BVH)**:
  - 10 estanterías con 100 libros cada una
  - Primero identificas la estantería correcta (1 prueba)
  - Luego buscas en esa estantería (100 pruebas)
  - Total: ~101 pruebas vs 1,000

### Estructura del árbol

```
              [Nodo raíz — caja que contiene TODO]
             /                                    \
  [Nodo izq — mitad izquierda]         [Nodo der — mitad derecha]
  /                       \             /                      \
[Esfera A]  [Esferas B,C]  [Esfera D]  [Esfera E]
```

Cada nodo almacena:
- Su AABB (bounding box de todos los descendientes)
- Hijo izquierdo y hijo derecho

### Construcción O(N log N)

```
1. Seleccionar eje aleatorio (X, Y o Z)
   → ¿Por qué aleatorio? Evita casos degenerados donde todos los objetos están alineados

2. Ordenar objetos por posición en ese eje
   objects.sort(key=lambda obj: obj.bounding_box().min.x)

3. Dividir por la mitad y construir recursivamente
   mid   = len(objects) // 2
   left  = BVHNode.create(objects[:mid])
   right = BVHNode.create(objects[mid:])

4. La caja del nodo = union(left.box, right.box)

Caso base: 1 objeto → retornar el objeto; 2 objetos → nodo con ambos como hijos
```

### Traversal con poda espacial

```
¿Rayo intersecta la caja del nodo?
    ├─ NO → descartar TODOS los descendientes (poda) 🚀
    └─ SÍ → probar ambos hijos
        ├─ Probar hijo izquierdo
        ├─ Si hay hit izquierdo, actualizar t_max para hijo derecho
        └─ Retornar el hit más cercano
```

### Impacto en rendimiento

| Operación | Sin BVH | Con BVH |
|-----------|---------|---------|
| Construcción | — | O(N log N) |
| Búsqueda promedio | O(N) | O(log N) |

Para una imagen 1920×1080 con 1,000,000 objetos y 10 rebotes:

```
Sin BVH:
  1920 × 1080 × 10 × 1,000,000 = 20,736,000,000,000 hits
  ≈ 20.7 TRILLONES de operaciones → ~5,000 horas

Con BVH:
  1920 × 1080 × 10 × log₂(1,000,000) ≈ 414,720,000 hits
  ≈ 415 MILLONES de operaciones → ~10 minutos

Aceleración: ~50,000×
```

El BVH es la diferencia entre impracticable y tiempo razonable.

---

## 5. HitRecord — Registro de Intersección

Cuando un rayo impacta un objeto, `HitRecord` almacena toda la información necesaria para calcular iluminación y scattering.

### Atributos y su lógica

#### `t` — Parámetro del rayo

Distancia desde el origen del rayo hasta el punto de impacto. Valores menores = objetos más cercanos = mayor prioridad.

```
Cámara en (0,0,0) → Rayo hacia (1,0,0)
├─ Esfera A en (3,0,0): t=2 ← MÁS CERCA (gana)
└─ Esfera B en (10,0,0): t=9 (descartada — oculta por A)
```

#### `normal` — Vector normal

Perpendicular a la superficie en el punto de impacto, siempre normalizado. Usos:
- Iluminación: `brightness = max(0, normal.dot(light_dir))`
- Dirección de reflexión/refracción
- Offset para shadow acne: `hit_point + normal * 0.001`

#### `emission` — Luz emitida

```
Vec3(0, 0, 0)     → No emite (objeto normal)
Vec3(1, 1, 1)     → Luz blanca suave
Vec3(15, 15, 15)  → Luz blanca intensa (Cornell Box default)
Vec3(50, 40, 20)  → Luz amarillenta muy intensa
```

Los valores > 1 son válidos en HDR — representan brillos que superan el rango [0,1] del display y se comprimen con gamma correction.

#### `fuzz` — Rugosidad del metal

```
fuzz = 0.0  → Espejo perfecto
fuzz = 0.1  → Metal muy pulido (cromo)
fuzz = 0.3  → Acero inoxidable
fuzz = 0.7  → Aluminio cepillado
fuzz = 1.0  → Metal muy rugoso (hierro oxidado)
```

Implementación: `scattered = reflected + fuzz * random_in_unit_sphere()`

#### `ior` — Índice de refracción

| Material | IOR | Efecto |
|----------|-----|--------|
| Vacío/Aire | 1.0 | Sin refracción |
| Agua | 1.33 | Doblez suave |
| Vidrio | 1.5 | Doblez medio (Cornell Box) |
| Cristal | 1.7 | Doblez notable |
| Diamante | 2.4 | Doblez intenso, brillos |

Ley de Snell:
```
n₁ * sin(θ₁) = n₂ * sin(θ₂)

Ejemplo: aire (1.0) → vidrio (1.5), θ₁ = 45°
1.0 * sin(45°) = 1.5 * sin(θ₂)
sin(θ₂) = 0.707 / 1.5 = 0.471
θ₂ ≈ 28.1°  ← el rayo se acerca a la normal ✓
```

---

## 6. Sphere — Geometría Esférica

### Definición matemática

Todos los puntos a distancia constante `r` de un centro `C`:
```
||P - C||² = r²
```

### Intersección rayo-esfera: derivación completa

```
Rayo: P(t) = O + t*D
Esfera: ||P - C||² = r²

Sustituyendo P = O + t*D:
||O + t*D - C||² = r²

Sea oc = O - C:
||oc + t*D||² = r²

Expandiendo:
(D·D)t² + 2(D·oc)t + (oc·oc - r²) = 0

Ecuación cuadrática: at² + bt + c = 0
  a = D · D
  b = 2(D · oc)
  c = oc·oc - r²

Discriminante: Δ = b² - 4ac
  Δ < 0 → No hay intersección (pasa de largo)
  Δ = 0 → Intersección tangente (roza)
  Δ > 0 → Dos intersecciones (entra y sale)
```

**Ejemplo numérico**:
```python
origin    = Vec3(0, 0, 0)
direction = Vec3(1, 0, 0)
center    = Vec3(5, 0, 0)
radius    = 2

oc = origin - center = Vec3(-5, 0, 0)
a  = direction·direction = 1
b  = 2*(direction·oc) = -10
c  = oc·oc - r² = 25 - 4 = 21
Δ  = 100 - 84 = 16  → Hay intersección ✓

t₁ = (10 - 4) / 2 = 3   → punto de entrada (3, 0, 0)
t₂ = (10 + 4) / 2 = 7   → punto de salida (7, 0, 0)

Normal en entrada:
normal = (P₁ - center) / radius
       = (Vec3(3,0,0) - Vec3(5,0,0)) / 2
       = Vec3(-1, 0, 0)  ← apunta hacia el origen ✓
```

**Por qué la normal es unitaria automáticamente**: `(point - center) / radius`. Como `||point - center|| = radius` (definición de esfera), la división produce magnitud 1 exacta — sin necesidad de `.normalize()`.

---

## 7. Quad — Cuadriláteros Planos

### Definición paramétrica

```
Un Quad se define por:
  Q  → punto origen (esquina)
  u  → vector primer lado
  v  → vector segundo lado

Vértices:
  Q ────────→ Q+u
  │             │
  ↓             ↓
  Q+v ─────→ Q+u+v

Cualquier punto: P(s,t) = Q + s*u + t*v,  s,t ∈ [0,1]
```

### Intersección en dos fases

**Fase 1 — Intersección rayo-plano**:

El quad está contenido en el plano `n · P = D` donde:
- `n = (u × v).normalize()` — normal del plano
- `D = n · Q` — distancia del plano al origen

```
Rayo en plano: n·(O + t*dir) = D
t = (D - n·O) / (n·dir)

Si n·dir ≈ 0 → rayo paralelo al plano → no hay intersección
```

**Ejemplo**:
```python
Q = Vec3(0, 2, 0)   # Quad horizontal en y=2
u = Vec3(4, 0, 0)
v = Vec3(0, 0, 3)

n = (u×v).normalize() = Vec3(0,1,0)  # apunta arriba
D = n · Q = 2

ray.origin    = Vec3(1, 0, 1)
ray.direction = Vec3(0, 1, 0)  # hacia arriba

t = (2 - 0) / 1 = 2
P = Vec3(1, 2, 1)  ← punto de impacto en el plano
```

**Fase 2 — Test de contención con coordenadas baricéntricas**:

```
P = Q + alpha*u + beta*v

Condición para estar dentro del quad:
  0 ≤ alpha ≤ 1  AND  0 ≤ beta ≤ 1
```

### Normales y orientación

Los vectores `u` y `v` negativos invierten la normal. Esto se usa en la Cornell Box para que todas las paredes tengan la normal apuntando hacia el interior:

```python
# Techo: normal apunta hacia abajo (interior)
Quad(Vec3(555,555,555), u=Vec3(-555,0,0), v=Vec3(0,0,-555), ...)
#                             ↑ negativo           ↑ negativo
```

---

## 8. Triangle — Geometría Triangular

### ¿Por qué triángulos?

Los triángulos son la primitiva universal de gráficos 3D:
- **3 puntos siempre definen un plano** (no así los quads)
- **Hardware GPU optimizado** específicamente para triángulos
- **Formato estándar** en archivos .obj, .fbx, .stl
- **Cualquier superficie** se puede aproximar con triángulos

El Stanford Bunny que usa este proyecto tiene ~69,000 triángulos.

### Coordenadas baricéntricas

Un punto P dentro del triángulo (v0, v1, v2):
```
P = v0 + u*(v1 - v0) + v*(v2 - v0)

Condiciones:
  u ≥ 0
  v ≥ 0
  u + v ≤ 1

(u=0, v=0) → v0    (u=1, v=0) → v1    (u=0, v=1) → v2
(u=1/3, v=1/3) → Centroide (centro del triángulo)
```

### Algoritmo Möller-Trumbore (1997)

El estándar de la industria. Ventajas clave:
- No requiere precalcular el plano
- Calcula `t` y coordenadas baricéntricas **simultáneamente**
- Evita divisiones hasta el final
- Solo usa productos cruz y puntos

**Derivación**:
```
Rayo: R(t) = O + t*D
Triángulo: T(u,v) = v0 + u*edge1 + v*edge2

Intersección: O + t*D = v0 + u*edge1 + v*edge2
Sea s = O - v0:
s = -t*D + u*edge1 + v*edge2

Sistema 3×3 resuelto con Regla de Cramer:
  h = D × edge2
  a = edge1 · h    (determinante)

  Si |a| < ε → paralelo, no hay intersección

  f = 1 / a
  u = f * (s · h)        → test: u ∈ [0,1]
  q = s × edge1
  v = f * (D · q)        → test: v ∈ [0,1] y u+v ≤ 1
  t = f * (edge2 · q)    → test: t ∈ [t_min, t_max]
```

**Ejemplo numérico completo**:
```python
v0 = Vec3(0, 0, 0)
v1 = Vec3(4, 0, 0)
v2 = Vec3(0, 4, 0)

ray.origin    = Vec3(1, 1, -5)
ray.direction = Vec3(0, 0, 1)   # hacia adelante

edge1 = Vec3(4, 0, 0)
edge2 = Vec3(0, 4, 0)

h = direction × edge2 = Vec3(-4, 0, 0)
a = edge1 · h = -16            ✓ (no paralelo)

f = -0.0625
s = origin - v0 = Vec3(1, 1, -5)

u = f * (s · h) = -0.0625 * (-4) = 0.25   ✓ en [0,1]

q = s × edge1 = Vec3(0, -20, -4)
v = f * (direction · q) = -0.0625 * (-4) = 0.25   ✓ en [0,1]
u + v = 0.5 ≤ 1   ✓

t = f * (edge2 · q) = -0.0625 * (-80) = 5   ✓

Punto de impacto: Vec3(1, 1, -5) + 5*Vec3(0,0,1) = Vec3(1, 1, 0)
Verificación baricéntrica: v0 + 0.25*edge1 + 0.25*edge2 = Vec3(1, 1, 0)  ✓
```

---

## 9. Muestreo Monte Carlo (utils)

### ¿Por qué muestreo aleatorio?

El path tracing resuelve la ecuación de renderizado, una integral que **no tiene solución analítica** para escenas complejas:

```
L_out = ∫_Ω f_r(ω) * L_in(ω) * cos(θ) dω
```

Solución: aproximar con muestras aleatorias (Monte Carlo):

```
L_out ≈ (1/N) Σ f_r(ω_i) * L_in(ω_i) * cos(θ_i)
```

**Convergencia**: Error ∝ 1/√N. Para reducir el error a la mitad se necesitan 4× más muestras. Por eso path tracing requiere muchas muestras por píxel.

### Rejection Sampling — el método de rechazo

Técnica para generar muestras de distribuciones complejas usando una región simple:

```
Algoritmo:
  1. Genera muestra en región SIMPLE (cubo/cuadrado)
  2. Test: ¿Cumple criterio para región COMPLEJA (esfera/círculo)?
  3. SÍ → acepta y retorna
  4. NO → rechaza y vuelve al paso 1
```

### `random_in_unit_sphere()`

**Por qué el método naïve está mal**:
```python
# ¡MAL! — concentra puntos cerca del centro
r     = random()
theta = random() * 2π
phi   = random() * π
```
Con `r = random()` uniforme, el 50% de puntos tienen `r < 0.5`, pero ese radio interior ocupa solo el 12.5% del volumen. Resultado: concentración antinatural en el centro.

**Solución — rejection sampling**:
```python
def random_in_unit_sphere():
    while True:
        p = Vec3(uniform(-1,1), uniform(-1,1), uniform(-1,1))
        if p.length() < 1.0:
            return p
```

Eficiencia: `V_esfera / V_cubo = (4/3)π / 8 ≈ 52.4%` → en promedio ~1.91 intentos.

**Usos**:
- Scattering difuso: `scatter_target = hit_point + normal + random_in_unit_sphere()`
- Reflexión fuzzy en metales: `reflected + fuzz * random_in_unit_sphere()`

### `random_in_unit_disk()` — Profundidad de campo

```python
def random_in_unit_disk():
    while True:
        p = Vec3(uniform(-1,1), uniform(-1,1), 0)
        if p.length() < 1.0:
            return p
```

Eficiencia: `π/4 ≈ 78.5%` → en promedio ~1.27 intentos (más eficiente que esfera).

**Cómo produce bokeh**: Cada muestra por píxel tiene un origen ligeramente diferente en el disco de la lente. Todos los rayos de un mismo píxel convergen en el **plano focal** — los objetos en ese plano se ven nítidos. Los objetos a diferente distancia producen un "círculo de confusión" → bokeh.

```
      Lente (apertura)
         .-----.
        |   🔍   |   Múltiples orígenes
         `-----'
           \|/
            •    Plano focal (nítido)
           /|\
          / | \   Objetos fuera → borrosos (bokeh)
```

### `generar_direccion_aleatoria(normal)` — Hemisferio orientado

Para materiales difusos, el rayo de rebote debe ir hacia el **exterior** de la superficie:

```python
def generar_direccion_aleatoria(normal):
    random_dir = random_in_unit_sphere().normalize()
    if random_dir.dot(normal) > 0.0:
        return random_dir       # Mismo hemisferio ✓
    else:
        return random_dir * -1  # Invertir para que quede en el hemisferio correcto
```

El producto punto determina el hemisferio:
```
n · d > 0 → ángulo < 90° → mismo hemisferio ✓
n · d < 0 → ángulo > 90° → hemisferio opuesto → invertir
```

**Distribución uniforme vs coseno-ponderado**:

La BRDF Lambertiana es `f_r = ρ/π`. Con muestreo uniforme, el estimador incluye `cos(θ)` explícitamente. Con muestreo coseno-ponderado (`PDF = cos(θ)/π`), el `cos(θ)` se cancela, reduciendo varianza y por tanto ruido.

### `load_obj()` — Cargar modelos 3D

**Formato OBJ** (Wavefront, estándar desde los 80s):

```
v x y z      # Vértice
f v1 v2 v3   # Cara (triángulo, índices 1-based)
```

**Atención**: OBJ usa índices **1-based**. Conversión: `idx_python = idx_obj - 1`.

**Triangulación de polígonos** (fan method):
```
Cuadrado (4 vértices):       Fan triangulación:
v3------v4                   v3------v4
|       |                     |\      |
|       |          →          | \  T2 |
|       |                     | T1\   |
v1------v2                   v1------v2

Polígono N vértices → (N-2) triángulos
```

**Transformaciones** (orden crítico — scale ANTES de offset):
```python
v_final = (v_original * scale) + offset
# Si inviertes el orden, el offset también se escala ❌
```

**Con modelos OBJ el BVH es obligatorio**:
```
Stanford Bunny: ~69,000 triángulos

Sin BVH: cada rayo prueba 69,000 triángulos → inviable
Con BVH: cada rayo prueba ~log₂(69,000) ≈ 17 → rápido
```

---

## 10. Path Tracing — El Pipeline Completo

### La Ecuación de Renderizado (Kajiya 1986)

```
L_out(x, ω_out) = L_e(x, ω_out) + ∫_Ω f_r(x, ω_in, ω_out) L_in(x, ω_in) cos(θ) dω_in

L_out  = Luz saliente (lo que vemos)
L_e    = Luz emitida por el objeto
f_r    = BRDF (función de reflexión del material)
L_in   = Luz entrante (recursión)
cos(θ) = Ley de Lambert
```

### Next Event Estimation (NEE) — Por qué es crítico

**Sin NEE** (path tracing puro), la probabilidad de que un rayo aleatorio golpee la pequeña luz del techo es ~1/500 a 1/1000. El 99%+ de rayos no contribuyen → imagen extremadamente ruidosa.

**Con NEE**: en cada rebote difuso se lanza un "shadow ray" **directamente hacia la luz**. La contribución directa está garantizada (si no hay oclusión), reduciendo el ruido 10-100× para el mismo número de muestras.

**`puede_ver_luz` — evitar double-counting**:

```
puede_ver_luz = True:
  Rayos primarios y especulares — SÍ pueden ver la luz directamente
  if hit.emission > 0: return emission  ✓

puede_ver_luz = False:
  Rayos difusos después de NEE — NO pueden ver la luz directamente
  if hit.emission > 0: return Vec3(0,0,0)  ✓
  Razón: la luz ya fue contada por NEE. Contarla de nuevo = 2× brillo ❌
```

### Fórmula NEE

```
L_direct = Σ_luces [ E_luz × ρ_superficie × cos(θ_superficie) × (Ω / π) ]

Ω = ángulo sólido subtendido por la luz:
    Ω = (Área_luz × cos(θ_luz)) / distancia²
```

**Ejemplo numérico**:
```
Superficie en piso (y=100), normal (0,1,0)
Luz en techo (y=554), área 130×105=13,650 unidades²
Distancia = 454, emisión = Vec3(15,15,15)

Ω = (13,650 × 1.0) / 454² ≈ 0.066 sr

L_direct = Vec3(15,15,15) × Vec3(0.73,0.73,0.73) × 1.0 × (0.066 / π)
         ≈ Vec3(0.23, 0.23, 0.23)
```

### Ley de Snell vectorial (para dieléctricos)

```
η₁ sin(θ₁) = η₂ sin(θ₂)

Implementación vectorial:
  cos_theta = min((-ray_dir)·normal, 1.0)
  r_perp    = (ray_dir + normal * cos_theta) * (η₁/η₂)
  r_parallel = normal * -√(1 - |r_perp|²)
  refracted  = r_perp + r_parallel

Reflexión Total Interna: si (η₁/η₂) * sin(θ₁) > 1 → sin(θ₂) imposible → solo reflexión
```

**Ejemplo**: vidrio → aire (1.5→1.0), θ_crítico = arcsin(1/1.5) = 41.8°. Por encima → reflexión perfecta (por eso el vidrio se ve como espejo en los bordes).

### Aproximación de Fresnel-Schlick

Determina probabilísticamente si reflejar o refractar:

```
R(θ) = R₀ + (1 - R₀)(1 - cos θ)⁵
R₀   = ((η₁ - η₂)/(η₁ + η₂))²

Ejemplo vidrio (1.5) desde aire:
  R₀ = ((1-1.5)/(1+1.5))² ≈ 0.04

  θ = 0° (perpendicular): R = 4%   → 96% refracta
  θ = 45°:                R ≈ 4.2%
  θ = 90° (rasante):      R = 100% → 0% refracta (espejo)

→ Por eso el agua se ve más reflectiva cuando la miras de lado ✓
```

### Árbol de recursión — ejemplo completo

```
Cámara → Rayo (depth=4)
           ↓ hit Esfera ROJA difusa
           ├─ NEE: Vec3(0.5, 0.0, 0.0)  ← luz directa roja
           └─ Rebote difuso (depth=3, puede_ver_luz=False)
               ↓ hit Pared BLANCA
               ├─ NEE: Vec3(0.3, 0.3, 0.3)
               └─ Rebote (depth=2)
                   ↓ hit Pared VERDE
                   ├─ NEE: Vec3(0.0, 0.2, 0.0)
                   └─ Rebote (depth=1)
                       ↓ hit Techo
                       ├─ NEE: Vec3(0.1, 0.1, 0.1)
                       └─ Rebote (depth=0) → Vec3(0,0,0)  ← CASO BASE

Desenrollando:
  depth=1: Vec3(0.1,0.1,0.1) + Vec3(0,0,0) × blanco  = Vec3(0.10, 0.10, 0.10)
  depth=2: Vec3(0.0,0.2,0.0) + Vec3(0.10,0.10,0.10) × verde
         = Vec3(0.00,0.20,0.00) + Vec3(0.012,0.045,0.015) = Vec3(0.012, 0.245, 0.015)
  depth=3: Vec3(0.3,0.3,0.3) + Vec3(0.012,0.245,0.015) × blanco
         = Vec3(0.309, 0.479, 0.311)
  depth=4: Vec3(0.5,0.0,0.0) + Vec3(0.309,0.479,0.311) × rojo
         = Vec3(0.5,0,0) + Vec3(0.201,0.031,0.020)
         = Vec3(0.701, 0.031, 0.020)  ← ROJO con tinte de color bleeding ✓
```

### Gamma correction

Los displays no son lineales: una entrada de 0.5 no produce 50% de brillo visual.

```
Corrección sRGB (γ = 2.2):
  rgb_display = rgb_linear^(1/2.2)

Ejemplo: Vec3(0.5, 0.3, 0.1) en espacio lineal
  r = 0.5^0.4545 ≈ 0.73
  g = 0.3^0.4545 ≈ 0.58
  b = 0.1^0.4545 ≈ 0.35

Sin gamma: imagen oscura, tonos medios comprimidos
Con gamma: imagen correcta con luminosidad natural ✓
```

### Renderizado paralelo

El path tracing es **embarazosamente paralelo** — cada fila de píxeles es completamente independiente. Se usa `multiprocessing.Pool` para evadir el GIL de Python y aprovechar todos los núcleos.

```
Eficiencia (Ley de Amdahl, 99% paralelizable):
  N=4  cores → ~3.7×
  N=8  cores → ~7.2×
  N=16 cores → ~13.5×
```

Modo secuencial (`use_parallel=False`) es útil para debugging: el output de `print` sale ordenado y sin interleaving entre procesos.

### Stratified Sampling — antialiasing mejorado

En lugar de muestras completamente aleatorias (que producen clustering), se divide el píxel en una cuadrícula y se toma una muestra aleatoria dentro de cada celda:

```
Random puro (100 muestras):         Stratified (10×10):
  . . . . ..                          . . . . . . . . . .
  .   . .                             . . . . . . . . . .
  .  .. .                             . . . . . . . . . .
  clustering → ruido irregular        uniforme → 30-50% menos ruido
```

---

## Resumen de Complejidades

| Componente | Construcción | Consulta |
|------------|-------------|---------|
| AABB hit | — | O(1), ~6 comparaciones |
| Sphere hit | — | O(1), ~30 operaciones |
| Triangle hit (Möller-Trumbore) | — | O(1), ~40 operaciones |
| BVH | O(N log N) | O(log N) promedio |
| `random_in_unit_sphere` | — | O(1) amortizado (~1.91 intentos) |
| `random_in_unit_disk` | — | O(1) amortizado (~1.27 intentos) |
| `load_obj` | O(V + F) | — |

## Conexión con Tecnologías Profesionales

Los algoritmos de este proyecto son los **mismos fundamentos** que usan:

| Sistema | Empresa | Usos |
|---------|---------|------|
| Hyperion | Disney | Frozen, Moana, Encanto |
| RenderMan | Pixar | Toy Story, Soul |
| Arnold | Autodesk | Industria VFX/cine |
| Cycles | Blender Foundation | Open source |
| OptiX / RT Cores | NVIDIA | GPU ray tracing (RTX) |
| Embree | Intel | CPU ray tracing de alto rendimiento |

La diferencia con producciones profesionales está en optimizaciones avanzadas (MIS, bidirectional path tracing, Russian roulette, denoising por ML, aceleración GPU), no en los algoritmos fundamentales.
