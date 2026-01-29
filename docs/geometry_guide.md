# Guía Completa de geometry.py: Conceptos Matemáticos y Técnicos

## Tabla de Contenidos

1. [Introducción](#introducción)
2 [BVH - Bounding Volume Hierarchy](#bvh---bounding-volume-hierarchy)
3. [AABB - Axis-Aligned Bounding Box](#aabb---axis-aligned-bounding-box)
4. [HitRecord - Registro de Intersección](#hitrecord---registro-de-intersección)
5. [Sphere - Geometría Esférica](#sphere---geometría-esférica)
6. [Quad - Cuadriláteros Planos](#quad---cuadriláteros-planos)
7. [Triangle - Geometría Triangular](#triangle---geometría-triangular)
8. [Conceptos Avanzados](#conceptos-avanzados)

---

## Introducción

El archivo `geometry.py` implementa las estructuras geométricas y algoritmos fundamentales para un **ray tracer** (trazador de rayos). Un ray tracer simula el comportamiento de la luz trazando rayos desde la cámara hacia la escena y calculando sus interacciones con los objetos.

### ¿Qué es Ray Tracing?

El ray tracing es una técnica de renderizado que simula la física de la luz:

1. **Lanza rayos** desde la cámara a través de cada píxel
2. **Detecta intersecciones** con objetos en la escena
3. **Calcula iluminación** en los puntos de impacto
4. **Simula rebotes** de luz (reflexión, refracción)
5. **Acumula color** para cada píxel

**Problema Principal**: En una escena con N objetos, cada rayo debe probar intersección con TODOS los objetos → O(N) por rayo.

**Solución**: Estructuras de aceleración espacial como el BVH.

---

## BVH - Bounding Volume Hierarchy

### ¿Qué es un BVH?

El **BVH** (Jerarquía de Volúmenes Envolventes) es una estructura de datos de **árbol binario** que organiza objetos geométricos en el espacio 3D para acelerar las pruebas de intersección rayo-objeto.

### Analogía: Biblioteca de Libros

Imagina que tienes 1,000 libros y quieres encontrar uno específico:

- **Sin organización (O(N))**: Revisa TODOS los 1,000 libros uno por uno
- **Con estanterías (BVH)**: 
  - 10 estanterías con 100 libros cada una
  - Primero identificas la estantería correcta (1 prueba)
  - Luego buscas en esa estantería (100 pruebas)
  - Total: ~101 pruebas vs 1,000

### Estructura del BVH

```
                  [Nodo Raíz - Caja grande que contiene TODO]
                 /                                          \
    [Nodo Izq - Mitad izquierda]                  [Nodo Der - Mitad derecha]
    /                         \                    /                        \
[Esfera A]  [Caja con esferas B y C]    [Esfera D]                   [Esfera E]
```

Cada nodo tiene:
- **Caja envolvente (AABB)**: Volumen rectangular que contiene todos sus descendientes
- **Hijo izquierdo**: Sub-árbol o objeto
- **Hijo derecho**: Sub-árbol o objeto

### Algoritmo de Construcción

```python
bvh_root = BVHNode.create(objects)
```

**Paso a paso**:

1. **Selecciona un eje aleatorio** (X, Y o Z)
   - ¿Por qué aleatorio? Evita casos degenerados donde todos los objetos están alineados

2. **Ordena objetos** según su posición en ese eje
   ```python
   # Si elegimos eje X, ordena por la coordenada X de cada caja
   objects.sort(key=lambda obj: obj.bounding_box().min.x)
   ```

3. **Divide por la mitad** y construye recursivamente
   ```python
   mid = len(objects) // 2
   left = BVHNode.create(objects[:mid])    # Primera mitad
   right = BVHNode.create(objects[mid:])   # Segunda mitad
   ```

4. **Crea caja envolvente** que contiene ambos hijos
   ```python
   full_box = left.bounding_box().union(right.bounding_box())
   ```

**Casos Base**:
- **1 objeto**: Retorna el objeto directamente (no crea nodo)
- **2 objetos**: Un nodo con ambos como hijos

### Algoritmo de Búsqueda (Traversal)

```python
hit = bvh_root.hit(ray, 0.001, float('inf'))
```

**Poda Espacial (Pruning)**:

```
¿Rayo intersecta caja del nodo?
    ├─ NO → retorna None (descarta TODOS los descendientes) 🚀
    └─ SÍ → prueba ambos hijos
        ├─ Prueba hijo izquierdo
        ├─ Si hay hit izquierdo, actualiza t_max para hijo derecho
        └─ Retorna el hit más cercano
```

**Ejemplo numérico**:

Imagina 8 esferas en una escena:

```
Sin BVH:
- Rayo 1: prueba 8 esferas → 8 pruebas
- Rayo 2: prueba 8 esferas → 8 pruebas
- ...
- 1000 rayos × 8 esferas = 8,000 pruebas

Con BVH (árbol balanceado):
- Nivel 1: 1 caja (toda la escena)
- Nivel 2: 2 cajas (mitades)
- Nivel 3: 4 cajas (cuartos)
- Nivel 4: 8 esferas (hojas)

Promedio por rayo:
- Prueba caja nivel 1: ✓ intersecta
- Prueba caja nivel 2 izq: ✗ NO intersecta → descarta 4 esferas
- Prueba caja nivel 2 der: ✓ intersecta
- Prueba caja nivel 3: ✓ intersecta → descarta 2 esferas
- Prueba 1-2 esferas finales

Total: ~3-4 pruebas vs 8 → **Reducción del 50-60%**
```

### Complejidad Computacional

| Operación | Sin BVH | Con BVH |
|-----------|---------|---------|
| Construcción | - | O(N log N) |
| Búsqueda (mejor caso) | O(N) | O(log N) |
| Búsqueda (peor caso) | O(N) | O(N) |
| Búsqueda (promedio) | O(N) | O(log N) |
| Memoria | O(N) | O(N) |

**Ejemplo con números reales**:
- Escena con 1,000,000 objetos
- Sin BVH: 1,000,000 pruebas por rayo
- Con BVH: ~log₂(1,000,000) = **~20 pruebas** por rayo
- **Aceleración de 50,000x**

---

## AABB - Axis-Aligned Bounding Box

### ¿Qué es una AABB?

Una **AABB** (Caja Envolvente Alineada con los Ejes) es el volumen rectangular **mínimo** alineado con los ejes X, Y, Z que contiene completamente un objeto o conjunto de objetos.

### Visualización

```
        max (x_max, y_max, z_max)
          +-------------------------+
         /|                        /|
        / |                       / |
       /  |                      /  |
      +-------------------------+   |
      |   |                     |   |
      |   |     OBJETO 3D       |   |
      |   +---------------------|---+
      |  /                      |  /
      | /                       | /
      |/                        |/
      +-------------------------+
    min (x_min, y_min, z_min)
```

Se define por **2 puntos**:
- **min**: `(x_mín, y_mín, z_mín)` - esquina "inferior-trasera-izquierda"
- **max**: `(x_máx, y_máx, z_máx)` - esquina "superior-frontal-derecha"

### ¿Por qué "Alineada con los Ejes"?

**Alineada**: Las caras de la caja son siempre **perpendiculares** a los ejes X, Y, Z (no puede rotar).

**Ventajas**:
- ✅ Intersección rayo-caja **muy rápida** (solo comparaciones)
- ✅ Construcción **trivial** (solo min/max)
- ✅ Operación unión **instantánea**

**Desventaja**:
- ❌ Puede contener mucho "espacio vacío" para objetos rotados

**Comparación con OBB** (Oriented Bounding Box - caja orientada):
```
Objeto diagonal dentro de AABB:     Mismo objeto en OBB:
    max                                  /-------------\
    +-----------------+                 /               \
    | ####         ###|                /    #########    \
    |  ####     ####  |               min   #########    max
    |   #########     |                \    #########    /
    |    #######      |                 \               /
    |     ####        |                  \-------------/
    min--------+

AABB desperdicia ~40% espacio      OBB ajustada (~10% desperdicio)
```

### Algoritmo de Intersección: Slabs

El test de intersección usa el **algoritmo de slabs** (Kay & Kajiya, 1986):

**Conceptalización**: Una caja 3D es la intersección de 3 pares de planos paralelos (slabs):

```
Slab X: entre x_min y x_max
Slab Y: entre y_min y y_max
Slab Z: entre z_min y z_max
```

**Para cada eje** (X, Y, Z):

1. Calcula dónde el rayo **entra** al slab (`t0`)
2. Calcula dónde el rayo **sale** del slab (`t1`)

```python
# Para el eje X:
invD = 1.0 / ray.direction.x
t0 = (box.min.x - ray.origin.x) * invD  # Entrada
t1 = (box.max.x - ray.origin.x) * invD  # Salida

# Si la dirección es negativa, intercambiar
if invD < 0:
    t0, t1 = t1, t0
```

3. **Intersecta los intervalos** de los 3 ejes:

```
Intervalos:
  Eje X: [t0_x, t1_x]
  Eje Y: [t0_y, t1_y]
  Eje Z: [t0_z, t1_z]

Intervalo final: [max(t0_x, t0_y, t0_z), min(t1_x, t1_y, t1_z)]

Si intervalo está vacío → NO hay intersección
```

**Ejemplo numérico**:

```python
ray.origin = (0, 0, 0)
ray.direction = (1, 1, 1).normalize()  # Diagonal
box.min = (2, 1, 3)
box.max = (5, 4, 6)

# Eje X:
t0_x = (2 - 0) / (1/√3) = 2√3 ≈ 3.46
t1_x = (5 - 0) / (1/√3) = 5√3 ≈ 8.66

# Eje Y:
t0_y = (1 - 0) / (1/√3) = 1√3 ≈ 1.73
t1_y = (4 - 0) / (1/√3) = 4√3 ≈ 6.93

# Eje Z:
t0_z = (3 - 0) / (1/√3) = 3√3 ≈ 5.20
t1_z = (6 - 0) / (1/√3) = 6√3 ≈ 10.39

# Intersección:
t_min = max(3.46, 1.73, 5.20) = 5.20  ✓ Entra en z=3
t_max = min(8.66, 6.93, 10.39) = 6.93  ✓ Sale en y=4

Intervalo válido: [5.20, 6.93] → HAY INTERSECCIÓN ✓
```

### Operación Union

Combina dos cajas en una que las contiene a ambas:

```python
box1 = AABB(Vec3(0,0,0), Vec3(2,2,2))
box2 = AABB(Vec3(1,1,1), Vec3(4,3,5))

# Union:
min_combinado = Vec3(
    min(0, 1) = 0,
    min(0, 1) = 0,
    min(0, 1) = 0
)
max_combinado = Vec3(
    max(2, 4) = 4,
    max(2, 3) = 3,
    max(2, 5) = 5
)

box_union = AABB(Vec3(0,0,0), Vec3(4,3,5))
```

---

## HitRecord - Registro de Intersección

### Propósito

Cuando un rayo impacta un objeto, necesitamos **mucha información** para calcular la iluminación correctamente. El `HitRecord` almacena todos estos datos.

### Atributos Detallados

#### 1. `t` (float) - Parámetro del Rayo

**Definición**: Distancia desde el origen del rayo hasta el punto de impacto.

```python
ray.origin = (0, 0, 0)
ray.direction = (1, 0, 0)  # Hacia la derecha

# Si t = 5:
hit_point = ray.origin + t * ray.direction
          = (0,0,0) + 5 * (1,0,0)
          = (5, 0, 0)
```

**Importancia de t**:
- Valores **menores** de t → objetos más **cercanos** a la cámara
- Usado para determinar **oclusión** (qué objeto está delante)

**Ejemplo**:
```
Cámara en (0,0,0) → Rayo hacia (1,0,0)
├─ Esfera A en (3,0,0): t=2 ✓ MÁS CERCA
└─ Esfera B en (10,0,0): t=9 (oculta por A, se descarta)
```

#### 2. `point` (Vec3) - Punto de Intersección

Coordenadas 3D exactas donde el rayo tocó la superficie.

```python
point = Vec3(5.23, -1.45, 8.92)
```

#### 3. `normal` (Vec3) - Vector Normal

**Vector perpendicular** a la superficie en el punto de impacto (siempre **normalizado**, magnitud=1).

```
Esfera de radio 2 centrada en (0,0,0):
  ├─ Punto de impacto: (2, 0, 0)
  └─ Normal: (1, 0, 0)  # Apunta radialmente hacia afuera
```

**Uso**: 
- Cálculo de **iluminación** (Ley de Lambert: `cos(θ) = normal · luz`)
- Dirección de **reflexión/refracción**

#### 4. `color` (Vec3) - Albedo

Color **base** del material (RGB en rango [0-1]).

```python
color = Vec3(0.8, 0.3, 0.3)  # Rojo apagado
      = (R=0.8, G=0.3, B=0.3)
```

**Albedo** = Fracción de luz que se **refleja** (no se absorbe).

**Ejemplos**:
- `Vec3(0.9, 0.9, 0.9)` → Casi blanco (refleja 90% de la luz)
- `Vec3(0.1, 0.1, 0.1)` → Casi negro (absorbe 90%, refleja 10%)
- `Vec3(0, 1, 0)` → Verde puro (absorbe rojo y azul completamente)

#### 5. `emission` (Vec3) - Luz Emitida

Luz que el objeto **emite** por sí mismo (para fuentes de luz).

```python
emission = Vec3(15, 15, 15)  # Luz blanca muy brillante
```

**Valores típicos**:
- `Vec3(0, 0, 0)` → No emite (objeto normal)
- `Vec3(1, 1, 1)` → Luz blanca suave
- `Vec3(15, 15, 15)` → Luz blanca intensa (bombilla)
- `Vec3(50, 40, 20)` → Luz amarillenta muy intensa (sol)

**¿Por qué valores > 1?**
En HDR (High Dynamic Range), los valores pueden exceder 1 para representar brillos intensos.

#### 6. `is_metal` (bool) - Material Metálico

Si `True`, el material es **reflectante especular** (como un espejo).

**Metales vs No-Metales**:
- **Metales**: Reflejan la luz como espejos (reflexión especular)
- **No-metales** (dieléctricos): Reflejo difuso + algo de especular

#### 7. `fuzz` (float) - Rugosidad

Controla cuán "borrosas" son las reflexiones metálicas.

```
fuzz = 0.0:  Espejo perfecto (reflexión perfectamente nítida)
  [Imagen Mental: Espejo de baño]

fuzz = 0.3:  Metal pulido (reflexión ligeramente borrosa)
  [Imagen Mental: Acero inoxidable]

fuzz = 0.7:  Metal rugoso (reflexión muy difusa)
  [Imagen Mental: Aluminio cepillado]

fuzz = 1.0:  Metal muy rugoso (casi difuso)
  [Imagen Mental: Hierro oxidado]
```

**Implementación**:
```python
# En el rebote del rayo:
reflected_dir = reflect(ray.direction, normal)

if is_metal and fuzz > 0:
    # Añade aleatoriedad proporcional a fuzz
    reflected_dir += fuzz * random_unit_vector()
```

**Efecto del aumento**:
- `fuzz ↑` → Reflexiones más **borrosas/difusas**
- `fuzz ↓` → Reflexiones más **nítidas/especulares**

#### 8. `is_dielectric` (bool) - Material Transparente

`True` para materiales como vidrio, agua, diamante que **refractan** la luz.

**Comportamiento**:
- Parte de la luz se **refleja** (como en un espejo)
- Parte se **refracta** (atraviesa y se dobla)

**Probabilidad de reflexión** (Ecuaciones de Fresnel):
- Ángulo rasante → más reflexión (ej: mirar agua desde arriba vs de lado)

#### 9. `ior` (float) - Índice de Refracción

Controla **cuánto se dobla** la luz al atravesar el material.

**Valores comunes**:

| Material | IOR | Efecto Visual |
|----------|-----|---------------|
| Vacío/Aire | 1.0 | Sin refracción |
| Agua | 1.33 | Doblez suave |
| Vidrio | 1.5 | Doblez medio |
| Cristal | 1.7 | Doblez notable |
| Diamante | 2.4 | Doblez intenso (brillo) |

**Ley de Snell**:
```
n₁ * sin(θ₁) = n₂ * sin(θ₂)

n₁ = IOR del medio 1 (ej: aire = 1.0)
n₂ = IOR del medio 2 (ej: vidrio = 1.5)
θ₁ = ángulo de incidencia
θ₂ = ángulo de refracción
```

**Ejemplo numérico**:
```
Rayo pasa de aire (IOR=1.0) a vidrio (IOR=1.5)
Ángulo de entrada: θ₁ = 45°

1.0 * sin(45°) = 1.5 * sin(θ₂)
0.707 = 1.5 * sin(θ₂)
sin(θ₂) = 0.707 / 1.5 = 0.471
θ₂ = arcsin(0.471) ≈ 28.1°

El rayo se "dobla" hacia la normal → de 45° a 28.1°
```

**Efecto del aumento de IOR**:
- `IOR ↑` → Luz se dobla **más** hacia la normal
- Materiales con IOR alto (diamante) → **más brillo y dispersión**

---

## Sphere - Geometría Esférica

### Definición Matemática

Una esfera es el conjunto de todos los puntos que están a una **distancia constante** (radio) de un punto central:

```
||P - C||² = r²

P = cualquier punto en la superficie
C = centro de la esfera
r = radio
|| || = magnitud del vector (distancia)
```

### Intersección Rayo-Esfera

**Problema**: ¿Dónde intersecta un rayo una esfera?

**Derivación matemática completa**:

1. **Rayo parametrizado**:
   ```
   Ray(t) = O + t*D
   O = origen del rayo
   D = dirección del rayo (normalizada)
   t = parámetro (distancia a lo largo del rayo)
   ```

2. **Esfera**:
   ```
   ||P - C||² = r²
   ```

3. **Sustitución** (P = punto en el rayo):
   ```
   ||O + t*D - C||² = r²
   ```

4. **Sea** `oc = O - C`:
   ```
   ||oc + t*D||² = r²
   ```

5. **Expandir el producto escalar**:
   ```
   (oc + t*D) · (oc + t*D) = r²
   oc·oc + 2t(D·oc) + t²(D·D) = r²
   (D·D)t² + 2(D·oc)t + (oc·oc - r²) = 0
   ```

6. **Ecuación cuadrática**: `at² + bt + c = 0`
   ```
   a = D · D
   b = 2(D · oc)
   c = oc·oc - r²
   ```

7. **Fórmula cuadrática**:
   ```
   t = (-b ± √(b² - 4ac)) / (2a)
   ```

8. **Discriminante**: `Δ = b² - 4ac`
   ```
   Δ < 0  → No hay intersección (rayo pasa de largo)
   Δ = 0  → Intersección tangente (roza la esfera)
   Δ > 0  → Dos intersecciones (entra y sale)
   ```

**Ejemplo numérico**:

```python
# Rayo
origin = Vec3(0, 0, 0)
direction = Vec3(1, 0, 0)  # Hacia la derecha

# Esfera
center = Vec3(5, 0, 0)
radius = 2

# Cálculos
oc = origin - center = Vec3(-5, 0, 0)
a = direction · direction = 1
b = 2 * (direction · oc) = 2 * (1*-5 + 0*0 + 0*0) = -10
c = oc·oc - r² = 25 - 4 = 21

# Discriminante
Δ = b² - 4ac = 100 - 84 = 16  ✓ Hay intersección

# Raíces
sqrt(Δ) = 4
t₁ = (-(-10) - 4) / 2 = 6 / 2 = 3  ✓ Entrada
t₂ = (-(-10) + 4) / 2 = 14 / 2 = 7  ✓ Salida

# Puntos de intersección
P₁ = origin + 3 * direction = (3, 0, 0)  # Entra
P₂ = origin + 7 * direction = (7, 0, 0)  # Sale

# Normal en P₁
normal = (P₁ - center) / radius
       = (3 - 5, 0, 0) / 2
       = (-2, 0, 0) / 2
       = (-1, 0, 0)  # Apunta hacia el origen
```

### Muestreo de Superficie

#### Muestreo Uniforme

Para generar un punto aleatorio **uniformemente distribuido** en la superficie de una esfera:

**Problema ingenuo** (INCORRECTO):
```python
# ESTO ESTÁ MAL:
theta = random() * 2π
phi = random() * π

# Genera MÁS puntos cerca de los polos!
```

**Solución correcta**:
```python
theta = random() * 2π  # Ángulo azimutal
u = random()  # Variable uniforme [0,1]
phi = arccos(2*u - 1)  # Ángulo polar

# Conversión a cartesianas:
x = sin(phi) * cos(theta)
y = sin(phi) * sin(theta)
z = cos(phi)

point = center + radius * Vec3(x, y, z)
```

**¿Por qué `phi = arccos(2*u - 1)`?**

La **densidad de área** en una esfera varía con la latitud:
- Ecuador: circunferencia máxima
- Polos: área mínima (un punto)

Si usamos `phi = u*π` directamente, obtendríamos:
```
Densidad de puntos:
  Polo Norte (φ=0°): MUCHOS puntos
  Ecuador (φ=90°): pocos puntos
  Polo Sur (φ=180°): MUCHOS puntos

  [Concentración en polos = MAL]
```

La transformación `arccos(2*u - 1)` **compensa** esta densidad desigual, distribuyendo uniformemente en **área superficial**.

### Muestreo de Ángulo Sólido

**Importance Sampling** para luces esféricas.

**Problema**: Queremos enviar rayos hacia una luz esférica, pero NO queremos desperdiciar rayos que se van en direcciones aleatorias.

**Conceptos clave**:

1. **Ángulo Sólido (Ω)**: "Cantidad de cielo" que ocupa un objeto visto desde un punto
   - Medido en **estereorradianes** (sr)
   - Esfera completa = 4π sr
   - Hemisferio = 2π sr

2. **Para una esfera vista desde un punto**:
   ```
   Ω = 2π(1 - cos(θ_max))
   
   donde:
   sin(θ_max) = R / d
   R = radio de la esfera
   d = distancia al centro
   ```

**Ejemplo numérico**:

```python
# Luz esférica
light_center = Vec3(10, 10, 10)
light_radius = 2

# Punto desde donde observamos
hit_point = Vec3(0, 0, 0)

# Distancia
direction_to_center = light_center - hit_point = (10, 10, 10)
dist_sq = 10² + 10² + 10² = 300
dist = √300 ≈ 17.32

# Ángulo del cono
sin_theta_max = radius / dist = 2 / 17.32 ≈ 0.115
theta_max = arcsin(0.115) ≈ 6.6°

# Ángulo sólido
cos_theta_max = √(1 - sin²(θ)) ≈ 0.993
Ω = 2π(1 - 0.993) = 2π × 0.007 ≈ 0.044 sr

# Interpretación: La luz ocupa solo 0.044/4π ≈ 0.35% del cielo
```

**Efecto de la distancia**:

```
Distancia d=5:
  θ_max = arcsin(2/5) ≈ 23.6°
  Ω ≈ 0.65 sr (luz ocupa ~5% del cielo)

Distancia d=20:
  θ_max = arcsin(2/20) ≈ 5.7°
  Ω ≈ 0.03 sr (luz ocupa ~0.24% del cielo)

Distancia d=100:
  θ_max ≈ 1.1°
  Ω ≈ 0.001 sr (luz casi un punto)

d ↑ → θ_max ↓ → Ω ↓ (luz ocupa menos "cielo")
```

---

## Quad - Cuadriláteros Planos

### Definición Paramétrica

Un Quad se define por:
- **Q**: Punto origen (una esquina)
- **u**: Vector que define el primer lado
- **v**: Vector que define el segundo lado

```
Los 4 vértices son:
Q ────────u────────→ Q+u
│                     │
v                     v
│                     │
↓                     ↓
Q+v ──────u──────→ Q+u+v
```

**Parametrización**:
```
P(s, t) = Q + s*u + t*v

donde s, t ∈ [0, 1]
```

**Ejemplos**:
```
s=0, t=0 → P = Q          (esquina origen)
s=1, t=0 → P = Q+u        (esquina derecha)
s=0, t=1 → P = Q+v        (esquina arriba)
s=1, t=1 → P = Q+u+v      (esquina diagonal)
s=0.5, t=0.5 → P = Q+0.5u+0.5v (centro)
```

### Intersección Rayo-Quad

**Algoritmo en 2 fases**:

#### Fase 1: Intersección Rayo-Plano

El quad está contenido en un **plano infinito**:

**Ecuación del plano**:
```
n · P = D

n = normal del plano = (u × v).normalize()
D = n · Q  (distancia desde el origen)
```

**Intersección rayo-plano**:
```
Rayo: R(t) = O + t*dir

Sustituyendo en el plano:
n · (O + t*dir) = D
n·O + t(n·dir) = D
t = (D - n·O) / (n·dir)

Si n·dir ≈ 0 → rayo paralelo al plano (no intersecta)
```

**Ejemplo numérico**:
```python
# Quad horizontal en y=2
Q = Vec3(0, 2, 0)
u = Vec3(4, 0, 0)
v = Vec3(0, 0, 3)

# Normal (apunta hacia arriba)
n = u × v = Vec3(0, 4*3, 0) = Vec3(0, 12, 0)
n.normalize() = Vec3(0, 1, 0)

# Parámetro D
D = n · Q = 0*0 + 1*2 + 0*0 = 2

# Ecuación del plano: y = 2

# Rayo desde abajo
ray.origin = Vec3(1, 0, 1)
ray.direction = Vec3(0, 1, 0)  # Hacia arriba

# Intersección
t = (2 - n·origin) / (n·direction)
  = (2 - 0) / (1)
  = 2

# Punto de intersección
P = origin + 2 * direction = Vec3(1, 2, 1) ✓
```

#### Fase 2: Test de Contención

¿Está P dentro de los límites del quad?

**Coordenadas baricéntricas**:
```
P = Q + alpha*u + beta*v

Resolver para alpha, beta:
  P - Q = alpha*u + beta*v

Usando productos cruz:
  alpha = w · ((P-Q) × v)
  beta = w · (u × (P-Q))

donde w = (u × v) / |u × v|²  (precalculado)
```

**Condición para estar dentro**:
```
0 ≤ alpha ≤ 1  AND  0 ≤ beta ≤ 1
```

**Continuando el ejemplo**:
```python
P = Vec3(1, 2, 1)  # Del paso anterior
planar_hit = P - Q = Vec3(1, 0, 1)

# Cálculo de w (precalculado en __init__)
n_vec = u × v = Vec3(0, 12, 0)
w = n_vec / (n_vec · n_vec)
  = Vec3(0, 12, 0) / 144
  = Vec3(0, 1/12, 0)

# Alpha
planar_hit × v = Vec3(1,0,1) × Vec3(0,0,3)
               = Vec3(0*3-1*0, 1*0-1*3, 1*0-0*1)
               = Vec3(0, -3, 0)
alpha = w · Vec3(0,-3,0) = 0*0 + (1/12)*(-3) + 0*0 = -0.25

# Alpha < 0 → Fuera del quad ✗
```

### Muestreo de Ángulo Sólido (Luces de Área)

Las luces de área (como paneles LED, ventanas) son muy comunes en escenas realistas.

**Algoritmo**:

1. **Punto aleatorio en el quad**:
   ```python
   r1, r2 = random(), random()
   random_point = Q + r1*u + r2*v
   ```

2. **Dirección hacia el punto**:
   ```python
   direction = (random_point - hit_point).normalize()
   distance² = ||random_point - hit_point||²
   ```

3. **Ángulo sólido**:
   ```
   área = |u × v|  (magnitud del producto cruz)
   cos_light = |normal · direction|  (ángulo con la luz)
   
   Ω = (área * cos_light) / distancia²
   ```

**Interpretación física**:

```
Luz de frente (cos ≈ 1):
  ┌────────┐
  │  LUZ   │ ← Normal apunta hacia nosotros
  └────────┘
  Ángulo sólido MÁXIMO

Luz de lado (cos ≈ 0):
  │  LUZ   │ ← Normal perpendicular
  Ángulo sólido MÍNIMO (casi 0)

cos_light corrige la proyección aparente
```

**Ejemplo numérico**:
```python
# Luz de área 2×2 en el techo
Q = Vec3(-1, 5, -1)
u = Vec3(2, 0, 0)
v = Vec3(0, 0, 2)
normal = Vec3(0, -1, 0)  # Apunta hacia abajo

# Punto en el suelo
hit_point = Vec3(0, 0, 0)

# Punto aleatorio (r1=0.5, r2=0.5 → centro)
random_point = Q + 0.5*u + 0.5*v = Vec3(0, 5, 0)

# Dirección y distancia
direction_vec = Vec3(0, 5, 0)
distance = 5
direction = Vec3(0, 1, 0)

# Área
area = |u × v| = |Vec3(0, 0, 4)| = 4

# Coseno
cos_light = |normal · direction| = |(-1) * 1| = 1

# Ángulo sólido
Ω = (4 * 1) / 25 = 0.16 sr

# Si nos alejamos a y=10:
distance = 10
Ω = (4 * 1) / 100 = 0.04 sr  (¼ del anterior)

distance ↑ → Ω ↓² (escala cuadráticamente)
```

---

## Triangle - Geometría Triangular

### Importancia

Los triángulos son la **primitiva fundamental** en gráficos 3D porque:

1. **Universales**: Cualquier superficie 3D puede aproximarse con triángulos
2. **Simples**: 3 puntos siempre definen un plano
3. **Eficientes**: Hardware GPU optimizado para triángulos
4. **Compatibilidad**: Formato estándar (.obj, .fbx, .stl)

### Coordenadas Baricéntricas

Un punto P dentro de un triángulo se puede expresar como:

```
P = v0 + u*(v1 - v0) + v*(v2 - v0)

donde:
  u ≥ 0
  v ≥ 0
  u + v ≤ 1
```

**Interpretación**:
```
(u=0, v=0) → P = v0          (vértice 0)
(u=1, v=0) → P = v1          (vértice 1)
(u=0, v=1) → P = v2          (vértice 2)

(u=0.5, v=0) → Punto medio del borde v0-v1
(u=0, v=0.5) → Punto medio del borde v0-v2
(u=0.5, v=0.5) → ¡FUERA! (u+v=1 está en el borde v1-v2)

(u=1/3, v=1/3) → Centroide (centro del triángulo)
```

### Algoritmo Möller-Trumbore

**El estándar de la industria** para intersección rayo-triángulo (1997).

**Ventajas**:
- ✅ No requiere precalcular el plano
- ✅ Calcula t y coordenadas baricéntricas simultáneamente
- ✅ Evita divisiones hasta el final (optimización)
- ✅ Solo usa productos cruz y escalares (rápido)

**Derivación matemática**:

1. **Ecuaciones simultáneas**:
   ```
   Rayo: R(t) = O + t*D
   Triángulo: T(u,v) = v0 + u*edge1 + v*edge2
   
   Intersección: R(t) = T(u,v)
   O + t*D = v0 + u*edge1 + v*edge2
   ```

2. **Reordenar**:
   ```
   O - v0 = -t*D + u*edge1 + v*edge2
   
   Sea s = O - v0:
   s = -t*D + u*edge1 + v*edge2
   ```

3. **Sistema lineal 3×3**:
   ```
   [-D, edge1, edge2] [t]   [s]
                       [u] = 
                       [v]
   ```

4. **Regla de Cramer** (usando determinantes):
   ```
   h = D × edge2
   a = edge1 · h    (determinante principal)
   
   Si |a| < ε → rayo paralelo al triángulo
   
   f = 1 / a
   u = f * (s · h)
   
   q = s × edge1
   v = f * (D · q)
   t = f * (edge2 · q)
   ```

**Implementación paso a paso**:

```python
def hit(ray, triangle):
    edge1 = v1 - v0
    edge2 = v2 - v0
    
    h = ray.direction × edge2
    a = edge1 · h
    
    # 1. Test paralelo
    if -ε < a < ε:
        return None  # Paralelo
    
    f = 1.0 / a
    s = ray.origin - v0
    
    # 2. Test u
    u = f * (s · h)
    if u < 0.0 or u > 1.0:
        return None  # Fuera
    
    # 3. Test v
    q = s × edge1
    v = f * (ray.direction · q)
    if v < 0.0 or u + v > 1.0:
        return None  # Fuera
    
    # 4. Calcular t
    t = f * (edge2 · q)
    if t < t_min or t > t_max:
        return None  # Fuera del rango
    
    # ¡Hit válido!
    point = ray.origin + t * ray.direction
    return HitRecord(t, point, normal, ...)
```

**Ejemplo numérico completo**:

```python
# Triángulo
v0 = Vec3(0, 0, 0)
v1 = Vec3(4, 0, 0)
v2 = Vec3(0, 4, 0)

# Rayo (hacia el centro del triángulo)
ray.origin = Vec3(1, 1, -5)
ray.direction = Vec3(0, 0, 1)  # Hacia adelante

# Paso 1: Edges
edge1 = v1 - v0 = Vec3(4, 0, 0)
edge2 = v2 - v0 = Vec3(0, 4, 0)

# Paso 2: h y a
h = direction × edge2
  = Vec3(0,0,1) × Vec3(0,4,0)
  = Vec3(-4, 0, 0)

a = edge1 · h
  = Vec3(4,0,0) · Vec3(-4,0,0)
  = -16  ✓ (no paralelo)

# Paso 3: f y s
f = 1 / -16 = -0.0625
s = origin - v0 = Vec3(1, 1, -5)

# Paso 4: u
u = f * (s · h)
  = -0.0625 * (Vec3(1,1,-5) · Vec3(-4,0,0))
  = -0.0625 * (-4)
  = 0.25  ✓ (en [0,1])

# Paso 5: q y v
q = s × edge1
  = Vec3(1,1,-5) × Vec3(4,0,0)
  = Vec3(0, -20, -4)

v = f * (direction · q)
  = -0.0625 * (Vec3(0,0,1) · Vec3(0,-20,-4))
  = -0.0625 * (-4)
  = 0.25  ✓ (en [0,1])

# Paso 6: Verificar u+v
u + v = 0.25 + 0.25 = 0.5 ≤ 1  ✓

# Paso 7: t
t = f * (edge2 · q)
  = -0.0625 * (Vec3(0,4,0) · Vec3(0,-20,-4))
  = -0.0625 * (-80)
  = 5  ✓

# Punto de intersección
P = origin + 5 * direction
  = Vec3(1, 1, -5) + 5*Vec3(0,0,1)
  = Vec3(1, 1, 0)

# Verificación usando baricéntricas
P = v0 + 0.25*edge1 + 0.25*edge2
  = Vec3(0,0,0) + 0.25*Vec3(4,0,0) + 0.25*Vec3(0,4,0)
  = Vec3(1, 1, 0)  ✓ CORRECTO
```

---

## Conceptos Avanzados

### 1. Shadow Acne (Acné de Sombra)

**Problema**: Cuando un rayo rebota en una superficie, puede re-intersectarse con la misma superficie debido a errores de punto flotante.

```
Superficie en y=0 (exacta)

Rayo rebota en (x=5, y=0.0000001, z=3)  ← Error numérico!
              ↑
        [Re-impacta la superficie]

Resultado: Puntos negros o píxeles ruidosos
```

**Solución**: `t_min = 0.001`

```python
hit = surface.hit(ray, t_min=0.001, t_max=inf)
#                        ↑
#            Ignora intersecciones muy cercanas
```

**Efecto**:
- `t_min = 0` → Acné de sombra (MUY MALO)
- `t_min = 0.001` → Sin acné (CORRECTO)
- `t_min = 1.0` → Gaps visibles (DEMASIADO GRANDE)

### 2. Importance Sampling

**Problema Naive**: M muestrear iluminación aleatoriamente en todo el hemisferio desperdicade muchos rayos.

```
Muestreo Uniforme (MALO):
  1000 rayos × 1 luz pequeña
  → Solo ~10 rayos golpean la luz
  → Desperdicia 99% de los rayos
  → Imagen MUY ruidosa
```

**Importance Sampling** (BUENO):
Concentra muestras donde más importan (hacia las luces).

```
Muestreo Directo de Luces:
  10 rayos hacia la luz (garantizados)
  → Imagen limpia con menos rayos
  → ~100× menos ruido
```

**Corrección matemática**:
```
Contribución = (BRDF * emisión * cos_surface) / PDF

PDF = Probability Density Function
Para muestreo de ángulo sólido:
  PDF = 1 / Ω

donde Ω = ángulo sólido de la luz
```

### 3. Índice de Refracción y Reflexión Total Interna

**Reflexión Total Interna** ocurre cuando la luz no puede refractarse porque el ángulo es demasiado grande.

**Condición** (de la Ley de Snell):
```
sin(θ₂) = (n₁/n₂) * sin(θ₁)

Si (n₁/n₂) * sin(θ₁) > 1 → ¡sin(θ₂) imposible!
→ Reflexión total interna
```

**Ejemplo**:
```
Rayo en agua (n=1.33) hacia aire (n=1.0)
Ángulo crítico: θ_c = arcsin(n₂/n₁) = arcsin(1/1.33) ≈ 48.6°

θ₁ < 48.6° → Refracción + algo de reflexión
θ₁ > 48.6° → Reflexión total (100%, como espejo)

[Por eso cuando buceas y miras hacia arriba en ángulo,
 ves reflejos del fondo submarino en la superficie]
```

### 4. Ecuaciones de Fresnel

Controlan **cuánta luz se refleja vs refracta** en dieléctricos.

**Aproximación de Schlick**:
```
R(θ) = R₀ + (1 - R₀)(1 - cos(θ))⁵

R₀ = ((n₁ - n₂)/(n₁ + n₂))²

θ = ángulo entre rayo y normal
```

**Ejemplo** (aire → vidrio, n₁=1, n₂=1.5):
```
R₀ = ((1-1.5)/(1+1.5))² = 0.04 = 4%

θ = 0° (perpendicular): R = 4%   → 96% refracta
θ = 45°: R ≈ 9%                  → 91% refracta
θ = 85° (rasante): R ≈ 85%       → 15% refracta

Ángulo grazing → más reflexión
(por eso el agua refleja más cuando la miras de lado)
```

### 5. Complejidad Computacional

**Tabla de rendimiento**:

| Operación | Complejidad | Ejemplo (N esferas) |
|-----------|-------------|---------------------|
| Hit AABB | O(1) | 6 comparaciones |
| Hit Sphere | O(1) | ~30 operaciones |
| Hit Triangle | O(1) | ~40 operaciones |
| Búsqueda sin BVH | O(N) | 1,000,000 hits |
| Construcción BVH | O(N log N) | ~20,000,000 ops |
| Búsqueda con BVH | O(log N) | ~20 hits |
| **Speedup** | **N / log N** | **50,000×** |

Para una imagen 1920×1080 con 10 rebotes:
```
Sin BVH: 
  1920 × 1080 × 10 × 1,000,000 = 20,736,000,000,000 hits
  ≈ 20.7 TRILLION operaciones

Con BVH:
  1920 × 1080 × 10 × 20 = 414,720,000 hits
  ≈ 415 MILLION operaciones

Tiempo de render:
  Sin BVH: ~5,000 horas
  Con BVH: ~10 minutos

BVH es la DIFERENCIA entre impracticable y tiempo real
```

---

## Resumen de Valores Clave y Sus Efectos

### t_min (normalmente 0.001)

- **Propósito**: Evitar shadow acne
- `↑ Aumentar` (ej. 0.01): Se eliminan más auto-intersecciones, pero pueden aparecer gaps
- `↓ Disminuir` (ej. 0.0001): Más precisión, pero más shadow acne
- **Óptimo**: 0.001 para la mayoría de escenas

### fuzz (rugosidad metálica, 0-1)

- **Propósito**: Controlar nitidez de reflexiones
- `↑ = 0 → 1`: Espejo perfecto → Metal rugoso difuso
- `↓ = 1 → 0`: Metal rugoso → Espejo perfecto
- **Ejemplos**: 0=espejo baño, 0.3=acero pulido, 0.7=aluminio, 1.0=hierro oxidado

### ior (índice de refracción, 1.0-2.5)

- **Propósito**: Controlar doblez de luz refractada
- `↑ Aumentar`: Más doblez, más brillo, más dispersión
- `↓ Disminuir`: Menos doblez, más transparente
- **Valores**: 1.0=aire, 1.33=agua, 1.5=vidrio, 2.4=diamante

### emission (intensidad de luz, 0-∞)

- **Propósito**: Luz emitida por fuentes
- `↑ Aumentar`: Luz más brillante, más iluminación en escena
- `↓ Disminuir`: Luz más tenue
- **Valores**: 0=sin luz, 1-5=luz suave, 10-20=bombilla, 50+=sol

### BVH Max Depth (profundidad del árbol)

- **Propósito**: Balance entre construcción y búsqueda
- Profundidad óptima: log₂(N) donde N = número de objetos
- `↑ Aumentar`: Más tiempo de construcción, búsquedas ligeramente más rápidas
- `↓ Disminuir`: Menos tiempo de construcción, búsquedas más lentas

---

## Conclusión

Este archivo implementa los **pilares fundamentales** de cualquier path tracer moderno:

1. **Acelleración espacial** (BVH) - hace posible renderizar escenas complejas
2. **Geometría básica** (Sphere, Triangle, Quad) - bloques de construcción universales  
3. **Intersección eficiente** (algoritmos optimizados) - bottleneck principal del rendimiento
4. **Muestreo inteligente** (importance sampling) - reduce ruido dramáticamente
5. **Materiales físicamente correctos** (Fresnel, IOR) - realismo visual

**Conexión con tecnologías comerciales** (NVIDIA RTX, etc.):

- BVH → RTX usa BVH en hardware dedicado (RT Cores)
- Möller-Trumbore → Algoritmo estándar en todas las GPUs
- Importance Sampling → Implementado en OptiX, Embree, etc.
- AABB intersection → Acelerado en hardware en GPUs modernas

Los conceptos aquí son los MISMOS que usan:
- **NVIDIA OptiX** (motor de ray tracing)
- **Disney's Hyperion** (renderer de produccin)
- **Blender Cycles** (motor de render open source)
- **Unreal Engine 5** (ray tracing en tiempo real)

La diferencia principal es **optimización** y **escala**, pero los algoritmos fundamentales son idénticos.
