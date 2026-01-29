# Guía Completa de utils.py: Utilidades y Muestreo Monte Carlo

## Tabla de Contenidos

1. [Introducción](#introducción)
2. [Monte Carlo Sampling](#monte-carlo-sampling)
3. [obtener_punto_luz_aleatorio](#obtener_punto_luz_aleatorio)
4. [random_in_unit_sphere](#random_in_unit_sphere)
5. [generar_direccion_aleatoria](#generar_direccion_aleatoria)
6. [random_in_unit_disk](#random_in_unit_disk)
7. [load_obj](#load_obj)
8. [Comparación de Métodos](#comparación-de-métodos)
9. [Aplicaciones Prácticas](#aplicaciones-prácticas)

---

## Introducción

El archivo `utils.py` contiene funciones de utilidad fundamentales para path tracing avanzado:

**Categorías**:

1. **Muestreo Aleatorio** (Monte Carlo):
   - `random_in_unit_sphere()` - Volumen esférico
   - `random_in_unit_disk()` - Área circular
   - `generar_direccion_aleatoria()` - Hemisferio orientado
   - `obtener_punto_luz_aleatorio()` - Luz de área

2. **Importación de Geometría**:
   - `load_obj()` - Carga modelos 3D

### ¿Por qué Muestreo Aleatorio?

Path tracing usa **Monte Carlo integration** para resolver la ecuación de renderizado:

```
L_out = ∫_Ω BRDF(ω) * L_in(ω) * cos(θ) dω
```

Esta integral es **imposible de resolver analíticamente** para escenas complejas.

**Solución**: Aproximar con muestras aleatorias:

```
L_out ≈ (1/N) Σ BRDF(ω_i) * L_in(ω_i) * cos(θ_i)
```

donde ω_i son direcciones **aleatorias** (Monte Carlo samples).

**Convergencia**:
- N=1 muestra → Muy ruidoso
- N=10 muestras → Ruidoso
- N=100 muestras → Aceptable  
- N=1000 muestras → Suave
- N=10000 muestras → Casi sin ruido

Error ∝ 1/√N (teorema Monte Carlo)

---

## Monte Carlo Sampling

### Conceptos Fundamentales

#### 1. Distribución Uniforme

**Uniforme** significa que todos los puntos/direcciones tienen la misma probabilidad.

**Ejemplo 1D**:
```python
x = random()  # [0, 1]

P(x ∈ [0.3, 0.4]) = 0.1  # 10%
P(x ∈ [0.7, 0.8]) = 0.1  # 10%

TODAS las intervalos del mismo tamaño = misma probabilidad
```

**Ejemplo 2D (cuadrado)**:
```python
x = random()  # [0, 1]
y = random()  # [0, 1]

Uniformemente distribuido en el cuadrado [0,1]²
```

#### 2. Rejection Sampling (Método de Rechazo)

Técnica para generar muestras de distribuciones complejas:

**Algoritmo General**:
```
1. Genera muestra en región SIMPLE (cubo, cuadrado)
2. Test: ¿Cumple criterio para región COMPLEJA (esfera, círculo)?
3. Si SÍ → ACEPTA, retorna muestra
4. Si NO → RECHAZA, vuelve al paso 1
```

**Ejemplo Visual - Círculo en Cuadrado**:

```
    +--------+
    |  ....  |   Cuadrado [-1,1]²
    | ..::.. |   
    |..:✓✓:.|   Círculo x²+y²<1
    | ..::.. |   
    |  ....  |   ✓ Aceptado (dentro)
    +--------+   ✗ Rechazado (fuera)
```

**Eficiencia**:
```
Eficiencia = Volumen_objetivo / Volumen_contenedor

Círculo en cuadrado: π/4 ≈ 78.5%
Esfera en cubo: π/6 ≈ 52.4%
```

#### 3. PDF (Probability Density Function)

**PDF** describe cómo de probable es cada resultado.

Para distribución **uniforme en volumen** (esfera):
```
PDF(p) = 1 / Volumen_total

Para esfera unitaria:
Volumen = (4/3)π
PDF = 1 / ((4/3)π) = 3/(4π)
```

Para distribución **uniforme en superficie** (hemisferio):
```
PDF(ω) = 1 / Área_hemisferio = 1 / (2π)
```

#### 4. Teorema de Convergencia Monte Carlo

```
Error ∝ 1 / √N

N = número de muestras

Ejemplo:
- N=100 → Error relativo ~10%
- N=10,000 → Error relativo ~1%
- N=1,000,000 → Error relativo ~0.1%

Para REDUCIR error a la MITAD, necesitas 4× más muestras
```

**Por qué √N y no N**:
- Es propiedad estadística de variables aleatorias independientes
- Convergencia LENTA pero GARANTIZADA
- Por eso path tracing necesita muchas muestras

---

## obtener_punto_luz_aleatorio

### Propósito

Simula **luces de área esférica** para sombras suaves (penumbra).

### Luces Puntuales vs Luces de Área

**Luz Puntual** (ideal, irreal):

```
        Luz •
           |\\
           | \\
           |  \\
        -------+-------
        Objeto |
               ▼
        Sombra DURA (100% oscuro)
```

**Luz de Área** (real):

```
      .------.
     |  LUZ   |  Múltiples puntos
      `------'
      .\|/|\.
     . \|/| .
    .  \|/|  .
   ---------+-------
        Objeto
         ▼
   ████▓▓▒▒░░  Penumbra (gradiente)
   UMBRA PENUMBRA
```

### Algoritmo

```python
def obtener_punto_luz_aleatorio(centro_luz, radio_luz):
    offset = Vec3(
        (random() - 0.5) * 2,  # [-1, 1]
        (random() - 0.5) * 2,  # [-1, 1]
        (random() - 0.5) * 2   # [-1, 1]
    ) * radio_luz
    return centro_luz + offset
```

**Paso a paso**:

1. `random() - 0.5` → [-0.5, 0.5]
2. `() * 2` → [-1, 1]
3. `Vec3(...) * radio_luz` → cubo [-radio, radio]³
4. `centro + offset` → Punto final

### Distribución

Este método genera puntos en un **CUBO**, NO una esfera:

```
       +----------+
      /|  Esfera /|    Más puntos en esquinas
     / | inscrita |    del cubo (fuera de esfera)
    +----------+ /
    |          |/
    +----------+
```

**Probabilidad por región**:
- Dentro de esfera: 52.4% del volumen del cubo
- Esquinas (fuera): 47.6% del volumen

**¿Es un problema?**

Para la MAYORÍA de escenas: **NO**
- Diferencia visual imperceptible
- Más simple/rápido que verdadera esfera
- Las sombras suaves siguen siendo realistas

Para simulaciones **físicamente precisas**: Usar `random_in_unit_sphere()` instead.

### Ejemplo Numérico

```python
centro = Vec3(0, 10, 0)  # Luz en el techo
radio = 0.5

# Generar 3 puntos
p1 = obtener_punto_luz_aleatorio(centro, radio)
# Ejemplo: Vec3(-0.2, 10.3, 0.4)

p2 = obtener_punto_luz_aleatorio(centro, radio)
# Ejemplo: Vec3(0.4, 9.8, -0.1)

p3 = obtener_punto_luz_aleatorio(centro, radio)
# Ejemplo: Vec3(0.1, 10.5, 0.2)

# Verificar que están cerca del centro
assert (p1 - centro).length() <= radio * √3  # √3 ≈ 1.73 diagonal del cubo
```

### Efecto del Radio

**Visualización de sombras**:

```
radio_luz = 0.0 (luz puntual):
    Objeto
      |
  ████████  Sombra DURA
  
radio_luz = 0.2:
    Objeto
      |
  ████▓▓▒▒  Penumbra SUTIL
  
radio_luz = 0.5:
    Objeto
      |
  ███▓▓▒▒░░  Penumbra VISIBLE
  
radio_luz = 1.0:
    Objeto
      |
  ██▓▓▒▒░░░░  Penumbra AMPLIA
```

**Ecuación de penumbra**:

```
Ancho_penumbra ≈ distancia_luz * (radio_luz / distancia_objeto)

Ejemplo:
- Luz a 10m,radio 0.5m
- Objeto a 8m
- Penumbra ≈ 10 * (0.5 / 8) ≈ 0.625m
```

### Uso en Path Tracing

```python
# Para sombras suaves, enviar MÚLTIPLES rayos hacia puntos diferentes

shadow_color = Vec3(0, 0, 0)
num_light_samples = 10

for _ in range(num_light_samples):
    # Punto aleatorio en luz
    punto_luz = obtener_punto_luz_aleatorio(centro_luz, radio_luz)
    
    # Rayo de sombra
    to_light = (punto_luz - hit_point).normalize()
    shadow_ray = Ray(hit_point + 0.001*normal, to_light)
    
    # Test de visibilidad
    if not scene.hit(shadow_ray, 0.001, distance_to_light):
        # No bloqueado → suma iluminación
        shadow_color += light_color
    # else: bloqueado → en sombra

# Promedio
shadow_color /= num_light_samples
```

**Resultados**:
- 1 muestra → sombras ruidosas/punteadas
- 10 muestras → sombras aceptables
- 100 muestras → sombras suaves

---

## random_in_unit_sphere

### El Problema: Distribución Naïve INCORRECTA

**Intento ingenuo** (MALO):

```python
# ¡ESTO ESTÁ MAL!
r = random()           # Radio [0, 1]
theta = random() * 2π  # Ángulo azimutal
phi = random() * π     # Ángulo polar

x = r * sin(phi) * cos(theta)
y = r * sin(phi) * sin(theta)
z = r * cos(phi)
```

**¿Por qué está MAL?**

Genera MÁS puntos cerca del **centro**:

```
Distribución de r:
  P(r) = r²  (volumen de cáscaras esféricas)
  
  Con r=random() uniforme:
  50% puntos tienen r < 0.5
  → Toda la mitad interna!
  
  Debería ser:
  12.5% puntos en radio 0-0.5 (volumen (0.5)³ de 1³)
```

**Visualización**:

```
INCORRECTO (ingenuo):        CORRECTO (rechazo):
     .....                        .  .  .
    .......                      .  .  .  .
   .........                    .  .  .  .  .
    .......                      .  .  .  .
     .....                        .  .  .

Concentrado centro          Uniforme en volumen
```

### La Solución: Rejection Sampling

```python
def random_in_unit_sphere():
    while True:
        p = Vec3(
            random.uniform(-1, 1),
            random.uniform(-1, 1),
            random.uniform(-1, 1)
        )
        if p.length() < 1.0:
            return p
```

**Por qué funciona**:

El cubo [-1,1]³ CONTIENE la esfera unitaria perfectamente:

```
Cubo: volumen = 2³ = 8
Esfera: volumen = (4/3)π ≈ 4.19

Ratio = 4.19 / 8 ≈ 0.524 = 52.4%

→ 52.4% de puntos aleatorios en cubo están en esfera
```

### Análisis Matemático de Eficiencia

**Probabilidad de aceptar**:

```
P(accept) = V_esfera / V_cubo
          = (4/3)π / 2³
          = (4/3)π / 8
          = π / 6
          ≈ 0.5236...
          ≈ 52.36%
```

**Número esperado de iteraciones**:

```
E[iteraciones] = 1 / P(accept)
               = 6 / π
               ≈ 1.909859...
               ≈ 1.91 veces
```

**Interpretación**:
- En promedio, necesitas ~2 intentos
- A veces 1 (suerte), a veces 5+ (mala suerte)
- Probabilidad ≥ 10 intentos: ~0.002% (muy raro)

### Ejemplo Numérico Detallado

```python
# Simular 5 intentos

# Intento 1:
p1 = Vec3(0.73, -0.45, 0.92)
length1 = sqrt(0.73² + 0.45² + 0.92²)
        = sqrt(0.5329 + 0.2025 + 0.8464)
        = sqrt(1.5818)
        = 1.258
# ✗ length > 1 → RECHAZAR

# Intento 2:
p2 = Vec3(-0.12, 0.88, -0.34)
length2 = sqrt(0.12² + 0.88² + 0.34²)
        = sqrt(0.0144 + 0.7744 + 0.1156)
        = sqrt(0.9044)
        = 0.951
# ✓ length < 1 → ACEPTAR
return Vec3(-0.12, 0.88, -0.34)
```

### Verificación de Uniformidad

**Test estadístico** (1M puntos):

```python
samples = [random_in_unit_sphere() for _ in range(1_000_000)]

# Verificar distribución de distancias
distances = [s.length() for s in samples]

# Histograma esperado (volumen de cáscaras)
# P(r < R) = R³ (volumen relativo)

counts = [0] * 10
for d in distances:
    bin_idx = int(d * 10)
    if bin_idx < 10:
        counts[bin_idx] += 1

# Esperado vs Observado:
# Bin [0.0-0.1]: Esperado 0.1³  = 0.1%, Observado ~0.1%
# Bin [0.1-0.2]: Esperado 0.2³  = 0.8%, Observado ~0.7%
# Bin [0.9-1.0]: Esperado 1.0³  = 100%, Observado ~27.1%
#                        (90% anterior)

# Distribución correcta ✓
```

### Usos en Path Tracing

#### 1. Scattering Difuso (Lambertian)

```python
# Material mate golpeado
scatter_target = hit_point + normal + random_in_unit_sphere()
scatter_direction = (scatter_target - hit_point).normalize()

# Nuevo rayo
bounce_ray = Ray(hit_point, scatter_direction)
```

#### 2. Fuzzy Reflection (Metales Rugosos)

```python
# Metal con rugosidad
reflected = reflect(incident, normal)
fuzzy_reflected = reflected + fuzz * random_in_unit_sphere()

# Mayor fuzz → más aleatoriedad → más difuso
```

#### 3. Subsurface Scattering (bajo superficie)

```python
# Simular dispersión bajo superficie
entry_point = hit_point
scatter_offset = random_in_unit_sphere() * scatter_radius

exit_point = entry_point + scatter_offset
```

---

## generar_direccion_aleatoria

### Propósito

Genera dirección en el **hemisferio** orientado por la normal, para materiales difusos.

### Hemisferio vs Esfera Completa

**Esfera completa**:
```
      •  •  •       Todas direcciones
    •        •      (360° en todas)
   •    🌍    •
    •        •
      •  •  •
```

**Hemisferio** (orientado por normal ↑):
```
      •  •  •       Solo direcciones "arriba"
    •        •      (solo un lado)
   •    💎    •
    ----------      Plano tangente
```

**En superficies**:
- Luz puede rebotar hacia AFUERA (hemisferio)
- Luz NO puede rebotar hacia ADENTRO (dentro del objeto)

### Algoritmo

```python
def generar_direccion_aleatoria(normal):
    # Paso 1: Dirección aleatoria en esfera
    random_dir = random_in_unit_sphere().normalize()
    
    # Paso 2: Test de hemisferio con producto punto
    if random_dir.dot(normal) > 0.0:
        return random_dir  # ✓ Mismo lado
    else:
        return random_dir * -1  # ✗ Invertir
```

### Producto Punto para Test de Hemisferio

**Matemáticas**:

```
normal · direction = |n| |d| cos(θ)

Como ambos son unitarios (|n| = |d| = 1):
normal · direction = cos(θ)

donde θ = ángulo entre los vectores
```

**Casos**:

```
n · d > 0  → cos(θ) > 0 → θ < 90°  → Mismo hemisferio ✓
n · d = 0  → cos(θ) = 0 → θ = 90°  → Plano tangente (borde)
n · d < 0  → cos(θ) < 0 → θ > 90°  → Hemisferio opuesto ✗
```

**Visualización**:

```
        Normal n = (0, 1, 0)
            ↑
            |
            |    d1 = (0.6, 0.8, 0)
            |   ↗     n · d1 = 0*0.6 + 1*0.8 + 0*0 = 0.8 > 0 ✓
            |  /
            | /
    --------+--------  Plano tangente (y=0)
            |
            |    d2 = (0.3, -0.9, 0.3)
            |   ↙     n · d2 = 0*0.3 + 1*(-0.9) + 0*0.3 = -0.9 < 0 ✗
            |        Invertir → (0.3, 0.9, 0.3) ✓
```

### Ejemplo Numérico Completo

```python
# Superficie horizontal apuntando arriba
normal = Vec3(0, 1, 0)

# Paso 1: Generar dirección en esfera
# (asumamos que random_in_unit_sphere retorna Vec3(0.2, -0.7, 0.3))
random_sphere = Vec3(0.2, -0.7, 0.3)
random_dir = random_sphere.normalize()

# Normalizar:
length = sqrt(0.2² + (-0.7)² + 0.3²)
       = sqrt(0.04 + 0.49 + 0.09)
       = sqrt(0.62)
       ≈ 0.787

normalized = Vec3(0.2/0.787, -0.7/0.787, 0.3/0.787)
          = Vec3(0.254, -0.889, 0.381)

# Paso 2: Test de hemisferio
dot = normal · normalized
    = 0*0.254 + 1*(-0.889) + 0*0.381
    = -0.889

# dot < 0 → hemisferio opuesto → INVERTIR
final_dir = normalized * -1
         = Vec3(-0.254, 0.889, -0.381)

# Verificar
final_dot = normal · final_dir
          = 0*(-0.254) + 1*0.889 + 0*(-0.381)
          = 0.889 > 0 ✓

return Vec3(-0.254, 0.889, -0.381)
```

### Distribución: Uniforme vs Coseno-ponderado

#### Uniforme en Hemisferio (esta función)

**Distribución**:
- Todos los ángulos θ ∈ [0°, 90°] equiprobables
- PDF = 1 / (2π) (área hemisferio)

**Características**:
- ✅ Simple y rápido
- ✅ Aproximación razonable
- ❌ NO es físicamente exacto (Lambertian)

#### Coseno-ponderado (Lambertian Ideal)

**Distribución**:
- Más muestras cerca de la normal (θ ≈ 0°)
- Menos muestras paralelas a superficie (θ ≈ 90°)
- PDF = cos(θ) / π

**Por qué coseno**:

La BRDF Lambertiana incluye cos(θ):

```
L_out = (ρ/π) ∫ L_in(ω) cos(θ) dω

Si muestreamos con PDF = cos(θ)/π:
L_out ≈ (ρ/π) * Σ [ L_in(ω_i) * cos(θ_i) / PDF(ω_i) ]
      = (ρ/π) * Σ [ L_in(ω_i) * cos(θ_i) / (cos(θ_i)/π) ]
      = ρ * Σ L_in(ω_i)

El cos(θ) se cancela → menos varianza → menos ruido
```

**Comparación visual**:

```
Uniforme:                Coseno-ponderado:
    • • •                    • • • •
  • • • • •                • • • • • •
 • • • • • •              • • • • • • •
-------------            ---------------
   Igual                 Más cerca normal

Ruido: MEDIO            Ruido: BAJO
```

### Uso en Path Tracing

```python
def trace_path(ray, scene, depth):
    if depth <= 0:
        return Vec3(0,0,0)
    
    hit = scene.hit(ray, 0.001, inf)
    if not hit:
        return sky_color(ray.direction)
    
    # Material difuso
    if not hit.is_metal and not hit.is_dielectric:
        # Generar dirección de rebote
        scatter_dir = generar_direccion_aleatoria(hit.normal)
        
        # Rayo rebotado
        scattered = Ray(hit.point, scatter_dir)
        
        # Recursión
        incoming = trace_path(scattered, scene, depth - 1)
        
        # Aplicar albedo
        return hit.color * incoming
    
    # ... metales/dieléctricos
```

---

## random_in_unit_disk

### Propósito

Genera punto 2D en disco (círculo) para **Depth of Field**.

### Profundidad de Campo (Depth of Field)

**Cámaras ideales** (pinhole):
- TODO enfocado
- No realista (ojos/cámaras tienen lentes)

**Cámaras reales**:
- Lente con apertura finita
- Solo plano focal nítido
- Antes/después → desenfoque

**Simulación con random_in_unit_disk**:

```
Lente (apertura)
    .-----.
   |   🔍   |  Múltiples puntos de origen
    `-----'
      \  |  /    Rayos desde diferentes puntos
       \ | /
        \|/
         •      Plano focal (nítido)
        /|\
       / | \    Objetos fuera de foco → borrosos
      /  |  \
```

### Algoritmo de Rechazo 2D

```python
def random_in_unit_disk():
    while True:
        p = Vec3(random.uniform(-1, 1), random.uniform(-1, 1), 0)
        if p.length() < 1.0:
            return p
```

**Eficiencia**:

```
P(accept) = Área_círculo / Área_cuadrado
          = π / 4
          ≈ 0.785
          = 78.5%

E[iteraciones] = 4 / π
               ≈ 1.27 veces
```

**Más eficiente que esfera** (78.5% vs 52.4%)!

### Visualización

```
    y
    ↑
-1  +-------+ 1
    |  ...  |   . = puntos aceptados (en disco)
    | ..:.. |   espacio = rechazados
 0  |.::o::.|   o = centro
    | ..:.. |
    |  ...  |
-1  +-------+ → x
   -1   0   1
```

**Test de pertenencia**:

```
x² + y² < 1  → Dentro del disco ✓
x² + y² ≥ 1  → Fuera del disco ✗
```

### Ejemplo Numérico

```python
# Intento 1:
p1 = Vec3(0.93, 0.85, 0)
dist1 = sqrt(0.93² + 0.85²)
      = sqrt(0.8649 + 0.7225)
      = sqrt(1.5874)
      ≈ 1.260
# ✗ dist ≥ 1 → RECHAZAR

# Intento 2:
p2 = Vec3(-0.45, 0.67, 0)
dist2 = sqrt(0.45² + 0.67²)
      = sqrt(0.2025 + 0.4489)
      = sqrt(0.6514)
      ≈ 0.807
# ✓ dist < 1 → ACEPTAR
return Vec3(-0.45, 0.67, 0)
```

### Depth of Field Implementation

```python
# Configuración cámara
camera_pos = Vec3(0, 0, -10)
aperture = 0.1          # Apertura (mayor = más desenfoque)
focus_distance = 10.0   # Distancia al plano nítido

# Calcular ejes de cámara
camera_forward = Vec3(0, 0, 1)
camera_right = Vec3(1, 0, 0)
camera_up = Vec3(0, 1, 0)

# Para cada píxel, generar rayo:
def get_camera_ray(pixel_x, pixel_y):
    # Dirección base (sin DOF)
    base_direction = calculate_pixel_direction(pixel_x, pixel_y)
    
    # Offset aleatorio en disco (lente)
    lens_sample = random_in_unit_disk() * aperture
    offset = lens_sample.x * camera_right + lens_sample.y * camera_up
    
    # Nuevo origen (punto en lente)
    ray_origin = camera_pos + offset
    
    # Punto focal (donde deben converger todos los rayos)
    focal_point = camera_pos + focus_distance * base_direction
    
    # Dirección desde punto de lente hacia punto focal
    ray_direction = (focal_point - ray_origin).normalize()
    
    return Ray(ray_origin, ray_direction)
```

### Efecto de Apertura

**Apertura = 0.0** (pinhole):
```
TODO NÍTIDO (sin DOF)
```

**Apertura = 0.05**:
```
Plano focal: NÍTIDO
Cerca/lejos: Ligeramente borroso
Bokeh: Casi imperceptible
```

**Apertura = 0.2**:
```
Plano focal: NÍTIDO
Cerca/lejos: Moderadamente borroso
Bokeh: Visible (círculos pequeños)
```

**Apertura = 0.5+**:
```
Plano focal: NÍTIDO
Cerca/lejos: MUY borroso
Bokeh: Muy evidente (círculos grandes)
```

**Ecuación de bokeh**:

```
Diámetro_bokeh ≈ aperture * |distancia_objeto - focus_distance| / distancia_objeto

Ejemplo:
- aperture = 0.2
- focus_distance = 10
- Objeto en z = 15 (5 unidades detrás)

bokeh ≈ 0.2 * |15 - 10| / 15
      = 0.2 * 5 / 15
      ≈ 0.067 unidades de imagen
```

### Método Polar (Alternativa sin Rechazo)

```python
def random_in_unit_disk_polar():
    # Sin bucle, siempre acepta
    r = sqrt(random())        # Radio (sqrt para distribución uniforme)
    theta = random() * 2 * π  # Ángulo
    
    x = r * cos(theta)
    y = r * sin(theta)
    return Vec3(x, y, 0)
```

**¿Por qué sqrt(random())?**

```
Para distribución uniforme en ÁREA:
P(r < R) = πR² / π = R²

Si usamos r = random():
  P(r < 0.5) = 0.5  (50% puntos en radio < 0.5)
  Pero área(0.5) = π(0.5)² = 0.25π (25% del área)
  ❌ MÁS puntos en centro

Con r = sqrt(random()):
  P(r < 0.5) = 0.25  (25% puntos en radio < 0.5)
  área(0.5) = 0.25π  (25% del área)
  ✓ Uniforme en área
```

**Comparación**:

| Aspecto | Rechazo | Polar |
|---------|---------|-------|
| Eficiencia | 78.5% | 100% |
| Operaciones | 2 random, 1 sqrt, comparación | 2 random, 1 sqrt, 1 sin, 1 cos |
| Complejidad | Simple | Trigonometría |
| Rendimiento | ~Equivalente | ~Equivalente |

Para la mayoría de casos, el **método de rechazo es suficiente** (más simple).

---

## load_obj

### Formato OBJ (Wavefront)

Estándar de la industria para geometría 3D.

**Desarrollado por**: Wavefront Technologies (1980s)

**Usado por**:
- Blender
- Maya
- 3DS Max
- Cinema 4D
- ZBrush
- Unity
- Unreal Engine

### Estructura Básica

**Tipos de líneas**:

```
# Comentario
v x y z          # Vértice
vt u v          # Coordenada textura (UV)
vn x y z        # Normal
f v1 v2 v3      # Cara (triángulo)
g nombre        # Grupo
o nombre        # Objeto
mtllib file.mtl # Biblioteca materiales
usemtl nombre   # Usar material
```

**Ejemplo completo** (cubo):

```
# Cube.obj
v -1.0 -1.0  1.0    # Vértice 1
v  1.0 -1.0  1.0    # Vértice 2
v  1.0  1.0  1.0    # Vértice 3
v -1.0  1.0  1.0    # Vértice 4
v -1.0 -1.0 -1.0    # Vértice 5
v  1.0 -1.0 -1.0    # Vértice 6
v  1.0  1.0 -1.0    # Vértice 7
v -1.0  1.0 -1.0    # Vértice 8

f 1 2 3      # Cara frontal triángulo 1
f 1 3 4      # Cara frontal triángulo 2
f 5 8 7      # Cara trasera triángulo 1
f 5 7 6      # Cara trasera triángulo 2
# ... más caras
```

### Implementación en load_obj

**Algoritmo**:

```
1. Inicializar listas vacías: vertices[], triangles[]
2. Abrir archivo
3. Para cada línea:
   a. Si empieza con 'v ' → parsear vértice
      - Aplicar escala: v * scale
      - Aplicar offset: v + offset
      - Agregar a vertices[]
   
   b. Si empieza con 'f ' → parsear cara
      - Extraer índices de vértices
      - Triangular si n > 3 (fan method)
      - Crear triángulos con color y material
      - Agregar a triangles[]

4. Retornar triangles[]
```

### Índices en OBJ

**CRÍTICO**: OBJ usa índices **1-based** (empiezan en 1):

```
Python/C: 0-based       OBJ: 1-based
vertices[0]       →     v 1
vertices[1]       →     v 2
vertices[2]       →     v 3

Conversión: idx_python = idx_obj - 1
```

**Índices negativos** (relativos al final):

```
v 0 0 0    # Vértice N-2
v 1 0 0    # Vértice N-1
v 0 1 0    # Vértice N

f -3 -2 -1  # Últimos 3 vértices

Conversión: idx = len(vertices) + idx_obj
```

### Formato de Caras

**Variantes**:

```
f v1 v2 v3              # Solo vértices
f v1/vt1 v2/vt2 v3/vt3  # Vértices + textura
f v1//vn1 v2//vn2 v3//vn3          # Vértices + normales
f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3 # Completo
```

**Parsing**:

```python
"f 12/34/56 78/90/11 23/45/67"
    ↓
parts = ["12/34/56", "78/90/11", "23/45/67"]

Para cada part:
    "12/34/56".split('/') = ["12", "34", "56"]
                              ↑    ↑    ↑
                              v   vt   vn
    
    Nos interesa solo el primer número (v)
    idx = int("12") = 12
    idx_python = 12 - 1 = 11
    vertices[11]
```

### Triangulación (Fan Method)

**Problema**: Caras pueden tener > 3 vértices:

```
f 1 2 3 4  (cuadrado, 4 vértices)
```

**Solución - Fan desde vértice 0**:

```
Cuadrado:          Fan:
v3------v4         v3------v4
|       |           |\      |
|       |           | \  T2 |
|       |           |  \    |
|       |           | T1\   |
v1------v2         v1------v2

Triángulos:
T1 = [v1, v2, v3]
T2 = [v1, v3, v4]

Polígono N vértices → (N-2) triángulos
```

**Código**:

```python
indices = [1, 2, 3, 4, 5]  # Pentágono

for i in range(1, len(indices) - 1):
    # i=1: triángulo [0, 1, 2]
    # i=2: triángulo [0, 2, 3]
    # i=3: triángulo [0, 3, 4]
    
    triangle = Triangle(
        vertices[indices[0]],      # Siempre vértice 0
        vertices[indices[i]],      # Vértice actual
        vertices[indices[i+1]]     # Vértice siguiente
    )
```

### Ejemplo Completo

```python
# Archivo: pyramid.obj
"""
# Pirámide
v 0 1 0       # 1: Ápice
v -1 0 -1     # 2: Base esquina 1
v 1 0 -1      # 3: Base esquina 2
v 1 0 1       # 4: Base esquina 3
v -1 0 1      # 5: Base esquina 4

f 1 2 3       # Cara lateral 1
f 1 3 4       # Cara lateral 2
f 1 4 5       # Cara lateral 3
f 1 5 2       # Cara lateral 4
f 2 4 3       # Base triángulo 1
f 2 5 4       # Base triángulo 2
"""

# Cargar
pyramidtriangles = load_obj(
    'pyramid.obj',
    color=Vec3(0.9, 0.7, 0.1),     # Dorado
    offset=Vec3(0, 5, 0),           # Elevada 5 unidades
    scale=2.0,                      # Doble tamaño
    material_params={'is_metal': True, 'fuzz': 0.3}
)

# Resultado:
# - 5 vértices leídos
# - 6 triángulos creados
# - Cada vértice escalado ×2 y elevado +5
```

### Transformaciones

**Scale**:

```python
v_original = Vec3(1, 2, 3)
scale = 2.5

v_scaled = v_original * scale
         = Vec3(1*2.5, 2*2.5, 3*2.5)
         = Vec3(2.5, 5.0, 7.5)
```

**Offset**:

```python
v_scaled = Vec3(2.5, 5.0, 7.5)
offset = Vec3(10, -3, 0)

v_final = v_scaled + offset
        = Vec3(2.5+10, 5.0-3, 7.5+0)
        = Vec3(12.5, 2.0, 7.5)
```

**Orden**: SIEMPRE scale ANTES de offset:

```
Correcto:  v_final = (v * scale) + offset
Incorrecto: v_final = (v + offset) * scale  ← Offset también se escala!
```

### Limitaciones de esta Implementación

| Característica | Soportado | Notas |
|----------------|-----------|-------|
| Vértices (v) | ✅ | Completo |
| Caras (f) | ✅ | Con triangulación |
| Índices negativos | ✅ | Relativos al final |
| Polígonos | ✅ | Fan method |
| Texturas (vt) | ❌ | Ignoradas |
| Normales (vn) | ❌ | Recalculadas por triángulo |
| Grupos (g) | ❌ | Todos en misma lista |
| Objetos (o) | ❌ | Todos juntos |
| Materiales (mtl) | ❌ | material_params manual |
| Smooth shading | ❌ | Flat shading (normal por triángulo) |

### Uso con BVH

**CRÍTICO**: Modelos OBJ suelen tener miles/millones de triángulos.

**Sin BVH**:

```python
dragon = load_obj('dragon.obj', Vec3(0.2, 0.8, 0.2))
# 871,414 triángulos

scene.extend(dragon)  # ❌ MUY LENTO

# Cada rayo prueba 871,414 triángulos
# Render 1920×1080: ~1.8 billones de tests
# Tiempo: ~Horas o días
```

**Con BVH**:

```python
dragon = load_obj('dragon.obj', Vec3(0.2, 0.8, 0.2))
dragon_bvh = BVHNode.create(dragon)  # Construir BVH

scene.append(dragon_bvh)  # ✓ RÁPIDO

# Cada rayo prueba ~log₂(871,414) ≈ 20 triángulos
# Render 1920×1080: ~41 millones de tests
# Tiempo: ~Minutos

# Aceleración: 43,570× más rápido
```

**Regla general**:

```
< 100 triángulos: BVH opcional
100-1000 triángulos: BVH recomendado
> 1000 triángulos: BVH OBLIGATORIO
```

---

## Comparación de Métodos

### Tabla de Eficiencia

| Función | Objetivo | Dimensión | Eficiencia | Iteraciones Esperadas |
|---------|----------|-----------|------------|-----------------------|
| `random_in_unit_sphere()` | Volumen esférico | 3D | 52.4% | ~1.91 |
| `random_in_unit_disk()` | Área circular | 2D | 78.5% | ~1.27 |
| `generar_direccion_aleatoria()` | Hemisferio | 3D superficie | ~50% | ~3.82* |
| `obtener_punto_luz_aleatorio()` | Cubo | 3D | 100% | 1 |

\* ~1.91 para esfera + normalización + 50% inversión

### Tabla de Uso

| Función | Uso Principal | Alternativas |
|---------|---------------|--------------|
| `random_in_unit_sphere()` | Scatter difuso, fuzzy reflection | Coseno-ponderado |
| `random_in_unit_disk()` | Depth of field | Método polar |
| `generar_direccion_aleatoria()` | Materiales Lambertian | Importancesampling |
| `obtener_punto_luz_aleatorio()` | Sombras suaves | Luz de área quad, importance sampling |
| `load_obj()` | Cargar modelos | Otros formatos (PLY, STL) |

---

## Aplicaciones Prácticas

### 1. Path Tracer Completo con DOF

```python
def render_scene_with_dof(width, height, samples_per_pixel):
    """Renderer con depth of field."""
    
    image = [[Vec3(0,0,0) for _ in range(width)] for _ in range(height)]
    
    # Configuración
    aperture = 0.1
    focus_distance = 10.0
    
    for y in range(height):
        for x in range(width):
            color = Vec3(0,0,0)
            
            # Múltiples muestras para antialiasing + DOF
            for _ in range(samples_per_pixel):
                # Offset píxel (antialiasing)
                u = (x + random()) / width
                v = (y + random()) / height
                
                # Offset lente (DOF)
                lens_offset = random_in_unit_disk() * aperture
                
                # Rayo con DOF
                ray = get_camera_ray_dof(u, v, lens_offset, focus_distance)
                
                # Trazar
                color += trace_ray(ray, scene, max_depth=10)
            
            # Promedio
            image[y][x] = color / samples_per_pixel
    
    return image
```

### 2. Sombras Suaves

```python
def calculate_soft_shadow(hit_point, normal, light_center, light_radius):
    """Calcula sombra suave con múltiples muestras."""
    
    shadow_factor = 0.0
    num_samples = 32
    
    for _ in range(num_samples):
        # Punto aleatorio en luz
        light_sample = obtener_punto_luz_aleatorio(light_center, light_radius)
        
        # Dirección hacia luz
        to_light = light_sample - hit_point
        distance = to_light.length()
        direction = to_light / distance  # Normalizar
        
        # Rayo de sombra
        shadow_origin = hit_point + 0.001 * normal
        shadow_ray = Ray(shadow_origin, direction)
        
        # Test visibilidad
        if not scene.hit(shadow_ray, 0.001, distance):
            shadow_factor += 1.0  # Visible
    
    # Fracción visible
    return shadow_factor / num_samples
```

### 3. Material Difuso

```python
def scatter_diffuse(hit_record):
    """Scatter para material Lambertian."""
    
    # Dirección aleatoria en hemisferio
    scatter_direction = generar_direccion_aleatoria(hit_record.normal)
    
    # Rayo scattereado
    scattered_ray = Ray(hit_record.point, scatter_direction)
    
    # Atenuación (albedo)
    attenuation = hit_record.color
    
    return scattered_ray, attenuation
```

### 4. Cargar Escena Compleja

```python
def build_complex_scene():
    """Construye escena con modelos OBJ."""
    
    scene = []
    
    # Suelo
    floor_tris = load_obj(
        'plane.obj',
        color=Vec3(0.8, 0.8, 0.8),
        scale=10.0
    )
    scene.extend(floor_tris)
    
    # Personaje
    character_tris = load_obj(
        'character.obj',
        color=Vec3(0.9, 0.7, 0.5),
        offset=Vec3(0, 0, 0),
        scale=1.5,
        material_params={'is_metal': False}
    )
    character_bvh = BVHNode.create(character_tris)
    scene.append(character_bvh)
    
    # Objeto metálico
    sword_tris = load_obj(
        'sword.obj',
        color=Vec3(0.9, 0.9, 0.95),
        offset=Vec3(2, 1, 0),
        scale=0.5,
        material_params={'is_metal': True, 'fuzz': 0.1}
    )
    sword_bvh = BVHNode.create(sword_tris)
    scene.append(sword_bvh)
    
    # BVH global
    scene_bvh = BVHNode.create(scene)
    
    return scene_bvh
```

---

## Resumen

### Conceptos Clave

1. **Monte Carlo**: Aproximar integrales con muestras aleatorias
2. **Rejection Sampling**: Generar distribuciones complejas efectivamente
3. **Hemisferio**: Solo direcciones "hacia afuera" de superficie
4. **Depth of Field**: Múltiples orígenes de rayo simulan lente real
5. **OBJ**: Formato estándar para geometría 3D

### Parámetros Importantes

| Parámetro | Rango | Efecto |
|-----------|-------|--------|
| `radio_luz` | 0-1+ | 0=puntual, 1=penumbra amplia |
| `aperture` | 0-0.5 | 0=todo nítido, 0.5=mucho bokeh |
| `focus_distance` | >0 | Distancia al plano nítido |
| `scale` (OBJ) | >0 | Tamaño del modelo |
| `samples` | 1-10000 | 1=ruidoso, 10000=suave |

### Complejidad

| Función | Tiempo | Espacio |
|---------|--------|---------|
| `random_in_unit_sphere()` | O(1) esperado | O(1) |
| `random_in_unit_disk()` | O(1) esperado | O(1) |
| `generar_direccion_aleatoria()` | O(1) esperado | O(1) |
| `obtener_punto_luz_aleatorio()` | O(1) | O(1) |
| `load_obj()` | O(V + F) | O(V + T) |

V = vértices, F = caras, T = triángulos

### Relación con Tecnologías

Estos métodos son usados en:
- **Disney Hyperion** (Frozen, Moana)
- **Pixar RenderMan** (Toy Story, etc.)
- **Blender Cycles** (open source)
- **V-Ray** (industria VFX)
- **Arnold** (industria)
- **OptiX** (NVIDIA GPU ray tracing)

Los algoritmos fundamentales son **idénticos**, solo difieren en optimizaciones y características avanzadas.
