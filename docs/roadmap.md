# Roadmap de estudio — Dominar Ray Tracing / Path Tracing

Temario estructurado en niveles. Cada nivel construye sobre el anterior. Los temas marcados con ★ son los más críticos para entender este proyecto en particular.

---

## Índice

1. [Nivel 1 — Fundamentos matemáticos](#nivel-1--fundamentos-matemáticos)
2. [Nivel 2 — Geometría computacional](#nivel-2--geometría-computacional)
3. [Nivel 3 — Física de la luz](#nivel-3--física-de-la-luz)
4. [Nivel 4 — Algoritmos de renderizado](#nivel-4--algoritmos-de-renderizado)
5. [Nivel 5 — Técnicas avanzadas de path tracing](#nivel-5--técnicas-avanzadas-de-path-tracing)
6. [Nivel 6 — Aceleración y optimización](#nivel-6--aceleración-y-optimización)
7. [Nivel 7 — Implementación y sistemas](#nivel-7--implementación-y-sistemas)
8. [Recursos recomendados](#recursos-recomendados)

---

## Nivel 1 — Fundamentos matemáticos

Base sin la cual nada de lo demás tiene sentido. Se usa constantemente en cada línea del renderer.

### Álgebra lineal ★

- **Vectores en R³**: suma, resta, escalar, magnitud, normalización
- **Producto punto (dot product)**: definición, interpretación geométrica, relación con el coseno del ángulo
  - Cuándo `a·b > 0`, `= 0`, `< 0` y qué significa en la práctica
  - Proyección de un vector sobre otro
- **Producto cruzado (cross product)**: definición, vector perpendicular, regla de la mano derecha
  - Cálculo del área de un paralelogramo
  - Construcción de normales de superficie
- **Bases ortonormales**: qué son, cómo construirlas a partir de un vector y un "up"
  - Proceso Gram-Schmidt simplificado
- **Transformaciones lineales**: rotación, traslación, escala
  - Matrices 4×4 homogéneas (aunque este proyecto no las usa, son fundamentales en general)
- **Cambio de base**: pasar coordenadas de espacio local a espacio mundo y viceversa

### Geometría analítica ★

- **Ecuación paramétrica de una recta**: `P(t) = O + t·D`
- **Ecuación del plano**: `n·P = d` y sus variantes
- **Ecuación de la esfera**: `||P - C||² = r²`
- **Intersección recta-plano**: derivación y solución
- **Intersección recta-esfera**: resolución de la ecuación cuadrática
- **Discriminante y sus casos**: sin intersección, tangente, dos puntos

### Cálculo ★

- **Integrales**: concepto de integral como suma de áreas infinitesimales
  - Integrales sobre superficies y sobre el hemisferio (relevante para la Rendering Equation)
- **Derivadas parciales**: usadas en la derivación de algunas BRDFs avanzadas
- **Ángulo sólido**: qué mide, cómo se calcula para esferas y quads planos
  - Relación: `dΩ = dA · cos(θ) / r²`
- **Distribuciones de probabilidad continuas**: PDF (función de densidad de probabilidad), CDF

### Trigonometría

- Ley de Snell en forma escalar: `η₁·sin(θ₁) = η₂·sin(θ₂)`
- Relación entre productos punto y cosenos para vectores unitarios
- `sin²(θ) + cos²(θ) = 1` y sus usos para pasar entre sin y cos
- Campo de visión (FOV) y su relación con `tan(θ/2)` para el viewport

---

## Nivel 2 — Geometría computacional

Cómo representar y manipular geometría de forma eficiente en código.

### Primitivas geométricas ★

- **Rayo**: representación paramétrica, normalización obligatoria de la dirección, rango `[t_min, t_max]`
- **Esfera**: intersección rayo-esfera completa (derivación + código)
  - Optimización `half-b` para evitar multiplicaciones innecesarias
- **Triángulo**: el algoritmo de Möller-Trumbore ★
  - Por qué no se precalcula el plano
  - Coordenadas baricéntricas y su interpretación
  - Backface culling opcional
- **Quad/Paralelogramo**: intersección rayo-plano + test de contención
  - Vector auxiliar `w` para coordenadas paramétricas
- **AABB (Bounding Box)**: ★
  - Algoritmo de slabs de Kay-Kajiya
  - Por qué funciona con infinitos en IEEE 754
  - `union` de dos AABBs

### Normales de superficie

- Cómo calcular normales para cada primitiva
- Cuándo orientar la normal hacia el rayo (double-sided surfaces)
- Normal de esfera: `(P - C) / r` ya está normalizada
- Normal de triángulo: producto cruzado de dos aristas
- Importancia de las normales en iluminación y scattering

### Coordenadas baricéntricas

- Definición y representación de puntos dentro de un triángulo
- Condición de validez: `u ≥ 0, v ≥ 0, u+v ≤ 1`
- Interpolación de atributos (colores, UV, normales) usando coordenadas baricéntricas

### Estructuras de aceleración ★

- **BVH (Bounding Volume Hierarchy)**:
  - Construcción: recursión, elección de eje, ordenamiento, partición
  - Traversal: poda por AABB, optimización con t_max reducido
  - Casos base: 1 objeto (hoja), 2 objetos
  - Complejidad O(n log n) construcción, O(log n) query
- **SAH (Surface Area Heuristic)**: criterio óptimo para elegir cómo partir los objetos (extensión del BVH básico)
- Otras estructuras (no en este proyecto pero importantes en general):
  - KD-Tree: partición del espacio, no de los objetos
  - Octree: subdivisión uniforme del espacio
  - Grilla uniforme: hashing espacial

---

## Nivel 3 — Física de la luz

El sustento teórico de por qué el renderer produce imágenes fotorrealistas.

### Radiometría ★

- **Flujo radiante (Φ)**: energía por unidad de tiempo [Watts]
- **Irradiancia (E)**: flujo por unidad de área [W/m²]
- **Radiancia (L)**: flujo por área por ángulo sólido [W/m²·sr] — la cantidad que mide el path tracer
- **Intensidad radiante (I)**: flujo por ángulo sólido
- Por qué trabajamos con radiancia y no con otra cantidad

### Interacción luz-materia ★

- **Reflexión difusa (Lambertiana)**: modelo y limitaciones
  - Por qué la BRDF es `albedo/π` y no solo `albedo`
  - Distribución coseno-ponderada
- **Reflexión especular perfecta**: ley de reflexión, derivación vectorial de `R = I - 2(I·N)N`
- **Microfacetas**: modelo conceptual que explica la rugosidad de metales (base de modelos avanzados como GGX/Beckmann)
- **Refracción**: Ley de Snell escalar y vectorial
  - Por qué cambia la velocidad de la luz en distintos medios
  - Índice de refracción y su variación con la longitud de onda (dispersión cromática)
- **Reflexión interna total**: condición `n₁·sin(θ) > n₂`, aplicaciones físicas
- **Efecto Fresnel**: por qué los materiales reflejan más en ángulos rasantes
  - Ecuaciones de Fresnel exactas (para estudio)
  - Aproximación de Schlick (para implementación)
- **Absorción**: Beer-Lambert law para medios participantes (niebla, vidrio coloreado)

### Conservación de energía

- Toda BRDF física debe satisfacer: `∫Ω fr(ωi, ωo)·cos(θi) dωi ≤ 1`
- Qué pasa si se viola: escenas infinitamente brillantes, inestabilidad numérica
- Cómo verificar que un material conserva energía

### Color y espectro

- RGB vs espectro completo: limitaciones del modelo RGB
- Gamma y espacio de color sRGB: por qué aplicar corrección gamma
- Temperatura de color: K → RGB aproximado
- Tonemapping: convertir HDR a LDR para visualización (Reinhard, ACES, filmic)

---

## Nivel 4 — Algoritmos de renderizado

La evolución histórica y conceptual de los algoritmos de síntesis de imagen.

### Modelos de iluminación local

- **Modelo de Phong**: ambient + diffuse + specular, limitaciones
- **Modelo de Blinn-Phong**: variante más eficiente y física
- Por qué estos modelos no producen iluminación global

### Ray Casting básico ★

- Inversión del camino de la luz: por qué lanzar rayos desde el ojo
- Generación de rayos primarios desde la cámara
- Test de intersección y selección del hit más cercano
- Coloreo simple con luz puntual

### Whitted Ray Tracing (1980)

- Shadow rays: cómo detectar oclusión
- Rayos especulares recursivos: reflexiones en espejos
- Rayos de refracción recursivos: vidrio básico
- Limitaciones: sin iluminación global, sin scattering difuso

### La Rendering Equation ★

- Formulación de Kajiya (1986): `Lo = Le + ∫ fr·Li·cos(θ) dωi`
- Interpretación de cada término
- Naturaleza recursiva: `Li` en un punto depende de `Lo` en otros puntos
- Por qué no tiene solución analítica en el caso general

### Path Tracing ★

- Estimación Monte Carlo de la integral
- Un rayo por sample: camino aleatorio desde el ojo hasta la luz
- Convergencia: error ∝ `1/√N`
- Por qué es físicamente correcto
- El problema del ruido (variance) y cómo reducirlo

### Bidirectional Path Tracing (BDPT)

- Idea: trazar caminos desde la luz Y desde la cámara, conectarlos
- Por qué reduce drásticamente el ruido en escenas con luz indirecta dominante
- Complexity vs. Vanilla PT

### Metropolis Light Transport (MLT)

- Idea: mutar caminos de luz exitosos en lugar de generar nuevos aleatoriamente
- Ideal para escenas con caustics o fuentes de luz muy ocultas
- Convergencia no uniforme: puede sobreexponer zonas

---

## Nivel 5 — Técnicas avanzadas de path tracing

Refinamientos sobre el path tracing básico que mejoran dramáticamente la calidad o velocidad.

### Variance Reduction ★

- **Next Event Estimation (NEE)** ★: muestrear la luz directamente en cada rebote difuso
  - Cómo evitar el doble conteo (flag `puede_ver_luz`)
  - Por qué solo funciona para materiales difusos (no especulares)
- **Importance Sampling** ★: muestrear más donde la función tiene valores altos
  - Muestreo coseno-ponderado para difusos
  - Muestreo de ángulo sólido para luces de área
  - PDF y corrección del peso: `f(x) / pdf(x)`
- **Multiple Importance Sampling (MIS)**: combinar múltiples estrategias de muestreo
  - Heurística de balance (Veach): `w_i = pdf_i / Σ pdf_j`
  - Reduce la varianza en materiales que mezclan difuso y especular
- **Russian Roulette**: terminar caminos de baja energía con probabilidad p y escalar por `1/(1-p)`
  - Elimina el sesgo de truncar a profundidad fija sin sesgar el estimador
- **Stratified Sampling** ★: subdividir el dominio de muestreo para evitar clustering

### Cámara avanzada ★

- **Modelo thin-lens**: profundidad de campo física
  - Relación entre apertura, distancia focal y círculo de confusión
- **Exposure y tonemapping**: simular la respuesta del sensor
- **Distorsión de lente**: barrel/pincushion (modelos avanzados)
- **Motion blur**: muestrear el tiempo del rayo para objetos en movimiento

### Materiales avanzados

- **Microfacet BRDFs**: GGX/Trowbridge-Reitz, Beckmann
  - Distribution (D), Fresnel (F), Geometry (G): el modelo Cook-Torrance
  - Importancia para metales rugosos físicamente plausibles
- **Subsurface Scattering**: piel, cera, mármol — la luz entra y sale en puntos distintos
- **Medios participantes**: niebla, humo, nubes — la luz interactúa con el volumen
- **Materiales anisotrópicos**: cepillado en una dirección (pelo, seda, metal cepillado)
- **Índice de refracción complejo**: metales conductores con componente imaginaria de IOR

### Luz y emisión

- **Luces de área**: muestreo correcto con corrección de PDF ★
- **Luces de entorno (HDRI)**: imagen esférica como fuente de luz global
  - Importance sampling de HDRI: muestrear por luminancia
- **Photon Mapping**: precalcular dónde caen los fotones de la luz
  - Útil para caustics y subsurface scattering
- **Irradiance Caching**: precalcular y reutilizar la irradiancia en puntos cercanos

---

## Nivel 6 — Aceleración y optimización

Cómo pasar de "correcto pero lento" a "correcto y rápido".

### Estructuras de aceleración avanzadas

- **SAH (Surface Area Heuristic)**: construcción óptima del BVH
  - Minimizar `costo(nodo) = C_traversal + (A_L/A_N)·N_L·C_hit + (A_R/A_N)·N_R·C_hit`
  - Binned SAH: aproximar con N cubos en lugar de evaluar cada primitiva
- **MBVH / QBVH**: nodos con 4 u 8 hijos (más cache-friendly)
- **HLBVH**: construcción en GPU usando Morton codes
- **Re-fitting**: actualizar el BVH para escenas dinámicas sin reconstruirlo completo

### Paralelización ★

- **CPU multi-core**: `multiprocessing` vs `threading` en Python (GIL)
  - Por qué `multiprocessing` para path tracing en Python
  - Granularidad: por fila, por tile (bloque de píxeles), por sample
- **GPU (CUDA/OptiX)**: el path tracing es inherentemente paralelo (un thread por rayo)
  - Divergencia de warps: problema cuando rayos del mismo warp toman caminos distintos
  - Memory coalescing: acceso eficiente a la geometría desde GPU
- **SIMD**: vectorizar el test de intersección para procesar 4-8 rayos simultáneamente
- **Tiling**: renderizar en bloques para mejorar la localidad de cache

### Denoising

- **Filtros de imagen**: Gaussian, bilateral — suavizan ruido pero pierden detalle
- **A-trous / SVGF**: denoising temporal para renders progresivos
- **Deep Learning Denoising**: NVIDIA OptiX AI Denoiser, Intel OIDN
  - Red neuronal entrenada en pares ruidoso/limpio
  - Permite renders de muy pocos samples con calidad de muchos

### Optimizaciones de código

- Evitar allocations en el hot path (el inner loop de intersección)
- Cache-friendly data layout: AoS vs SoA para vértices y triángulos
- Precomputar valores en construcción (normales, `w` de Quad, área)
- Early exit: retornar `None` lo antes posible en los tests de intersección

---

## Nivel 7 — Implementación y sistemas

Aspectos de ingeniería de software específicos de renderers de producción.

### Arquitectura de un renderer

- **Scene graph**: representación jerárquica de la escena
- **Material system**: separación de datos (parámetros) y comportamiento (shaders)
- **Plugin system**: cargar geometría, materiales y samplers sin recompilar
- **Instancing**: reusar geometría con distintas transformaciones (mil copias del mismo árbol)

### Formatos y assets

- **OBJ / FBX / glTF**: formatos de modelos 3D, sus diferencias y limitaciones
- **EXR (OpenEXR)**: formato HDR de 32 bits por canal para imágenes de render
- **HDRI**: imágenes esféricas para iluminación de entorno
- **Texturas y UV mapping**: cómo aplicar imágenes a superficies

### Verificación y debugging

- **Test de conservación de energía**: un material en una caja espejo debe estabilizarse, no crecer
- **Test de furnace**: esfera blanca en entorno blanco uniforme debe ser invisible
  - Si la BRDF es correcta, el albedo final es exactamente 1.0
- **Visualización de AOVs**: renderizar normales, profundidad, albedo por separado para debuggear
- **Comparación con referencia**: la Cornell Box tiene valores medidos reales

### Métricas de calidad

- **MSE / RMSE**: error cuadrático medio respecto a la referencia
- **SSIM**: índice de similitud estructural
- **Variance**: medir el ruido numéricamente, no solo visualmente
- **Convergence plots**: ruido vs. número de samples para comparar técnicas

---

## Recursos recomendados

### Libros — orden sugerido de lectura

| Título | Autor | Nivel | Enfoque |
|---|---|---|---|
| *Ray Tracing in One Weekend* | Peter Shirley | Principiante | Implementación práctica desde cero |
| *Ray Tracing: The Next Week* | Peter Shirley | Intermedio | BVH, texturas, motion blur |
| *Ray Tracing: The Rest of Your Life* | Peter Shirley | Avanzado | Monte Carlo, importance sampling |
| *Physically Based Rendering (PBRT)* | Pharr, Jakob, Humphreys | Avanzado | El libro de referencia de la industria |
| *Real-Time Rendering* | Akenine-Möller et al. | Intermedio | Rasterización y técnicas híbridas |
| *Mathematics for 3D Game Programming* | Lengyel | Fundamentos | Álgebra lineal aplicada a gráficos |

### Papers fundamentales

| Paper | Año | Aporte |
|---|---|---|
| *An Improved Illumination Model for Shaded Display* (Whitted) | 1980 | Ray tracing con reflexión y refracción |
| *The Rendering Equation* (Kajiya) | 1986 | Formulación matemática completa |
| *Robust Monte Carlo Methods for Light Transport* (Veach) | 1997 | MIS, bidirectional PT, MLT |
| *Fast, Minimum Storage Ray-Triangle Intersection* (Möller-Trumbore) | 1997 | Algoritmo estándar rayo-triángulo |
| *Importance Sampling of Area Lights* (Ureña et al.) | 2013 | Muestreo por ángulo sólido de quads |
| *Spatiotemporal Variance-Guided Filtering* (Schied et al.) | 2017 | Denoising temporal moderno |

### Cursos online

| Recurso | Plataforma | Enfoque |
|---|---|---|
| *CS 348B: Image Synthesis* | Stanford (YouTube) | Completo, académico |
| *Scratchapixel 2.0* | scratchapixel.com | Implementación paso a paso, gratis |
| *Ray Tracing Gems* | realtimerendering.com | Técnicas modernas, gratis |
| *PBRT Book online* | pbrt.org | Referencia completa, gratis |

### Implementaciones de referencia

| Proyecto | Lenguaje | Por qué estudiarlo |
|---|---|---|
| **PBRT v4** | C++ | El renderer académico de referencia |
| **Mitsuba 3** | Python/C++ | Differentiable rendering, moderno |
| **Nori** | C++ | Renderer educativo de ETH Zurich |
| **tinyraytracer** | C++ | Implementación mínima en ~200 líneas |
| **smallpt** | C++ | Path tracer en 99 líneas, clásico |

---

## Orden de estudio sugerido para este proyecto

```
Semana 1-2:   Nivel 1 completo (matemática)
              → Leer: Ray Tracing in One Weekend

Semana 3-4:   Nivel 2 (geometría) + Nivel 3 (física)
              → Leer: Ray Tracing: The Next Week

Semana 5-6:   Nivel 4 (algoritmos) + Rendering Equation
              → Leer: Ray Tracing: The Rest of Your Life

Semana 7-8:   Nivel 5 (NEE, importance sampling)
              → Leer: Veach thesis cap. 9 (MIS)

Semana 9+:    Nivel 6 (optimización) + Nivel 7 (sistemas)
              → Leer: PBRT capítulos 4, 13, 14
```

Los primeros tres libros de Peter Shirley son **gratuitos online** en `raytracing.github.io` y son el mejor punto de partida para ir de cero a un path tracer funcional en pocas semanas.
