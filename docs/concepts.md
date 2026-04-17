# Conceptos fundamentales — Ray Tracing y Path Tracing

Este documento explica desde cero cómo funciona el renderizado fotorrealista: qué problema resuelve, cómo se llegó a él históricamente, y la matemática que lo sustenta.

---

## Tabla de contenidos

1. [El problema: cómo generar una imagen 3D](#1-el-problema-cómo-generar-una-imagen-3d)
2. [Sin trazado de rayos vs con trazado de rayos: la diferencia fundamental](#2-sin-trazado-de-rayos-vs-con-trazado-de-rayos-la-diferencia-fundamental)
3. [La evolución: de los píxeles proyectados a la luz simulada](#3-la-evolución-de-los-píxeles-proyectados-a-la-luz-simulada)
4. [Rasterización: el enfoque clásico](#4-rasterización-el-enfoque-clásico)
5. [Ray Casting: ver con rayos](#5-ray-casting-ver-con-rayos)
6. [Ray Tracing: luz que rebota](#6-ray-tracing-luz-que-rebota)
7. [Path Tracing: la ecuación completa](#7-path-tracing-la-ecuación-completa)
8. [La física de la luz](#8-la-física-de-la-luz)
9. [Matemática de intersecciones](#9-matemática-de-intersecciones)
10. [Materiales y BRDF](#10-materiales-y-brdf)
11. [Monte Carlo: resolver integrales con azar](#11-monte-carlo-resolver-integrales-con-azar)
12. [Aceleración: BVH](#12-aceleración-bvh)
13. [La Cornell Box](#13-la-cornell-box)

---

## 1. El problema: cómo generar una imagen 3D

Una imagen es una grilla de píxeles. Cada píxel tiene un color. El problema central del renderizado 3D es:

> **¿De qué color debe ser cada píxel, dada una escena 3D con objetos, luces y una cámara?**

La respuesta física real es: el color de un píxel depende de cuánta luz llega al sensor de la cámara desde esa dirección. La luz sale de fuentes luminosas, rebota en superficies, se absorbe, se refracta, y finalmente una fracción llega al ojo.

Simular eso exactamente es computacionalmente imposible: hay infinitos fotones viajando en infinitas direcciones. Todos los métodos de renderizado son aproximaciones a ese proceso físico ideal.

---

## 2. Sin trazado de rayos vs con trazado de rayos: la diferencia fundamental

Antes de entrar en el detalle de cada tecnología, conviene entender qué cambia radicalmente cuando se introduce el trazado de rayos.

### Sin trazado de rayos: rasterización

En la rasterización, el motor 3D trabaja objeto por objeto. Toma cada triángulo de la escena, lo proyecta sobre la pantalla y calcula su color usando fórmulas matemáticas simples que simulan grosso modo cómo funciona la luz. Esas fórmulas no saben nada de lo que hay alrededor del objeto: no saben si hay una pared roja al lado que le daría un tinte rojizo, no saben si otro objeto lo tapa parcialmente para crear una sombra suave, no saben si hay una superficie brillante cerca que lo debería reflejar.

Para compensar, los videojuegos y motores en tiempo real inventan trucos: precalculan sombras y las guardan en texturas (shadow maps), almacenan reflexiones en cubos estáticos (cubemaps), simulan iluminación indirecta con sonidos de capturas de luz precalculadas (lightmaps o irradiance probes). Cada truco tapa un hueco que la rasterización no puede resolver de forma física, pero todos son aproximaciones que fallan en ciertos ángulos, con ciertos movimientos o con ciertas configuraciones de la escena.

El resultado es rápido, muy rápido. Pero la imagen tiene ese sabor artificial que es difícil de eliminar: las sombras tienen bordes duros o artefactos de resolución, las reflexiones no se actualizan si los objetos se mueven, la iluminación indirecta es plana y uniforme.

### Con trazado de rayos: simulación física

El trazado de rayos abandona los trucos. En lugar de trabajar objeto por objeto, trabaja píxel por píxel simulando el camino real que seguiría la luz. Para cada píxel, lanza un rayo desde el ojo hacia la escena y pregunta: ¿qué le pasa a este rayo? ¿golpea algo? ¿cómo rebota? ¿llega eventualmente a una fuente de luz?

La consecuencia directa es que todos los fenómenos de la luz emergen de forma natural, sin trucos:

- Las **sombras** aparecen solas porque si un rayo de sombra lanzado desde un punto hacia la luz es bloqueado por otro objeto, ese punto está en sombra.
- Las **reflexiones** son correctas porque se lanza un rayo en la dirección de reflexión y se ve qué hay en esa dirección, en tiempo real.
- La **iluminación indirecta** aparece porque los rayos rebotan en superficies difusas y llevan el color de esa superficie consigo.
- El **color bleeding** (el tinte rojizo que proyecta una pared roja sobre todo lo que está cerca) es gratis: ningún truco especial, simplemente los rayos que rebotan en la pared roja llegan a las otras superficies cargando ese color rojo.

El costo es el tiempo de cómputo. Simular la física real de la luz es órdenes de magnitud más caro que proyectar triángulos. Esa tensión entre calidad física y velocidad es lo que ha impulsado 50 años de evolución en esta tecnología.

---

## 3. La evolución: de los píxeles proyectados a la luz simulada

Lo que hoy llamamos "ray tracing" no nació completo. Fue una cadena de avances donde cada generación resolvía los problemas que la anterior dejaba sin respuesta. Esta es esa cadena, en orden cronológico.

---

### Paso 0 — Rasterización (sin ray tracing, años 70 en adelante)

Antes de que existiera el ray tracing, los gráficos 3D funcionaban proyectando la geometría sobre la pantalla y coloreando cada píxel con un modelo de iluminación local: cuánta luz directa llega desde una fuente puntual, sin considerar rebotes ni sombras reales.

Lo que tenía: velocidad, suficiente para renderizar en tiempo real.

Lo que le faltaba: todo lo que depende de la relación entre objetos. Las sombras, las reflexiones, la iluminación indirecta, el color bleeding. Todo se hacía con trucos y aproximaciones.

---

### Paso 1 — Ray Casting (1968, Arthur Appel)

Arthur Appel propuso en 1968 la idea central: en lugar de trabajar objeto por objeto, invertir el proceso y lanzar rayos desde el ojo hacia la escena.

La idea de invertir es práctica. En la realidad, la luz sale de las fuentes y viaja en todas las direcciones. La enorme mayoría de esos fotones nunca llegan al ojo. Es un desperdicio monumental simularlos. Appel observó que si invertimos el camino —lanzamos rayos desde el ojo hacia la escena— solo procesamos los rayos que efectivamente contribuyen a la imagen.

Con el ray casting básico se podía determinar qué objeto era visible desde cada píxel (visibilidad primaria) y lanzar un rayo de sombra hacia las luces para saber si el punto estaba iluminado o en sombra.

Lo que resolvió: visibilidad correcta, sombras duras sin trucos.

Lo que le faltaba: no había reflexiones ni refracciones. Los objetos brillantes o de vidrio seguían sin poderse representar correctamente. Y la iluminación seguía siendo local: los rebotes entre superficies no existían.

---

### Paso 2 — Ray Tracing de Whitted (1980, Turner Whitted)

En 1980, Turner Whitted publicó el paper que puso el ray tracing en el mapa. Su aporte fue hacer el proceso recursivo: cuando un rayo golpea una superficie especular, en lugar de colorear el píxel con iluminación local, se lanza otro rayo en la dirección de reflexión y se ve qué hay en esa dirección. Si golpea vidrio, se lanza también un rayo refractado. Y así recursivamente, hasta una profundidad máxima.

Whitted produjo una imagen famosa con una esfera metálica y una de vidrio encima de un tablero a cuadros. Reflexiones perfectas, refracción con Fresnel, sombras correctas. Para la época era magia.

Lo que resolvió: reflexiones especulares recursivas, refracción con Snell, sombras más correctas.

Lo que le faltaba: las superficies difusas (mate) seguían usando iluminación local. La iluminación indirecta entre superficies mate no existía. No había color bleeding, no había luz suave rebotando en las esquinas. La imagen de Whitted se veía física para los reflejos, pero artificial para las superficies mate.

---

### Paso 3 — Distributed Ray Tracing (1984, Robert Cook, Thomas Porter, Loren Carpenter — Pixar)

El paper "Distributed Ray Tracing" de Cook et al. introdujo una idea elegante: en lugar de lanzar un único rayo perfectamente especular, distribuir los rayos de forma aleatoria alrededor de la dirección ideal.

Si distribuyes las muestras en el tiempo, obtienes motion blur sin costo adicional. Si las distribuyes sobre el disco de la apertura de la cámara, obtienes depth of field real (desenfoque de profundidad). Si las distribuyes sobre el área de la fuente de luz en lugar de un punto, obtienes sombras suaves. Si las distribuyes alrededor de la dirección de reflexión, obtienes reflexiones borrosas (glossy, como el metal cepillado).

Todo esto surgió del mismo principio: muestrear aleatoriamente en lugar de usar un único rayo determinista.

Lo que resolvió: sombras suaves, depth of field, motion blur, reflexiones glossy. Todo con el mismo framework.

Lo que le faltaba: la iluminación indirecta entre superficies difusas seguía sin estar. Dos paredes mate no se iluminaban mutuamente. El color bleeding aún era imposible.

---

### Paso 4 — Path Tracing (1986, James Kajiya)

Kajiya formalizó en 1986 la **Rendering Equation**: la descripción matemática exacta y completa de cómo se distribuye la luz en una escena. Esta ecuación captura todo: emisión, reflexión difusa y especular, transmisión, iluminación directa e indirecta.

El problema es que la ecuación es recursiva e imposible de resolver analíticamente. Kajiya propuso estimarla con Monte Carlo: lanzar muchos rayos desde el ojo, que cada rayo tome un camino aleatorio rebotando en las superficies hasta llegar a una fuente de luz, y promediar todos los resultados.

Esto es el **path tracing**. Es el primer algoritmo que produce iluminación global correcta sin trucos: luz que rebota entre superficies difusas, color bleeding, iluminación suave en rincones, todo surge naturalmente de seguir los caminos de la luz.

El costo: necesita muchas muestras por píxel para promediar el ruido estadístico. Con pocas muestras, la imagen tiene grano (el "noise" característico del path tracing). Con muchas, converge a la solución físicamente correcta.

Lo que resolvió: iluminación global completa. Por primera vez, la luz se simulaba de verdad, no se aproximaba.

Lo que le faltaba: convergencia lenta, especialmente con fuentes de luz pequeñas o caminos de luz difíciles (cáusticas bajo el vidrio, esquinas en penumbra). El path tracing puro es ineficiente cuando los caminos de luz útiles son raros.

---

### Paso 5 — Bidirectional Path Tracing y Metropolis Light Transport (1993–1997)

El path tracing lanza rayos desde la cámara. Pero ¿qué pasa si la fuente de luz está en una habitación conectada por una rendija estrecha? Los rayos desde la cámara tienen muy pocas probabilidades de atravesar esa rendija, y la imagen tarda mucho en converger.

**Bidirectional Path Tracing** (Lafortune & Willems, 1993): se lanza rayos tanto desde la cámara como desde las luces, y se conectan los caminos que se "encuentran" en el medio. Los caminos de luz difíciles se vuelven mucho más eficientes de encontrar.

**Metropolis Light Transport** (Veach & Guibas, 1997): va más lejos. Cuando encuentra un camino de luz que contribuye significativamente a la imagen, lo *muta* ligeramente para explorar rutas similares en lugar de seguir muestreando al azar. Es un algoritmo de cadena de Markov (MCMC) aplicado al transporte de luz. Especialmente efectivo en escenas con cáusticas complejas o iluminación extremadamente indirecta.

Lo que resolvieron: escenas con iluminación difícil de encontrar. Imágenes que tardarían días con path tracing puro se podían renderizar en horas.

---

### Paso 6 — Photon Mapping (1996–2001, Henrik Wann Jensen)

Una aproximación diferente al problema. En lugar de un solo algoritmo de una pasada, Jensen propuso dos pasadas:

**Pasada 1 (fotones):** Emitir fotones desde las fuentes de luz hacia la escena. Dejar que reboten. Almacenar dónde aterrizan en un mapa espacial (el "photon map").

**Pasada 2 (render):** Renderizar con ray tracing convencional, pero cuando se necesita estimar la iluminación indirecta en un punto, consultar el mapa de fotones para ver cuánta energía aterrizó cerca.

La ventaja principal: las cáusticas (el patrón de luz concentrada que proyecta una esfera de vidrio sobre la mesa) son naturales con photon mapping porque los fotones las forman físicamente en la primera pasada. Con path tracing puro, las cáusticas convergen extraordinariamente despacio.

Lo que resolvió: cáusticas realistas, convergencia más rápida en ciertos escenarios.

Lo que le faltaba: es un algoritmo con sesgo (bias) — introduce un error sistemático controlable pero no garantiza la convergencia exacta a la solución correcta de la misma forma que el path tracing.

---

### Paso 7 — Ray Tracing en tiempo real en GPU (2018, NVIDIA RTX)

Hasta 2018, todo el ray tracing de calidad era offline: horas o días de cómputo en CPU para producir una imagen. Los videojuegos y aplicaciones en tiempo real eran territorio exclusivo de la rasterización.

NVIDIA introdujo en 2018 las GPUs RTX con unidades de hardware dedicadas a acelerar el BVH traversal y el test de intersección rayo-triángulo: las RT Cores. Al mismo tiempo, Microsoft estandarizó DirectX Raytracing (DXR) como API para acceder a estas capacidades desde cualquier aplicación.

El resultado: ray tracing en tiempo real, a 60 fps o más. No path tracing completo (eso sigue siendo demasiado lento para tiempo real), sino una combinación híbrida: rasterización para la mayoría del frame, y ray tracing para los efectos que más lo benefician (reflexiones, sombras suaves, iluminación global aproximada).

Lo que resolvió: la brecha entre calidad offline y tiempo real se redujo drásticamente. Los videojuegos pueden tener reflexiones de ray tracing, sombras correctas y algo de iluminación global, todo a 60 fps.

Lo que le faltaba: sigue sin ser path tracing completo. Las muestras son limitadas, el ruido es agresivo. Aquí entra la IA.

---

### Paso 8 — Denoising con IA y path tracing en tiempo real (2018–hoy)

El ruido del path tracing a pocas muestras es el obstáculo principal para el tiempo real. La solución moderna: entrenar una red neuronal para que aprenda a limpiar ese ruido.

**DLSS 3.5 Ray Reconstruction** de NVIDIA, **Intel Open Image Denoise**, y el denoiser de OptiX son ejemplos de redes neuronales que toman una imagen ruidosa con 1-4 samples por píxel y producen una imagen limpia que parece tener cientos. Se entrenan con millones de pares (imagen ruidosa, imagen limpia de referencia) y aprenden a reconocer los patrones del ruido de Monte Carlo.

El resultado combinado hoy: hardware de ray tracing + denoising neural + técnicas de reconstrucción temporal (aprovechar frames anteriores para refinar el frame actual) permite tener algo muy cercano a path tracing completo en tiempo real.

**Lumen** de Unreal Engine 5 es otro ejemplo: un sistema de iluminación global dinámica en tiempo real que combina ray tracing de distancia (signed distance fields) y ray tracing de hardware para aproximar la iluminación global de forma eficiente a 60 fps.

---

### Resumen de la línea del tiempo

| Año | Tecnología | Qué aportó | Qué le faltó |
|-----|-----------|-----------|--------------|
| ~70s | Rasterización | Velocidad en tiempo real | Todo lo físico (rebotes, GI real, sombras correctas) |
| 1968 | Ray Casting (Appel) | Visibilidad correcta, sombras duras reales | Reflexiones, refracción, iluminación indirecta |
| 1980 | Ray Tracing de Whitted | Reflexiones y refracciones recursivas | Iluminación indirecta en superficies mate |
| 1984 | Distributed Ray Tracing (Cook) | Sombras suaves, DOF, motion blur, glossy | Iluminación global difusa (color bleeding) |
| 1986 | Path Tracing (Kajiya) | Iluminación global completa y física | Convergencia lenta, especialmente en caminos difíciles |
| 1993–1997 | Bidirectional PT / MLT | Escenas con iluminación difícil de alcanzar | Complejidad algorítmica, más difícil de implementar |
| 1996–2001 | Photon Mapping (Jensen) | Cáusticas realistas, convergencia mejor | Sesgo en la estimación, no convergencia exacta |
| 2018 | RT Cores GPU / DXR | Ray tracing en tiempo real (híbrido) | No es path tracing completo, pocas muestras = ruido |
| 2018–hoy | Denoising neural (DLSS, OIDN) | Imagen limpia de pocas muestras, GI en tiempo real | Sigue siendo una aproximación, no física exacta |

El camino fue claro: cada generación resolvía el problema más urgente de la anterior y descubría el siguiente. Hoy, la línea entre "offline" y "tiempo real" se está borrando. Un videojuego moderno con RTX y DLSS produce imágenes que habrían requerido horas de render offline hace 10 años.

---

## 4. Rasterización: el enfoque clásico

La rasterización es el método que usa prácticamente todo videojuego y GPU en tiempo real. En lugar de simular la luz, proyecta la geometría 3D sobre la pantalla.

### Cómo funciona

```
Para cada triángulo en la escena:
  1. Proyectar sus 3 vértices al plano 2D de la pantalla
  2. Determinar qué píxeles cubre (rasterizar)
  3. Para cada píxel cubierto: calcular color con fórmulas locales
```

El color se calcula con modelos simplificados: luz ambiente, difusa (Lambertiano) y especular (Phong o Blinn-Phong). Estos modelos **no simulan luz global**: no hay rebotes reales, no hay sombras indirectas, no hay reflexiones físicas.

### Limitaciones

- **Sombras**: requieren técnicas adicionales (shadow maps), que son aproximaciones con artefactos.
- **Reflexiones**: requieren cubemaps o screen-space reflections, que son aproximaciones que fallan en muchos casos.
- **Iluminación global**: casi imposible en tiempo real. Lo que se ve como "iluminación global" en videojuegos es precalculado (lightmaps) o aproximado con irradiance probes.
- **Refracción física**: muy difícil de hacer correctamente.

La rasterización es extraordinariamente rápida (millones de triángulos por segundo), pero sacrifica precisión física.

---

## 5. Ray Casting: ver con rayos

La idea central del ray casting es invertir el camino de la luz. En la realidad, los fotones van de la fuente al ojo. Pero solo una fracción infinitesimal de todos los fotones emitidos llega al ojo. Simular eso directamente sería ineficiente.

**La inversión:** en lugar de emitir fotones desde la luz, lanzamos rayos desde el ojo hacia la escena.

```
Para cada píxel (x, y):
  1. Lanzar un rayo desde la cámara a través del píxel
  2. Encontrar el objeto más cercano que intersecta el rayo
  3. Calcular el color en ese punto
```

Esta inversión es válida porque las leyes de la óptica son reversibles en el tiempo: un camino de luz de A→B es físicamente equivalente a B→A.

### El rayo

Un rayo se define como una semirrecta:

```
P(t) = O + t · D

donde:
  O = origen (posición de la cámara)
  D = dirección (vector unitario hacia el píxel)
  t = distancia a lo largo del rayo (t > 0)
```

El objeto más cercano es el que da el menor `t` positivo al resolver la ecuación de intersección.

### Ray Casting puro (Whitted, 1980)

Arthur Appel en 1968 propuso el concepto. Turner Whitted en 1980 lo formalizó con el primer ray tracer con reflexiones y refracción. En el modelo de Whitted, al golpear un objeto:

- Si es difuso: iluminación local con shadow rays a las luces
- Si es reflectante: lanzar un rayo especular recursivo
- Si es transparente: lanzar un rayo refractado recursivo

Esto ya producía imágenes con reflexiones y refracciones correctas, algo imposible con rasterización. Pero seguía usando iluminación local para las superficies mate (sin rebotes difusos reales).

---

## 6. Ray Tracing: luz que rebota

El ray tracing moderno extiende el modelo de Whitted para incluir rebotes difusos. La idea fundamental:

> Un punto en una superficie difusa no solo recibe luz directamente de las fuentes: también recibe luz que rebotó en otras superficies. Esa luz indirecta es la que crea el color bleeding (el rojizo de la pared roja que se proyecta sobre el techo blanco) y la iluminación suave en zonas sin línea de visión directa con la luz.

El problema es que para calcular cuánta luz llega a un punto difuso desde cualquier dirección del hemisferio, habría que lanzar infinitos rayos. Aquí entra el path tracing.

---

## 7. Path Tracing: la ecuación completa

### La Ecuación de Renderizado

En 1986, James Kajiya formuló la **Rendering Equation**, la descripción matemática exacta de cómo se distribuye la luz en una escena:

```
Lo(x, ωo) = Le(x, ωo) + ∫Ω fr(x, ωi, ωo) · Li(x, ωi) · cos(θi) dωi
```

En palabras:

> La luz que sale de un punto `x` en dirección `ωo` es igual a la luz que emite el material más la integral, sobre todas las direcciones del hemisferio, de la luz que llega multiplicada por cómo el material la refleja en la dirección de salida.

- `Lo(x, ωo)` — luz saliente del punto x en dirección ωo
- `Le(x, ωo)` — luz emitida (solo para fuentes de luz)
- `fr(x, ωi, ωo)` — BRDF: cómo el material refleja de ωi a ωo
- `Li(x, ωi)` — luz que llega desde dirección ωi
- `cos(θi)` — factor de ángulo: la luz que llega en ángulo rasante contribuye menos
- `dωi` — integral sobre todas las direcciones del hemisferio

El problema: `Li(x, ωi)` es recursivo. La luz que llega desde una dirección es a su vez la luz saliente de otro punto en esa dirección, que a su vez depende de otros puntos... La ecuación se contiene a sí misma.

### La solución: Monte Carlo

La integral sobre el hemisferio no tiene solución analítica en el caso general. El path tracing la **estima numéricamente** usando Monte Carlo:

```
Para calcular el color de un píxel:
  1. Lanzar N rayos desde el ojo (samples)
  2. Para cada rayo:
     a. Encontrar punto de impacto
     b. Muestrear UNA dirección aleatoria del hemisferio
     c. Lanzar rayo en esa dirección
     d. Repetir recursivamente hasta profundidad máxima o hit de luz
  3. Promediar los N resultados
```

Cada "camino" de luz (path) es una cadena de rebotes desde la cámara hasta una fuente de luz. La estimación Monte Carlo converge a la solución exacta de la ecuación de renderizado a medida que aumenta N.

### Convergencia

El error del estimador Monte Carlo cae como `1/√N`. Para reducir el ruido a la mitad, se necesita cuadruplicar el número de muestras. Por eso el path tracing siempre tiene ruido a muestras bajas (el "grain" o "noise" característico).

```
samples=100   → error ≈ 10%
samples=400   → error ≈ 5%
samples=1600  → error ≈ 2.5%
samples=10000 → error ≈ 1%
```

---

## 8. La física de la luz

### Qué es la luz

La luz es radiación electromagnética en el rango visible (~380nm a ~700nm). Para ray tracing, la modelamos como partículas (fotones) que viajan en línea recta hasta interactuar con materia.

Los colores RGB en renderizado representan la radiancia: energía por área por ángulo sólido. No es lo mismo que el color perceptual del ojo humano, pero es una aproximación suficiente para imágenes fotorrealistas.

### Cómo interactúa la luz con la materia

Cuando un rayo golpea una superficie, ocurre alguna combinación de:

**Reflexión difusa (Lambertiana)**
La superficie dispersa la luz uniformemente en todas las direcciones del hemisferio. Una hoja de papel, una pared mate, la madera. El color del material absorbe parte de la luz y refleja el resto. No hay dirección preferida.

**Reflexión especular**
La superficie refleja la luz en una dirección específica: el ángulo de reflexión igual al ángulo de incidencia. Un espejo perfecto. Los metales tienen reflexión especular coloreada (el oro refleja más en amarillo que en azul).

**Refracción**
La luz pasa a través del material, cambiando de velocidad y por tanto de dirección. El vidrio, el agua, el diamante. La cantidad de curvatura depende del índice de refracción (IOR).

**Absorción**
La energía de la luz se convierte en calor. Todos los materiales absorben algo. Un material negro absorbe casi todo.

**Emisión**
El material genera luz por sí mismo (bombillas, el sol, pantallas). En el modelo matemático, esto es el término `Le` de la ecuación de renderizado.

### Conservación de energía

Un material físicamente plausible nunca puede reflejar más luz de la que recibe. La BRDF (ver sección 8) debe satisfacer esta condición. Si no, el renderer se comporta de manera inestable (escena infinitamente brillante).

---

## 9. Matemática de intersecciones

El núcleo computacional del ray tracing es resolver "¿en qué punto golpea este rayo a este objeto?"

### Rayo-Esfera

Una esfera se define como todos los puntos P a distancia r del centro C:

```
||P - C||² = r²
```

Un punto en el rayo es `P(t) = O + t·D`. Sustituyendo:

```
||O + t·D - C||² = r²
```

Sea `oc = O - C`:

```
(oc + t·D)·(oc + t·D) = r²
t²(D·D) + 2t(D·oc) + (oc·oc - r²) = 0
```

Ecuación cuadrática `at² + bt + c = 0` con:
```
a = D·D = 1  (si D está normalizado)
b = 2(D·oc)
c = oc·oc - r²
```

Discriminante: `Δ = b² - 4ac`
- `Δ < 0`: sin intersección (rayo pasa de largo)
- `Δ = 0`: intersección tangente
- `Δ > 0`: dos intersecciones (entrada y salida)

```
t = (-b ± √Δ) / 2a
```

Se toma el t menor positivo (la entrada), dentro del rango `[t_min, t_max]`.

### Rayo-Plano (para Quad)

Un plano se define por su normal `n` y un punto `Q` en él:

```
n · P = n · Q = D
```

Sustituyendo el rayo:

```
n · (O + t·D) = D
t = (D - n·O) / (n·D)
```

Si `n·D ≈ 0`, el rayo es paralelo al plano (sin intersección).

Una vez obtenido el punto de intersección, se verifica si está dentro del quad usando coordenadas baricéntricas:

```
P = Q + α·u + β·v

α y β se obtienen con productos cruzados y el vector auxiliar w = (u×v)/|u×v|²
La intersección es válida si 0 ≤ α ≤ 1 y 0 ≤ β ≤ 1
```

### Rayo-Triángulo (Möller-Trumbore, 1997)

El algoritmo más eficiente para triángulos. Resuelve el sistema lineal que iguala el punto en el rayo con el punto en el triángulo:

```
O + t·D = v0 + u·(v1-v0) + v·(v2-v0)
```

Esto se convierte en un sistema 3×3 que se resuelve mediante la regla de Cramer usando productos cruzados y puntos:

```
h = D × edge2
a = edge1 · h          (si a ≈ 0 → rayo paralelo al triángulo)

f = 1/a
s = O - v0
u = f · (s · h)        (0 ≤ u ≤ 1 para estar dentro)

q = s × edge1
v = f · (D · q)        (v ≥ 0 y u+v ≤ 1 para estar dentro)

t = f · (edge2 · q)    (distancia a lo largo del rayo)
```

Ventaja: no precalcula el plano, trabaja directamente con los vértices. O(1) con un número fijo de operaciones.

### La normal en el punto de impacto

La normal es el vector perpendicular a la superficie en el punto de intersección. Es esencial para calcular ángulos de incidencia, reflexión y refracción.

- **Esfera**: `n = (P - C) / r` (normalizado porque `||P-C|| = r`)
- **Quad / Triángulo**: normal precalculada en el constructor: `n = (u × v).normalize()`

---

## 10. Materiales y BRDF

### Qué es la BRDF

La **Bidirectional Reflectance Distribution Function** (BRDF) describe cómo un material refleja la luz. Dado un rayo de entrada en dirección `ωi` y una dirección de salida `ωo`, la BRDF `fr(ωi, ωo)` da la proporción de luz que se refleja de una a otra.

Propiedades que debe cumplir toda BRDF física:
1. **No negatividad**: `fr ≥ 0`
2. **Reciprocidad de Helmholtz**: `fr(ωi, ωo) = fr(ωo, ωi)`
3. **Conservación de energía**: la luz total reflejada no supera la incidente

### BRDF Lambertiana (materiales difusos)

La BRDF más simple. La luz se refleja igual en todas las direcciones:

```
fr(ωi, ωo) = albedo / π
```

El factor `π` normaliza la integral sobre el hemisferio para que la energía se conserve. El `albedo` (entre 0 y 1) indica qué fracción de la luz se refleja (1 = espejo blanco perfecto, 0 = negro absoluto).

La dirección del rayo rebotado se elige aleatoriamente en el hemisferio orientado por la normal. Promediando muchas muestras, se aproxima la integral.

### Reflexión especular (metales)

La reflexión especular perfecta sigue la ley de reflexión:

```
R = I - 2(I·N)·N
```

Donde `I` es la dirección incidente (normalizada) y `N` es la normal. El ángulo de reflexión es igual al ángulo de incidencia.

La "fuzziness" o rugosidad del metal añade una perturbación aleatoria al vector reflejado:

```
R_fuzzy = R + fuzz · random_in_unit_sphere()
```

Esto simula microsuperficies irregulares que dispersan la reflexión. `fuzz=0` es un espejo perfecto; `fuzz=1` la reflexión es casi aleatoria.

### Refracción y Fresnel (vidrio)

#### Ley de Snell

Cuando la luz pasa de un medio con índice de refracción `η₁` a otro con `η₂`, cambia de dirección:

```
η₁ · sin(θ₁) = η₂ · sin(θ₂)
```

En forma vectorial, la dirección refractada se calcula descomponiéndola en componente perpendicular y paralela a la normal:

```
r_perp  = (η₁/η₂) · (I + cos(θ₁)·N)
r_paral = -√(1 - |r_perp|²) · N
r = r_perp + r_paral
```

#### Reflexión interna total

Si `η₁/η₂ · sin(θ₁) > 1`, la ecuación de Snell no tiene solución: la luz no puede salir del medio más denso y se refleja completamente. Es el fenómeno que hace que el fondo de una piscina parezca brillante cuando se mira en ángulo raso.

#### Efecto Fresnel

En la práctica, un material transparente como el vidrio refleja más luz a ángulos rasantes. Si mirás una ventana de frente ves a través; si la mirás desde un ángulo muy inclinado, ves un reflejo casi perfecto.

La aproximación de Schlick simplifica el cálculo:

```
r₀ = ((η₁ - η₂) / (η₁ + η₂))²
R(θ) = r₀ + (1 - r₀) · (1 - cos θ)⁵
```

`R(θ)` es la probabilidad de reflexión vs refracción. En el path tracer, se elige aleatoriamente: con probabilidad `R(θ)` el rayo refleja, con probabilidad `1 - R(θ)` refracta. Promediando muchas muestras, se obtiene el efecto Fresnel correcto.

---

## 11. Monte Carlo: resolver integrales con azar

### El problema

La ecuación de renderizado requiere calcular una integral sobre el hemisferio:

```
∫Ω fr(ωi, ωo) · Li(ωi) · cos(θi) dωi
```

Esta integral no tiene solución analítica en el caso general (escenas arbitrarias). La dimensión del problema es enorme: para cada punto de la escena, en cada dirección, es necesario evaluar la luz que llega, que a su vez depende de todos los demás puntos.

### La solución Monte Carlo

El **estimador Monte Carlo** aproxima una integral muestreando aleatoriamente:

```
∫ f(x) dx ≈ (1/N) · Σ f(xᵢ) / p(xᵢ)
```

Donde `p(xᵢ)` es la probabilidad de haber elegido la muestra `xᵢ`. Si las muestras son uniformes en el dominio, `p(xᵢ)` es constante y se simplifica.

Aplicado al path tracing: lanzar N rayos por píxel, calcular el color de cada uno siguiendo un camino aleatorio hasta la luz, y promediar. Cada rayo es una muestra de la integral.

### Importance Sampling

Si las muestras se eligen de forma inteligente (más muestras donde la función tiene valores altos), el estimador converge más rápido. Esto se llama **importance sampling**.

El componente `cos(θ)` de la ecuación de renderizado hace que las direcciones cercanas a la normal contribuyan más. Muestrear más en esas direcciones (distribución coseno-ponderada) reduce el número de muestras necesarias para la misma calidad.

### Next Event Estimation (NEE)

El path tracing puro espera que el rayo llegue a una fuente de luz por azar, lo que puede ser muy ineficiente si la luz es pequeña. La **NEE** mejora esto muestreando directamente hacia la luz en cada rebote difuso:

```
Para cada rebote difuso:
  1. Calcular contribución directa: shadow ray hacia la luz (NEE)
  2. Calcular contribución indirecta: rayo en dirección aleatoria (path tracing)
  3. Combinar ambas
```

Para evitar contar la luz dos veces (una por NEE y otra si el rayo indirecto golpea la luz), los rayos indirectos ignoran la emisión de las fuentes de luz cuando las golpean.

### Muestreo estratificado

En lugar de generar los N samples completamente aleatorios en el píxel, el muestreo estratificado divide el píxel en subregiones y genera una muestra por subregión. Esto reduce la varianza porque las muestras quedan más uniformemente distribuidas, mejorando la convergencia por el mismo número de rayos.

---

## 12. Aceleración: BVH

### El problema de escalar

Un path tracer ingenuo prueba cada rayo contra cada objeto de la escena: O(n) por rayo. Con 69.000 triángulos y millones de rayos, esto es prohibitivo.

### Bounding Volume Hierarchy

El **BVH** es un árbol binario que organiza los objetos espacialmente. Cada nodo del árbol tiene:
- Una **AABB** (Axis-Aligned Bounding Box) que envuelve todos sus objetos
- Dos hijos (nodos o geometría)

```
         [AABB grande]
        /             \
  [AABB izq]      [AABB der]
   /      \        /      \
[Esf1] [Esf2] [Quad1]  [Tri...]
```

### Traversal O(log n)

Al probar un rayo contra el BVH:

```
1. Si el rayo no golpea la AABB del nodo → descartar TODO el subárbol
2. Si golpea → probar ambos hijos recursivamente
3. Retornar el hit más cercano
```

El descarte masivo de subárboles reduce la complejidad de O(n) a O(log n) en promedio. Para el Bunny con 69.000 triángulos, en lugar de probar 69.000 intersecciones por rayo, se prueban unas ~16-17 (log₂(69000) ≈ 16).

### AABB: test de intersección (algoritmo de slabs)

La AABB es el test de rechazo rápido. Una caja alineada con los ejes se puede ver como la intersección de 3 pares de planos (slabs):

```
Para cada eje (X, Y, Z):
  t0 = (min_i - origin_i) / direction_i   (entrada al slab)
  t1 = (max_i - origin_i) / direction_i   (salida del slab)

  Si direction < 0: intercambiar t0 y t1

  t_enter = max(t_enter, t0)
  t_exit  = min(t_exit,  t1)

Si t_exit < t_enter → no hay intersección
```

Solo 6 divisiones y comparaciones, mucho más barato que una intersección esfera-rayo.

### Construcción del BVH

Los objetos se dividen recursivamente eligiendo un eje aleatorio y ordenándolos por la posición de su bounding box en ese eje. Se dividen por la mitad y se construyen los subárboles recursivamente.

```
create(objects):
  Si 1 objeto → retornar objeto directamente
  Si 2 objetos → crear nodo con ambos como hojas
  Si > 2 → elegir eje, ordenar, dividir a la mitad, recursión
```

---

## 13. La Cornell Box

### Qué es

La **Cornell Box** es una escena de referencia estándar en síntesis de imagen, creada en 1984 por el grupo de investigación de Donald Greenberg en Cornell University (de ahí el nombre). Es una caja simple con:

- Paredes **izquierda roja** y **derecha verde** (difusas)
- **Techo, piso y pared de fondo blancos** (difusos)
- Una **fuente de luz rectangular** en el techo
- Dos **objetos** en el interior (originalmente dos cajas, modernamente esferas u otros)

La caja real física fue construida y medida con instrumentos para obtener los parámetros exactos de color y geometría. Las imágenes renderizadas se pueden comparar directamente con fotografías de la caja real.

### Para qué sirve

La Cornell Box se usa como **benchmark de renderizado** porque:

**1. Tiene todos los fenómenos de luz relevantes**
- Iluminación directa desde la fuente de área
- Iluminación indirecta (la luz que rebota en las paredes blancas ilumina el techo)
- Color bleeding: el rojo de la pared izquierda y el verde de la derecha se proyectan sobre las superficies blancas cercanas
- Sombras suaves (la fuente de luz tiene área, no es puntual)
- Cáusticas si hay vidrio

**2. Es medible y verificable**
Existe una fotografía de la caja física real. Un renderer que produce la misma imagen está simulando la luz correctamente. Es una forma objetiva de validar la precisión física de un algoritmo.

**3. Es mínima pero completa**
Usa la geometría más simple posible (cajas rectangulares) pero incluye todos los fenómenos que hacen difícil el renderizado global. No hay distractores.

### Por qué el color bleeding es importante

El color bleeding es uno de los efectos más difíciles de renderizar correctamente, y es la razón principal por la que la Cornell Box fue diseñada. En una escena real con una pared roja, la luz que rebota en ella imparte un tono rojizo a todo lo que la rodea. La rasterización clásica no puede simular esto; el path tracing lo produce naturalmente porque los rayos difusos llevan el color de las superficies en que rebotan.

### La Cornell Box en este proyecto

La escena está definida en `src/scene/cornell_box.py`. Los valores numéricos de los colores y geometría están basados en las especificaciones publicadas por el equipo de Cornell:

```
Pared roja:   albedo = (0.65, 0.05, 0.05)
Pared verde:  albedo = (0.12, 0.45, 0.15)
Paredes blancas: albedo = (0.73, 0.73, 0.73)
Luz:          emisión = (15, 15, 15), posición centrada en el techo
```

La fuente de luz tiene área (es un `Quad`), lo que produce sombras suaves naturalmente. La intensidad `15` (muy por encima de `1.0`) representa que la luz es mucho más brillante que las superficies reflectantes — necesario para que la iluminación indirecta sea visible sin sobreexponer.

### El rol de los objetos centrales

Los dos objetos en el interior de la caja son los protagonistas visuales:

- Una **esfera de vidrio** (DielectricMaterial) muestra refracción, reflexión interna total y efecto Fresnel. La cáustica (el patrón de luz concentrada debajo) requiere muchos rebotes para formarse correctamente.
- Una **esfera metálica** (MetalMaterial) muestra reflexión especular y refleja el interior de la caja.

Alternativamente, el **Stanford Bunny** (69.000 triángulos) demuestra que el path tracer puede manejar geometría arbitrariamente compleja gracias al BVH.

---

## Resumen: el camino completo de un píxel

```
1. Se lanza un rayo desde la cámara a través del píxel
   └─ La cámara thin-lens añade offset aleatorio para DOF

2. El rayo traversa el BVH → O(log n) tests de intersección
   └─ AABB rechaza ramas enteras del árbol

3. Se encuentra el punto de impacto más cercano
   └─ Möller-Trumbore / cuadrática / plano-rayo

4. Se evalúa el material en ese punto
   │
   ├─ EmissiveMaterial → retornar luz emitida
   │
   ├─ DiffuseMaterial  → NEE (shadow ray a la luz) + rayo difuso aleatorio
   │   └─ BRDF Lambertiana: fr = albedo/π
   │
   ├─ MetalMaterial    → rayo reflejado (R = I - 2(I·N)N) ± fuzz
   │
   └─ DielectricMaterial → refractar (Snell) o reflejar (Fresnel probabilístico)
       └─ Si reflexión interna total → siempre reflejar

5. Recursión hasta profundidad máxima o impacto con fuente de luz

6. El color acumulado es una muestra de la Rendering Equation

7. Después de N samples, se promedia → estimador Monte Carlo
   └─ Gamma correction (^1/2.2) → conversión a [0,255]
```

La imagen final es el resultado de estimar la Rendering Equation de Kajiya para cada píxel mediante Monte Carlo, usando el path de luz que conecta cada píxel con las fuentes de luz a través de rebotes físicamente correctos.
