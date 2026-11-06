## Que significa cada parametro de los presents de main.py

Los parámetros que ves dentro de la clase Preset en main.py agrupan la configuración de renderizado, cámara y escena de forma simplificada.

Aquí tienes el significado detallado de cada uno:

### Parámetros de Render (Cómo se genera la imagen)
* ```width``` y ```height```: Definen la resolución, esto es, el ancho y alto en píxeles de la imagen final (por ejemplo: ```800``` y ```800```).
* ```samples```: Es la cantidad de "rayos de luz" o muestras que se lanzan para procesar un único píxel. Entre **mayor** sea este número, la imagen tendrá **menos ruido ("grano")**, pero tardará más tiempo en renderizarse. Para pruebas rápidas usas 50 o 100, y para un render final limpio 1000 a 2000.
* ```max_depth```: Indica el **número máximo de veces que la luz puede rebotar** en la escena.
    * Si es poco (ej. 4), la luz no entra bien al cristal o a las sombras y se ve oscuro.
    * Si es alto (ej. 10 a 12), obtendrás sombras muy ricas, reflejos realistas y cáusticas completas.
* ```output_path```: Es la ubicación y nombre del archivo donde se va a guardar la fotografía, generalmente un archivo ".png" dentro de la carpeta ```output/```.
* ```use_parallel``` (```True``` por defecto): Activa el renderizado multinúcleo. Permite que el programa use toda la potencia de tu CPU, procesando varias líneas de la imagen al mismo tiempo, lo cual es fundamental para ahorrar tiempo.
* ```gamma``` (```2.2``` por defecto): Aplica corrección de color Gamma apropiada para pantallas comunes (estándar sRGB), evitando que los colores lineales salgan apagados u oscuros.

### Parámetros de Cámara (Cómo se ve la captura)
* ```fov``` (```40.0``` por defecto): Field of View (Campo de Visión). Define en grados qué tan ancha es la mirada de la cámara (muy parecido al zoom). 40° es la medida clásica que se usa para mostrar la Cornell Box y que la caja ocupe todo el marco.
* ```aperture``` (```0.0``` por defecto): Simula la apertura de la cámara física y controla la **Profundidad de Campo (DOF o desenfoque)**.
    * Si es ```0.0``` (cámara PinHole), toda la escena estará siempre completamente nítida.
    * Un valor alto (ej. 20.0) creará un fondo/frente borroso (efecto bokeh fotográfico).
* ```focus_distance``` (```None``` por defecto): Se utiliza sólo si se usa ```aperture``` mayor a cero. Es la medida que le dice a la cámara exactamente qué lugar debe salir enfocado. Al enviarse como ```None```, el motor asume automáticamente que debe enfocar siempre al centro de la escena.

### Parámetros de Escena (Qué vamos a renderizar)
* ```scene_mode```: Simplemente indica qué objetos habrá dentro de la habitación principal. Por código vienen tres constantes en juego:
    * ```SceneMode.SPHERES```: Carga la clásica esfera de cristal transparente y otra esfera metálica.
    * ```SceneMode.BUNNY```: Ubica la prueba poligonal del "Stanford Bunny" asumiendo el tipo de refracción del cristal.
    * ```SceneMode.BOXES```: Pone bloques rectangulares (cajas) clásicas de paredes difusas dentro de la habitación.

----

## Cuando dices que el sample es la cantidad de rayos de luz que se envia a un unico pixel, quiere decir que por cada pixel de los 800x800, enviará 400 (por ejemplo)? 

¡Exactamente! Has entendido perfectamente la magnitud matemática que hay detrás del motor de render.

Si tu configuración tiene una resolución de 800x800 y le asignas ```samples=400```, la cámara disparará matemáticamente **400 rayos de luz distintos por cada uno de los píxeles**.

Si hacemos la matemática rápida, es impresionante:
* Píxeles totales: ```800 x 800 = 640,000 píxeles```
* Rayos disparados desde la cámara: ```640,000 píxeles × 400 muestras =``` **256,000,000 de rayos iniciales**.

¡Y eso es solo el principio! Porque si tú configuras el ```max_depth=8``` (ocho rebotes máximos), cada uno de esos 256 millones de rayos trazará un recorrido y rebotará en hasta 8 paredes u objetos diferentes antes de llegar a la fuente de luz. Por lo cual la computadora termina calculando fácilmente **miles de millones de intersecciones matemáticas (álgebra lineal) para generar una sola imagen**.

#### ¿Por qué se envían 400 rayos al mismo píxel?
Podrías pensar: "Si lanzo 400 rayos por el mismo agujero, ¿no verán todos exactamente lo mismo?"

La respuesta es **no**, por dos razones fundamentales en el _Path Tracing_:

1. **La luz indirecta es aleatoria**: Cuando el rayo viaja y toca una superficie mate (como una pared blanca de cartón), el rayo de luz rebota en una dirección aleatoria. Cada uno de los 400 rayos rebotará en una dirección infinitesimalmente distinta a los demás. Al promediar la luz que encuentran estos 400 caminos aleatorios cruzados, averiguamos el color real de ese píxel (a este método se le llama integración de Monte Carlo). Si solo enviaras **1 rayo** en un espacio mate, arrojaría un color al azar, haciendo que toda la imagen se vea sumamente granulada (con ruido).
2. **Suavizado de bordes microscópico (Anti-Aliasing)**: Los 400 rayos de un píxel no viajan exactamente al mismísimo centro milimétrico de ese cuadradito. El programa los dispara dispersándolos ligeramente dentro de los bordes o fronteras de ese único píxel cuadrado _(conocido como stratified sampling)_. Como unos rayos impactarán en el objeto 3D y otros fallarán (impactando el fondo), al promediar el color, tus objetos tendrán bordes perfectamente difuminados y curvos en lugar de verse "pixelados", como los escalones de los videojuegos de los años 90.

Por todo este monumental esfuerzo computacional es que el parámetro ```use_parallel=True``` es crítico en tu código, ¡le permite a tu procesador repartir ese cuarto de billón de rayos entre todos sus núcleos al mismo tiempo!

----

## Para que se vea super perfecto, cual sería la configuración extrema ultra de un renderizado? Más alla de que la computadora aguante o no

Si ignoramos las limitaciones de la vida computacional real y buscamos la perfección absoluta y total (lo que en la industria del cine o animación 3D se conoce como una imagen Ground Truth o "Verdad Base"), tendríamos que llevar la ecuación geométrica al límite.

La magia del Path Tracing es que converge hacia la pura realidad física entre más se calculen sus variables. Para una configuración astronómica, podrías añadir este preset debajo del de ```"ULTRA"``` en el arreglo ```PRESETS``` de tu ```main.py```:

```python
"NIGHTMARE_PERFECTION": Preset(
    width=3840, height=2160,       # Resolución 4K nativa
    samples=10000,                 # Eliminación matemática de cualquier grano o ruido
    max_depth=50,                  # Rebotes prácticamente infinitos
    output_path="output/render_perfecto_4k.png",
    aperture=0.0,                  # Nítido por toda la escena
),
```

#### ¿Por qué estos parámetros significan "perfección"?
Aquí la explicación física detrás de esos escalofriantes números:

1. **Resolución 4K (3840 x 2160)**: Son 8.2 millones de píxeles. Esto garantizaría que las fronteras entre un objeto metálico curvo y el fondo sean matemáticamente indetectables a simple vista, logrando un anti-aliasing fenomenal.
2. **```samples = 10000``` (10,000 rayos por píxel)**: Aquí radica la verdadera perfección. En el renderizado físico real de cristales (donde la luz tiene que rebotar miles de veces en ángulos diminutos para formarte cáusticas —los increíbles reflejos arqueados de luz que proyecta una copa de vidrio sobre una mesa—), el ruido o "grano" cuesta una enormidad eliminarlo. Un error estadístico de ```1/√10000``` significa que tu error en cálculo de iluminación es del microscópico 1%. Es una captura de la realidad.
3. **```max_depth = 50``` (50 rebotes físicos de luz)**: Generalmente, al décimo o doceavo rebote la energía de la luz está muy desgastada (las paredes oscuras absorben intensidad). Sin embargo, cuando juntas dos **objetos de vidrio entrelazados** o pones la cámara dentro de una esfera de vidrio mirando otra de agua, la luz puede quedar rebotando "atrapada" en modo de espejos infinitos antes de escapar. Computar **50 rebotes** te asegura que ni una microscópica gota de luz ambiental se va a descartar como oscuridad de forma artificial.

**Disclaimer para la vida real:** Procesar una sola foto con esa configuración (_8.2 millones de píxeles × 10,000 muestras por píxel = asombrosos 82 mil millones de rayos iniciales_)... inclusive con un procesador actual de 16-32 núcleos corriendo en paralelo al 100%, ¡probablemente tardaría entre **tres a cinco días ininterrumpidos** en renderizar una sola imagen estática en formato Python puro!

----

## Esto como se hace actualmente? Con super computadoras o utilizando tecnologia DLSS? Donde generamos una imagen pequeña perfecta y luego lo redimensionamos con IA

Has dado justo en el clavo con lo segundo, pero resulta que hoy en día la industria se divide en dos mundos complementarios (Cine vs. Videojuegos), y cada uno aborda este problema "imposible" de formas distintas pero utilizando exactamente la intuición que acabas de mencionar.

Así es como logramos hacer esto actualmente sin que se derritan nuestras computadoras:

**1\. El mundo del Cine y Animación (Avatar, Pixar, Disney)**
En la pantalla grande no podemos darnos el lujo de "inventar" píxeles porque fallos milimétricos arruinan la inmersión de una película de 200 millones de dólares. Sin embargo, no esperan a tirar 10.000 muestras; usan **dos armas principales**:

- **Granjas de Render (Render Farms / Supercomputadoras)**: Estudios como Pixar o Wētā FX (los creadores de Avatar) no usan una PC brutal, sino miles de computadoras ultra-potentes trabajando en red conectadas en granjas inmensas. Mientras tu PC procesa una línea, la granja reparte la carga para que, aun así, **un solo fotograma** completo a 4K de _Avatar: The Way of Water_ tarde entre 12 y 24 horas en completarse en la granja. ¡Y recuerda que necesitan 24 fotogramas por cada segundo!
- **Aceleración por hardware (GPU) y "AI Denoising"**: Hace unos años revolucionaron la física. Se dieron cuenta de que envés de calcular los matemáticamente tediosos **10,000 mil rayos por píxel**, podían disparar solo unos **500 a 1000 rayos** (que generarán una imagen correcta, pero ruidosa) y luego pasan esa imagen por una Inteligencia Artificial matemática conocida como **Denoiser** (ej. _NVIDIA OptiX AI Denoiser_ o _Intel OIDN_). Esa IA fue entrenada previamente con millones de fotos _Ground Truth_ perfectas vs _Tus fotos con Ruido_, y aprendió perfectamente cómo suavizar el ruido exacto respetando que el cristal siga luciendo igual de duro, limpio y brillante. Esto bajó los tiempos de cálculo de semanas a cuestión de horas.

**2\. El mundo del Tiempo Real y Videojuegos (Cyberpunk 2077 con Path Tracing)**
Para los videojuegos la cosa es extremadamente hostil. ¡Necesitan que no tarde tres días, sino sólo **16 milisegundos** por foto para llegar a los 60 FPS (cuadros por segundo) y que puedas jugarlo fluído! Aquí es donde tu idea encaja a la perfección:

- **Hardware Dedicado (Tarjetas RTX)**: En tu código usas fuerza bruta de la CPU para calcular cada matemática de choque, pero tarjetas como las RTX de NVIDIA tienen componentes de silicio soldados ("RT Cores") creados físicamente con un único propósito en la vida: cruzar de forma estúpida y obscenamente rápida cientos de millones de rayos y rebotarlos contra las mallas de los objetos en microsegundos.
- **Pocos Rayos (El gran truco)**: En lugar de disparar 400 muestras por píxel (como lo tienes arriba en tu código de ```MEDIUM```), Cyberpunk 2077 literalmente dispara 1, o incluso a veces menos de **1 solo rayo promedio por cada píxel**. La imagen interna que resulta es una papilla horrorosa e irreconocible de ruido puro blanco y negro.
- **Temporal Denoising & DLSS**: Esa papilla pasa por complejas capas de Inteligencia Artificial que hacen exactamente dos cosas:
    - Miran los fotogramas anteriores (historial temporal) del juego para predecir si ayer hubo luz en ese píxel por donde ahora no pasó el rayo y tapan los huecos del ruido rápido.
    - A esa imagen limpia microscópica en baja resolución de píxeles (ej: generada en un 720p base falso) se le aplica el algoritmo mágico de **NVIDIA DLSS 3.5**, reconstruyendo las reflexiones e inyectándole millones de píxeles hiper-escalados hasta formar tu pantalla en 4K.

Básicamente, la IA actual tiene tan memorizado el comportamiento de la luz, que le basta una pre-visualización espantosa a bajísima resolución con mínimos rebotes físicos, para alucinar de maravilla y deducir cuál debió ser el resultado matemático correcto final.